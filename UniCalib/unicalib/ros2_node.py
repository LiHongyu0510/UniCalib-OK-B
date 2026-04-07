"""
UniCalib ROS2 节点
提供 Action Server 接口供外部调用标定流水线
"""
from __future__ import annotations
import logging
import sys

logger = logging.getLogger(__name__)


def main():
    """ROS2 节点主入口。"""
    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=sys.argv)
        node = UniCalibNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except ImportError:
        logger.error("rclpy not available. Run inside ROS2 environment.")
        sys.exit(1)


try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import ParameterDescriptor
    from std_msgs.msg import String

    class UniCalibNode(Node):
        """UniCalib ROS2 节点 —— 提供标定触发和状态发布。"""

        def __init__(self):
            super().__init__("unicalib_node")

            # 参数
            self.declare_parameter(
                "config_path", "",
                ParameterDescriptor(description="UniCalib 配置文件路径"))
            self.declare_parameter(
                "data_path", "",
                ParameterDescriptor(description="数据目录或 rosbag 路径"))
            self.declare_parameter(
                "output_dir", "/root/calib_ws/results",
                ParameterDescriptor(description="结果输出目录"))
            self.declare_parameter(
                "auto_start", False,
                ParameterDescriptor(description="是否启动时自动运行标定"))

            # 状态发布
            self._status_pub = self.create_publisher(
                String, "/unicalib/status", 10)

            # 触发订阅
            self._trigger_sub = self.create_subscription(
                String, "/unicalib/trigger", self._on_trigger, 10)

            self.get_logger().info("UniCalib node started.")
            self._publish_status("idle")

            # 自动启动
            if self.get_parameter("auto_start").value:
                self._run_calibration()

        def _on_trigger(self, msg: String):
            """接收触发消息，开始标定。"""
            stage = msg.data if msg.data else "all"
            self.get_logger().info(f"Received calibration trigger: stage={stage}")
            self._run_calibration(stage=stage)

        def _run_calibration(self, stage: str = "all"):
            """在独立线程中运行标定流水线。"""
            import threading
            thread = threading.Thread(
                target=self._calib_thread,
                args=(stage,),
                daemon=True
            )
            thread.start()

        def _calib_thread(self, stage: str):
            """标定线程。"""
            config_path = self.get_parameter("config_path").value
            data_path = self.get_parameter("data_path").value
            output_dir = self.get_parameter("output_dir").value

            if not config_path or not data_path:
                self.get_logger().error(
                    "config_path and data_path parameters must be set.")
                self._publish_status("error: missing parameters")
                return

            self._publish_status(f"running_{stage}")
            self.get_logger().info(f"Starting calibration (stage={stage})...")

            try:
                from .core.system import UniCalibSystem
                system = UniCalibSystem(config_path)

                import yaml
                import tempfile
                with open(config_path) as f:
                    config = yaml.safe_load(f)
                config.setdefault("system", {})["output_dir"] = output_dir

                if stage == "all":
                    results = system.run_full_pipeline(data_path)
                else:
                    results = system.run_stage(stage, data_path)

                self._publish_status("completed")
                self.get_logger().info("Calibration completed successfully.")

            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")
                self._publish_status(f"error: {str(e)[:100]}")

        def _publish_status(self, status: str):
            msg = String()
            msg.data = status
            self._status_pub.publish(msg)

except ImportError:
    # rclpy 不可用时提供占位类
    class UniCalibNode:  # type: ignore
        pass
