"""
python3 h265_mp4.py <db3文件根目录> <输出MP4绝对路径>
示例：python3 h265_mp4.py  /mnt/db3_files /autocity/bag_decode/utput.mp4"
"""

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import sys
import logging
import subprocess
import datetime


class DB3Merger:
    def __init__(self, output_mp4_path, fps=10):
        self.output_mp4_path = output_mp4_path  # 最终MP4绝对路径（用户输入）
        self.temp_dir = os.path.dirname(output_mp4_path)  # 临时文件目录（与MP4同目录）
        self.ffmpeg_path = "/autocity/sweeper/h265_decode/ffmpeg"
        self.fps = fps  # 视频帧率（10Hz，与原始数据匹配）
        self.all_h265_paths = []  # 存储提取的H.265路径（按db3修改时间排序）
        # 确保临时目录存在
        os.makedirs(self.temp_dir, exist_ok=True)

    def read_ros2bag(self, file_path):
        try:
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(uri=file_path, storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions('', '')
            reader.open(storage_options, converter_options)

            topic_types = reader.get_all_topics_and_types()
            if not topic_types:
                logging.warning(f".db3文件无有效话题：{file_path}，已跳过")
                return None, None

            # 取第一个视频话题（默认唯一）
            type_map = {topic.name: topic.type for topic in topic_types}
            video_topic = list(type_map.keys())[0]
            return video_topic, self.message_generator(reader, type_map)
            
        except Exception as e:
            logging.error(f"读取.db3文件失败 {file_path}：{str(e)}")
            return None, None

    def message_generator(self, reader, type_map):
        while reader.has_next():
            try:
                topic, data, timestamp = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                # 确保是有效H.265帧（含data字段且非空）
                if hasattr(msg, 'data') and len(msg.data) > 0:
                    yield msg  # 仅返回msg（无需时间戳，后续按db3时间排序）
            except Exception as e:
                logging.warning(f"跳过异常帧（{file_path}）：{str(e)}")
                continue

    def extract_h265(self, file_path):
        video_topic, msg_generator = self.read_ros2bag(file_path)
        if not video_topic or not msg_generator:
            return None

        # 生成H.265临时文件名（含.db3标识，避免重复）
        db3_basename = os.path.splitext(os.path.basename(file_path))[0]
        h265_filename = f"temp_{db3_basename}_{video_topic}.h265"
        h265_path = os.path.join(self.temp_dir, h265_filename)

        try:
            # 写入H.265帧数据
            with open(h265_path, "wb") as f:
                frame_count = 0
                for msg in msg_generator:
                    f.write(msg.data)
                    frame_count += 1

            # 验证H.265有效性（非空）
            if os.path.getsize(h265_path) == 0:
                logging.warning(f"H.265文件为空：{h265_path}，已删除")
                os.remove(h265_path)
                return None

            logging.info(f"提取完成：{os.path.basename(file_path)} → 共{frame_count}帧 → {os.path.basename(h265_path)}")
            return h265_path

        except Exception as e:
            logging.error(f"提取H.265失败 {file_path}：{str(e)}")
            if os.path.exists(h265_path):
                os.remove(h265_path)
            return None

    def get_db3_sorted_by_filename(self, db3_dir):
        db3_files = []
        for root, _, files in os.walk(db3_dir):
            for file in files:
                if file.endswith('.db3'):
                    db3_path = os.path.join(root, file)
                    db3_filename = os.path.basename(db3_path)
                    db3_files.append((db3_filename, db3_path))  

        if not db3_files:
            logging.warning(f"在目录 {db3_dir} 中未找到任何.db3文件")
            return []

        db3_files_sorted = sorted(db3_files, key=lambda x: x[0])
        sorted_db3_paths = [path for _, path in db3_files_sorted]

        logging.info(f"\n找到 {len(sorted_db3_paths)} 个.db3文件，按文件名升序排序如下：")
        for idx, path in enumerate(sorted_db3_paths, 1):
            filename = os.path.basename(path)
            mtime = os.path.getmtime(path)
            readable_time = datetime.datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M:%S")
            logging.info(f"  {idx}. 文件名：{filename} → 完整路径：{path}")
        logging.info("")

        return sorted_db3_paths

    def merge_h265_files(self):
        """按.db3修改时间顺序合并所有H.265文件"""
        if not self.all_h265_paths:
            logging.error("没有可合并的H.265文件")
            return None

        # 合并后的H.265临时路径
        merged_h265_path = os.path.join(self.temp_dir, "merged_all_temp.h265")

        try:
            with open(merged_h265_path, "wb") as out_f:
                for idx, h265_path in enumerate(self.all_h265_paths, 1):
                    if not os.path.exists(h265_path):
                        logging.warning(f"第{idx}个H.265文件不存在，已跳过：{os.path.basename(h265_path)}")
                        continue

                    # 二进制拼接H.265（确保流连续）
                    with open(h265_path, "rb") as in_f:
                        out_f.write(in_f.read())

                    logging.debug(f"已合并第{idx}/{len(self.all_h265_paths)}个H.265：{os.path.basename(h265_path)}")

            logging.info(f"\n所有H.265合并完成！合并后路径：{merged_h265_path}")
            logging.info(f"合并文件大小：{round(os.path.getsize(merged_h265_path)/1024/1024, 2)} MB\n")
            return merged_h265_path

        except Exception as e:
            logging.error(f"合并H.265失败：{str(e)}", exc_info=True)
            if os.path.exists(merged_h265_path):
                os.remove(merged_h265_path)
            return None

    def convert_to_mp4(self, merged_h265_path):
        """将合并后的H.265转换为指定路径的MP4"""
        if not merged_h265_path or not os.path.exists(merged_h265_path):
            logging.error("合并后的H.265文件不存在，无法转换MP4")
            return None

        # FFmpeg命令：强制10Hz帧率，避免播放过快
        command = [
            self.ffmpeg_path,
            '-y',  # 覆盖已存在的MP4
            '-f', 'hevc',  # 输入格式为H.265
            '-r', str(self.fps),  # 输入帧率（10Hz）
            '-i', merged_h265_path,  # 合并后的H.265
            '-vcodec', 'libx264',
            '-crf', '28',  # H.264编码质量控制（值越小画质越好，22-28为平衡区间）
            '-preset', 'medium',  # 编码速度/画质权衡（medium=默认平衡，slow=画质更好但速度慢
            '-r', str(self.fps),  # 输出帧率（锁定10Hz）
            '-vsync', 'vfr',  # 按时间戳同步，避免跳帧
            self.output_mp4_path  # 最终MP4绝对路径
        ]

        try:
            logging.info(f"开始转换MP4：{os.path.basename(merged_h265_path)} → {os.path.basename(self.output_mp4_path)}")
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # 检查FFmpeg执行结果
            if result.returncode != 0:
                logging.error(f"FFmpeg转换失败：{result.stderr}")
                # 清理空MP4文件
                if os.path.exists(self.output_mp4_path) and os.path.getsize(self.output_mp4_path) == 0:
                    os.remove(self.output_mp4_path)
                return None

            # 验证MP4有效性
            if os.path.exists(self.output_mp4_path) and os.path.getsize(self.output_mp4_path) > 0:
                # 读取MP4时长（兼容旧FFmpeg）
                info_cmd = [self.ffmpeg_path, '-i', self.output_mp4_path, '-v', 'error']
                info_result = subprocess.run(info_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                duration_match = __import__('re').search(r'Duration: (\d+:\d+:\d+\.\d+)', info_result.stderr)
                duration = duration_match.group(1) if duration_match else "未知"

                logging.info(f"MP4转换完成！")
                logging.info(f"路径：{self.output_mp4_path}")
                logging.info(f"时长：{duration} | 大小：{round(os.path.getsize(self.output_mp4_path)/1024/1024, 2)} MB")
                return self.output_mp4_path
            else:
                logging.error("生成的MP4为空或不存在")
                return None

        except Exception as e:
            logging.error(f"MP4转换异常：{str(e)}", exc_info=True)
            if os.path.exists(self.output_mp4_path):
                os.remove(self.output_mp4_path)
            return None

    def clean_temp_files(self):
        """清理所有临时文件（H.265），保留最终MP4"""
        logging.info("\n开始清理临时文件...")
        for h265_path in self.all_h265_paths:
            if os.path.exists(h265_path):
                try:
                    os.remove(h265_path)
                    logging.debug(f"已删除临时H.265：{os.path.basename(h265_path)}")
                except Exception as e:
                    logging.warning(f"删除临时文件失败 {os.path.basename(h265_path)}：{str(e)}")

        # 清理合并的临时H.265
        merged_temp = os.path.join(self.temp_dir, "merged_all_temp.h265")
        if os.path.exists(merged_temp):
            os.remove(merged_temp)
            logging.debug(f"已删除合并临时H.265：{os.path.basename(merged_temp)}")

        logging.info("临时文件清理完成！")

    def run(self, db3_dir):
        try:
            # 1. 按.db3修改时间排序
            sorted_db3_paths = self.get_db3_sorted_by_filename(db3_dir)
            if not sorted_db3_paths:
                logging.error("未找到任何有效.db3文件，程序退出")
                return False

            # 2. 按顺序提取每个.db3的H.265
            logging.info("开始按顺序提取H.265...")
            for db3_path in sorted_db3_paths:
                h265_path = self.extract_h265(db3_path)
                if h265_path:
                    self.all_h265_paths.append(h265_path)  # 按提取顺序（即db3修改时间顺序）存储

            if not self.all_h265_paths:
                logging.error("所有.db3均未提取到有效H.265，程序退出")
                return False

            # 3. 合并H.265（按.db3修改时间顺序）
            merged_h265 = self.merge_h265_files()
            if not merged_h265:
                logging.error("H.265合并失败，程序退出")
                self.clean_temp_files()
                return False

            # 4. 转换为MP4
            final_mp4 = self.convert_to_mp4(merged_h265)
            if not final_mp4:
                logging.error("MP4转换失败，程序退出")
                self.clean_temp_files()
                return False

            # 5. 清理临时文件
            self.clean_temp_files()

            # 6. 最终结果
            logging.info("\n" + "="*60)
            logging.info(f"最终MP4路径：{final_mp4}")
            logging.info(f"文件大小：{round(os.path.getsize(final_mp4)/1024/1024, 2)} MB")
            logging.info("="*60)
            return True

        except Exception as e:
            logging.error(f"程序运行异常：{str(e)}", exc_info=True)
            self.clean_temp_files()
            return False


def main():
    # 配置日志（同时输出到控制台和文件）
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.FileHandler(os.path.join(os.path.dirname(__file__), "db3_merge.log")),
            logging.StreamHandler()
        ]
    )

    # 1. 检查命令行参数（第二个参数为MP4绝对路径）
    if len(sys.argv) != 3:
        logging.error("用法错误！正确格式：")
        logging.error(f"python3 {sys.argv[0]} <db3文件根目录> <输出MP4绝对路径>")
        logging.error(f"示例：python3 {sys.argv[0]} /mnt/db3_files /autocity/bag_decode/merged_output.mp4")
        sys.exit(1)

    # 2. 解析参数
    db3_dir = sys.argv[1]  # 第一个参数：.db3文件根目录
    output_mp4_path = sys.argv[2]  # 第二个参数：MP4绝对路径

    # 3. 验证参数合法性
    if not os.path.isdir(db3_dir):
        logging.error(f".db3根目录不存在：{db3_dir}")
        sys.exit(1)
    # 验证MP4输出目录是否可写
    output_mp4_parent = os.path.dirname(output_mp4_path)
    if not os.access(output_mp4_parent, os.W_OK):
        logging.error(f"MP4输出目录不可写（请检查权限）：{output_mp4_parent}")
        sys.exit(1)

    # 4. 执行主流程
    merger = DB3Merger(output_mp4_path=output_mp4_path, fps=10)
    success = merger.run(db3_dir=db3_dir)
    sys.exit(0 if success else -1)


if __name__ == "__main__":
    main()