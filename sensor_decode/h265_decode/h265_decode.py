import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import cv2
import sys

class Unpack:
    def __init__(self, file_path):
        self.file_path = file_path
        self.topics = {}
        self.msg_cache = []

    def read_ros2bag(self):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        topic_names = [topic_name for topic_name in type_map.keys()][0]

        # 使用实例方法
        self.topics = {
            'topic': topic_names,
            'msgs': self.message_generator(reader, type_map),
            'save_path': os.path.join('.', os.path.splitext(os.path.basename(self.file_path))[0], topic_names)
        }
        os.makedirs(self.topics['save_path'], exist_ok=True)

    def message_generator(self, reader, type_map):
        while reader.has_next():
            try:
                topic, data, timestamp = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                if hasattr(msg, 'data'):
                    timestamp_str = str(timestamp)[:13]
                    self.msg_cache.append((msg, timestamp_str))
                    yield (msg, timestamp_str)
            except Exception as e:
                break

    def save_h265_data(self):
        h265_name = self.topics['topic'] + '.h264'
        output_file_path = os.path.join(self.topics['save_path'], h265_name)
        print('Saving h265 video....')
        try:
            with open(output_file_path, "wb") as f:
                for msg, _ in self.topics['msgs']:
                    f.write(msg.data)
        except Exception as e:
            print(f"Error opening file for writing: {e}")
            return

        cap = cv2.VideoCapture(output_file_path)
        if not cap.isOpened():
            print(f"Error opening video file: {output_file_path}")
            return
        self.topics['video'] = cap
        print(f'h265 Video saved in {output_file_path}')

    def get_image(self):
        msg_gen = iter(self.msg_cache)
        while True:
            try:
                ret, img = self.topics['video'].read()
                if not ret:
                    break
                msg, timestamp = next(msg_gen)
                frame_id = msg.header.frame_id
                img_name = os.path.join(self.topics['save_path'], f"{self.topics['topic']}_{frame_id}_{timestamp}.jpg")
                try:
                    cv2.imencode('.jpg', img)[1].tofile(img_name)
                    print(f"image saved in {img_name}")
                except Exception as e:
                    print(f"Error saving image: {e}")
            except StopIteration:
                break
            except Exception as e:
                print(f"Error processing video frame: {e}")
                break

    def run(self):
        self.read_ros2bag()
        self.save_h265_data()
        self.get_image()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <decode_file>")
        sys.exit(1)

    file_path = sys.argv[1]
    unpack = Unpack(file_path)
    unpack.run()