import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import cv2
import sys
import logging
import glob
import threading
class Unpack:
    def __init__(self, file_path, save_path, mode):
        self.file_path = file_path
        self.save_path = save_path
        self.mode = mode
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
        if self.mode == "bev_mode":
            basename = os.path.basename(self.file_path)
            t1, t2 = basename.split('_')[2:4]
            bag_time = f"{t1}_{t2[:4]}"
            save_path = os.path.join(self.save_path, topic_names, bag_time)
        else:
            save_path = os.path.join(self.save_path, topic_names)
        self.topics = {
            'topic': topic_names,
            'msgs': self.message_generator(reader, type_map),
            'save_path': save_path
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
                logging.error(f"Error reading next message: {e}")
                break

    def save_h265_data(self):
        h265_name = os.path.splitext(os.path.basename(self.file_path))[0] + '.h265'
        output_file_path = os.path.join(self.topics['save_path'], h265_name)
        logging.info('Saving h265 video...')
        try:
            with open(output_file_path, "wb") as f:
                for msg, _ in self.topics['msgs']:
                    f.write(msg.data)
        except Exception as e:
            logging.error(f"Error opening file for writing: {e}")
            return

        cap = cv2.VideoCapture(output_file_path)
        if not cap.isOpened():
            logging.error(f"Error opening video file: {output_file_path}")
            return
        self.topics['video'] = cap
        logging.info(f'h265 Video saved in {output_file_path}')

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
                    if self.mode == "bev_mode":
                        cv2.imwrite(img_name, img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                    else:
                        cv2.imencode('.jpg', img)[1].tofile(img_name)
                        logging.info(f"Image saved in {img_name}")
                except Exception as e:
                    logging.error(f"Error saving image: {e}")
            except StopIteration:
                break
            except Exception as e:
                logging.error(f"Error processing video frame: {e}")
                break
    def delete_files(self, extension = '.h265'):
        pattern = os.path.join(self.topics['save_path'], f"*{extension}")

        files_to_delete = glob.glob(pattern)

        for file_path in files_to_delete:
            try:
                os.remove(file_path)
                print(f"Deleted file: {file_path}")
            except Exception as e:
                print(f"Error deleting file {file_path}: {e}")
    def run(self):
        self.read_ros2bag()
        self.save_h265_data()
        self.get_image()
        self.delete_files()

def map_db3_to_parent_dir(folder_path):
    """
    创建一个字典，将 .db3 文件的上两级目录名作为键，文件路径作为值（列表形式）。

    :param folder_path: 根文件夹路径
    :return: 一个字典，键为 .db3 文件的上两级目录名，值为文件路径列表
    """
    dir_file_map = {}

    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.db3'):
                file_path = os.path.join(root, file)  # 获取文件完整路径
                # 获取上两级目录名
                parent_dir = os.path.basename(os.path.dirname(os.path.dirname(file_path)))

                # 将文件路径添加到对应键的列表中
                if parent_dir not in dir_file_map:
                    dir_file_map[parent_dir] = []
                dir_file_map[parent_dir].append(file_path)
    
    return dir_file_map

def process_files_in_dir(dir_name, file_paths, save_path, mode):
    print(f"线程启动：处理目录 {dir_name}")
    for file_path in file_paths:
        # 模拟文件处理逻辑
        try:
            print(f"解析文件：{file_path}")
            if file_path.endswith('.db3'):
                #file_path = os.path.join(root, file)
                unpack = Unpack(file_path, save_path, mode)
                unpack.run()
        except Exception as e:
            print(f"解析文件失败：{file_path}, 错误信息：{e}")
    print(f"线程完成：目录 {dir_name} 的文件解析完成")


def process_map_with_threads(dir_file_map, save_path, mode):
    threads = []
    
    for dir_name, file_paths in dir_file_map.items():
        # 创建线程
        thread = threading.Thread(target=process_files_in_dir, args=(dir_name, file_paths, save_path, mode))
        threads.append(thread)
        thread.start()  # 启动线程

    # 等待所有线程完成
    for thread in threads:
        thread.join()
    print("所有图像解析完成") 


def process_folder(folder_path, save_path, files_map):
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.db3'):
                file_path = os.path.join(root, file)
                unpack = Unpack(file_path, save_path)
                unpack.run()



if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    if len(sys.argv) < 3:
        logging.error(f"Usage: python {sys.argv[0]} <folder_path> <save_path>")
        sys.exit(1)

    folder_path = sys.argv[1]
    save_path = sys.argv[2]
    mode = sys.argv[3] if len(sys.argv) == 4 else ""
    result = map_db3_to_parent_dir(folder_path)
    process_map_with_threads(result, save_path, mode)
    #process_folder(folder_path, save_path, result)