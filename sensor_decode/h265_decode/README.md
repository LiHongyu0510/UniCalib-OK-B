# 使用说明
给脚本添加执行权限：
```
chmod +x h265_decode.py
```
执行命令：

```
python3 h265_decode.py 二进制文件
示例：python3 h265_decode.py center_right_20240730_114547_bag_0.db3
```
```
python3 h265_decode_percertion.py 文件夹路径 文件保存路径
示例：python3 h265_decode_percertion.py */video   */jpeg

如果是bev数据
python3 h265_decode_percertion.py */video   */jpeg    bev_mode
```