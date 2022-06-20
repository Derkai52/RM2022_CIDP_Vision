import matplotlib.pyplot as plt
import numpy as np

# 本示例用于能量机关分析数据，示例数据文件为 energy_data.txt，可根据实际情况更改代码及数据格式

# 解决中文乱码问题(加入以下两行代码)
plt.rcParams['font.sans-serif']=['SimHei'] # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False # 用来正常显示负号

# with open("bind_result_yaw.txt", "r") as f: # 填入实际的数据文件路径！
with open("ori_yaw.txt", "r") as f: # 填入实际的数据文件路径！
    send_lines = f.read().splitlines()
send_infos = [line.split(" ") for line in send_lines]
print(send_infos)

#
with open("bind_yaw.txt", "r") as f: # 填入实际的数据文件路径！
    send_lines = f.read().splitlines()
dsend_infos = [line.split(" ") for line in send_lines]
print(dsend_infos)
#

with open("fusion_yaw.txt", "r") as f: # 填入实际的数据文件路径！
    send_lines = f.read().splitlines()
ddsend_infos = [line.split(" ") for line in send_lines]
print(dsend_infos)

m_yaw = [float(info[0]) for info in send_infos] # yaw测量值
c_yaw = [-(float(info[0]))+0.24 for info in dsend_infos] # yaw滤波值
p_yaw = [float(info[0]) for info in ddsend_infos] # yaw预测值

plt.title("能量机关分析表")
times = range(len(m_yaw)) # 获取数据长度
plt.plot(times, m_yaw,   label="m_yaw 测量值")
plt.plot(times, c_yaw,  label="c_yaw 滤波值", color="r")
plt.plot(times, p_yaw,  label="p_yaw 预测值", color="g")
# plt.scatter(recv_times, TargetAngle,label="TargetAngle")
# plt.scatter(recv_times, RealAngle,  label="RealAngle")
plt.legend()
# plt.savefig("./卡尔曼分析表.jpg")
plt.show()

