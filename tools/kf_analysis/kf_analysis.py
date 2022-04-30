import matplotlib.pyplot as plt
import numpy as np

# 本示例用于卡尔曼分析数据，示例数据文件为 kf_data.txt，可根据实际情况更改代码及数据格式

# 解决中文乱码问题(加入以下两行代码)
plt.rcParams['font.sans-serif']=['SimHei'] # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False # 用来正常显示负号

with open("kf_data_sample.txt", "r") as f: # 填入实际的数据文件路径！
    send_lines = f.read().splitlines()
send_infos = [line.split(" ") for line in send_lines]
print(send_infos)

m_yaw = [float(info[0]) for info in send_infos] # yaw测量值
c_yaw = [float(info[1]) for info in send_infos] # yaw滤波值
p_yaw = [float(info[2]) for info in send_infos] # yaw预测值

plt.title("卡尔曼分析表")
times = range(len(m_yaw)) # 获取数据长度
plt.plot(times, m_yaw,   label="m_yaw 测量值")
plt.plot(times, c_yaw,  label="c_yaw 滤波值", color="r")
plt.plot(times, p_yaw,  label="p_yaw 预测值", color="g")
# plt.scatter(recv_times, TargetAngle,label="TargetAngle")
# plt.scatter(recv_times, RealAngle,  label="RealAngle")
plt.legend()
# plt.savefig("./卡尔曼分析表.jpg")
plt.show()

