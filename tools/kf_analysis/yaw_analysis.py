# import matplotlib.pyplot as plt
# import numpy as np

# # 本示例用于卡尔曼分析数据，示例数据文件为 kf_data.txt，可根据实际情况更改代码及数据格式
#
# # 解决中文乱码问题(加入以下两行代码)
# plt.rcParams['font.sans-serif']=['SimHei'] # 用来正常显示中文标签
# plt.rcParams['axes.unicode_minus'] = False # 用来正常显示负号
# plt.title("Yaw Analysis Table")
#
# # optimization before
# plt.subplot(2,1,1)
# plt.text(0.5 ,0.5, 'optimization before',ha='center',va='center',size=24,alpha=.5)
# with open("yaw_data_not_bind.txt", "r") as f: # 填入实际的数据文件路径！
#     optimization_before = f.read().splitlines()
# send_infos = [line.split(" ") for line in optimization_before]
# print(send_infos)
#
# m_yaw1 = [float(info[0])+7 for info in send_infos] # yaw测量值
# c_yaw1 = [-(float(info[1])) for info in send_infos] # yaw滤波值
# p_yaw1 = [float(info[2]) for info in send_infos] # yaw预测值
# p_yaw_filter1 = [float(info[3])+7 for info in send_infos] # yaw预测值
# fusion_filter1 = [float(info[4]) for info in send_infos] # yaw预测值
#
# times = range(len(m_yaw1)) # 获取数据长度
# plt.plot(times, m_yaw1,   label="m_yaw 测量值", color="b")
# plt.plot(times, c_yaw1,  label="mcu_yaw 当前值", color="r")
# plt.plot(times, p_yaw1,  label="fusion 融合值", color="g")
# plt.plot(times, p_yaw_filter1,  label="m_yaw_filter 融合值", color="y")
# plt.plot(times, fusion_filter1,  label="fusion_filter 融合值", color="c")
# # plt.scatter(recv_times, TargetAngle,label="TargetAngle")
# # plt.scatter(recv_times, RealAngle,  label="RealAngle")
#
#
# # optimization after
# plt.subplot(2,1,2)
# plt.text(0.5,0.5, 'optimization after',ha='center',va='center',size=24,alpha=.5)
# with open("yaw_data_bind.txt", "r") as f: # 填入实际的数据文件路径！
#     optimization_after = f.read().splitlines()
# send_infos = [line.split(" ") for line in optimization_after]
# print(send_infos)
#
#
# m_yaw = [float(info[0])+14 for info in send_infos] # yaw测量值
# c_yaw = [-(float(info[1])) for info in send_infos] # yaw云台值
# p_yaw = [float(info[2]) for info in send_infos] # yaw融合值
# p_yaw_filter = [float(info[3])+14 for info in send_infos] # yaw消除误差后预测值
# fusion_filter = [float(info[4]) for info in send_infos] # yaw预测值
#
# times = range(len(m_yaw)) # 获取数据长度
# plt.plot(times, m_yaw,   label="m_yaw 测量值", color="b")
# plt.plot(times, c_yaw,  label="mcu_yaw 当前值", color="r")
# plt.plot(times, p_yaw,  label="fusion 融合值", color="g")
# plt.plot(times, p_yaw_filter,  label="m_yaw_filter 融合值", color="y")
# plt.plot(times, fusion_filter,  label="fusion_filter 融合值", color="c")
# # plt.scatter(recv_times, TargetAngle,label="TargetAngle")
# # plt.scatter(recv_times, RealAngle,  label="RealAngle")
#
#
# plt.legend()
# # plt.savefig("./Yaw分析表.jpg")
# plt.show()
#



import matplotlib.pyplot as plt
import numpy as np

# 本示例用于卡尔曼分析数据，示例数据文件为 kf_data.txt，可根据实际情况更改代码及数据格式

# 解决中文乱码问题(加入以下两行代码)
plt.rcParams['font.sans-serif']=['SimHei'] # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False # 用来正常显示负号
plt.title("Yaw Analysis Table")

# optimization before

with open("yaw_test_6.txt", "r") as f: # 填入实际的数据文件路径！
    optimization_before = f.read().splitlines()
send_infos = [line.split(" ") for line in optimization_before]
print(send_infos)

m_yaw1 = [float(info[0])+16.7 for info in send_infos] # yaw测量值
c_yaw1 = [-(float(info[1])) for info in send_infos] # yaw滤波值
p_yaw1 = [float(info[2]) for info in send_infos] # yaw预测值
p_yaw_filter1 = [float(info[3]) for info in send_infos] # yaw预测值
fusion_filter1 = [float(info[4]) for info in send_infos] # yaw预测值

times = range(len(m_yaw1)) # 获取数据长度
plt.plot(times, m_yaw1,   label="m_yaw 测量值", color="b")
plt.plot(times, c_yaw1,  label="mcu_yaw 当前值", color="r")
plt.plot(times, p_yaw1,  label="fusion 融合值", color="g")
plt.plot(times, p_yaw_filter1,  label="m_yaw_fake 未对齐融合值", color="y")
plt.plot(times, fusion_filter1,  label="fusion_filter 融合值", color="c")
# plt.scatter(recv_times, TargetAngle,label="TargetAngle")
# plt.scatter(recv_times, RealAngle,  label="RealAngle")



plt.legend()
# plt.savefig("./Yaw分析表.jpg")
plt.show()

