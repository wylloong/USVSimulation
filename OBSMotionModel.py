# -*- coding: utf-8 -*-
from collections import deque # 引入队列，先进先出
import math
import matplotlib.pyplot as plt
import VesselClass

class OBSMotionModel(VesselClass.Vessels):
    # 初始化
    def __init__(self, vesselNo):
        # __init__方法的第一个参数永远是self，表示创建的实例本身
        VesselClass.Vessels.__init__(self, vesselNo) # 使用父类名称直接调用
        # 输入：初始位置
        self.UpdateLocation(0, 0)
        self.UpdateKineticChara(0, 0)

    def OBSRestart(self,targetSpeed,targetSpeedAngle,Ini_x, Ini_y):
        # 更新当前状态
        self.UpdateLocation(Ini_x, Ini_y)
        self.UpdateKineticChara(targetSpeed,targetSpeedAngle)

    # 输入：目标速度，目标航向
    def UpdateMotionParameters(self,targetSpeed,targetSpeedAngle):
        self.UpdateKineticChara(targetSpeed, targetSpeedAngle)

    def UpdateLocationParameters(self, Ini_x, Ini_y):
        self.UpdateLocation(Ini_x, Ini_y)

    # 运动模型实现算法
    def MotionModelIteration(self,iteraInterval):
        return self.BoatMotionItera(iteraInterval)

# 单元测试
if __name__=="__main__":
    figure=plt.figure()
    ax=figure.add_subplot(111)
    ax.axis("equal")  # 设置图像显示的时候XY轴比例
    plt.grid(True)  # 添加网格
    plt.ion()  # interactive mode on
    # 运动模型初始化
    obsMotionModel = OBSMotionModel(1)
    obsMotionModel.OBSRestart(10,60,200,100)

    for x in range(300):
        Curr_X,Curr_Y=obsMotionModel.MotionModelIteration(0.1)
        ax.scatter(Curr_X,Curr_Y)
        plt.pause(0.01)
    print("Location X= %f ,Y=%f" % (Curr_X, Curr_Y))

    plt.ioff()
    plt.show()
