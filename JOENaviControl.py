# -*- coding: utf-8 -*-
import USVMotionModelBase
import GeoCommonBase
import matplotlib.pyplot as plt
import time
import math
from collections import deque # 引入队列，先进先出

class USVControl(USVMotionModelBase.USVMotionModel):
    def __init__(self,start_x,start_y,boatid):
        super(USVControl,self).__init__(boatid) # 如果子类实现这个函数，就覆盖了父类的这个函数，既然继承父类，就要在这个函数里显式调用一下父类的__init__()
        self.tar_RotationalVelo=0 # rotational angular velocity
        self.tar_ArrivedTime=0
        self.tar_ArrivedRadius=0
        self.tar_lastGeoX = 0
        self.tar_lastGeoY = 0
        self.Arrivedflag=False # 抵达标志位
        self.ArrivedNO=-1
        self.tar_GeoX=start_x
        self.tar_GeoY=start_y
        self.Current_X=start_x
        self.Current_Y=start_y
        self.lineSight=0
        self.OACri=False
        self.navi_Velocity = 0
        self.dTrajectory=0
        self.dHead=0
        self.outPutMessage=0

    def UpdateWaypoint(self,tar_x,tar_y,tar_arrivedRadius,tar_velocity):
        self.tar_lastGeoX=self.tar_GeoX-0.01
        self.tar_lastGeoY = self.tar_GeoY
        self.tar_GeoX = tar_x
        self.tar_GeoY = tar_y
        self.tar_ArrivedRadius = tar_arrivedRadius
        self.TargetSpeed=tar_velocity
        self.TargetHead = 90 - math.atan2(self.tar_GeoY - self.tar_lastGeoY,
                                      self.tar_GeoX - self.tar_lastGeoX) * 180 / math.pi  # 正北方向

    def UpdateWaypointforSwitchPath(self,tar_x,tar_y,tar_arrivedRadius,tar_velocity,tar_lastGeoX,tar_lastGeoY):
        # self.tar_lastGeoX=self.tar_GeoX
        # self.tar_lastGeoY = self.tar_GeoY
        self.tar_GeoX = tar_x
        self.tar_GeoY = tar_y
        self.tar_ArrivedRadius = tar_arrivedRadius


        self.TargetSpeed=tar_velocity
        self.TargetHead=90 - math.atan2(self.tar_GeoY - tar_lastGeoY,
                                    self.tar_GeoX - tar_lastGeoX) * 180 / math.pi  # 正北方向

    # 更新运动状态
    def UpdateMotionState(self,curr_x,curr_y,curr_heading,curr_velocity):
        self.Current_X=curr_x
        self.Current_Y=curr_y
        self.TargetHead=curr_heading
        self.velocity=curr_velocity
        self.direction=curr_heading
        self.que=deque([curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,
                        curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity,curr_velocity],maxlen=16)
        self.UpdateBoatOut(self.direction)

    def UpdateLastWaypoint(self,last_x,last_y):
        self.tar_lastGeoX = last_x
        self.tar_lastGeoY = last_y

    def UpdateOAState(self,OA_cricial,tar_heading,tar_velocity):
        self.OACri = OA_cricial
        self.TargetSpeed = tar_velocity
        self.TargetHead= tar_heading

    def UpdateTargetVelocity(self,tar_velocity):
        self.TargetSpeed = tar_velocity

    def USVCtrIteration(self,timeInterval):


        # 判断是否到达目标航点
        distance2Waypoint=GeoCommonBase.DistanceofPoints(self.Current_X,self.Current_Y,self.tar_GeoX,self.tar_GeoY)
        #self.globalTime+=timeInterval
        #print("距离目标 %f"%(distance2Waypoint))
        if distance2Waypoint<self.tar_ArrivedRadius:
            print("Arrived Target Waypoint!")
            self.Arrivedflag=True
            self.ArrivedNO+=1
            #self.passTime=self.globalTime
        else:
            # 避障中，向固定方向行驶
            if self.OACri==False:
                # 偏航距计算
                A, B, C = GeoCommonBase.GeneralEquation(self.tar_lastGeoX, self.tar_lastGeoY, self.tar_GeoX,
                                                        self.tar_GeoY)
                dTra = math.fabs(A * self.Current_X + B * self.Current_Y + C) / math.sqrt(A ** 2 + B ** 2)
                CurrPnt2WP = 90 - math.atan2(self.Current_Y - self.tar_lastGeoY,
                                             self.Current_X - self.tar_lastGeoX) * 180 / math.pi  # 正北方向
                # 目标参数
                self.tar_Head = 90 - math.atan2(self.tar_GeoY - self.Current_Y,
                                                self.tar_GeoX - self.Current_X) * 180 / math.pi  # 正北方向
                # 偏航距符号
                dTemp = CurrPnt2WP - self.tar_Head

                if dTemp < 0 and dTemp > -180:
                    # 船在航线左侧，期望右转，符号为-
                    dTra = -dTra
                else:
                    # 船在航线右侧，期望左转，符号为+
                    dTra = +dTra
            else:
                dTra=0

            # 速度限制
            if (self.TargetSpeed > 10):
                self.TargetSpeed = 10

            self.dTrajectory = dTra
            self.UpdateInputParameters(self.TargetSpeed, self.TargetHead)  # 更新目标运动参数

            # 航向偏差
            dHeading = self.TargetHead - self.direction  # 航向偏差
            if (dHeading > 180):
                dHeading = dHeading - 360
            if (dHeading < -180):
                dHeading = dHeading + 360
            self.dHead=dHeading
            # -180< 左转 <0  0< 右转 <180
            # 速度偏差
            dVelocity=self.TargetSpeed-self.velocity

            if math.fabs(dTra)>30: # 太大了不客观
                dTra=dTra/(math.fabs(dTra))*30
            self.MotionModelIteration(timeInterval,dHeading,dVelocity,dTra)

            if(self.outPutMessage%10==0):
                print("当前航向: %f ; 目标航向: %f ; 航向偏差: %f ; 偏航距: %f ; 当前航速: %f m/s "%(self.direction,self.TargetHead,self.dHead,dTra,self.velocity))
            self.outPutMessage += 1

if __name__=="__main__":
    figure = plt.figure(224)
    ax = figure.add_subplot(221)
    plt.grid(True)  # 添加网格
    ax1 = figure.add_subplot(222)
    plt.grid(True)  # 添加网格
    ax2 = figure.add_subplot(223)
    plt.grid(True)  # 添加网格
    ax3 = figure.add_subplot(224)
    plt.grid(True)  # 添加网格
    ax.axis("equal")  # 设置图像显示的时候XY轴比例

    plt.ion()  # interactive mode on

    # 绘制目标点
    WP_x=[0,300,300]
    WP_y=[0,200,500]
    WP_time=[0,30,20]
    WP_radius=[0,30,30]
    ax.plot(WP_x,WP_y)

    usvCtr=USVControl(0,0,0)
    #print(usvCtr.__dict__)
    usvCtr.UpdateWaypoint(0,0,10,10)
    TimeAxis=0
    while True:
        if usvCtr.Arrivedflag==True:
            usvCtr.Arrivedflag=False
            if usvCtr.ArrivedNO==(len(WP_x)-1):
                break
            # tar_x,tar_y,tar_arrivedRadius,tar_velocity
            usvCtr.UpdateWaypoint(WP_x[usvCtr.ArrivedNO+1],WP_y[usvCtr.ArrivedNO+1],WP_radius[usvCtr.ArrivedNO+1],10)
        #for x in range(1000):
        TimeAxis+=1
        usvCtr.USVCtrIteration(0.1)
        # 位置
        ax.scatter(usvCtr.Current_X, usvCtr.Current_Y)
        # 速度
        ax1.scatter(TimeAxis,usvCtr.velocity)
        # 航向角
        ax2.scatter(TimeAxis, usvCtr.direction)
        # 角速度
        ax3.scatter(TimeAxis, usvCtr.AngSpeedNew)
        #time.sleep(0.1)
        #plt.pause(0.01)

    plt.ioff()  # 关闭交互模式
    # 界面显示
    plt.show()
