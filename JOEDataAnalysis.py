# -*- coding: utf-8 -*-
# JOE论文仿真数据分析
import collections
import SQLiteRW
import matplotlib.pyplot as plt
import JOENaviControl
import GeoCommonBase
import OBSMotionModel # 障碍物运动模型
import math

Point = collections.namedtuple("Point", ["x", "y"])
TDPoint = collections.namedtuple("TDPoint", ["x", "y","z"])

if __name__=="__main__":
    # <editor-fold desc="初始化环境配置">
    unitheight = 10
    unitlength=16* 1852 / 3600 * unitheight  # 网格的单位长度
    unitwidth=16* 1852 / 3600 * unitheight  # 网格的单位宽度
    wp_arrRadius=30 # 抵达半径
    iteraTimeInterval=0.1 # USV迭代周期
    # </editor-fold>

    # <editor-fold desc="人机交互初始化">
    plt.close()
    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    # ax.axis("equal")
    plt.ion()
    # </editor-fold>

    # 路径设置
    plannedPathList = []
    PlannedPath_xList=[5,8,11,19,21,22,35,45]
    PlannedPath_yList = [44, 44, 40, 40,44,44,44,44]
    PlannedPath_zList = [0, 3, 10, 18,24,25,51,61]
    isSwitchList=[False for i in range(len(PlannedPath_xList)-1)]
    hasNavigatedIndex=0
    pathIndex=0

    while pathIndex<len(PlannedPath_xList):
        plannedPathList.append(TDPoint(PlannedPath_xList[pathIndex],PlannedPath_yList[pathIndex],PlannedPath_zList[pathIndex]))
        pathIndex+=1

    print(plannedPathList)

    USV_curr_x=plannedPathList[0].x * unitwidth
    USV_curr_y=plannedPathList[0].y * unitlength
    USV_curr_z = plannedPathList[0].z * unitheight
    del plannedPathList[0]  # 使用del删除对应下标的元素
    # 目标航路点 ""wp_x","wp_y","wp_time","wp_arrRadius","LocalWP","wp_velocity","WP_criTime"
    USV_tar_x = plannedPathList[0].x * unitwidth
    USV_tar_y = plannedPathList[0].y * unitlength
    USV_tar_z = plannedPathList[0].z * unitheight
    print(plannedPathList)

    # <editor-fold desc="初始化USV和障碍物的状态">
    # 障碍物
    # 180度,20kn,对遇
    OBS_1 = OBSMotionModel.OBSMotionModel(1)
    OBS_1.OBSRestart(5 * 1852 / 3600, 88, 830, 3500)

    # 90度,18kn,左交叉
    OBS_2 = OBSMotionModel.OBSMotionModel(2)
    OBS_2.OBSRestart(9 * 1852 / 3600, 10, 2730, 1450)

    # 240度,20kn,右交叉
    OBS_3 = OBSMotionModel.OBSMotionModel(3)
    OBS_3.OBSRestart(4 * 1852 / 3600, 190, 2930, 4450)

    # USV
    USVControl = JOENaviControl.USVControl(USV_curr_x, USV_curr_y, 0)  # USV控制模型
    wp_velocity=GeoCommonBase.DistanceofPoints(USV_curr_x,USV_curr_y,USV_tar_x,USV_tar_y)/(USV_tar_z-USV_curr_z)
    # UpdateMotionState(self,curr_x,curr_y,curr_heading,curr_velocity)
    USVControl.UpdateMotionState(USV_curr_x,USV_curr_y, 90, 16* 1852 / 3600)
    USVControl.UpdateWaypoint(USV_tar_x, USV_tar_y, wp_arrRadius, wp_velocity)
    # </editor-fold>
    systemTime=0

    USV_last_x=0
    USV_last_y=0

    while(len(plannedPathList)>0): # 仍然有航路没有行驶
        # USV 状态  航行控制
        systemTime+=iteraTimeInterval
        USV_last_x = USVControl.Current_X
        USV_last_y = USVControl.Current_Y

        # 障碍物运动迭代
        OBS_1.MotionModelIteration(iteraTimeInterval)
        OBS_2.MotionModelIteration(iteraTimeInterval)
        OBS_3.MotionModelIteration(iteraTimeInterval)


        if USVControl.Arrivedflag == True:
            USVControl.Arrivedflag = False
            # del plannedPathList[0] # 删除抵达的目标点
            if len(plannedPathList)==0:
                print("抵达所有目标点,程序可以退出")
                break
            else:
                USV_tar_x = plannedPathList[0].x * unitwidth
                USV_tar_y = plannedPathList[0].y * unitlength
                USV_tar_z = plannedPathList[0].z * unitheight
                # "wp_x","wp_y","wp_time","wp_arrRadius","LocalWP","wp_velocity","WP_criTime"
                wp_velocity=GeoCommonBase.DistanceofPoints(USVControl.Current_X,USVControl.Current_Y,USV_tar_x,USV_tar_y)/(USV_tar_z-systemTime)
                USVControl.UpdateWaypoint(USV_tar_x, USV_tar_y, wp_arrRadius,wp_velocity)
                print("USV 当前位置：(%f , %f)  目标位置：(%f , %f) 速度：%f m/s" %(USVControl.Current_X,USVControl.Current_Y,USV_tar_x,USV_tar_y,wp_velocity))
        elif(USV_tar_z-systemTime<1):
            # 超过抵达时间，自动切换驶向下一航点
            if len(plannedPathList)==1:
                print("抵达所有目标点,程序可以退出")
                break
            relativeDis = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, USV_tar_x,
                                                         USV_tar_y)
            print("距离目标点： %f m. 虽然未抵达航点，但是时间约束暂时切换下一航点"%(relativeDis))
            USV_tar_lastGeoX = plannedPathList[0].x * unitwidth
            USV_tar_lastGeoY = plannedPathList[0].y * unitlength
            # del plannedPathList[0] # 删除抵达的目标点
            USV_tar_x = plannedPathList[0].x * unitwidth
            USV_tar_y = plannedPathList[0].y * unitlength
            USV_tar_z = plannedPathList[0].z * unitheight
            # "wp_x","wp_y","wp_time","wp_arrRadius","LocalWP","wp_velocity","WP_criTime"
            wp_velocity = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, USV_tar_x,
                                                         USV_tar_y) / (USV_tar_z - systemTime)
            USVControl.UpdateWaypointforSwitchPath(USV_tar_x, USV_tar_y, wp_arrRadius, wp_velocity, USV_tar_lastGeoX,
                                                   USV_tar_lastGeoY)
            print("USV 当前位置：(%f , %f)  目标位置：(%f , %f) 速度：%f m/s" % (
            USVControl.Current_X, USVControl.Current_Y, USV_tar_x, USV_tar_y, wp_velocity))


        # 切换路径
        if(abs(plannedPathList[0].z*unitheight-systemTime)<=1 and isSwitchList[hasNavigatedIndex]==False and len(plannedPathList)>1):
            isSwitchList[hasNavigatedIndex]=True
            hasNavigatedIndex+=1
            USV_tar_lastGeoX = plannedPathList[0].x * unitwidth
            USV_tar_lastGeoY = plannedPathList[0].y * unitlength
            del plannedPathList[0]
            USV_tar_x = plannedPathList[0].x * unitwidth
            USV_tar_y = plannedPathList[0].y * unitlength
            USV_tar_z = plannedPathList[0].z * unitheight
            # "wp_x","wp_y","wp_time","wp_arrRadius","LocalWP","wp_velocity","WP_criTime"
            wp_velocity = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, USV_tar_x,
                                                         USV_tar_y) / (USV_tar_z - systemTime)
            USVControl.UpdateWaypointforSwitchPath(USV_tar_x, USV_tar_y, wp_arrRadius, wp_velocity, USV_tar_lastGeoX,
                                                   USV_tar_lastGeoY)
            print("USV 当前位置：(%f , %f)  目标位置：(%f , %f)  速度：%f m/s" % (
                USVControl.Current_X, USVControl.Current_Y, USV_tar_x, USV_tar_y, wp_velocity))
            print("第 %d 次航路切换成功"%(hasNavigatedIndex))
        elif(abs(plannedPathList[0].z*unitheight-systemTime)<=1 and len(plannedPathList)==1):
            break

        # USV运动迭代
        wp_velocity = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, USV_tar_x,
                                                     USV_tar_y) / (USV_tar_z - systemTime)
        USVControl.UpdateTargetVelocity(wp_velocity)
        USVControl.USVCtrIteration(iteraTimeInterval)

        if(int(systemTime*10)%20==0):
            relativeDis1 = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, OBS_1.Current_X, OBS_1.Current_Y)
            relativeDis2 = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, OBS_2.Current_X,
                                                          OBS_2.Current_Y)
            relativeDis3 = GeoCommonBase.DistanceofPoints(USVControl.Current_X, USVControl.Current_Y, OBS_3.Current_X,
                                                          OBS_3.Current_Y)
            minDis=min(relativeDis1,relativeDis2,relativeDis3)
            print("距离障碍物最近距离： %f "%(minDis))
            plt.pause(0.01)

        if (int(systemTime * 10) % 50 == 0):
            ax.scatter(OBS_1.Current_X, OBS_1.Current_Y,color="y")
            ax.scatter(OBS_2.Current_X, OBS_2.Current_Y,color="g")
            ax.scatter(OBS_3.Current_X, OBS_3.Current_Y,color="k")

        if (int(systemTime * 10) % 20 == 0):
            USV_x_list=[]
            USV_y_list = []
            USV_x_list.append(USV_last_x)
            USV_x_list.append(USVControl.Current_X)
            USV_y_list.append(USV_last_y)
            USV_y_list.append(USVControl.Current_Y)
            print(USV_x_list)
            plt.plot(USV_x_list,USV_y_list,'-',color="r",linewidth=2)
            # ax.scatter(USVControl.Current_X, USVControl.Current_Y, color="r",linewidths=0.01)  # 人机界面展示

    plt.ioff() # 交互模式关闭
    plt.show()
