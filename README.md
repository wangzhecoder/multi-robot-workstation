# Multi_Robot_SLAM Launch Tutorial

[TOC]

## 启动仿真环境

### 启动步骤：

依次执行如下命令：

> roslaunch multi_robot_slam start.launch 
>
> rosrun rqt_image_view rqt_image_view
>
> roslaunch multi_robot_slam imageConverter.launch
>
> > 对于机器人1，因为有手柄，所以启动手柄驱动：roslaunch turtlebot_teleop ps3_teleop.launch
> >
> > 对机器人2，3，没有手柄，启动键盘控制：roslaunch turtlebot_teleop keyboard_teleop.launch

至此，单机器人任务程序启动完毕。

接下来启动机器人1电脑上的(由于最终版的multiRobWorkstation程序我还没拷贝到另两台电脑，拷贝之后任意一台都可以)：

> rosrun multi_robot_workstation multiRobWorkstation
>
> rosrun rviz rviz

至此全部程序启动完毕。

### 操作步骤

1.首先控制三台机器人移动，当三台机器人都有数据传给workstation程序后方可开始匹配与地图融合，否则程序将不会开始匹配与融合。

2.当三台机器人都已发送数据且workstation程序已接收到，先控制机器人1对环境进行建图扫描（_程序中设定让机器人2，3的场景与机器人1的进行比较，后期完善可修改程序安排机器人两两之间相互对比，这里为了实验方便，将这部分去掉了_），待机器人1扫描完毕，机器人2，3可按任意顺序进行扫描建图。

3.地图融合结果在rviz中观察。rviz需添加

> 5个map监视项，分别监视/map/map1, /map/map2, /map/map3
>
> 2个visualization_mark_array监视项，分别监视/visualization_marker2 ,/visualization_marker3,
>
> 1个marker监视项,监视/visualization_marker_array

## 启动真机环境

### 启动步骤：

依次执行如下命令：

> roslaunch rbx1_bringup turtlebot_minimal_create11.launch
>
> roslaunch rbx1_bringup turtlebot_fake_laser_freenect.launch
>
> roslaunch slam_karto karto_ddemo.launch
>
> roslaunch multi_robot_slam imageConverter.launch 
>
> > 在作为workstation的电脑上启动：
> >
> > rosrun multi_robot_workstation multiRobWorkstation
>
> roslaunch turtlebot_teleop ps3_teleop.launch或roslaunch turtlebot_teleop keyboard_teleop.launch
>
> (可选)：rosrun rqt_image_view rqt_image_view

### 操作步骤

同仿真环境

# 注意事项

## 数据传输不通

此时查看`roslaunch multi_robot_slam start.launch`终端下是否存在大片红色错误，若有，则另开一个新的终端重新启动`roslaunch master_discovery_fkie master_discovery.launch`和`roslaunch master_sync_fkie master_sync.launch`，一般情况下，问题将得以解决。若依然无效，查看网络是否连接正常，若网络连接正常，那就重启任务吧
