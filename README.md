# ROS Packages for Franka Emika Panda

## 1. Package franka_controllers

> 功能：针对Franka Panda机器人的自定义控制器。<br>
> 启动：
>> (1)主从遥操作控制器（franka_teleoperation_controller）：roslaunch franka_controllers franka_teleoperation_controller.launch <br>

## 2. Package franka_teleoperation

> 功能：作为遥操作的从操作手节点，完成以下三项功能：（1）接收主操作手发送的运动控制指令并将其映射到从操作手对应的工作空间；（2）将映射后的运动控制指令发送给从操作手进行执行；（3）开启从操作手的Hand的触觉感知功能并实时发布触觉感知数据以反馈给主操作手感知。<br>
> 启动：
>> (1)遥操作从操作手节点（单独功能）：roslaunch franka_teleoperation slave_franka.launch <br>
>> (2)在Gazebo仿真器中实现遥操作从操作手节点（单独功能）：roslaunch franka_teleoperation slave_franka_gazebo.launch
