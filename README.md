# ROS Packages for Franka Emika Panda

## 1. Package franka_controllers

> 功能：针对Franka Panda机器人的自定义控制器。<br>
> 启动：
>> (1)笛卡尔空间控制器：
>>>- 阻抗控制器：roslaunch franka_controllers franka_cartesian_impedance_controller.launch
>>>- 位姿控制器：roslaunch franka_controllers franka_cartesian_pose_controller.launch
>>>- 速度控制器：roslaunch franka_controllers franka_cartesian_velocity_controller.launch
>>
>> (2)关节空间控制器：
>>>- 阻抗控制器：roslaunch franka_controllers franka_joint_impedance_controller.launch
>>>- 位置控制器：roslaunch franka_controllers franka_joint_position_controller.launch
>>>- 速度控制器：roslaunch franka_controllers franka_joint_velocity_controller.launch
>>
>> (3)C空间控制器：
>>>- 力控制器：roslaunch franka_controllers franka_force_controller.launch
>>>- 力示教控制器：roslaunch franka_controllers franka_force_teaching_controller.launch

## 2. Package franka_teleoperation

> 功能：作为遥操作的从操作手节点，完成以下三项功能：
>>（1）接收主操作手发送的运动控制指令并将其映射到从操作手对应的工作空间；<br>
>>（2）将映射后的运动控制指令发送给从操作手进行执行；<br>
>>（3）开启从操作手的Hand的触觉感知功能并实时发布触觉感知数据以反馈给主操作手感知。
>
> 启动：
>> (1)遥操作从操作手节点（单独功能）：roslaunch franka_teleoperation slave_franka.launch <br>
>> (2)在Gazebo仿真器中实现遥操作从操作手节点（单独功能）：roslaunch franka_teleoperation slave_franka_gazebo.launch
