# 数字舵机控制功能包

这个ROS功能包用于使用PWM控制数字舵机，适用于树莓派平台。

## 依赖

- ROS (tested on ROS Noetic)
- pigpio库 (用于树莓派GPIO控制)
- 标准PWM控制的数字舵机

## 安装pigpio库

在树莓派上安装pigpio库:

```bash
sudo apt-get update
sudo apt-get install pigpio python-pigpio python3-pigpio
```

## 功能包编译

```bash
cd ~/catkin_ws/
catkin_make
```

## 使用方法

1. 连接数字舵机到树莓派
   - X轴舵机连接到GPIO18
   - Y轴舵机连接到GPIO19

2. 启动舵机控制节点：

```bash
rosrun digital_servo digital_servo_node
```

## 订阅的话题

- `/x_angle_slider` (std_msgs/Int32)：X轴舵机角度控制（0-180度）
- `/y_angle_slider` (std_msgs/Int32)：Y轴舵机角度控制（0-180度）
- `angle_data` (geometry_msgs/Point)：从相机节点接收的角度数据
- `camera_mode` (std_msgs/Int32)：相机模式控制 