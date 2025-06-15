#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point # 导入Point消息类型

# 全局配置参数，方便调整
# 初始角度值（从数据值转换为角度值）
INITIAL_X_ANGLE = 168  # 约168度
INITIAL_Y_ANGLE = 156  # 约156度

# 角度范围限制（从数据值转换为角度值）
X_MIN_ANGLE = 147  # 约147度
X_MAX_ANGLE = 198  # 约198度
Y_MIN_ANGLE = 135  # 约135度
Y_MAX_ANGLE = 180  # 约180度

# 舵机速度（仅显示用）
SERVO_SPEED = 50  # 更新为50与main.cpp匹配

# 角度转换函数 (虽然不再直接发布数据值，但保留用于显示)
def angle_to_data(angle):
    """将角度值(0-240度)转换为数据值(0-1000)"""
    if angle > 240:
        angle = 240
    if angle < 0:
        angle = 0
    data = round((angle * 1000) / 240)
    if data > 1000:
        data = 1000
    return data

def data_to_angle(data):
    """将数据值(0-1000)转换为角度值(0-240度)"""
    if data > 1000:
        data = 1000
    if data < 0:
        data = 0
    return round((data * 240) / 1000)

# 确保消息类型正确导入
try:
    # Python 3
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    # Python 2
    import Tkinter as tk
    import ttk

class ServoControllerGUI:
    def __init__(self, master):
        self.master = master
        master.title("舵机控制器 (增量模式)")
        
        # 设置窗口大小
        master.geometry("500x300")
        
        # 创建ROS节点
        rospy.init_node('servo_controller_gui')
        
        # 创建发布者 - 改为发布增量
        self.increment_pub = rospy.Publisher('angle_data', Point, queue_size=10)
        
        # 存储当前值，用于避免重复发布（以角度值存储）
        self.current_x_angle = INITIAL_X_ANGLE
        self.current_y_angle = INITIAL_Y_ANGLE
        
        # 创建角度控制框架
        self.create_x_angle_frame()
        self.create_y_angle_frame()
        
        # 添加精确输入框架
        self.create_precise_input_frame()
        
        # 添加速度固定说明标签和角度范围提示
        info_frame = ttk.Frame(master)
        info_frame.grid(row=3, column=0, padx=10, pady=5, sticky="w")
        
        speed_info = ttk.Label(info_frame, text="舵机控制模式: 增量控制 (0.1度/步)")
        speed_info.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        
        x_range_info = ttk.Label(info_frame, text="X轴角度范围: {}-{}度".format(X_MIN_ANGLE, X_MAX_ANGLE))
        x_range_info.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        
        y_range_info = ttk.Label(info_frame, text="Y轴角度范围: {}-{}度".format(Y_MIN_ANGLE, Y_MAX_ANGLE))
        y_range_info.grid(row=2, column=0, padx=10, pady=5, sticky="w")
        
        # 控制按钮
        self.create_control_buttons()
        
        # 初始化发布计时器 (200ms更新一次，降低发布频率)
        self.timer = master.after(200, self.publish_values)
        
        # 设置关闭窗口时的处理
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 初始化时不需要发布任何值，舵机保持原位
        # rospy.sleep(1.0) # 等待发布者完全初始化
        # self.publish_all_values()
    
    def create_x_angle_frame(self):
        frame = ttk.LabelFrame(self.master, text="X轴舵机角度控制 (度)")
        frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        # 使用0-240度范围
        self.x_angle_var = tk.IntVar(value=INITIAL_X_ANGLE)
        self.x_angle_scale = ttk.Scale(frame, from_=0, to=240, variable=self.x_angle_var, 
                                      orient=tk.HORIZONTAL, length=350)
        self.x_angle_scale.grid(row=0, column=0, padx=10, pady=10)
        
        self.x_angle_label = ttk.Label(frame, text=str(INITIAL_X_ANGLE)+"°")
        self.x_angle_label.grid(row=0, column=1, padx=10, pady=10)
        
        # 添加跟踪变量变化
        self.x_angle_var.trace("w", self.update_x_angle_label)
    
    def create_y_angle_frame(self):
        frame = ttk.LabelFrame(self.master, text="Y轴舵机角度控制 (度)")
        frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        # 使用0-240度范围
        self.y_angle_var = tk.IntVar(value=INITIAL_Y_ANGLE)
        self.y_angle_scale = ttk.Scale(frame, from_=0, to=240, variable=self.y_angle_var, 
                                      orient=tk.HORIZONTAL, length=350)
        self.y_angle_scale.grid(row=0, column=0, padx=10, pady=10)
        
        self.y_angle_label = ttk.Label(frame, text=str(INITIAL_Y_ANGLE)+"°")
        self.y_angle_label.grid(row=0, column=1, padx=10, pady=10)
        
        # 添加跟踪变量变化
        self.y_angle_var.trace("w", self.update_y_angle_label)
    
    def create_precise_input_frame(self):
        frame = ttk.LabelFrame(self.master, text="精确角度输入 (度)")
        frame.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        
        # X轴精确输入
        ttk.Label(frame, text="X轴:").grid(row=0, column=0, padx=5, pady=5)
        self.x_entry = ttk.Entry(frame, width=8)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)
        self.x_entry.insert(0, str(INITIAL_X_ANGLE))
        
        # Y轴精确输入
        ttk.Label(frame, text="Y轴:").grid(row=0, column=2, padx=5, pady=5)
        self.y_entry = ttk.Entry(frame, width=8)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)
        self.y_entry.insert(0, str(INITIAL_Y_ANGLE))
        
        # 设置按钮
        set_btn = ttk.Button(frame, text="设置角度", command=self.set_precise_angles)
        set_btn.grid(row=0, column=4, padx=10, pady=5)
    
    def set_precise_angles(self):
        try:
            # 获取并验证X轴输入
            x_value = int(self.x_entry.get())
            if x_value < 0:
                x_value = 0
                self.x_entry.delete(0, tk.END)
                self.x_entry.insert(0, str(0))
            elif x_value > 240:
                x_value = 240
                self.x_entry.delete(0, tk.END)
                self.x_entry.insert(0, str(240))
                
            # 获取并验证Y轴输入
            y_value = int(self.y_entry.get())
            if y_value < 0:
                y_value = 0
                self.y_entry.delete(0, tk.END)
                self.y_entry.insert(0, str(0))
            elif y_value > 240:
                y_value = 240
                self.y_entry.delete(0, tk.END)
                self.y_entry.insert(0, str(240))
            
            # 更新滑块值
            self.x_angle_var.set(x_value)
            self.y_angle_var.set(y_value)
            
            # 立即发布
            self.publish_all_values()
            
        except ValueError:
            rospy.logerr("输入的角度值必须是整数")
    
    def create_control_buttons(self):
        frame = ttk.Frame(self.master)
        frame.grid(row=4, column=0, padx=10, pady=10)
        
        self.reset_btn = ttk.Button(frame, text="重置到默认位置", command=self.reset_to_default)
        self.reset_btn.grid(row=0, column=0, padx=10, pady=10)
        
        self.start_stop_var = tk.BooleanVar(value=True)
        self.start_stop_btn = ttk.Button(frame, text="停止发布", command=self.toggle_publishing)
        self.start_stop_btn.grid(row=0, column=1, padx=10, pady=10)
        
        # 添加立即发布按钮
        self.publish_now_btn = ttk.Button(frame, text="立即发布", command=self.publish_all_values)
        self.publish_now_btn.grid(row=0, column=2, padx=10, pady=10)
    
    # 标签更新函数 - 显示角度值
    def update_x_angle_label(self, *args):
        value = self.x_angle_var.get()
        self.x_angle_label.config(text=str(value)+"°")
        # 同时更新输入框
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, str(value))
    
    def update_y_angle_label(self, *args):
        value = self.y_angle_var.get()
        self.y_angle_label.config(text=str(value)+"°")
        # 同时更新输入框
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, str(value))
    
    # 发布增量值
    def publish_all_values(self):
        try:
            x_angle = self.x_angle_var.get()
            y_angle = self.y_angle_var.get()
            
            # 计算角度差并转换为增量步数
            x_diff = x_angle - self.current_x_angle
            y_diff = y_angle - self.current_y_angle
            
            # 每0.1度为一步
            x_steps = int(round(x_diff * 10))
            y_steps = int(round(y_diff * 10))
            
            if x_steps != 0 or y_steps != 0:
                point_msg = Point()
                point_msg.x = x_steps
                point_msg.y = y_steps
                point_msg.z = 0 # z轴未使用
                
                self.increment_pub.publish(point_msg)
                rospy.loginfo("已发布增量: X=%d 步, Y=%d 步", x_steps, y_steps)
                
                # 更新当前角度值
                self.current_x_angle = x_angle
                self.current_y_angle = y_angle
            
        except Exception as e:
            rospy.logerr("发布消息时出错: %s", str(e))
    
    # 定时检查滑块值变化并发布增量
    def publish_values(self):
        if self.start_stop_var.get():
            self.publish_all_values()
        
        # 重新安排计时器
        self.timer = self.master.after(200, self.publish_values)  # 200ms检查一次
    
    # 重置舵机到默认位置
    def reset_to_default(self):
        self.x_angle_var.set(INITIAL_X_ANGLE)  # 设置X轴角度为默认值
        self.y_angle_var.set(INITIAL_Y_ANGLE)  # 设置Y轴角度为默认值
        # 立即发布更新后的值
        self.publish_all_values()
    
    # 切换发布状态
    def toggle_publishing(self):
        self.start_stop_var.set(not self.start_stop_var.get())
        if self.start_stop_var.get():
            self.start_stop_btn.config(text="停止发布")
        else:
            self.start_stop_btn.config(text="开始发布")
            # 当停止发布时，可以发送一个0增量来确保舵机停止
            try:
                point_msg = Point()
                point_msg.x = 0
                point_msg.y = 0
                point_msg.z = 0
                self.increment_pub.publish(point_msg)
                rospy.loginfo("已发布停止命令 (增量: X=0, Y=0)")
            except Exception as e:
                rospy.logerr("发布停止命令时出错: %s", str(e))
    
    # 处理窗口关闭
    def on_closing(self):
        if self.timer:
            self.master.after_cancel(self.timer)
        rospy.signal_shutdown("GUI closed")
        self.master.destroy()

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = ServoControllerGUI(root)
        
        # 使得窗口居中显示
        window_width = root.winfo_reqwidth()
        window_height = root.winfo_reqheight()
        position_right = int(root.winfo_screenwidth()/2 - window_width/2)
        position_down = int(root.winfo_screenheight()/2 - window_height/2)
        root.geometry("+{}+{}".format(position_right, position_down))
        
        root.mainloop()
    except rospy.ROSInterruptException:
        pass 