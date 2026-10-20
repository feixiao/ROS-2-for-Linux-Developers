# 机器人底层核心遥测报文 (Robot Core Telemetry)
std_msgs/Header header

# 电池百分比 [0.0 - 100.0]
float32 battery_percentage

# CPU 与电机温度 (摄氏度)
float32 cpu_temperature
float32 motor_temperature

# 当前运行模式状态码 (0: IDLE, 1: MANUAL, 2: AUTONOMOUS, 3: EMERGENCY_STOP)
uint8 status_code

# 底盘世界坐标系位姿
float64 x
float64 y
float64 theta
