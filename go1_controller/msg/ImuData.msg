# IMU数据消息
# 
# 字段说明：
# quaternion: 四元数，用于表示机器人的姿态，顺序为 [w, x, y, z]
# gyroscope: 陀螺仪数据，用于测量角速度，单位为弧度/秒，顺序为 [x, y, z]
# accelerometer: 加速度计数据，用于测量加速度，单位为米/秒²，顺序为 [x, y, z]

float64[] quaternion
float64[] gyroscope
float64[] accelerometer