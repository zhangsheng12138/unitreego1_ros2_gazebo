# 电机控制指令消息
# 
# 字段说明：
# mode: 电机控制模式
# q: 目标关节位置（弧度）
# dq: 目标关节速度（弧度/秒）
# kp: 位置控制比例增益
# kd: 速度控制微分增益
# tau: 目标关节力矩（牛顿·米）

uint8 mode
float32 q
float32 dq
float32 kp
float32 kd
float32 tau
