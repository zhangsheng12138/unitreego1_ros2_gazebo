#include "unitree_joint_control_tool.h"

// float 版本限幅
float clamp(float &val, float min_val, float max_val)
{
    val = std::min(std::max(val, min_val), max_val);
    return val;
}

// double 版本限幅
double clamp(double &val, double min_val, double max_val)
{
    val = std::min(std::max(val, min_val), max_val);
    return val;
}

// 带滤波的速度计算
double computeVel(double currentPos, double lastPos, double lastVel, double period)
{
    return lastVel * 0.35f + 0.65f * (currentPos - lastPos) / period;
}

// 核心：电机力矩控制算法
double computeTorque(double currentPos, double currentVel, ServoCmd &cmd)
{
    double targetPos = cmd.pos;
    double targetVel = cmd.vel;
    double targetTorque = cmd.torque;
    double posStiffness = cmd.posStiffness;
    double velStiffness = cmd.velStiffness;

    // 特殊值判断：停止对应控制模式
    if (fabs(targetPos - posStopF) < 1e-6)
        posStiffness = 0;
    if (fabs(targetVel - velStopF) < 1e-6)
        velStiffness = 0;

    // 力矩计算公式：
    // 输出力矩 = Kp*(位置误差) + Kd*(速度误差) + 前馈力矩
    double calcTorque = posStiffness * (targetPos - currentPos)
                      + velStiffness * (targetVel - currentVel)
                      + targetTorque;

    return calcTorque;
}
