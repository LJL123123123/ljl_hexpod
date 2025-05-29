#ifndef USER_H
#define USER_H

#define PI 3.1415926
#define TIME_STEP 64
#define MAX_SPEED 6.28

#define Hex_zl 1
#define LEG_NUM 6 //6
#define hex_L1 90.00 //实物已改成90

#define hex_L2 100
#define hex_L3 100
#define XYLL (hex_L1+hex_L2+hex_L3*sin(18/180*PI))//198.401699
#define ZLL hex_L3*cos(18/180*PI)//95.105652

#define hex_L 160*1.5 // 实际移动距离为80
#define hex_H  80 // 腿抬高高度
#define hex_nn 600 // 采样数（4的倍数）
#define hex_T 2
#define hex_speed 1

#define motor_m1 1.3
#define motor_m2 2.5 
#define motor_m3 0.1

#define motor_I1 0.0001176
#define motor_I2 0.000196
#define motor_I3 0.0000392

#define leg_L1 0.1
#define leg_L2 0.1
#define leg_L3 0.13
#define gravity 9.81
// #define has_stopped true
// #define stopped_time 

inline double degree2rad(double degree)
{
    double rad = degree / 180.00 * PI;
    return rad;
}
inline double rad2degree(double rad)
{
    double degree = rad / PI * 180.00;
    return degree;
}

inline void theta2q(double* theta, double* q)
{
    q[1] = theta[1];
    q[2] = theta[2];
    q[3] = -(-theta[3] - degree2rad(72.0) - q[2]);
}

inline void q2theta(double* theta, double* q)
{
    theta[1] = q[1];
    theta[2] = q[2];
    theta[3] = q[3] + degree2rad(72.0) - q[2];
}

#define old_key  -1

#endif 