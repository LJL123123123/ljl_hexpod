#define PI 3.1415926
#define TIME_STEP 64
#define MAX_SPEED 6.28

#define Hex_zl 1
#define LEG_NUM 2
#define hex_L1 90.00 //实物已改成90
#define hex_L2 100
#define hex_L3 100
#define XYLL (hex_L1+hex_L2+hex_L3*sin(18/180*PI))//198.401699
#define ZLL hex_L3*cos(18/180*PI)//95.105652

#define hex_L 160*1.0 // 实际移动距离为80
#define hex_H  80 // 腿抬高高度
#define hex_nn 600 // 采样数（4的倍数）
#define hex_T 2
#define hex_speed 1

#define old_key  -1