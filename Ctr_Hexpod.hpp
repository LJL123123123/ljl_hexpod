#ifndef HEXPOD_HPP
#define HEXPOD_HPP

#include "RobotRunner.hpp"
#include "DMmotor.hpp"
#include "Timer.h"
#include "Math_Tool.hpp"

 #include <iostream>
 #include <cmath>
 #include <iostream>
 #include <chrono>
 #include <ctime>

 #include "Eigen/Dense"
 #include "user.h"
 
 double first_reset = 0;
    
 
double reset_1 = 0;
double reset_2 = 0; 
class HexPodController : public RobotController
{
    public:
        DMmotor m_LegMotor[3*LEG_NUM];
        Module<HexPodController> m_update_module;
        HexPodController():RobotController("HexPodController"),
        m_LegMotor{DMmotor("leg0_motor1"),DMmotor("leg0_motor2"),DMmotor("leg0_motor3")
                ,DMmotor("leg1_motor1"),DMmotor("leg1_motor2"),DMmotor("leg1_motor3")
                ,DMmotor("leg2_motor1"),DMmotor("leg2_motor2"),DMmotor("leg2_motor3")
                ,DMmotor("leg3_motor1"),DMmotor("leg3_motor2"),DMmotor("leg3_motor3")
                ,DMmotor("leg4_motor1"),DMmotor("leg4_motor2"),DMmotor("leg4_motor3")
                ,DMmotor("leg5_motor1"),DMmotor("leg5_motor2"),DMmotor("leg5_motor3")
            },


        // m_LegMotor{DMmotor("leg0_motor1"),DMmotor("leg0_motor2"),DMmotor("leg0_motor3")
        //             ,DMmotor("leg1_motor1"),DMmotor("leg1_motor2"),DMmotor("leg1_motor3")，
        //             DMmotor("leg2_motor1"),DMmotor("leg2_motor2"),DMmotor("leg2_motor3")
        //            },

        m_update_module{MT_CONTROLLER,this,&HexPodController::update,"HexPodController_update_module"}
        {};
        
        struct Leg 
        {
        double gama;
        LowPassFilter<double>  Pos_filter[3];
        LowPassFilter<double>  Vel_filter[3];
        LowPassFilter<double> Touq_filter[3];
        };

        struct Leg legs[LEG_NUM];
                
        // void init()用于对六足机器人控制器进行初始化操作，包括查找通信接口、设置腿部角度参数以及初始化电机的控制参数
        void init() override{
            std::cout<<11<<std::endl;

            DM_USB2CAN* massage_ptr = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN1"));/*多写一个变量*/
            DM_USB2CAN* massage_ptr1 = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN2"));
            DM_USB2CAN* massage_ptr2 = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN3"));
            DM_USB2CAN* massage_ptr3 = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN4"));
            DM_USB2CAN* massage_ptr4 = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN5"));
            DM_USB2CAN* massage_ptr5 = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN6"));
            
            // if(massage_ptr == nullptr || massage_ptr1 == nullptr)
            // {
            // if(massage_ptr == nullptr || massage_ptr1 == nullptr||massage_ptr2 == nullptr)
            // {
            if(massage_ptr == nullptr || massage_ptr1 == nullptr||massage_ptr2 == nullptr
            ||massage_ptr3 == nullptr || massage_ptr4 == nullptr||massage_ptr5 == nullptr)
            {
                fprintf(stderr,"can not find communication interface\n");
                return;
            }   
            //查找通信接口
            //通过 PeriodicTaskManager 单例对象的 FindTask 方法查找名为 
            //"DM_USB2CAN1" 和 "DM_USB2CAN2" 的任务，这两个任务代表了两个不同的通信接口
            //dynamic_cast<DM_USB2CAN*>xxzxzxz DM_USB2CAN 类型的指针，
            //因为后续需要使用 DM_USB2CAN 类的功能来与电机进行通信。
            //if 语句：检查是否成功找到这两个通信接口，如果任意ss一个指针为 nullptr，
            //则输出错误信息到标准错误流，并终止 init 函数的执行。

            legs[0].gama =  degree2rad(0.0);
            legs[2].gama =  degree2rad(0.0);
            legs[4].gama =  degree2rad(0.0);
            legs[1].gama =  degree2rad(0.0);
            legs[3].gama =  degree2rad(0.0);
            legs[5].gama =  degree2rad(0.0);
            for (int j = 0; j < LEG_NUM; j++)
            {
                for(int i = 0; i < 3; i++)
                {
                    if(j == 0)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr,DMMode::MIT);/*串口在这*/
                    else if(j == 1)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr1,DMMode::MIT);
                    else if(j == 2)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr2,DMMode::MIT);
                    else if(j == 3)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr3,DMMode::MIT);
                    else if(j == 4)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr4,DMMode::MIT);
                    else if(j == 5)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr5,DMMode::MIT);
                   
                    fprintf(stderr,"leg%d_motor%d:m_LegMotor[%d]:%d\n",j,i,j*3+i,m_LegMotor[j*3+i].id);

                    m_LegMotor[j*3+i].control_p_des = 0;
                    m_LegMotor[j*3+i].control_k_d = 0.8f;
                    m_LegMotor[j*3+i].control_k_p = 25.0f;
                    m_LegMotor[j*3+i].control_v_des = 0.5f;
                }
            }
            
            
            // std::cout<<13<<std::endl;
        };
        
        //弧度，角度，theta q转换
        double degree2rad(double degree)
        {
            double rad = degree / 180.00 * PI;
            return rad;
        }

        double rad2degree(double rad)
        {
            double degree = rad / PI * 180.00;
            return degree;
        }

        void theta2q(double* theta, double* q)
        {
            q[1] = theta[1];q[2] = theta[2];
            q[3] = -(-theta[3] - degree2rad(72.0) - q[2]);
        }

        void q2theta(double* theta, double* q)   
        {
            theta[1] = q[1];theta[2] = q[2];
            theta[3] = q[3] + degree2rad(72.0) - q[2];
        }

        Eigen::Vector3d Hex_leg_p,Hex_tehta;

        void FK(double gama)
        {
            Hex_leg_p << (hex_L1 + hex_L2 * cos(Hex_tehta[1]) + hex_L3 * cos(Hex_tehta[1] + Hex_tehta[2])) * cos(Hex_tehta[0] + gama)
                        ,(hex_L1 + hex_L2 * cos(Hex_tehta[1]) + hex_L3 * cos(Hex_tehta[1] + Hex_tehta[2])) * sin(Hex_tehta[0] + gama)
                        ,hex_L2 * sin(Hex_tehta[1]) + hex_L3 * sin(Hex_tehta[1] + Hex_tehta[2]);
        }

        void IK(double gama, double* pi, double* p0, double* theta) //输出rad
        {
            double M = (p0[2] - pi[2]) * cos(gama) - (p0[1] - pi[1]) * sin(gama);
            double N = (p0[1] - pi[1]) * cos(gama) + (p0[2] - pi[2]) * sin(gama);
            double P = p0[3] - pi[3];

            
            theta[1] = atan(M / N);


            double A = M - hex_L1 * sin(theta[1]);
            double B = N - hex_L1 * cos(theta[1]);

            theta[3] = -acos((A * A + B * B + P * P - hex_L2 * hex_L2 - hex_L3 * hex_L3) / (2 * hex_L2 * hex_L3));

            double x = p0[1] - pi[1];
            double y = p0[2] - pi[2];
            double z = p0[3] - pi[3];

            A = sqrt(x * x + y * y) - hex_L1;
            B = sqrt(z * z);

            int error = 0;
            double temp_value = 0;

            if ((A * A + z * z) > (hex_L2 * hex_L2 + hex_L3 * hex_L3))
            {
                /*printf("高机位\n");*/
                double delta = asin(hex_L3 * -sin(theta[3]) / sqrt(A * A + B * B));            
                if (A <= 0)
                {
                    double ttan = atan(A / B);
                    theta[2] = (delta + ttan) - PI / 2;
                }
                else
                {
                    double ttan = atan(B / A);
                    theta[2] = (delta - ttan);
                }

            }
            else
            {
                /*printf("低机位\n");*/
                double ccos2 = acos((A * A + B * B + hex_L2 * hex_L2 - hex_L3 * hex_L3) / (2 * sqrt(A * A + B * B) * hex_L2));
                double ttan = atan(B / A);
                theta[2] = (ccos2 - ttan);
            }

        }

        Eigen::Matrix3d hex_J;
        Eigen::Vector3d hex_v_0d,hex_omega;

        void Jacbi(double gama,int leg_n)
    {
        Hex_tehta << m_LegMotor[leg_n*3+0].feedback_pos,
                     m_LegMotor[leg_n*3+1].feedback_pos,
                     m_LegMotor[leg_n*3+2].feedback_pos;
      
        double s1_gamma = sin(Hex_tehta[0] + gama); // 根据实际情况更新
        double c1_gamma = cos(Hex_tehta[0] + gama);
        double c1 = cos(Hex_tehta[0]);
        double c2 = cos(Hex_tehta[1]);
        double c3 = cos(Hex_tehta[2]);
        double s1 = sin(Hex_tehta[0]);
        double s2 = sin(Hex_tehta[1]);
        double s3 = sin(Hex_tehta[2]);
        double c2_3 = cos(Hex_tehta[1]+Hex_tehta[2]);
        double s2_3 = sin(Hex_tehta[1]+Hex_tehta[2]);

        double J_v[9];
        J_v[0] = -s1_gamma * (hex_L1 + hex_L2 * c2 + hex_L3 * c2_3);
        J_v[1] = -c1_gamma * (hex_L2 * s2 + hex_L3 * s2_3);
        J_v[2] = -c1_gamma * hex_L3 * s2_3;
    
        J_v[3] = c1_gamma * (hex_L1 + hex_L2 * c2 + hex_L3 * c2_3);
        J_v[4] = -s1_gamma * (hex_L2 * s2 + hex_L3 * s2_3);
        J_v[5] = -s1_gamma * hex_L3 * s2_3;
    
        J_v[6] = 0;
        J_v[7] = hex_L2 * c2 + hex_L3 * c2_3;
        J_v[8] = hex_L3 * c2_3;

        hex_J << J_v[0], J_v[1], J_v[2],  // 矩阵元素
         J_v[3], J_v[4], J_v[5],
         J_v[6], J_v[7], J_v[8];

        // std::cout << "hex_J: " << hex_J.transpose() << std::endl;//打印出来是转置
        hex_omega = hex_J.inverse() * hex_v_0d;
    }

        Eigen::Vector3d hex_p_0d,hex_v_0f,hex_f_d,hex_touq;//目标
        Eigen::Matrix3d hex_K_p,hex_K_d;


        //void gait_creat函数的主要功能是根据给定的时间 t 生成机器人腿部的运动轨迹，具体包括腿部在水平方向（x 方向）和垂直方向（z 方向）的位置和速度信息。
        //这个函数通常用于实现机器人的步态规划，通过控制腿部在不同时刻的位置和速度，使机器人能够按照预定的方式移动。
        
        //start 是一个布尔变量，用于标记是否为首次执行该函数。
        bool start = false;
        
        //void leg_controll对指定腿部的电机进行控制参数的设置和调整，以实现对腿部运动的精确控制。
        //它接收两个参数：leg_n 表示腿部的编号，q 是一个指向包含关节角度信息的数组的指针，单位为弧度（rad）
        void leg_controll(int leg_n, double* q ) 
        {            
            if(start == false)
            {
                for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[leg_n*3+i].control_k_d = 0.8f;
                    m_LegMotor[leg_n*3+i].control_k_p = 25.0f;
                }
            }

            //调用该函数计算指定腿部的雅可比矩阵。
            //雅可比矩阵描述了机器人关节空间和操作空间之间的速度关系，
            //在运动控制中用于将关节速度转换为末端执行器的速度，
            //或者反过来计算为了达到特定的末端执行器速度所需的关节速度。

            Jacbi(legs[leg_n].gama,leg_n);


            //调用该函数计算指定腿部所需的扭矩。

            double temp[] = { 0,0,0,0 };

            theta2q(q, q);

            //根据腿部的编号，对特定的关节角度进行取反操作。
            if(leg_n==1||leg_n==2||leg_n==5)
            q[3] = -q[3];
            if(leg_n==0||leg_n==3||leg_n==4)
            q[2] = -q[2];
            // if(leg_n == 1)


            for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[leg_n*3+i].control_v_des = legs[leg_n].Vel_filter[i].filter(hex_omega[i]);
                    m_LegMotor[leg_n*3+i].control_p_des = legs[leg_n].Pos_filter[i].filter(q[i+1]);  
                    m_LegMotor[leg_n*3+i].control_torque = legs[leg_n].Touq_filter[i].filter(hex_touq[i]);    

                    // m_LegMotor[i].control_torque = hex_touq[i];           
                    // 滤波的目的是平滑速度信号，减少噪声的影响。
                }
            
        }

        void leg_controll11(int leg_n, double* position,double* speed,double* tor) 
        {            
            if(start == false)
            {
                for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[leg_n*3+i].control_k_d = 0.8f;
                    m_LegMotor[leg_n*3+i].control_k_p = 25.0f;
                }
            }

            // theta2q(position, position);

            if(leg_n==1||leg_n==2||leg_n==5)
            position[3] = -position[3];
            if(leg_n==0||leg_n==3)
            position[2] = -position[2];

            for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[leg_n*3+i].control_v_des = legs[leg_n].Vel_filter[i].filter(speed[i]);
                    m_LegMotor[leg_n*3+i].control_p_des = legs[leg_n].Pos_filter[i].filter(position[i+1]);  
                    m_LegMotor[leg_n*3+i].control_torque = legs[leg_n].Touq_filter[i].filter(tor[i]);    

                }
            
        }
        // static double error = 0;
        //std::chrono::time_point start_time = std::chrono::high_resolution_clock::now();
        
        //声明和初始化了一些变量，其中包括获取高精度时钟时间点以及声明系统时钟时间点变量
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point start_time1;
        int num = 0;
        double t = 0,tx = 0,tt;
        double flag_1 = 1,flag_2 = 1,flag_k1 = 1,flag_k2 = 1;

        //void mv_forword函数的主要功能是控制六足机器人向前移动。它通过计算时间、生成步态轨迹、进行逆运动学求解，
        //最终控制机器人腿部的运动。函数在首次调用时会记录开始时间，打印相关信息，后续每次调用都会更新时间并根据当前时间生成相应的腿部运动控制参数。
       
        void mv_forword() 
        {

            reset_1 = 0;
            reset_2 = 0;
            double theta1,theta2,theta3,d1,d2,d3,dd1,dd2,dd3,ddd1,ddd2,ddd3;
            double theta4,theta5,theta6,d4,d5,d6,dd4,dd5,dd6,ddd4,ddd5,ddd6;
            double tue1,tue2,tue3,tue4,tue5,tue6;
            if(start == false)//当 start 为 false 时，表示函数首次被调用。
            {
                std::cout << "mv_left运行" << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                //std::cout << start << std::endl;
                auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                std::tm* start_time_tm = std::localtime(&start_time_t);

                // Print start_time
                std::cout << "Start start = true;time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
            }
        
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

            tx = static_cast<double>(duration) / 1000000;


            tt=hex_speed*tx; 
            
            double t = (tt - (int)(tt / hex_T) * hex_T) * M_PI;

            double c = cos(t),s = sin(t);

            theta1 = 20*s;d1 = 20*c;dd1 = -20*s;ddd1 = -20*c;
            
            if(t < M_PI/2 || t > 3*M_PI/2){
                theta2 = 50*sin(t + M_PI/2 );
                d2 =  50*cos(t + M_PI/2 );
                dd2 = -50*sin(t + M_PI/2 );
                ddd2 =  -50*cos(t + M_PI/2 );
              }
              else {
                theta2 = 4.12*sin(t + M_PI/2 );
                d2 =  4.12*cos(t + M_PI/2 );
                dd2 = -4.12*sin(t + M_PI/2 );
                ddd2 =  -4.12*cos(t + M_PI/2 );
            }
            
            theta3 = 9.64*sin(t);
            if (theta3 < 0)
            {
                theta3 = -theta3;
            }

            if(t == 0 ||t == M_PI || t == 2* M_PI){
                d3 = 0; 
                dd3 = 0;
                ddd3 = 0;
            }
            else {
                if(t > M_PI){
                    d3 = 9.64*cos(t - M_PI); 
                dd3 = -9.64*sin(t - M_PI);
                ddd3 = -9.64*cos(t - M_PI); 
                }
                else{
                    d3 = 9.64*cos(t); 
                    dd3 = -9.64*sin(t);
                ddd3 = -9.64*cos(t); 
                }
                
            }

            tue1 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1))*(dd1*dd1 + d1*ddd1) - 
            motor_m3*2*leg_L2*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(-sin(theta2* M_PI / 180.0))*d2*d1*dd1;

            if(tue1 > 7)tue1 = 6.5;
            if(tue1 < -7)tue1 = -6.5;

            if(tue1 > 0 && tue1 < 3)tue1 = 3;
            if(tue1 < 0 && tue1 > -3)tue1 = -3;


            if (t < M_PI) {
                tue2 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd2*dd2 + d2*ddd2) + 
                motor_m3*leg_L2*d1*d1*d2*(leg_L2*cos(theta2 * M_PI / 180.0) + leg_L1)*sin(theta2 * M_PI / 180.0) + 
                motor_m3*gravity*cos(theta2 * M_PI / 180.0) * leg_L2 * d2;
               
            } else {
                tue2 = 6;
            }

            if(tue2 > 7)tue2 = 6.5;
            if(tue2 < -7)tue2 = -6.5;

            if(tue2 > 0 && tue2 < 3)tue2 = 3;
            if(tue2 < 0 && tue2 > -3)tue2 = -3;

            tue3 = motor_I3*(dd3*dd3 + d3*ddd3);

            if(tue3 < 0)tue3 = tue3 - 3;
            else tue3 = tue3 + 3;

            double ttt = tt + hex_T/2;
            double t2 = (ttt - (int)(ttt / hex_T) * hex_T) * M_PI;
            double c2 = cos(t2),s2 = sin(t2);

            theta4 = 20*s2;d4 = 20*c2;dd4 = -20*s2;ddd4 = -20*c2;
            
            if(t2 < M_PI/2 || t2 > 3*M_PI/2){
                theta5 = 50*sin(t2 + M_PI/2 );
                d5 =  50*cos(t2 + M_PI/2 );
                dd5 = -50*sin(t2 + M_PI/2 );
                ddd5 =  -50*cos(t2 + M_PI/2 );
              }
              else {
                theta5 = 4.12*sin(t2 + M_PI/2 );
                d5 =  4.12*cos(t2 + M_PI/2 );
                dd5 = -4.12*sin(t2 + M_PI/2 );
                ddd5 =  -4.12*cos(t2 + M_PI/2 );
            }
            
            theta6 = 9.64*sin(t2);
            if (theta6 < 0)
            {
                theta6 = -theta6;
            }

            if(t2 == 0 ||t2 == M_PI || t2 == 2* M_PI){
                d6 = 0; 
                dd6 = 0;
                ddd6 = 0;
            }
            else {
                if(t2 > M_PI){
                    d6 = 9.64*cos(t2 - M_PI);
                dd6 = -9.64*sin(t2 - M_PI);
                ddd6 = -9.64*cos(t2 - M_PI);
                }
                else{
                    d6 = 9.64*cos(t2); 
                    dd6 = -9.64*sin(t2);
                ddd6 = -9.64*cos(t2); 
                }
                
            }

            tue4 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1))*(dd4*dd4 + d4*ddd4) - 
            motor_m3*2*leg_L2*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(-sin(theta5* M_PI / 180.0))*d5*d4*dd4;

            if(tue4 > 7)tue4 = 6.5;
            if(tue4 < -7)tue4 = -6.5;
            if(tue4 > 0 && tue4 < 3)tue4 = 3;
            if(tue4 < 0 && tue4 > -3)tue4 = -3;

            if (t2 < M_PI) {
                tue5 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd5*dd5 + d5*ddd5) + 
                motor_m3*leg_L2*d4*d4*d5*(leg_L2*cos(theta5 * M_PI / 180.0) + leg_L1)*sin(theta5 * M_PI / 180.0) + 
                motor_m3*gravity*cos(theta5 * M_PI / 180.0) * leg_L2 * d5;
               
            } else {
                tue5 = 6;
            }

            if(tue5 > 0 && tue5 < 3)tue5 = 3;
            if(tue5 < 0 && tue5 > -3)tue5 = -3;

            tue6 = motor_I3*(dd6*dd6 + d6*ddd6);

            if(tue6 < 0)tue6 = tue6 - 3;
            else tue6 = tue6 + 3;

            double tor04[] = {0,tue1,tue2,tue3};
            double tor2[] = {0,tue1,tue2,tue3};
            double tor15[] = {0,tue4,tue5,tue6};
            double tor3[] = {0,tue4,tue5,tue6};
            double speed04[] = {0,d1,d2,d3};
            double speed15[] = {0,d4,d5,d6};
            double speed2[] = {0,d1,d2,d3};
            double speed3[] = {0,d4,d5,d6};
            double position04[] = {0,degree2rad(theta1),degree2rad(theta2),degree2rad(theta3)};
            double position15[] = {0,degree2rad(-theta4),degree2rad(theta5),degree2rad(theta6)};
            double position2[] = {0,degree2rad(-theta1),degree2rad(theta2),degree2rad(theta3)};
            double position3[] = {0,degree2rad(theta4),degree2rad(theta5),degree2rad(theta6)};

            leg_controll11(0,position04,speed04,tor04);
            leg_controll11(4,position04,speed04,tor04);
            leg_controll11(2,position2,speed2,tor2);

            leg_controll11(1,position15,speed15,tor15);
            leg_controll11(5,position15,speed15,tor15);
            leg_controll11(3,position3,speed3,tor3);

            // first_stop = 0;
           
            if(start == false)
                {                
                    start = true;
                    //确保后续调用时不再执行初始化和时间记录的操作。
                }
       }


        //void mv_stop()停止机器人的运动，将机器人的状态重置为初始状态。它会对机器人的腿部滤波器参数、电机控制参数进行调整，
        //并调用 leg_controll 函数对部分腿部进行控制，最后将启动标记 start 置为 false。
    void mv_stop()
    {   
        num = 0 ;
        double q[] = { 0,0,0,0 };
        hex_omega << 0,0,0;
        //表示将关节角度设置为初始位置。
        //hex_omega代表角速度，将其所有元素设置为 0，意味着停止所有的旋转运动。

        //将位置滤波器的上一次滤波值 previousFilteredValue 设置为 0，速度滤波器的上一次滤波值设置为当前的角速度 hex_omega[i]，扭矩滤波器的上一次滤波值设置为 0。
        //调整滤波器的平滑系数 alpha，位置滤波器的 alpha 为 1e-2，速度滤波器的 alpha 为 1e-8，扭矩滤波器的 alpha 为 1e-10。较小的 alpha 值会使滤波器对新数据的响应更慢，起到平滑的作用。
        for(int j=0;j<LEG_NUM;j++)
            for(int i=0;i<3;i++)
            {
                legs[j].Pos_filter[i].previousFilteredValue = 0;
                legs[j].Pos_filter[i].alpha = 1e-2;
                //这里设置为 1e-2（即 0.01）。较小的 alpha 值意味着滤波器对新输入信号的响应较慢，会更多地依赖于上一次的滤波输出。
                //在停止运动的场景下，这样可以使位置信号的变化更加平滑，避免因突然的位置变化而导致电机产生过大的控制输出。
                legs[j].Vel_filter[i].previousFilteredValue = hex_omega[i];
                legs[j].Vel_filter[i].alpha = 1e-8;
                //这是因为在停止运动时，期望速度为 0，通过将上一次的滤波输出设置为当前角速度，可以让滤波器在后续的过程中逐渐将速度信号平滑地调整到 0。
                //意味着速度滤波器对新输入信号的响应极其缓慢，会极大地抑制速度信号的快速变化。、
                //这样可以避免因速度信号的噪声或突然变化而导致电机的不稳定控制，确保机器人平稳地停止运动。
                legs[j].Touq_filter[i].previousFilteredValue = 0;
                legs[j].Touq_filter[i].alpha = 1e-10;
                m_LegMotor[i].control_k_d = 0.1f;
                m_LegMotor[i].control_k_p = 5.0f;
            }
        leg_controll(0, q);
        leg_controll(1, q);
        leg_controll(2, q);
        leg_controll(3, q);
        leg_controll(4, q);
        leg_controll(5, q);
        start = false;  
        }

    void mv_reset() 
        {
            double theta1,theta2,theta3,d1,d2,d3,dd1,dd2,dd3,ddd1,ddd2,ddd3;
            double theta4,theta5,theta6,d4,d5,d6,dd4,dd5,dd6,ddd4,ddd5,ddd6;
            double tue1,tue2,tue3,tue4,tue5,tue6;

            if(start == false)//当 start 为 false 时，表示函数首次被调用。
            {
                std::cout << "mv_reset运行" << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                //std::cout << start << std::endl;
                auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                std::tm* start_time_tm = std::localtime(&start_time_t);

                // Print start_time
                std::cout << "Start start = true;time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
            }
        
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

            tx = static_cast<double>(duration) / 1000000;

            tt = hex_speed*tx; 
            
            double t = (tt - (int)(tt / hex_T) * hex_T) * M_PI;

            if((t >= 0 &&t > 0.1)||t < M_PI + 0.1 && t > M_PI - 0.1||(t > 2*M_PI - 0.1&& t >= 2*M_PI)){
                reset_1 = 1;
            }

            if(reset_1 == 0){
                double c = cos(t),s = sin(t);

                theta1 = 20*s;d1 = 20*c;dd1 = -20*s;ddd1 = -20*c;
                
                if(t < M_PI/2 || t > 3*M_PI/2){
                    theta2 = 50*sin(t + M_PI/2 );
                    d2 =  50*cos(t + M_PI/2 );
                    dd2 = -50*sin(t + M_PI/2 );
                    ddd2 =  -50*cos(t + M_PI/2 );
                }
                else {
                    theta2 = 4.12*sin(t + M_PI/2 );
                    d2 =  4.12*cos(t + M_PI/2 );
                    dd2 = -4.12*sin(t + M_PI/2 );
                    ddd2 =  -4.12*cos(t + M_PI/2 );
                }
                
                theta3 = 9.64*sin(t);
                if (theta3 < 0)
                {
                    theta3 = -theta3;
                }

                if(t == 0 ||t == M_PI || t == 2* M_PI){
                    d3 = 0; 
                    dd3 = 0;
                    ddd3 = 0;
                }
                else {
                    if(t > M_PI){
                        d3 = 9.64*cos(t - M_PI); 
                    dd3 = -9.64*sin(t - M_PI);
                    ddd3 = -9.64*cos(t - M_PI); 
                    }
                    else{
                        d3 = 9.64*cos(t); 
                        dd3 = -9.64*sin(t);
                    ddd3 = -9.64*cos(t); 
                    }
                    
                }

                tue1 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1))*(dd1*dd1 + d1*ddd1) - 
                motor_m3*2*leg_L2*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(-sin(theta2* M_PI / 180.0))*d2*d1*dd1;

                if(tue1 > 7)tue1 = 6.5;
                if(tue1 < -7)tue1 = -6.5;

                if(tue1 > 0 && tue1 < 3)tue1 = 3;
                if(tue1 < 0 && tue1 > -3)tue1 = -3;

                if (t < M_PI) {
                    tue2 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd2*dd2 + d2*ddd2) + 
                    motor_m3*leg_L2*d1*d1*d2*(leg_L2*cos(theta2 * M_PI / 180.0) + leg_L1)*sin(theta2 * M_PI / 180.0) + 
                    motor_m3*gravity*cos(theta2 * M_PI / 180.0) * leg_L2 * d2;
                
                } else {
                    tue2 = 6;
                }

                if(tue2 > 7)tue2 = 6.5;
                if(tue2 < -7)tue2 = -6.5;

                if(tue2 > 0 && tue2 < 3)tue2 = 3;
                if(tue2 < 0 && tue2 > -3)tue2 = -3;

                tue3 = motor_I3*(dd3*dd3 + d3*ddd3);

                if(tue3 < 0)tue3 = tue3 - 3;
                else tue3 = tue3 + 3;
                }

            else{
                theta1 = 0;
                theta2 = 0;
                theta3 = 0;
                tue1 = 6;
                tue2 = 6;
                tue3 = 6;
                d1 = 0;
                d2 = 0;
                d3 = 0;
            }


            double ttt = tt + hex_T/2;
            double t2 = (ttt - (int)(ttt / hex_T) * hex_T) * M_PI;

            if((t2 >= 0 &&t2 > 0.1)||t2 < M_PI + 0.1 && t2 > M_PI - 0.1||(t2 > 2*M_PI - 0.1&& t2 >= 2*M_PI)){
                reset_2 = 1;
            }

            if(reset_2 == 0){
                double c2 = cos(t2),s2 = sin(t2);
                theta4 = 20*s2;d4 = 20*c2;dd4 = -20*s2;ddd4 = -20*c2;
                
                if(t2 < M_PI/2 || t2 > 3*M_PI/2){
                    theta5 = 50*sin(t2 + M_PI/2 );
                    d5 =  50*cos(t2 + M_PI/2 );
                    dd5 = -50*sin(t2 + M_PI/2 );
                    ddd5 =  -50*cos(t2 + M_PI/2 );
                }
                else {
                    theta5 = 4.12*sin(t2 + M_PI/2 );
                    d5 =  4.12*cos(t2 + M_PI/2 );
                    dd5 = -4.12*sin(t2 + M_PI/2 );
                    ddd5 =  -4.12*cos(t2 + M_PI/2 );
                }
                
                theta6 = 9.64*sin(t2);
                if (theta6 < 0)
                {
                    theta6 = -theta6;
                }

                if(t2 == 0 ||t2 == M_PI || t2 == 2* M_PI){
                    d6 = 0; 
                    dd6 = 0;
                    ddd6 = 0;
                }
                else {
                    if(t2 > M_PI){
                        d6 = 9.64*cos(t2 - M_PI);
                    dd6 = -9.64*sin(t2 - M_PI);
                    ddd6 = -9.64*cos(t2 - M_PI);
                    }
                    else{
                        d6 = 9.64*cos(t2); 
                        dd6 = -9.64*sin(t2);
                    ddd6 = -9.64*cos(t2); 
                    }
                    
                }

                tue4 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1))*(dd4*dd4 + d4*ddd4) - 
                motor_m3*2*leg_L2*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(-sin(theta5* M_PI / 180.0))*d5*d4*dd4;

                if(tue4 > 7)tue4 = 6.5;
                if(tue4 < -7)tue4 = -6.5;
                if(tue4 > 0 && tue4 < 3)tue4 = 3;
                if(tue4 < 0 && tue4 > -3)tue4 = -3;

                if (t2 < M_PI) {
                    tue5 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd5*dd5 + d5*ddd5) + 
                    motor_m3*leg_L2*d4*d4*d5*(leg_L2*cos(theta5 * M_PI / 180.0) + leg_L1)*sin(theta5 * M_PI / 180.0) + 
                    motor_m3*gravity*cos(theta5 * M_PI / 180.0) * leg_L2 * d5;
                
                } else {
                    tue5 = 6;
                }

                if(tue5 > 0 && tue5 < 3)tue5 = 3;
                if(tue5 < 0 && tue5 > -3)tue5 = -3;

                tue6 = motor_I3*(dd6*dd6 + d6*ddd6);

                if(tue6 < 0)tue6 = tue6 - 3;
                else tue6 = tue6 + 3;

            }
            else{
                theta4 = 0;
                theta5 = 0;
                theta6 = 0;
                tue4 = 6;
                tue5 = 6;
                tue6 = 6;
                d4 = 0;
                d5 = 0;
                d6 = 0;
            }

            double tor04[] = {0,tue1,tue2,tue3};
            double tor2[] = {0,tue1,tue2,tue3};
            double tor15[] = {0,tue4,tue5,tue6};
            double tor3[] = {0,tue4,tue5,tue6};
            double speed04[] = {0,d1,d2,d3};
            double speed15[] = {0,d4,d5,d6};
            double speed2[] = {0,d1,d2,d3};
            double speed3[] = {0,d4,d5,d6};
            double position04[] = {0,degree2rad(theta1),degree2rad(theta2),degree2rad(theta3)};
            double position15[] = {0,degree2rad(-theta4),degree2rad(theta5),degree2rad(theta6)};
            double position2[] = {0,degree2rad(-theta1),degree2rad(theta2),degree2rad(theta3)};
            double position3[] = {0,degree2rad(theta4),degree2rad(theta5),degree2rad(theta6)};


            m_LegMotor[0].feedback_pos = position04[1];


            leg_controll11(0,position04,speed04,tor04);
            leg_controll11(4,position04,speed04,tor04);
            leg_controll11(2,position2,speed2,tor2);


            leg_controll11(1,position15,speed15,tor15);
            leg_controll11(5,position15,speed15,tor15);
            leg_controll11(3,position3,speed3,tor3);

            // first_stop = 0;
           
            if(start == false)
                {                
                    start = true;
                    //确保后续调用时不再执行初始化和时间记录的操作。
                }
       }


    void set_zero()
    {
        for(int i = 0; i < 3; i++)
        {
            for(int j=0;j<LEG_NUM;j++)
            {
            m_LegMotor[j*3+i].save_zero();
            }
         }
    }

    void mv_left()
        {
            reset_1 = 0;
            reset_2 = 0;
            double theta1,theta2,theta3,d1,d2,d3,dd1,dd2,dd3,ddd1,ddd2,ddd3;
            double theta4,theta5,theta6,d4,d5,d6,dd4,dd5,dd6,ddd4,ddd5,ddd6;
            double tue1,tue2,tue3,tue4,tue5,tue6;
            if(start == false)//当 start 为 false 时，表示函数首次被调用。
            {
                std::cout << "mv_left运行" << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                //std::cout << start << std::endl;
                auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                std::tm* start_time_tm = std::localtime(&start_time_t);

                // Print start_time
                std::cout << "Start start = true;time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
            }
        
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

            tx = static_cast<double>(duration) / 1000000;
            // if(stop_time != 0){
            //     time_error = tx - stop_time;
            // }
            
            tt=hex_speed*tx; 
            
            double t = (tt - (int)(tt / hex_T) * hex_T) * M_PI;

            double c = cos(t),s = sin(t);

            theta1 = 20*s;d1 = 20*c;dd1 = -20*s;ddd1 = -20*c;
            
            if(t < M_PI/2 || t > 3*M_PI/2){
                theta2 = 40*sin(t + M_PI/2 );
                d2 =  40*cos(t + M_PI/2 );
                dd2 = -40*sin(t + M_PI/2 );
                ddd2 =  -40*cos(t + M_PI/2 );
              }
              else {
                theta2 = 4.12*sin(t + M_PI/2 );
                d2 =  4.12*cos(t + M_PI/2 );
                dd2 = -4.12*sin(t + M_PI/2 );
                ddd2 =  -4.12*cos(t + M_PI/2 );
            }
            
            theta3 = 9.64*sin(t);
            if (theta3 < 0)
            {
                theta3 = -theta3;
            }

            if(t == 0 ||t == M_PI || t == 2* M_PI){
                d3 = 0; 
                dd3 = 0;
                ddd3 = 0;
            }
            else {
                if(t > M_PI){
                    d3 = 9.64*cos(t - M_PI); 
                dd3 = -9.64*sin(t - M_PI);
                ddd3 = -9.64*cos(t - M_PI); 
                }
                else{
                    d3 = 9.64*cos(t); 
                    dd3 = -9.64*sin(t);
                ddd3 = -9.64*cos(t); 
                }
                
            }

            tue1 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1))*(dd1*dd1 + d1*ddd1) - 
            motor_m3*2*leg_L2*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(-sin(theta2* M_PI / 180.0))*d2*d1*dd1;

            if(tue1 > 7)tue1 = 6.5;
            if(tue1 < -7)tue1 = -6.5;

            if(tue1 > 0 && tue1 < 3)tue1 = 3;
            if(tue1 < 0 && tue1 > -3)tue1 = -3;


            if (t < M_PI) {
                tue2 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd2*dd2 + d2*ddd2) + 
                motor_m3*leg_L2*d1*d1*d2*(leg_L2*cos(theta2 * M_PI / 180.0) + leg_L1)*sin(theta2 * M_PI / 180.0) + 
                motor_m3*gravity*cos(theta2 * M_PI / 180.0) * leg_L2 * d2;
               
            } else {
                tue2 = 6;
            }

            if(tue2 > 7)tue2 = 6.5;
            if(tue2 < -7)tue2 = -6.5;

            if(tue2 > 0 && tue2 < 3)tue2 = 3;
            if(tue2 < 0 && tue2 > -3)tue2 = -3;

            tue3 = motor_I3*(dd3*dd3 + d3*ddd3);

            if(tue3 < 0)tue3 = tue3 - 3;
            else tue3 = tue3 + 3;

/////另一组
            double ttt = tt + hex_T/2;
            double t2 = (ttt - (int)(ttt / hex_T) * hex_T) * M_PI;
            double c2 = cos(t2),s2 = sin(t2);

            theta4 = 20*s2;d4 = 20*c2;dd4 = -20*s2;ddd4 = -20*c2;
            
            if(t2 < M_PI/2 || t2 > 3*M_PI/2){
                theta5 = 40*sin(t2 + M_PI/2 );
                d5 =  40*cos(t2 + M_PI/2 );
                dd5 = -40*sin(t2 + M_PI/2 );
                ddd5 =  -40*cos(t2 + M_PI/2 );
              }
              else {
                theta5 = 4.12*sin(t2 + M_PI/2 );
                d5 =  4.12*cos(t2 + M_PI/2 );
                dd5 = -4.12*sin(t2 + M_PI/2 );
                ddd5 =  -4.12*cos(t2 + M_PI/2 );
            }
            
            theta6 = 9.64*sin(t2);
            if (theta6 < 0)
            {
                theta6 = -theta6;
            }

            if(t2 == 0 ||t2 == M_PI || t2 == 2* M_PI){
                d6 = 0; 
                dd6 = 0;
                ddd6 = 0;
            }
            else {
                if(t2 > M_PI){
                    d6 = 9.64*cos(t2 - M_PI);
                dd6 = -9.64*sin(t2 - M_PI);
                ddd6 = -9.64*cos(t2 - M_PI);
                }
                else{
                    d6 = 9.64*cos(t2); 
                    dd6 = -9.64*sin(t2);
                ddd6 = -9.64*cos(t2); 
                }
                
            }

            tue4 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1))*(dd4*dd4 + d4*ddd4) - 
            motor_m3*2*leg_L2*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(-sin(theta5* M_PI / 180.0))*d5*d4*dd4;

            if(tue4 > 7)tue4 = 6.5;
            if(tue4 < -7)tue4 = -6.5;
            if(tue4 > 0 && tue4 < 3)tue4 = 3;
            if(tue4 < 0 && tue4 > -3)tue4 = -3;
            if (t2 < M_PI) {
                tue5 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd5*dd5 + d5*ddd5) + 
                motor_m3*leg_L2*d4*d4*d5*(leg_L2*cos(theta5 * M_PI / 180.0) + leg_L1)*sin(theta5 * M_PI / 180.0) + 
                motor_m3*gravity*cos(theta5 * M_PI / 180.0) * leg_L2 * d5;
               
            } else {
                tue5 = 6;
            }

            if(tue5 > 0 && tue5 < 3)tue5 = 3;
            if(tue5 < 0 && tue5 > -3)tue5 = -3;

            tue6 = motor_I3*(dd6*dd6 + d6*ddd6);

            if(tue6 < 0)tue6 = tue6 - 3;
            else tue6 = tue6 + 3;

            double tor04[] = {0,tue1,tue2,tue3};
            double tor2[] = {0,tue1,tue2,tue3};
            double tor15[] = {0,tue4,tue5,tue6};
            double tor3[] = {0,tue4,tue5,tue6};
            double speed04[] = {0,d1,d2,d3};
            double speed15[] = {0,d4,d5,d6};
            double speed2[] = {0,d1,d2,d3};
            double speed3[] = {0,d4,d5,d6};
            double position04[] = {0,degree2rad(theta1),degree2rad(theta2),degree2rad(theta3)};
            double position15[] = {0,degree2rad(theta4),degree2rad(theta5),degree2rad(theta6)};
            double position2[] = {0,degree2rad(theta1),degree2rad(theta2),degree2rad(theta3)};
            double position3[] = {0,degree2rad(theta4),degree2rad(theta5),degree2rad(theta6)};

            leg_controll11(0,position04,speed04,tor04);
            leg_controll11(4,position04,speed04,tor04);
            leg_controll11(2,position2,speed2,tor2);

            leg_controll11(1,position15,speed15,tor15);
            leg_controll11(5,position15,speed15,tor15);
            leg_controll11(3,position3,speed3,tor3);

            // first_stop = 0;
           
            if(start == false)
                {                
                    start = true;
                    //确保后续调用时不再执行初始化和时间记录的操作。
                }

    }

    void mv_right()
    {
        double theta1,theta2,theta3,d1,d2,d3,dd1,dd2,dd3,ddd1,ddd2,ddd3;
        double theta4,theta5,theta6,d4,d5,d6,dd4,dd5,dd6,ddd4,ddd5,ddd6;
        double tue1,tue2,tue3,tue4,tue5,tue6;
        reset_1 = 0;
        reset_2 = 0;
        if(start == false)//当 start 为 false 时，表示函数首次被调用。
        {
            std::cout << "mv_right运行" << std::endl;
            start_time = std::chrono::high_resolution_clock::now();
            //std::cout << start << std::endl;
            auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm* start_time_tm = std::localtime(&start_time_t);

            // Print start_time
            std::cout << "Start start = true;time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
        }
    
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

        tx = static_cast<double>(duration) / 1000000;
        // if(stop_time != 0){
        //     time_error = tx - stop_time;
        // }
        
        tt=hex_speed*tx; 
        
        double t = (tt - (int)(tt / hex_T) * hex_T) * M_PI;

        double c = cos(t),s = sin(t);

        theta1 = 20*s;d1 = 20*c;dd1 = -20*s;ddd1 = -20*c;
        
        if(t < M_PI/2 || t > 3*M_PI/2){
            theta2 = 40*sin(t + M_PI/2 );
            d2 =  40*cos(t + M_PI/2 );
            dd2 = -40*sin(t + M_PI/2 );
            ddd2 =  -40*cos(t + M_PI/2 );
          }
          else {
            theta2 = 4.12*sin(t + M_PI/2 );
            d2 =  4.12*cos(t + M_PI/2 );
            dd2 = -4.12*sin(t + M_PI/2 );
            ddd2 =  -4.12*cos(t + M_PI/2 );
        }
        
        theta3 = 9.64*sin(t);
        if (theta3 < 0)
        {
            theta3 = -theta3;
        }

        if(t == 0 ||t == M_PI || t == 2* M_PI){
            d3 = 0; 
            dd3 = 0;
            ddd3 = 0;
        }
        else {
            if(t > M_PI){
                d3 = 9.64*cos(t - M_PI); 
            dd3 = -9.64*sin(t - M_PI);
            ddd3 = -9.64*cos(t - M_PI); 
            }
            else{
                d3 = 9.64*cos(t); 
                dd3 = -9.64*sin(t);
            ddd3 = -9.64*cos(t); 
            }
            
        }

        tue1 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1))*(dd1*dd1 + d1*ddd1) - 
        motor_m3*2*leg_L2*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(-sin(theta2* M_PI / 180.0))*d2*d1*dd1;

        if(tue1 > 7)tue1 = 6.5;
        if(tue1 < -7)tue1 = -6.5;

        if(tue1 > 0 && tue1 < 3)tue1 = 3;
        if(tue1 < 0 && tue1 > -3)tue1 = -3;


        if (t < M_PI) {
            tue2 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd2*dd2 + d2*ddd2) + 
            motor_m3*leg_L2*d1*d1*d2*(leg_L2*cos(theta2 * M_PI / 180.0) + leg_L1)*sin(theta2 * M_PI / 180.0) + 
            motor_m3*gravity*cos(theta2 * M_PI / 180.0) * leg_L2 * d2;
           
        } else {
            tue2 = 6;
        }

        if(tue2 > 7)tue2 = 6.5;
        if(tue2 < -7)tue2 = -6.5;

        if(tue2 > 0 && tue2 < 3)tue2 = 3;
        if(tue2 < 0 && tue2 > -3)tue2 = -3;

        tue3 = motor_I3*(dd3*dd3 + d3*ddd3);

        if(tue3 < 0)tue3 = tue3 - 3;
        else tue3 = tue3 + 3;

/////另一组
        double ttt = tt + hex_T/2;
        double t2 = (ttt - (int)(ttt / hex_T) * hex_T) * M_PI;
        double c2 = cos(t2),s2 = sin(t2);

        theta4 = 20*s2;d4 = 20*c2;dd4 = -20*s2;ddd4 = -20*c2;
        
        if(t2 < M_PI/2 || t2 > 3*M_PI/2){
            theta5 = 40*sin(t2 + M_PI/2 );
            d5 =  40*cos(t2 + M_PI/2 );
            dd5 = -40*sin(t2 + M_PI/2 );
            ddd5 =  -40*cos(t2 + M_PI/2 );
          }
          else {
            theta5 = 4.12*sin(t2 + M_PI/2 );
            d5 =  4.12*cos(t2 + M_PI/2 );
            dd5 = -4.12*sin(t2 + M_PI/2 );
            ddd5 =  -4.12*cos(t2 + M_PI/2 );
        }
        
        theta6 = 9.64*sin(t2);
        if (theta6 < 0)
        {
            theta6 = -theta6;
        }

        if(t2 == 0 ||t2 == M_PI || t2 == 2* M_PI){
            d6 = 0; 
            dd6 = 0;
            ddd6 = 0;
        }
        else {
            if(t2 > M_PI){
                d6 = 9.64*cos(t2 - M_PI);
            dd6 = -9.64*sin(t2 - M_PI);
            ddd6 = -9.64*cos(t2 - M_PI);
            }
            else{
                d6 = 9.64*cos(t2); 
                dd6 = -9.64*sin(t2);
            ddd6 = -9.64*cos(t2); 
            }
            
        }

        tue4 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1))*(dd4*dd4 + d4*ddd4) - 
        motor_m3*2*leg_L2*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(-sin(theta5* M_PI / 180.0))*d5*d4*dd4;

        if(tue4 > 7)tue4 = 6.5;
        if(tue4 < -7)tue4 = -6.5;
        if(tue4 > 0 && tue4 < 3)tue4 = 3;
        if(tue4 < 0 && tue4 > -3)tue4 = -3;
        if (t2 < M_PI) {
            tue5 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd5*dd5 + d5*ddd5) + 
            motor_m3*leg_L2*d4*d4*d5*(leg_L2*cos(theta5 * M_PI / 180.0) + leg_L1)*sin(theta5 * M_PI / 180.0) + 
            motor_m3*gravity*cos(theta5 * M_PI / 180.0) * leg_L2 * d5;
           
        } else {
            tue5 = 6;
        }

        if(tue5 > 0 && tue5 < 3)tue5 = 3;
        if(tue5 < 0 && tue5 > -3)tue5 = -3;

        tue6 = motor_I3*(dd6*dd6 + d6*ddd6);

        if(tue6 < 0)tue6 = tue6 - 3;
        else tue6 = tue6 + 3;

        double tor04[] = {0,tue1,tue2,tue3};
        double tor2[] = {0,tue1,tue2,tue3};
        double tor15[] = {0,tue4,tue5,tue6};
        double tor3[] = {0,tue4,tue5,tue6};
        double speed04[] = {0,d1,d2,d3};
        double speed15[] = {0,d4,d5,d6};
        double speed2[] = {0,d1,d2,d3};
        double speed3[] = {0,d4,d5,d6};
        double position04[] = {0,degree2rad(-theta1),degree2rad(theta2),degree2rad(theta3)};
        double position15[] = {0,degree2rad(-theta4),degree2rad(theta5),degree2rad(theta6)};
        double position2[] = {0,degree2rad(-theta1),degree2rad(theta2),degree2rad(theta3)};
        double position3[] = {0,degree2rad(-theta4),degree2rad(theta5),degree2rad(theta6)};

        leg_controll11(0,position04,speed04,tor04);
        leg_controll11(4,position04,speed04,tor04);
        leg_controll11(2,position2,speed2,tor2);

        leg_controll11(1,position15,speed15,tor15);
        leg_controll11(5,position15,speed15,tor15);
        leg_controll11(3,position3,speed3,tor3);

        // first_stop = 0;
       
        if(start == false)
            {                
                start = true;
                //确保后续调用时不再执行初始化和时间记录的操作。
            }

}

    void mv_back()
    {
        double theta1,theta2,theta3,d1,d2,d3,dd1,dd2,dd3,ddd1,ddd2,ddd3;
        double theta4,theta5,theta6,d4,d5,d6,dd4,dd5,dd6,ddd4,ddd5,ddd6;
        double tue1,tue2,tue3,tue4,tue5,tue6;
        reset_1 = 0;
        reset_2 = 0;
        if(start == false)//当 start 为 false 时，表示函数首次被调用。
        {
            std::cout << "mv_back运行" << std::endl;
            start_time = std::chrono::high_resolution_clock::now();
            //std::cout << start << std::endl;
            auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::tm* start_time_tm = std::localtime(&start_time_t);

            // Print start_time
            std::cout << "Start start = true;time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
        }
    
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

        tx = static_cast<double>(duration) / 1000000;
        // if(stop_time != 0){
        //     time_error = tx - stop_time;
        // }
        
        tt=hex_speed*tx; 
        
        double t = (tt - (int)(tt / hex_T) * hex_T) * M_PI;

        double c = cos(t),s = sin(t);

        theta1 = 20*s;d1 = 20*c;dd1 = -20*s;ddd1 = -20*c;
        
        if(t < M_PI/2 || t > 3*M_PI/2){
            theta2 = 50*sin(t + M_PI/2 );
            d2 =  50*cos(t + M_PI/2 );
            dd2 = -50*sin(t + M_PI/2 );
            ddd2 =  -50*cos(t + M_PI/2 );
          }
          else {
            theta2 = 4.12*sin(t + M_PI/2 );
            d2 =  4.12*cos(t + M_PI/2 );
            dd2 = -4.12*sin(t + M_PI/2 );
            ddd2 =  -4.12*cos(t + M_PI/2 );
        }
        
        theta3 = 9.64*sin(t);
        if (theta3 < 0)
        {
            theta3 = -theta3;
        }

        if(t == 0 ||t == M_PI || t == 2* M_PI){
            d3 = 0; 
            dd3 = 0;
            ddd3 = 0;
        }
        else {
            if(t > M_PI){
                d3 = 9.64*cos(t - M_PI); 
            dd3 = -9.64*sin(t - M_PI);
            ddd3 = -9.64*cos(t - M_PI); 
            }
            else{
                d3 = 9.64*cos(t); 
                dd3 = -9.64*sin(t);
            ddd3 = -9.64*cos(t); 
            }
            
        }

        tue1 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1))*(dd1*dd1 + d1*ddd1) - 
        motor_m3*2*leg_L2*(leg_L2*cos(theta2* M_PI / 180.0) + leg_L1)*(-sin(theta2* M_PI / 180.0))*d2*d1*dd1;

        if(tue1 > 7)tue1 = 6.5;
        if(tue1 < -7)tue1 = -6.5;

        if(tue1 > 0 && tue1 < 3)tue1 = 3;
        if(tue1 < 0 && tue1 > -3)tue1 = -3;


        if (t < M_PI) {
            tue2 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd2*dd2 + d2*ddd2) + 
            motor_m3*leg_L2*d1*d1*d2*(leg_L2*cos(theta2 * M_PI / 180.0) + leg_L1)*sin(theta2 * M_PI / 180.0) + 
            motor_m3*gravity*cos(theta2 * M_PI / 180.0) * leg_L2 * d2;
           
        } else {
            tue2 = 6;
        }

        if(tue2 > 7)tue2 = 6.5;
        if(tue2 < -7)tue2 = -6.5;

        if(tue2 > 0 && tue2 < 3)tue2 = 3;
        if(tue2 < 0 && tue2 > -3)tue2 = -3;

        tue3 = motor_I3*(dd3*dd3 + d3*ddd3);

        if(tue3 < 0)tue3 = tue3 - 3;
        else tue3 = tue3 + 3;

/////另一组
        double ttt = tt + hex_T/2;
        double t2 = (ttt - (int)(ttt / hex_T) * hex_T) * M_PI;
        double c2 = cos(t2),s2 = sin(t2);

        theta4 = 20*s2;d4 = 20*c2;dd4 = -20*s2;ddd4 = -20*c2;
        
        if(t2 < M_PI/2 || t2 > 3*M_PI/2){
            theta5 = 50*sin(t2 + M_PI/2 );
            d5 =  50*cos(t2 + M_PI/2 );
            dd5 = -50*sin(t2 + M_PI/2 );
            ddd5 =  -50*cos(t2 + M_PI/2 );
          }
          else {
            theta5 = 4.12*sin(t2 + M_PI/2 );
            d5 =  4.12*cos(t2 + M_PI/2 );
            dd5 = -4.12*sin(t2 + M_PI/2 );
            ddd5 =  -4.12*cos(t2 + M_PI/2 );
        }
        
        theta6 = 9.64*sin(t2);
        if (theta6 < 0)
        {
            theta6 = -theta6;
        }

        if(t2 == 0 ||t2 == M_PI || t2 == 2* M_PI){
            d6 = 0; 
            dd6 = 0;
            ddd6 = 0;
        }
        else {
            if(t2 > M_PI){
                d6 = 9.64*cos(t2 - M_PI);
            dd6 = -9.64*sin(t2 - M_PI);
            ddd6 = -9.64*cos(t2 - M_PI);
            }
            else{
                d6 = 9.64*cos(t2); 
                dd6 = -9.64*sin(t2);
            ddd6 = -9.64*cos(t2); 
            }
            
        }

        tue4 = (motor_m2*leg_L1*leg_L1+motor_I1 + motor_m3*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1))*(dd4*dd4 + d4*ddd4) - 
        motor_m3*2*leg_L2*(leg_L2*cos(theta5* M_PI / 180.0) + leg_L1)*(-sin(theta5* M_PI / 180.0))*d5*d4*dd4;

        if(tue4 > 7)tue4 = 6.5;
        if(tue4 < -7)tue4 = -6.5;
        if(tue4 > 0 && tue4 < 3)tue4 = 3;
        if(tue4 < 0 && tue4 > -3)tue4 = -3;
        if (t2 < M_PI) {
            tue5 = (motor_m3*leg_L2*leg_L2 + motor_I2)*(dd5*dd5 + d5*ddd5) + 
            motor_m3*leg_L2*d4*d4*d5*(leg_L2*cos(theta5 * M_PI / 180.0) + leg_L1)*sin(theta5 * M_PI / 180.0) + 
            motor_m3*gravity*cos(theta5 * M_PI / 180.0) * leg_L2 * d5;
           
        } else {
            tue5 = 6;
        }

        if(tue5 > 0 && tue5 < 3)tue5 = 3;
        if(tue5 < 0 && tue5 > -3)tue5 = -3;

        tue6 = motor_I3*(dd6*dd6 + d6*ddd6);

        if(tue6 < 0)tue6 = tue6 - 3;
        else tue6 = tue6 + 3;

        double tor04[] = {0,tue1,tue2,tue3};
        double tor2[] = {0,tue1,tue2,tue3};
        double tor15[] = {0,tue4,tue5,tue6};
        double tor3[] = {0,tue4,tue5,tue6};
        double speed04[] = {0,d1,d2,d3};
        double speed15[] = {0,d4,d5,d6};
        double speed2[] = {0,d1,d2,d3};
        double speed3[] = {0,d4,d5,d6};
        double position04[] = {0,degree2rad(-theta1),degree2rad(theta2),degree2rad(theta3)};
        double position15[] = {0,degree2rad(theta4),degree2rad(theta5),degree2rad(theta6)};
        double position2[] = {0,degree2rad(theta1),degree2rad(theta2),degree2rad(theta3)};
        double position3[] = {0,degree2rad(-theta4),degree2rad(theta5),degree2rad(theta6)};

        leg_controll11(0,position04,speed04,tor04);
        leg_controll11(4,position04,speed04,tor04);
        leg_controll11(2,position2,speed2,tor2);

        leg_controll11(1,position15,speed15,tor15);
        leg_controll11(5,position15,speed15,tor15);
        leg_controll11(3,position3,speed3,tor3);

        // first_stop = 0;
       
        if(start == false)
            {                
                start = true;
                //确保后续调用时不再执行初始化和时间记录的操作。
            }

}

        enum STATE {
            NUL,FORWARD,BACK,RIGHT,LEFT,RESET,STOP
        };
        
        STATE hex_state ;
        // static int old_key = -1;

        double zs = 0, xs = 0, zsv = 0, xsv = 0, xp = 0, zp = 0, xt = 0, zt = 0;
        double pi[4] = {0, 0, 0, 0}, p_JI[4] = {0, 0, 0, 0}, t_JI[4] = {0, 0, 0, 0}, q_JI[4] = {0, 0, 0, 0};
        double p_OU[4] = {0, 0, 0, 0}, t_OU[4] = {0, 0, 0, 0}, q_OU[4] = {0, 0, 0, 0};

        //void update()根据 hex_state 的不同值，执行不同的机器人运动控制逻辑，用于更新机器人的状态和控制其运动。
        //主要功能包括停止运动、向前运动、后退、向右移动、向左移动以及重置等操作。
        void update()
        {

            switch (hex_state)
            {
            case STOP:
                mv_reset();
                break;
            case FORWARD:
                mv_forword();
                break;
            case BACK:
                mv_back();
                break;
            case RIGHT:
                mv_right();
                break;
            case LEFT:
                mv_left();
                break;
            case RESET:
                mv_stop();
                break;
            default:
                break;
            }

        }

};
#endif

