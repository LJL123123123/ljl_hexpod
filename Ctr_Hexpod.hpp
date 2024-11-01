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


    
class HexPodController : public RobotController
{
    public:
        LowPassFilter<double> Pos_filter[3];
        LowPassFilter<double> Vel_filter[3];
        LowPassFilter<double> Touq_filter[3];
        DMmotor m_LegMotor[3];
        Module<HexPodController> m_update_module;
        std::string motorNames[3];
        double gamma = 0 ; 
        int leg_n = 0;  

        HexPodController():RobotController("HexPodController"),
        m_LegMotor{DMmotor("leg1_motor1"),DMmotor("leg1_motor2"),DMmotor("leg1_motor3")},

        m_update_module{MT_CONTROLLER,this,&HexPodController::update,"HexPodController_update_module"}
        {};

        
             
        void init() override{
            std::cout<<11<<std::endl;
            DM_USB2CAN* massage_ptr = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN1"));
            if(massage_ptr == nullptr)
            {
                fprintf(stderr,"can not find communication interface\n");
                return;
            }            

                        
            for(int i = 0; i < 3; i++)
            {
                m_LegMotor[i].Connectmotor(leg_n*3+i+1,MotorMode::MIT_MODE,massage_ptr,DMMode::MIT);
                m_LegMotor[i].control_p_des = 0;
                m_LegMotor[i].control_k_d = 0.8f;
                m_LegMotor[i].control_k_p = 25.0f;
                m_LegMotor[i].control_v_des = 0.5f;
            }
            // std::cout<<13<<std::endl;
        };

        
        
        Eigen::Vector3d Hex_leg_p;
        Eigen::Vector3d Hex_tehta;
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

            /*******************************/

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

            /********************************/

            // for (int i = 1; i < 4; ++i) {
            //                     std::cout << "theta[" << i << "] = " << theta[i]<< std::endl;
            //                  };

        }

        Eigen::Matrix3d hex_J;
        Eigen::Vector3d hex_v_0d;
        Eigen::Vector3d hex_omega;  

        void Jacbi(double gama)
    {
      
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

        hex_J << J_v[0], J_v[1], J_v[2],      // 矩阵元素
         J_v[3], J_v[4], J_v[5],
         J_v[6], J_v[7], J_v[8];

        // std::cout << "hex_J: " << hex_J.transpose() << std::endl;//打印出来是转置
        hex_omega = hex_J.inverse() * hex_v_0d;
    }
        
        Eigen::Vector3d hex_p_0d;//目标
        Eigen::Vector3d hex_v_0f;
        Eigen::Matrix3d hex_K_p;
        Eigen::Matrix3d hex_K_d;
        Eigen::Vector3d hex_f_d;
        Eigen::Vector3d hex_touq;
        void touque_caul(double gama)
        {
            Hex_tehta<<m_LegMotor[0].feedback_pos, m_LegMotor[1].feedback_pos, m_LegMotor[2].feedback_pos;
            FK(gama);
            
            hex_v_0f<<m_LegMotor[0].feedback_vel, m_LegMotor[1].feedback_vel, m_LegMotor[2].feedback_vel;
            hex_v_0f = hex_J * hex_v_0f;            

            hex_K_p<< 1, 0, 0,     0, 1, 0,     0, 0, 1;
            hex_K_d<< 1, 0, 0,     0, 1, 0,     0, 0, 1;
            hex_f_d = hex_K_p * (hex_p_0d-Hex_leg_p) + hex_K_d * (hex_v_0d-hex_v_0f);
            hex_touq = hex_J.trace() * hex_f_d;
        }
        
        bool start = false;        
        void leg_controll( double* q ) //输入rad
        {
            if(start == false)
            {
                for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[i].control_k_d = 0.8f;
                    m_LegMotor[i].control_k_p = 25.0f;
                }
            }
            
            Hex_tehta << m_LegMotor[0].feedback_pos,
                     m_LegMotor[1].feedback_pos,
                     m_LegMotor[2].feedback_pos;

            Jacbi(gamma);
            touque_caul(gamma);

            double temp[] = { 0,0,0,0 };
            theta2q(q, q);

            if(leg_n==1||leg_n==2||leg_n==5)
            q[3] = -q[3];
            if(leg_n==0||leg_n==3||leg_n==4)
            q[2] = -q[2];

            for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[i].control_v_des = Vel_filter[i].filter(hex_omega[i]);
                    m_LegMotor[i].control_p_des = Pos_filter[i].filter(q[i+1]);  
                    m_LegMotor[i].control_torque = -Touq_filter[i].filter(hex_touq[i]);                     
                }
            
        }
        double q[4];
        void update()
        {            
            leg_controll( q );            
        }

        
        

        // void update_init_state()
    // {
    //     for(int i = 0; i < orbit_length; i++)
    //     {
    //         if(!m_Is_motor_init[i])
    //         {
    //             if(m_init_state[i] == set)
    //             {
    //                 m_OrbitMotor[i].m_MotorMode = MotorMode::MIT_MODE;
    //                 m_OrbitMotor[i].control_p_des = 0;
    //                 m_OrbitMotor[i].control_v_des = 0;
    //                 m_OrbitMotor[i].control_k_p = 0;
    //                 m_OrbitMotor[i].control_k_d = 0;
    //                 m_OrbitMotor[i].control_torque = 0.25;
    //                 m_init_state[i] = waiting;
    //             }
    //             else if(m_init_state[i] == waiting)
    //             {
    //                 if(abs(m_OrbitMotor[i].feedback_vel)>1.0)
    //                 {
    //                     stuck_time[i] = Gloabl_Timer::Instance()->getSeconds();
    //                 }
    //                 if(Gloabl_Timer::Instance()->getSeconds() - stuck_time[i] > 0.5)
    //                 {
    //                     m_init_state[i] = zero;
    //                 }
    //             }
    //             else if(m_init_state[i] == zero)
    //             {
    //                 m_OrbitMotor[i].save_zero();
    //                 if(abs(m_OrbitMotor[i].feedback_pos) <0.01)
    //                 {
    //                     m_init_state[i] = reset;
    //                 }
    //             }
    //             else if (m_init_state[i] == reset)
    //             {
    //                 m_OrbitMotor[i].m_MotorMode = MotorMode::POS_VEC_MODE;
    //                 m_OrbitMotor[i].control_k_d = 1.0f;
    //                 m_OrbitMotor[i].control_k_p = 10.0f;
    //                 m_OrbitMotor[i].control_p_des = -20.0f;
    //                 m_OrbitMotor[i].control_v_des = 20.0f;
    //                 m_OrbitMotor[i].control_torque = 0.0;
    //                 m_position_set[i] = -m_OrbitMotor[i].control_p_des;
    //                 Pos_filter[i].previousFilteredValue = m_position_set[i];
    //                 Pos_filter[i].alpha = 0.01;
    //                 if(abs(m_OrbitMotor[i].feedback_pos-m_OrbitMotor[i].control_p_des)<0.05f) 
    //                 {
    //                     m_Is_motor_init[i] = true;
    //                     m_OrbitMotor[i].m_MotorMode = MotorMode::MIT_MODE;
    //                     m_OrbitMotor[i].control_v_des = 0.0f;
    //                     m_OrbitMotor[i].control_k_d = 1.0f;
    //                     m_OrbitMotor[i].control_k_p = 10.0f;
    //                 } 
    //             }
    //         }
    //     }
    // }
};
#endif