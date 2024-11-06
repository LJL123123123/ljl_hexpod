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
        
        DMmotor m_LegMotor[3*LEG_NUM];
        Module<HexPodController> m_update_module;

        HexPodController():RobotController("HexPodController"),
        m_LegMotor{DMmotor("leg0_motor1"),DMmotor("leg0_motor2"),DMmotor("leg0_motor3")
                ,DMmotor("leg1_motor1"),DMmotor("leg1_motor2"),DMmotor("leg1_motor3")},

        m_update_module{MT_CONTROLLER,this,&HexPodController::update,"HexPodController_update_module"}
        {};

        struct Leg {
        double gama;
        LowPassFilter<double> Pos_filter[3];
        LowPassFilter<double> Vel_filter[3];
        LowPassFilter<double> Touq_filter[3];

        };
        struct Leg legs[LEG_NUM];
                
        void init() override{
            std::cout<<11<<std::endl;
            DM_USB2CAN* massage_ptr = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN1"));/*多写一个变量*/
            DM_USB2CAN* massage_ptr1 = dynamic_cast<DM_USB2CAN*> (PeriodicTaskManager::Instance()->FindTask("DM_USB2CAN2"));
            if(massage_ptr == nullptr || massage_ptr1 == nullptr)
            {
                fprintf(stderr,"can not find communication interface\n");
                return;
            }            

            legs[0].gama =  degree2rad(-20.0);
            legs[2].gama =  degree2rad(0.0);
            legs[4].gama =  degree2rad(+20.0);
            legs[1].gama =  degree2rad(+20.0);
            legs[3].gama =  degree2rad(0.0);
            legs[5].gama =  degree2rad(-20.0);
            for (int j = 0; j < LEG_NUM; j++)
            {
                for(int i = 0; i < 3; i++)
                {
                    if(j == 0)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr,DMMode::MIT);/*串口在这*/
                    else if(j == 1)
                    m_LegMotor[j*3+i].Connectmotor(j*3+i+1,MotorMode::MIT_MODE,massage_ptr1,DMMode::MIT);
                    fprintf(stderr,"leg%d_motor%d:m_LegMotor[%d]:%d\n",j,i,j*3+i,m_LegMotor[j*3+i].id);
                    m_LegMotor[j*3+i].control_p_des = 0;
                    m_LegMotor[j*3+i].control_k_d = 0.8f;
                    m_LegMotor[j*3+i].control_k_p = 25.0f;
                    m_LegMotor[j*3+i].control_v_des = 0.5f;
                }
            }
            
            
            // std::cout<<13<<std::endl;
        };

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
            q[1] = theta[1];
            q[2] = theta[2];
            q[3] = -(-theta[3] - degree2rad(72.0) - q[2]);
        }

        void q2theta(double* theta, double* q)
        {
            theta[1] = q[1];
            theta[2] = q[2];
            theta[3] = q[3] + degree2rad(72.0) - q[2];
        }
        
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
        void touque_caul(double gama,int leg_n)
        {
            Hex_tehta<<m_LegMotor[leg_n*3+0].feedback_pos, m_LegMotor[leg_n*3+1].feedback_pos, m_LegMotor[leg_n*3+2].feedback_pos;
            FK(gama);
            
            hex_v_0f<<m_LegMotor[leg_n*3+0].feedback_vel, m_LegMotor[leg_n*3+1].feedback_vel, m_LegMotor[leg_n*3+2].feedback_vel;
            hex_v_0f = hex_J * hex_v_0f;            

            hex_K_p<< 1, 0, 0,     0, 1, 0,     0, 0, 1;
            hex_K_d<< 1, 0, 0,     0, 1, 0,     0, 0, 1;
            hex_f_d = hex_K_p * (hex_p_0d-Hex_leg_p) + hex_K_d * (hex_v_0d-hex_v_0f);
            hex_touq = hex_J.trace() * hex_f_d;
        }

        void gait_creat(double t, double* xs, double* zs, double* xv, double* zv)
        {
        double s = (t - (int)(t / hex_T) * hex_T);
        //printf("s=%lf\n", s);
        if (s < hex_T / 2)
        {
            *zs = hex_H / 0.04 * (2.5723 * pow(s, 3) - 7.717 * pow(s, 4) + 7.717 * pow(s, 5) - 2.5723 * pow(s, 6));
            *xs = 0.5*hex_L / 0.2 * (1.4267 * pow(s, 3) - 1.28 * pow(s, 4) - 0.52 * pow(s, 5) + 0.5733 * pow(s, 6) - 0.1);

            *zv = hex_H / 0.04 * (2.5723 * pow(s, 2) * 3 - 7.717 * pow(s, 3) * 4 + 7.717 * pow(s, 4) * 5 - 2.5723 * pow(s, 5) * 6);
            *xv = 0.5*hex_L / 0.2 * (1.4267 * pow(s, 2) * 3 - 1.28 * pow(s, 3) * 4 - 0.52 * pow(s, 4) * 5 + 0.5733 * pow(s, 5) * 6);
        }
        else
        {
            *zs = 0;
            *xs = 0.5*hex_L / 0.2 * (1.4267 * pow(hex_T - s, 3) - 1.28 * pow(hex_T - s, 4) - 0.52 * pow(hex_T - s, 5) + 0.5733 * pow(hex_T - s, 6) - 0.1);

            *zv = 0;
            *xv = 0.5*hex_L / 0.2 * (1.4267 * pow(hex_T - s, 2) * 2 - 1.28 * pow(hex_T - s, 3) * 4 - 0.52 * pow(hex_T - s, 4) * 5 + 0.5733 * pow(hex_T - s, 5) * 6);
        }

        }

        
        bool start = false;
        void leg_controll(int leg_n, double* q ) //输入rad
        {            
            if(start == false)
            {
                for(int i = 0; i < 3; i++)
                {
                    m_LegMotor[leg_n*3+i].control_k_d = 0.8f;
                    m_LegMotor[leg_n*3+i].control_k_p = 25.0f;
                }
            }
                        
            Jacbi(legs[leg_n].gama,leg_n);
            touque_caul(legs[leg_n].gama,leg_n);

            double temp[] = { 0,0,0,0 };
            theta2q(q, q);

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
                }
            
        }

        // static double error = 0;
        //std::chrono::time_point start_time = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point start_time1;
        double t = 0;
        double tx = 0;
        int num = 0;
        double flag_1 = 1;
        double flag_2 = 1;
        double flag_k1 = 1;
        double flag_k2 = 1;

        void mv_forword(double* p_JI, double* p_OU, double* t_JI, double* t_OU, double* q_JI, double* q_OU,
        double* pi)
        {
            if(start == false)
            {
                std::cout << "mv_forward运行" << std::endl;
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
        t=hex_speed*tx; // ��΢��ת��Ϊ��

        /******************************/

        double xs=0,zs=0,xsv=0,zsv=0;
        gait_creat(t, &xs, &zs, &xsv, &zsv);
        p_JI[1] = XYLL* cos(legs[0].gama);
        p_JI[2] = xs + XYLL * sin(legs[0].gama);
        p_JI[3] = zs - ZLL;
        hex_p_0d << p_JI[1],p_JI[2],p_JI[3];    
        hex_v_0d << 0.0,xsv,zsv;        
        IK(legs[0].gama, pi, p_JI, q_JI);
        // if(q_JI[3]>0)
        // q_JI[3]-=PI/2;
        // else
        // q_JI[3]+=PI/2;
        leg_controll(0, q_JI);

        /**************************************/

        double xp=0,zp=0,xpv=0,zpv=0;
        gait_creat(t + (double)1*hex_T / 2, &xp, &zp, &xpv, &zpv);
        p_OU[1] = XYLL * cos(legs[1].gama);
        p_OU[2] = xp - XYLL * sin(legs[1].gama);
        p_OU[3] = zp - ZLL;
        hex_p_0d << p_OU[1],p_OU[2],p_OU[3];
        hex_v_0d << 0.0,xpv,zpv;
        IK(-legs[1].gama, pi, p_OU, q_OU);
        q_OU[1] = -q_OU[1];
        // if(q_OU[3]>0)
        // q_OU[3]-=PI/2;
        // else
        // q_OU[3]+=PI/2;      
        leg_controll(1, q_OU);

        /****************************************/

        // p_JI[1] = XYLL * cos( legs[3].gama);
        // p_JI[2] = *xs;
        // p_JI[3] = *zs - ZLL;

        // p_OU[1] = XYLL * cos( legs[4].gama);
        // p_OU[2] = *xp;
        // p_OU[3] = *zp - ZLL;


        // p_JI[2] = p_JI[2];
        // p_OU[2] = p_OU[2];
        // IK(0, pi, p_JI, q_JI);
        // IK(0, pi, p_OU, q_OU);
        // q_JI[1] = -q_JI[1];
        // leg_controll(3, q_JI, flag_1, flag_k1);
        // leg_controll(4, q_OU, flag_2, flag_k2);


        // p_JI[1] = XYLL * cos( legs[5].gama);
        // p_JI[2] = *xs;
        // p_JI[3] = *zs - ZLL;

        // p_OU[1] = XYLL * cos( legs[6].gama);
        // p_OU[2] = *xp;
        // p_OU[3] = *zp - ZLL;

        // p_JI[2] = p_JI[2] + XYLL * sin( legs[5].gama);
        // p_OU[2] = p_OU[2] - XYLL * sin( legs[6].gama);
        // IK(0, pi, p_JI, q_JI);
        // IK(0, pi, p_OU, q_OU);
        // q_OU[1] = -q_OU[1];
        // leg_controll(5, q_JI, flag_1, flag_k1);
        // leg_controll(6, q_OU, flag_2, flag_k2);

        // num++;
        //  if(tx-1.0<0.01 && tx-1.0>-0.01)
        // {
        //     std::cout << num << std::endl;
        // }

        // if(std::abs(std::fmod(t, T/2) - T/2) < 0.0001)
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     std::cout << "1" << std::endl;
        // }
        if(start == false)
            {                
                start = true;
            }

    }

        void si_mv_forword(double* zs, double* xs, double* zp, double* xp, double* zt, double* xt,
            double* p_JI, double* p_OU, double* t_JI, double* t_OU, double* q_JI, double* q_OU,
            double* pi)
    {
        if(start == false)
            {
                std::cout << "si_mv_forward运行" << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                start = true;
                //std::cout << start << std::endl;
                auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                std::tm* start_time_tm = std::localtime(&start_time_t);

                // Print start_time
                std::cout << "Start time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
            }
        m_LegMotor[0].control_p_des = 15;
        m_LegMotor[0].control_v_des = 0.5f;
    }
       
        void mv_stop()
        {   
        num = 0 ;
        double q[] = { 0,0,0,0 };
        hex_omega << 0,0,0;
        for(int j=0;j<LEG_NUM;j++)
            for(int i=0;i<3;i++)
            {
                legs[j].Pos_filter[i].previousFilteredValue = 0;
                legs[j].Pos_filter[i].alpha = 1e-2;
                legs[j].Vel_filter[i].previousFilteredValue = hex_omega[i];
                legs[j].Vel_filter[i].alpha = 1e-8;
                legs[j].Touq_filter[i].previousFilteredValue = 0;
                legs[j].Touq_filter[i].alpha = 1e-10;

                m_LegMotor[i].control_k_d = 0.1f;
                m_LegMotor[i].control_k_p = 5.0f;
            }
        leg_controll(0, q);
        // leg_controll(5, q);
        leg_controll(1, q);
        // leg_controll(6, q);
        // leg_controll(3, q);
        // leg_controll(4, q);
        
        start = false;
        //std::cout << start << std::endl;
        }

        enum STATE {
            STOP,
            FORWARD,
            BACK,
            RIGHT,
            LEFT,
            RESET
        };
        STATE hex_state;
        // static int old_key = -1;

        double zs = 0;
        double xs = 0;
        double zsv = 0;
        double xsv = 0;
        double xp = 0;
        double zp = 0;
        double xt = 0;
        double zt = 0;
        double pi[4] = { 0, 0, 0, 0 };

        double p_JI[4] = { 0, 0, 0, 0 };
        double t_JI[4] = { 0, 0, 0, 0 };
        double q_JI[4] = { 0, 0, 0, 0 };

        double p_OU[4] = { 0, 0, 0, 0 };
        double t_OU[4] = { 0, 0, 0, 0 };
        double q_OU[4] = { 0, 0, 0, 0 };

        void update()
        {
            // m_LegMotor[0].control_p_des = 0.0f;
            // m_LegMotor[1].control_p_des = 0.0f;
            // m_LegMotor[2].control_p_des = 0.0f;
            // if(m_Is_motor_init[x])
            // {
            //     m_position_set[x] = clamp(m_position_set[x],(float)X_MIN,(float)X_MAX);
            //     m_OrbitMotor[x].control_p_des = -Pos_filter[x].filter(m_position_set[x]);          
            // }
            // else
            // {
            //     update_init_state();
            // }

            switch (hex_state)
            {
            case STOP:
                mv_stop();
                break;
            case FORWARD:
                mv_forword(
                    p_JI, p_OU, t_JI, t_OU, q_JI, q_OU,
                    pi);

                break;
            case BACK:
                // si_mv_forword(&zs, &xs, &zp, &xp, &zt, &xt,
                //     p_JI, p_OU, t_JI, t_OU, q_JI, q_OU,
                //     pi);

                break;
                case RIGHT:
                    // mv_right(&zs, &xs, &zp, &xp,
                    //     p_JI, p_OU, t_JI, t_OU, q_JI, q_OU,
                    //     pi);
                    break;
                case LEFT:
                    // mv_left(&zs, &xs, &zp, &xp,
                    //     p_JI, p_OU, t_JI, t_OU, q_JI, q_OU,
                    //     pi);
            case RESET:
                for (int t = 1; t < 10; t++)
                    for (int j = 1; j <= LEG_NUM; j++)
                        for (int i = 1; i <= 3; i++)
                        {
                            if(j<=3);
                            //dm0->reset(legs[j].motor_id[i]);
                            else;
                            //dm1->reset(legs[j].motor_id[i]);
                        }

                break;
            default:
                // ����������ֵ�ǰState����
                break;
            }

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