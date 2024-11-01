#ifndef HEX_GAIT_HPP
#define HEX_GAIT_HPP

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
#include "Ctr_Hexpod.hpp"

    
class Hex_gait : public RobotController
{
    public:
        Module<Hex_gait> Hex_gait_update_module;
        Hex_gait() : RobotController("Hex_gait") ,
        Hex_gait_update_module{MT_CONTROLLER,this,&Hex_gait::update,"Hex_gait_update_module"}
        {};
        HexPodController Leg[LEG_NUM];

        void init () override
        {
            Leg[0].gamma =  degree2rad(-20.0);
            // Leg[2].gamma =  degree2rad(0.0);
            // Leg[4].gamma =  degree2rad(+20.0);
            // Leg[1].gamma =  degree2rad(+20.0);
            // Leg[3].gamma =  degree2rad(0.0);
            // Leg[5].gamma =  degree2rad(-20.0);
            for(int i = 0;i<LEG_NUM;i++)
            Leg[i].leg_n=i;

            
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

        std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
        std::chrono::_V2::system_clock::time_point start_time1;
        double t = 0;
        double tx = 0;
        int num = 0;

        void mv_forword(double* p_JI, double* p_OU, double* t_JI, double* t_OU, double* q_JI, double* q_OU,
        double* pi)
        {
            
            if(Leg[0].start == false)
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
        p_JI[1] = XYLL* cos(Leg[0].gamma);
        p_JI[2] = xs + XYLL * sin(Leg[0].gamma);
        p_JI[3] = zs - ZLL;
        Leg[0].hex_p_0d << p_JI[1],p_JI[2],p_JI[3];    
        Leg[0].hex_v_0d << 0.0,xsv,zsv;        
        Leg[0].IK(0, pi, p_JI, Leg[0].q);
        // if(q_JI[3]>0)
        // q_JI[3]-=PI/2;
        // else
        // q_JI[3]+=PI/2;


        /**************************************/

        // double xp=0,zp=0,xpv=0,zpv=0;
        // gait_creat(t + (double)1*hex_T / 2, &xp, &zp, &xpv, &zpv);
        // p_OU[1] = XYLL * cos(legs[2].gama);
        // p_OU[2] = xp - XYLL * sin(legs[2].gama);
        // p_OU[3] = zp - ZLL;
        // hex_p_0d << p_OU[1],p_OU[2],p_OU[3];
        // hex_v_0d << 0.0,xpv,zpv;
        // IK(0, pi, p_OU, q_OU);
        // q_OU[1] = -q_OU[1];
        // if(q_OU[3]>0)
        // q_OU[3]-=PI/2;
        // else
        // q_OU[3]+=PI/2;      
        // leg_controll(2, q_OU);

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
        if(Leg[0].start == false)
            {                
                Leg[0].start = true;
            }

    }

    //     void si_mv_forword(double* zs, double* xs, double* zp, double* xp, double* zt, double* xt,
    //         double* p_JI, double* p_OU, double* t_JI, double* t_OU, double* q_JI, double* q_OU,
    //         double* pi)
    // {
    //     if(Leg[0].start == false)
    //         {
    //             std::cout << "si_mv_forward运行" << std::endl;
    //             start_time = std::chrono::high_resolution_clock::now();
    //             Leg[0].start = true;
    //             //std::cout << start << std::endl;
    //             auto start_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //             std::tm* start_time_tm = std::localtime(&start_time_t);

    //             // Print start_time
    //             std::cout << "Start time: " << std::put_time(start_time_tm, "%Y-%m-%d %H:%M:%S") << std::endl;
    //         }
    //     Leg[0].m_LegMotor[0].control_p_des = 15;
    //     Leg[0].m_LegMotor[0].control_v_des = 0.5f;
    // }
       
        void mv_stop()
        {   
        num = 0 ;
        for(int i=1;i<=3;i++)
        Leg[0].q[i] = 0;
        Leg[0].hex_omega << 0.0d,0.0d,0.0d;
        for(int i=0;i<3;i++)
        {
            Leg[0].Pos_filter[i].previousFilteredValue = 0;
            Leg[0].Pos_filter[i].alpha = 1e-2;
            Leg[0].Vel_filter[i].previousFilteredValue = Leg[0].hex_omega[i];
            Leg[0].Vel_filter[i].alpha = 1e-8;
            Leg[0].Touq_filter[i].previousFilteredValue = 0;
            Leg[0].Touq_filter[i].alpha = 1e-10;

            Leg[0].m_LegMotor[i].control_k_d = 0.1f;
            Leg[0].m_LegMotor[i].control_k_p = 5.0f;        
        }
        // leg_controll(5, q);
        // leg_controll(2, q);
        // leg_controll(6, q);
        // leg_controll(3, q);
        // leg_controll(4, q);
        
        Leg[0].start = false;
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
        
};
#endif