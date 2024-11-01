#include "Timer.h"
#include "IrisTac_Project.hpp"
#include "KeyboardReader.hpp"
#include <stdio.h>
#include "Math_Tool.hpp"
#include "Ctr_Hexpod.hpp"
#include "./include/user.h"

#include <QtWidgets/QApplication>
#include "mainwindow.h"

#include "Hex_gait.hpp"
 
#define NEED_PLOT 1
#define MAX_Y -5
#define MIN_Y 5
// #define CHANNEL_NUM 3*LEG_NUM
#define CHANNEL_NUM 3

#if NEED_PLOT
    
    CurvePlotFifo* fifo = nullptr;
    void Plotfunc(int argc, char *argv[])
    {
        QApplication a(argc, argv);
        MainWindow w(MIN_Y,MAX_Y,CHANNEL_NUM);
        fifo = w.GetFIFO();
        w.show();
        a.exec();
    }
#endif


int main(int argc, char *argv[])
{
    Gloabl_Timer::Instance();
    PeriodicTaskManager::Instance();

    DM_USB2CAN test = DM_USB2CAN("DM_USB2CAN1",100,0.001,921600,"/dev/ttyACM0",128,1024,8);
    test.start();

    IrisTac_Project Project;
    Project.start();

    RobotRunner::Instance()->Print_Controller();
    RobotRunner::Instance()->Print_RobotPart();
    RobotRunner::Instance()->Print_Module();

    #if NEED_PLOT
    std::thread t(Plotfunc,argc,argv);
    #endif

    KeyboardReader reader;
    reader.initialize();

    int64_t lastTimeMs = 0; 
    int64_t lastTimeMs2 = 0; 
    int64_t lastTimeMs3 = 0; 
    int64_t lastTimeMs4 = 0; 
    bool Is_changing_LED = true;
    int photo_num = 0;
    float led_num = 0;
    
    Project.m_hexpodController.hex_state = Project.m_hexpodController.STOP;
    while(1)
    {
        reader.readEvents();

        #if NEED_PLOT
        if (Gloabl_Timer::hasElapsedMs(10, lastTimeMs2))
        {
            if(fifo!=nullptr)
            {
                QVector<double> newData;
                double key = QTime::currentTime().msecsSinceStartOfDay() / 1000.0;
                newData.append(key);
                // newData.append(Project.m_hexpodController.Leg[0].q[1]);
                // newData.append(Project.m_hexpodController.Leg[0].q[2]);
                // newData.append(Project.m_hexpodController.Leg[0].q[3]);
                // newData.append(Project.m_hexpodController.m_LegMotor[0].feedback_pos);
                // newData.append(Project.m_hexpodController.m_LegMotor[1].feedback_pos);
                // newData.append(Project.m_hexpodController.m_LegMotor[2].feedback_pos);                
                newData.append(Project.m_hexpodController.Leg[0].m_LegMotor[0].control_p_des);
                newData.append(Project.m_hexpodController.Leg[0].m_LegMotor[1].control_p_des);
                newData.append(Project.m_hexpodController.Leg[0].m_LegMotor[2].control_p_des);
                // newData.append(Project.m_hexpodController.m_LegMotor[0].control_v_des);
                // newData.append(Project.m_hexpodController.m_LegMotor[1].control_v_des);
                // newData.append(Project.m_hexpodController.m_LegMotor[2].control_v_des);                
                // newData.append(Project.m_hexpodController.m_LegMotor[0].feedback_vel);
                // newData.append(Project.m_hexpodController.m_LegMotor[1].feedback_vel);
                // newData.append(Project.m_hexpodController.m_LegMotor[2].feedback_vel);
                // newData.append(Project.m_hexpodController.m_LegMotor[0].control_torque);
                // newData.append(Project.m_hexpodController.m_LegMotor[1].control_torque);
                // newData.append(Project.m_hexpodController.m_LegMotor[2].control_torque);
                // newData.append(Project.m_hexpodController.m_LegMotor[0].feedback_torque);
                // newData.append(Project.m_hexpodController.m_LegMotor[1].feedback_torque);
                // newData.append(Project.m_hexpodController.m_LegMotor[2].feedback_torque);
                if (!fifo->isFull())
                {
                    fifo->write(newData);
                }            
            }
        }
        #endif
        if (Gloabl_Timer::hasElapsedMs(100, lastTimeMs))
        {   
            // printf("stderr","%f",Project.m_hexpodController.m_LegMotor[0].control_p_des);
            if(reader.isKeyPressed(reader.getKeyCode("F")))
            {
                // Project.m_hexpodController.m_LegMotor = 
                Project.m_hexpodController.hex_state = Project.m_hexpodController.FORWARD;
            }
            else if(reader.isKeyPressed(reader.getKeyCode("S")))
            {
                Project.m_hexpodController.hex_state = Project.m_hexpodController.STOP;
            }
            else if(reader.isKeyPressed(reader.getKeyCode("B")))
            {
                Project.m_hexpodController.hex_state = Project.m_hexpodController.BACK;
            }
            else if(reader.isKeyPressed(reader.getKeyCode("S")))
            {

            }
            else if(reader.isKeyPressed(reader.getKeyCode("R")))
            {

            }
        }
    }
}
