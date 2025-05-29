#include "Timer.h"
#include "IrisTac_Project.hpp"
#include "KeyboardReader.hpp"
#include <stdio.h>
#include "Math_Tool.hpp"
#include "Ctr_Hexpod.hpp"
#include "./include/user.h"


// #include "Hex_gait.hpp"
 

int main(int argc, char *argv[])
{
    Gloabl_Timer::Instance();
    PeriodicTaskManager::Instance();

    DM_USB2CAN test = DM_USB2CAN("DM_USB2CAN1",100,0.001,921600,"can1",128,1024,8);
    DM_USB2CAN test1 = DM_USB2CAN("DM_USB2CAN2",100,0.001,921600,"can2",128,1024,8);
    DM_USB2CAN test2 = DM_USB2CAN("DM_USB2CAN3",100,0.001,921600,"can3",128,1024,8);
    
    DM_USB2CAN test3 = DM_USB2CAN("DM_USB2CAN4",100,0.001,921600,"can4",128,1024,8);
    DM_USB2CAN test4 = DM_USB2CAN("DM_USB2CAN5",100,0.001,921600,"can5",128,1024,8);
    DM_USB2CAN test5 = DM_USB2CAN("DM_USB2CAN6",100,0.001,921600,"can6",128,1024,8);

    test.start();
    test1.start();
    test2.start();
    test3.start();
    test4.start();
    test5.start();
    
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
    
    Project.m_hexpodController.hex_state = Project.m_hexpodController.NUL;
    while(1)
    {
        reader.readEvents();

        
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
            else if(reader.isKeyPressed(reader.getKeyCode("L")))
            {
                Project.m_hexpodController.hex_state = Project.m_hexpodController.LEFT;
            }
            else if(reader.isKeyPressed(reader.getKeyCode("R")))
            {
                Project.m_hexpodController.hex_state = Project.m_hexpodController.RIGHT;
            }
            else if(reader.isKeyPressed(reader.getKeyCode("E")))
            {
                Project.m_hexpodController.hex_state = Project.m_hexpodController.RESET;
            }
            // std::cout << "hex_state"<<Project.m_hexpodController.hex_state<< std::endl;
        }
    }
}
