#ifndef IRISTAC_PROJECT_HPP
#define IRISTAC_PROJECT_HPP
#include "Ctr_Hexpod.hpp"
#include "Ctr_IrisTacController.hpp"
class IrisTac_Project : public RobotRunner
{
    public:
        IrisTac_Project():RobotRunner(this){}
        HexPodController m_hexpodController;
};

#endif