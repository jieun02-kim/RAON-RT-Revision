/*****************************************************************************
*	Name: main.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Sample application of the AxisEPOS4 EtherCAT Slave
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include <cstdio>
#include <unistd.h>
#include "ConfigRobot.h"
#include "Indy7Ctrl.h"
#include "ExtInterface.h"


void signal_handler     (int nSignal);

CConfigRobot    *g_cConfigRobot;
CRobotIndy7        *g_cRobot;
CExtInterface   *g_cExtInterface;

int main(int argc, char **argv)
{
    g_cConfigRobot = NULL;
    g_cRobot = NULL;
    g_cExtInterface = NULL;

    DBG_LOG_INFO("=======================ROBOT APPLICATION STARTED!======================");

    int nOpt;
    BOOL bDefConfig = TRUE;
    TSTRING myFile = TSTRING(DEFAULT_CONFIGURATION_FULLPATH);
    
    while ((nOpt = getopt(argc, argv, "f:")) != -1)
    {
        switch (nOpt)
        {
        case 'f':
            myFile = TSTRING(optarg);
            DBG_LOG_INFO("Using configuration file: %s", myFile.c_str());
            bDefConfig = FALSE;
            break;
        default:
            break;
        }
    }
    if (TRUE == bDefConfig)
    {
        DBG_LOG_INFO("Using default configuration file: %s", myFile.c_str());
    }

    BOOL bSimMode = TRUE;
    g_cConfigRobot = new CConfigRobot();
    
    /* Load Configuration File */
    
    if (FALSE == IsExistFile(myFile.c_str()))
    {
        DBG_LOG_ERROR("Cannot read configration file %s!", myFile.c_str());
        return FALSE;
    }
    g_cConfigRobot->ReadConfiguration(myFile);
    bSimMode = g_cConfigRobot->GetSystemConf().bSim;

    
    g_cRobot = new CRobotIndy7(g_cConfigRobot);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);


    /* Lock all current and future pages from preventing of being paged to swap */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Determine whether to execute aging on start-up
    // g_cRobot->EnableAgingTest(g_cConfigRobot->GetSystemConf().bAging, g_cConfigRobot->GetSystemConf().nAgingDur);

    /* Initialize Robot */
    if (FALSE == g_cRobot->Init(bSimMode))
    {
        DBG_LOG_ERROR("Cannot Start Control Application!");
        return RET_FAIL;
    }

    while (!g_cRobot->CheckStopTask())
    {
        cpu_relax();
        usleep(1000);
    }

    // g_cRobot->WriteDataLog();

    DBG_LOG_INFO("=======================ROBOT APPLICATION ENDED!=======================");
    return RET_SUCC;
}

void signal_handler(int nSignal)
{
    DBG_LOG_INFO("RT-POSIX has been signalled [%d]", nSignal);
    g_cRobot->DeInit();
}
