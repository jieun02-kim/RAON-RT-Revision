/*
 * Test program to demonstrate conditional hardware/simulation operation
 * Usage: ./test_simulation [sim]
 *   - No arguments: Run with hardware (EtherCAT)
 *   - "sim" argument: Run with MuJoCo simulation
 */

#include "Indy7Ctrl.h"
#include <iostream>
#include <signal.h>

CRobotIndy7* g_pRobot = nullptr;

void signal_handler(int sig)
{
    std::cout << "\nShutting down..." << std::endl;
    if (g_pRobot)
    {
        g_pRobot->DeInit();
        delete g_pRobot;
        g_pRobot = nullptr;
    }
    exit(0);
}

int main(int argc, char* argv[])
{
    // Install signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Check command line arguments
    bool useSimulation = false;
    if (argc > 1 && std::string(argv[1]) == "sim")
    {
        useSimulation = true;
        std::cout << "Running in SIMULATION mode with MuJoCo visualization" << std::endl;
    }
    else
    {
        std::cout << "Running in HARDWARE mode with EtherCAT" << std::endl;
    }
    
    // Create robot controller
    g_pRobot = new CRobotIndy7();
    
    // Initialize with simulation flag
    if (!g_pRobot->Init(useSimulation))
    {
        std::cerr << "Failed to initialize robot controller" << std::endl;
        delete g_pRobot;
        return -1;
    }
    
    std::cout << "Robot controller initialized successfully" << std::endl;
    std::cout << "Mode: " << (useSimulation ? "SIMULATION" : "HARDWARE") << std::endl;
    std::cout << "Press Ctrl+C to quit..." << std::endl;
    
    // Enable robot control
    if (!g_pRobot->EnableRobot())
    {
        std::cerr << "Failed to enable robot" << std::endl;
        g_pRobot->DeInit();
        delete g_pRobot;
        return -1;
    }
    
    std::cout << "Robot enabled. Control loop running..." << std::endl;
    
    // Main loop - just keep running until signal
    while (true)
    {
        sleep(1);
        
        // Print robot state periodically
        static int counter = 0;
        if (++counter % 5 == 0)  // Every 5 seconds
        {
            if (g_pRobot->IsMoving())
            {
                std::cout << "Robot is moving..." << std::endl;
            }
            else
            {
                std::cout << "Robot is stationary" << std::endl;
            }
            
            // Example: Show first joint position
            ROBOT_STATE state;
            if (g_pRobot->GetCurrentRobotState(state))
            {
                std::cout << "Joint 0 position: " << state.pos[0] << " rad" << std::endl;
            }
        }
    }
    
    return 0;
}
