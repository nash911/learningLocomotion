/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   L E A R N   A P P L I C A T I O N                                                                          */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

// System includes
#include <time.h>
#include <stdexcept>
#include <cstdlib>

#include "Robot.h"
#include "SimulationOpenRave.h"
#include "Y1ModularRobot.h"
#include "LearningAlgorithm.h"
#include "QLearning.h"
#include "ApproximateQLearning.h"
//#include "FileHandler.h"

#define ROBOT_OPENRAVE
//#define ROBOT_Y1

#ifdef ROBOT_OPENRAVE
#define AVERAGE_BROADCAST_PERIOD 0.0025
#else
#define AVERAGE_BROADCAST_PERIOD 0.01
#endif

//#define CUMUALATIVE_DISTANCE
#define FORWARD_DISTANCE

//std::string note("No Notes");

int main(int argc, char* argv[])
{
    /* initialize random seed: */
    srand (time(NULL));

    vector<double> actions;
    //actions.push_back(-2.0);
    actions.push_back(-1.0);
    actions.push_back(0.0);
    actions.push_back(1.0);
    //actions.push_back(2.0);

    double state_min = -75.0;
    double state_max = 75.0;
    double state_res = 15.0;

    unsigned int W_size = 6;

    char* filename;

    Robot *robot = NULL;

    SimulationOpenRave simuOR_robot;
    Y1ModularRobot y1_robot;

#ifdef ROBOT_OPENRAVE
    robot = &simuOR_robot;
#elif defined(ROBOT_Y1)
    y1_robot.set_serial_port(argv[1], BAUD_RATE);
    robot = &y1_robot;
#else
    std::cerr << "MorphoMotion Error: Learn." << std::endl
              << "int main(int, char*) method." << std::endl
              << "Robot Environment needs to be defined!. " << std::endl;
    exit(1);
#endif


    LearningAlgorithm *l = NULL;
    l = new QLearning(robot, actions, state_min, state_max, state_res, ALPHA, GAMA, EPSILON);
    //l = new ApproximateQLearning(robot, actions, state_min, state_max, state_res, ALPHA, GAMA, EPSILON, W_size);

#ifdef ROBOT_OPENRAVE
    simuOR_robot.init_simu_env("Dummy");
#elif defined(ROBOT_Y1)
    //FileHandler parametersFileHandler(parameter_file, robot, NULL, controller, &mlp);
#endif

    //--Initialise the robot with 0 degrees to the motor and move it to the initial position--//
    robot->reset_robot();

    //--Initialise Controller--//
    l->init_controller(0.0);

    cout << endl << "argc: " << argc << endl;

    if(argc == 3)
    {
        if(argv[2][0] == '-'  && (argv[2][1] == 'c' || argv[2][1] == 'e'))
        {
            filename = "Q_Pi.dat";
            l->init_q_val(filename);

            filename = "V.dat";
            l->init_v_val(filename);

            filename = "R.dat";
            l->init_steps(filename);
        }
        else if(argv[2][0] == '-' && argv[2][1] == 'l')
        {
            l->initialise_Q(Q_INIT_VAL);
            l->initialise_V(V_INIT_VAL);
        }
        else
        {
            std::cerr << "LearningLocomotion Error: Learn." << std::endl
                      << "int main(int, char*) method." << std::endl
                      << "Invalid command line parameters" << std::endl
                      << "Usage: ./Learn -[Robot Type (m/t/q/y/l)] -[Mode (l/c/e)]" << std::endl;
            exit(1);
        }
    }
    else
    {
        std::cerr << "LearningLocomotion Error: Learn." << std::endl
                  << "int main(int, char*) method." << std::endl
                  << "Incurrent number of command line parameters. " << std::endl
                  << "Usage: ./Learn -[Robot Type (m/t/q/y/l)] -[Mode (l/c/e)]" << std::endl;
        exit(1);
    }

    char keyboard_key;
    std::cout << std::endl << "Press a key to start" << std::endl;
    std::cin.get(keyboard_key);

    if(argv[2][0] == '-'  && (argv[2][1] == 'l' || argv[2][1] == 'c'))
    {
        //-- Start (-l) or Continue (-c) Learning--//
        l->start_learning("Epsilon_Greedy");
    }
    else if(argv[2][0] == '-'  && argv[2][1] == 'e')
    {
        //-- Evaluate (-e)--//
        l->start_evaluation();
    }

    //char keyboard_key;
    std::cout << std::endl << "Press a key to end" << std::endl;
    std::cin.get(keyboard_key);

    delete l;

    cout << endl << "Finished Learning..." << endl << endl;
    return 0;
}
