/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   L E A R N I N G   A L G O R I T H M   H E A D E R                                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef LEARNINGALGORITHM_H
#define LEARNINGALGORITHM_H

#include <cmath>
#include <vector>
#include <thread>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#include "Robot.h"
#include "ServoFeedback.h"
#include "GraphFile.h"
//#include "OscillationAnalyzer_OutputSignal.h"

#define _USE_MATH_DEFINES

//#define Q 81 //-- Quit/Next
#define q 113
#define SPACE 32 //-- Pause
//#define P 112 //-- Previous
//#define p 80

//#define ACTIVITY_LOG
//#define DEBUGGER
//#define EVERY_STEP_DEBUGGER

//#define CUMULATIVE_DISTANCE
#define CUMULATIVE_DISTANCE_STEPSIZE 10

//#define NOISE
#define NOISE_MEAN 0
#define NOISE_SD 5

#define ALPHA 0.1
#define GAMA 0.5
#define EPSILON 0.1
#define LAMDA 0.9

#define Q_INIT_VAL 5.0
#define V_INIT_VAL 0.0
#define STEP_LIMIT 25000
#define CUTOFF_TIME 2.0

#define INVALID_ACTION_PENALTY -10.0
#define IDLE_PENALTY 0 //-0.1
#define USELESS_ACTION_PENALTY 0 //-20.0
#define WRONG_STATE_PENALTY 0 //-0.2
#define EXIT_STATE_PENALTY -1.0

#define R_ALPHA 0.1

#define EXPLORE_FUNCTION

using namespace std;

class LearningAlgorithm
{

public:

    // DEFAULT CONSTRUCTOR
    LearningAlgorithm(void);

    // CONSTRUCTOR WITH ROBOT OBJECT
    LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double);

    // CONSTRUCTOR WITH PRIMARY and SECONDARY ROBOT OBJECTS
    LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double);


    // DESTRUCTOR
    virtual ~LearningAlgorithm(void);

    // METHODS

    void reset_controller(void);

    void actuate_module(const unsigned int, double);
    void actuate_all_modules(const vector<double>&);

    bool read_servo_positions_with_time(void); // TODO: This should be implemented as a seperate thread.
    void read_servo_positions_with_time_THREAD(void);

    void set_robot_primary(Robot*);
    Robot* get_robot_primary(void);
    void set_robot_secondary(Robot*);
    Robot* get_robot_secondary(void);
    void set_servo_max(double);
    double get_servo_max(void);
    void set_servo_min(double);
    double get_servo_min(void);

    void set_EKF_dt(const double);
    double get_EKF_dt(void);
    void set_EKF_r(const double);
    double get_EKF_r(void);
    void set_EKF_qf(const double);
    double get_EKF_qf(void);

    double calculate_random_uniform(double, double) const;
    double calculate_random_normal(double, double)  const;

    void changemode(int);
    int kbhit (void);

    //-- VIRTUAL FUNCTIONS --//
    virtual void set_default(void);
    virtual void init_controller(const double);
    virtual void start_learning(const string) = 0;
    virtual void learn(const string) = 0;

    //--Learning Functions--//
    void set_S(void);
    void set_A(const vector<double>);
    void set_Q(void);
    void set_V(void);
    void set_N(void);
    void initialise_Q(const double);
    void initialise_V(const double);

    unsigned int get_state_indx(const vector<double>) const; //--Search and return the index of a state in the state vector _S--//
    bool valid_action(const unsigned int, const unsigned int) const; //--Simulate an action a and check if the action a in the current state s leads to a valide next state s'--//
    vector<double> roundoff_to_closest_state(const vector<double>) const; //--Given current servo positions, find the closest state in state vector _S--//
    unsigned int expected_next_state(const unsigned int, const unsigned int) const; //--Given a state s and action a, calculate the expected next state s'--//
    bool state_visited(const unsigned int) const; //--Check if a given state s was previously visited or not--//
    bool Q_visited(const unsigned int, const unsigned int) const; //--Check if a given state action pair q(a,s) was previously visited or not--//
    void update_N(const unsigned int, const unsigned int);

    /*unsigned int choose_action_epsilon_greedy(const unsigned int, const double) const;
    unsigned int choose_action_epsilon_soft(const unsigned int, const double) const;
    double get_Q_max_a(const unsigned int) const;*/
    bool is_max_a(const unsigned int, const unsigned int) const;
    double N(const unsigned int, const unsigned int) const;
    double act(const unsigned int, const unsigned int, unsigned int&);
    void reset_robot_to_s(const unsigned int);

    void print_vector(const vector<double>) const;

    void init_s_map(void);
    void update_s_map(const unsigned int);
    void plot_s_map(void) const;

    void init_q_map(void);
    void update_q_map(const unsigned int, const unsigned int);
    void plot_q_map(void) const;

    void init_a_map(void);
    void update_a_map(const unsigned int);
    void plot_a_map(void) const;

    void init_q_val(void);
    void init_q_val(char*);
    void plot_q_val(const unsigned int, const unsigned int);
    void save_q_val() const;
    void update_q_graph(void) const;

    void init_v_val(char*);
    void save_v_val() const;
    void update_v_graph(void) const;

    void create_training_dataFile(void) const;
    void create_maxA_dataFile(void) const;

    void init_steps(char*);

protected:
    Robot *robot_primary;
    Robot *robot_secondary;
    //OscillationAnalyzer_OutputSignal *oscAnlz;
    double servo_max;
    double servo_min;
    unsigned int number_of_modules;
    vector<ServoFeedback*> servo_feedback;

    //-- Ext Kalman Filter parameters
    double EKF_dt;
    double EKF_r;
    double EKF_qf;

    vector<vector<double> > _S;
    vector<vector<double> > _A;
    vector<vector<double> > _Q;
    //vector<vector<double> > _V;
    vector<double> _V;
    vector<vector<unsigned int> > _N; //--No. of times Q(S,A) hase been visited--//

    double state_space_min;
    double state_space_max;
    double state_space_resolution;

    double _alpha;
    double _gama;
    double _epsilon;
    double _k;

    unsigned int _steps;
    unsigned int zero_a;

    vector<vector<unsigned int> > s_map;
    vector<vector<unsigned int> > q_map;
    vector<unsigned int> a_map;
    vector<vector<double> > q_val;
};


#endif
