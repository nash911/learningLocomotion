/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   Q   L E A R N I N G   C L A S S   H E A D E R                                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef QLEARNING_H
#define QLEARNING_H

#include "LearningAlgorithm.h"

//using namespace learning;

class QLearning: public LearningAlgorithm
{

public:

    // DEFAULT CONSTRUCTOR
    QLearning(void);

    // CONSTRUCTOR WITH MLP and ROBOT OBJECT
    QLearning(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double);

    // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
    QLearning(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double);

    // DESTRUCTOR
    ~QLearning(void);

    // METHODS
    /*void init_local_variables(Flood::Vector<double>&,
                            Flood::Vector<double>&,
                            Flood::Vector<bool>&);*/

    unsigned int choose_action_greedy(const unsigned int) const;
    unsigned int choose_action_epsilon_greedy(const unsigned int, const double) const;
    unsigned int choose_action_epsilon_soft(const unsigned int, const double) const;
    double get_Q_max_a(const unsigned int) const;


    //-- VIRTUAL FUNCTIONS
    virtual void set_default(void);
    virtual void init_controller(const double);
    virtual void start_learning(const string);
    virtual void learn(const string);
    virtual void start_evaluation();
    virtual void evaluate(const double);
};

#endif
