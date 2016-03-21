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


#ifndef APPROXIMATEQLEARNING_H
#define APPROXIMATEQLEARNING_H

#include "LearningAlgorithm.h"

class ApproximateQLearning: public LearningAlgorithm
{

public:

  // DEFAULT CONSTRUCTOR
  ApproximateQLearning(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  ApproximateQLearning(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double, const unsigned int);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  ApproximateQLearning(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double, const unsigned int);

  // DESTRUCTOR
  ~ApproximateQLearning(void);

  // METHODS
  /*void init_local_variables(Flood::Vector<double>&,
                            Flood::Vector<double>&,
                            Flood::Vector<bool>&);*/

  double normalise_state(const double) const;
  double normalise_action(const double) const;
  vec get_features(const unsigned int, const unsigned int) const;
  double get_Q_prediction(const unsigned int, const unsigned int) const;

  unsigned int choose_action_epsilon_greedy(const unsigned int, const double) const;
  unsigned int choose_action_epsilon_soft(const unsigned int, const double) const;
  unsigned int choose_action_epsilon_greedy_ExpFunc(const unsigned int, const double) const;
  unsigned int choose_action_epsilon_soft_ExpFunc(const unsigned int, const double) const;
  double get_Q_max_a(const unsigned int) const;
  double get_Q_max_a_ExpFunc(const unsigned int) const;
  double N(const unsigned int, const unsigned int) const;


  //void J() const;


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(const double);
  virtual void start_learning(const string);
  virtual void learn(const string);

private:
  vec W;
  //vec features;
};

#endif
