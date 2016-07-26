/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   Q   L E A R N I N G   C L A S S                                                                            */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

//#include <cmath>
//#include <vector>

#include "QLearning.h"

QLearning::QLearning(void):LearningAlgorithm()
{
}


QLearning::QLearning(Robot* pointer_robot_primary,
                     const vector<double> actions,
                     const double s_min,
                     const double s_max,
                     const double s_res,
                     const double alpha,
                     const double gama,
                     const double epsilon):LearningAlgorithm(pointer_robot_primary,
                                                             actions,
                                                             s_min,
                                                             s_max,
                                                             s_res,
                                                             alpha,
                                                             gama,
                                                             epsilon)
{
    previous_joint_angle.assign(robot_primary->get_number_of_modules(), 0.0);
}


QLearning::QLearning(Robot* pointer_robot_primary,
                     Robot* pointer_robot_secondary,
                     const vector<double> actions,
                     const double s_min,
                     const double s_max,
                     const double s_res,
                     const double alpha,
                     const double gama,
                     const double epsilon):LearningAlgorithm(pointer_robot_primary,
                                                             pointer_robot_secondary,
                                                             actions,
                                                             s_min,
                                                             s_max,
                                                             s_res,
                                                             alpha,
                                                             gama,
                                                             epsilon)
{
    previous_joint_angle.assign(robot_primary->get_number_of_modules(), 0.0);
}


// DESTRUCTOR
QLearning::~QLearning(void)
{
    //LearningAlgorithm::~LearningAlgorithm();
}


unsigned int QLearning::choose_action_greedy(const unsigned int s) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }

    if(_A.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }


    //Action index--//
    vector<unsigned int> greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] > _Q[s][greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }
    cout << endl <<  "   Greedy Action: "; print_vector(_A[greedy_a_indx]); cout << "  with Q Val: " << _Q[s][greedy_a_indx];

    //--Sort greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] == _Q[s][greedy_a_indx])
        {
            greedy.push_back(a);
        }
    }

    //--Choose a greedy action at random uniformly--//
    random_uniform = calculate_random_uniform(0.0,1.0);
    p = 1.0/greedy.size();

    //--Initialise greedy action to be the last greedy action by default--//
    action = greedy[greedy.size()-1];

    for(unsigned int a=0; a<greedy.size(); a++)
    {
        if(random_uniform < (p*(a+1)))
        {
            action = greedy[a];
            break;
        }
    }

    return action;
}


unsigned int QLearning::choose_action_epsilon_greedy(const unsigned int s, const double epsilon) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }

    if(_A.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }


    //Action index--//
    vector<unsigned int> greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] > _Q[s][greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }
    cout << endl <<  "   Greedy Action: "; print_vector(_A[greedy_a_indx]); cout << "  with Q Val: " << _Q[s][greedy_a_indx];

    //--Sort greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] == _Q[s][greedy_a_indx])
        {
            greedy.push_back(a);
        }
    }

    //--IF Ɛ--//
    if(this->calculate_random_uniform(0.0,1.0) < epsilon)
    {
        //--Select an action at random uniformly--//
        random_uniform = calculate_random_uniform(0.0,1.0);
        p = 1.0/_A.size();

        for(unsigned int a=0; a<_A.size(); a++)
        {
            if(random_uniform < (p*(a+1)))
            {
                action = a;
                break;
            }
        }

        return action;
    }
    else
    {
        //--Choose a greedy action at random uniformly--//
        random_uniform = calculate_random_uniform(0.0,1.0);
        p = 1.0/greedy.size();

        //--Initialise greedy action to be the last greedy action by default--//
        action = greedy[greedy.size()-1];

        for(unsigned int a=0; a<greedy.size(); a++)
        {
            if(random_uniform < (p*(a+1)))
            {
                action = greedy[a];
                break;
            }
        }

        return action;
    }
}


unsigned int QLearning::choose_action_epsilon_soft(const unsigned int s, const double epsilon) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_soft(const unsigned int, const double) method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }

    if(_A.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_soft(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //Action index--//
    vector<unsigned int> greedy;
    vector<unsigned int> non_greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] > _Q[s][greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }

    //--Sort greedy and non-greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] == _Q[s][greedy_a_indx])
        {
            greedy.push_back(a);
        }
        else
        {
            non_greedy.push_back(a);
        }
    }

    //--IF non-greedy actions exist--//
    if(non_greedy.size())
    {
        //--AND IF a non-greedy action should be selected--//
        if(calculate_random_uniform(0.0,1.0) < epsilon)
        {
            //--Choose a non-greedy action at random uniformly--//
            random_uniform = calculate_random_uniform(0.0,1.0);
            p = 1.0/non_greedy.size();

            for(unsigned int a=0; a<non_greedy.size(); a++)
            {
                if(random_uniform < (p*(a+1)))
                {
                    action = non_greedy[a];
                    break;
                }
            }

            return action;
        }
    }

    //--IF NOT then choose a greedy action at random uniformly--//
    random_uniform = calculate_random_uniform(0.0,1.0);
    p = 1.0/greedy.size();

    for(unsigned int a=0; a<greedy.size(); a++)
    {
        if(random_uniform < (p*(a+1)))
        {
            action = greedy[a];
            break;
        }
    }

    return action;
}


double QLearning::get_Q_max_a(const unsigned int s) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }

    double max_Q = _Q[s][0];

    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] > max_Q)
        {
            max_Q = _Q[s][a];
        }
    }

    return max_Q;
}


void QLearning::init_controller(const double delta_time)
{
    LearningAlgorithm::init_controller(delta_time);
}


void QLearning::set_default(void)
{
    LearningAlgorithm::set_default();
}


/*void QLearning::init_local_variables(void)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {

  }
}*/


void QLearning::start_evaluation()
{
    //--Reset controller.
    reset_controller();

    robot_primary->set_receive_broadcast(true);

    std::thread ctrl(&QLearning::evaluate, this, EVALUATION_PERIOD);
    std::thread read_broadcast(&QLearning::read_servo_positions_with_time_THREAD, this);

    ctrl.join();
    read_broadcast.join();
}

void QLearning::evaluate(const double evaluation_period)
{

    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "void learn(const double, const double, const double, const string) method." << endl
             << "_Q size cannot be 0: " << _Q.size()
             << endl;

        exit(0);
    }

    double dt = 0;
    unsigned long evaluation_start_time = robot_primary->get_elapsed_evaluation_time();
    unsigned long previous_speed_calc_time = robot_primary->get_elapsed_evaluation_time();
    double totalDistanceTravelledEuc = 0;
    vector<double> rot(3);

    double cumu_r = 0;
    double avg_r = 0;
    bool correct_action = false;
    bool exit_state = false;
    bool evaluation_complete = false;

    //--Reward Graph--//
    fstream R_curve;
    if(_steps == 0)
    {
        remove("R.dat");
    }
    R_curve.open("R.dat",ios::out | ios::app);

    //--Speed Graph--//
    fstream Speed_graph;
    if(_steps == 0)
    {
        remove("Speed.dat");
    }
    Speed_graph.open("Speed.dat",ios::out | ios::app);

#ifdef RECORD_SERVO
    remove("ref.dat");
    remove("servo.dat");
#endif

    //--A--//
    unsigned int a;

    //--R--//
    double r;

    //--Initialise S to start state--//
    vector<double> start_state;
    start_state.assign(number_of_modules, 0.0);

    unsigned int s = get_state_indx(start_state);

    cout << endl << "s: " << s << endl;

    //--S'--//
    unsigned int sp = 0;

    do
    {
        do
        {
            do
            {
                //--Choose greedy action A from S--//
                a = choose_action_greedy(s);

                //--Check if the choosen action A is valid for current state S--//
                correct_action = valid_action(s, a);

                if(!correct_action)
                {
                    //--Penalise Invalid Actions--//
                    _Q[s][a] = INVALID_ACTION_PENALTY;

                    cout << "   Invalid Action!";
                }
            }while(!correct_action);

            //--Take action A, observe R, S'--//
            r = act(s, a, sp);

            //--Check if the robot has entered the exit state--//
            robot_primary->get_robot_rotation(rot);
            if((rot[1] >= 50.0 ||rot[1] <= -50.0) || (rot[2] >= 100.0 ||rot[2] <= -100.0))
            {
                r = r + EXIT_STATE_PENALTY;
                exit_state = true;
            }

            //--Augmenting reward--//
            r = exp(r*3.0)-1;

            //--Record R for graph--//
            if(r > -10.0)
            {
                cumu_r = cumu_r + r;
                avg_r = avg_r + (R_ALPHA * (r - avg_r));
                R_curve << _steps << " " << cumu_r/_steps << " " << avg_r << " " << r <<  endl;
            }

            //--Estimate Q(S,A)--//
            if(exit_state)
            {
                //--Q(S,A) <-- Q(S,A) + 𝛼[R - Q(S,A)]
                _Q[s][a] = _Q[s][a] + _alpha*(r - _Q[s][a]) + IDLE_PENALTY;
            }
            else
            {
                //--Q(S,A) <-- Q(S,A) + 𝛼[R + ɣmax_aQ(S',a) - Q(S,A)]
                _Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]) + IDLE_PENALTY;
            }

            //--Speed of locomotion--//
            dt = (robot_primary->get_elapsed_evaluation_time() - previous_speed_calc_time)/1000000.0;
            if((int)dt % 5 == 0)
            {
                double speed = fabs(robot_primary->get_cumulative_distance_travelled_x()) / dt;
                Speed_graph << _steps << " " << speed <<  endl;

                previous_speed_calc_time = robot_primary->get_elapsed_evaluation_time();
                robot_primary->reset_cumulative_distance_travelled_x();
            }

            //--S <-- S'--//
            if(exit_state)
            {
                s = get_state_indx(start_state);
            }
            else
            {
                s = sp;
            }

            totalDistanceTravelledEuc = robot_primary->calculate_total_distance_travelled_euclidean();

            cout << endl << "Evaluation" << endl << "Step: " << _steps++
                 << "  R: " << r  << "   Euc Dist: " << totalDistanceTravelledEuc
                 << endl;

            if(evaluation_period >= 0)
            {
                dt = (robot_primary->get_elapsed_evaluation_time() - evaluation_start_time)/1000000.0;
                if(dt > evaluation_period)
                {
                    evaluation_complete = true;
                }

            }
            else
            {
                evaluation_complete = false;
            }

        }while(totalDistanceTravelledEuc < BOUNDRY && !exit_state && !evaluation_complete);

        robot_primary->reset_robot_position();
        if(!exit_state)
        {
            reset_robot_to_s(s);
        }
        exit_state = false;
    }while(!evaluation_complete);

    R_curve.close();
    Speed_graph.close();

    robot_primary->set_receive_broadcast(false);
    while(robot_primary->get_broadcast_thread());
    return;

}


void QLearning::start_learning(const string policy_type)
{
    //--Reset controller.
    reset_controller();

    robot_primary->set_receive_broadcast(true);

    std::thread ctrl(&QLearning::learn, this, policy_type);
    std::thread read_broadcast(&QLearning::read_servo_positions_with_time_THREAD, this);

    ctrl.join();
    read_broadcast.join();
}


void QLearning::learn(const string policy_type)
{

    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "void learn(const double, const double, const double, const string) method." << endl
             << "_Q size cannot be 0: " << _Q.size()
             << endl;

        exit(0);
    }

    unsigned int episodes=0;
    double dt = 0;
    unsigned long previous_time = 0;
    unsigned long previous_Q_save_time = 0;
    unsigned long previous_speed_calc_time = robot_primary->get_elapsed_evaluation_time();
    double totalDistanceTravelledEuc = 0;
    vector<double> rot(3);

    unsigned int s_visited = 0;
    unsigned int q_visited = 0;
    unsigned int inv_act_selected = 0;
    double cumu_r = 0;
    double avg_r = 0;
    bool correct_action = false;
    bool exit_state = false;

    init_s_map();
    init_q_map();
    init_a_map();
    init_q_val();

    fstream L_curve;
    remove("Q.dat");
    L_curve.open("Q.dat",ios::out);
    /*if(policy_type.compare("Epsilon_Greedy")==0)
    {
        L_curve.open("Q_Greedy.dat",ios::out);
    }
    else if(policy_type.compare("Epsilon_Soft")==0)
    {
        L_curve.open("Q_Soft.dat",ios::out);
    }*/
    L_curve << _steps << " " << episodes << endl;

    //--Reward Graph--//
    fstream R_curve;
    if(_steps == 0)
    {
        remove("R.dat");
    }
    R_curve.open("R.dat",ios::out | ios::app);

    //--Speed Graph--//
    fstream Speed_graph;
    if(_steps == 0)
    {
        remove("Speed.dat");
    }
    Speed_graph.open("Speed.dat",ios::out | ios::app);

    //--A--//
    unsigned int a;

    //--R--//
    double r;

    //--Initialise S to start state--//
    vector<double> start_state;
    start_state.assign(number_of_modules, 0.0);

    unsigned int s = get_state_indx(start_state);

    cout << endl << "s: " << s << endl;

    //--S'--//
    unsigned int sp = 0;

    do
    {
        do
        {
            do
            {
                //--Choose A from S using policy derived from Q--//
                if(policy_type.compare("Epsilon_Greedy")==0)
                {
                    //--(eg: Ɛ-greedy)--//
                    a = choose_action_epsilon_greedy(s,_epsilon);
                }
                else if(policy_type.compare("Epsilon_Soft")==0)
                {
                    //--(eg: Ɛ-soft)--//
                    a = choose_action_epsilon_soft(s,_epsilon);
                }
                else
                {
                    cerr << "Morphomotion Error: QLearning class." << endl
                         << "void learn(const double, const double, const double, const string) method." << endl
                         << "Unknown policy type: " << policy_type
                         << endl;

                    exit(0);
                }

                //--Check if the choosen action A is valid for current state S--//
                correct_action = valid_action(s, a);

                if(!correct_action)
                {
                    //--Penalise Invalid Actions--//
                    _Q[s][a] = _Q[s][a] + INVALID_ACTION_PENALTY;

                    inv_act_selected++;

                    if(!Q_visited(s,a))
                    {
                        q_visited++;
                        update_q_map(s,a);
                    }
                }
            }while(!correct_action);

            //--Take action A, observe R, S'--//
            r = act(s, a, sp);

            //--Check if the robot has entered the exit state--//
            robot_primary->get_robot_rotation(rot);
            if((rot[1] >= 50.0 ||rot[1] <= -50.0) || (rot[2] >= 100.0 ||rot[2] <= -100.0))
            {
                r = r + EXIT_STATE_PENALTY;
                exit_state = true;
            }

            //--Augmenting reward--//
            r = exp(r*3.0)-1;

            //--Record R for graph--//
            if(r > -10.0)
            {
                cumu_r = cumu_r + r;
                avg_r = avg_r + (R_ALPHA * (r - avg_r));
                R_curve << _steps << " " << cumu_r/_steps << " " << avg_r << " " << r <<  endl;
            }

            //--Record S for graph--//
            if(!state_visited(s))
            {
                s_visited++;
            }
            update_s_map(s);
            plot_s_map();

            //--Record Q(s,a) for graph--//
            if(!Q_visited(s,a))
            {
                q_visited++;
            }

            update_q_map(s,a);
            update_a_map(a);

            dt = (robot_primary->get_elapsed_evaluation_time() - previous_time)/1000000.0;

            //--Update graph files--//
            if(dt > 100.0)
            {
                plot_q_map();
                plot_q_val(s,a);
                plot_a_map();
                update_q_graph();
                update_v_graph();

                create_training_dataFile();

                previous_time = robot_primary->get_elapsed_evaluation_time();
            }

            //--Update Learning Curve graph file--//
            L_curve << _steps << " " << s_visited << " " << q_visited << " " << inv_act_selected << endl;

            //--Estimate Q(S,A)--//
            if(exit_state)
            {
                //--Q(S,A) <-- Q(S,A) + 𝛼[R - Q(S,A)]
                _Q[s][a] = _Q[s][a] + _alpha*(r - _Q[s][a]) + IDLE_PENALTY;
            }
            else
            {
                //--Q(S,A) <-- Q(S,A) + 𝛼[R + ɣmax_aQ(S',a) - Q(S,A)]
                //_Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]);
                _Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]) + IDLE_PENALTY;
            }

            //--Estimate V(S)--//
            if(exit_state)
            {
                //--V(S) <-- V(S) + 𝛼[R - V(S)]
                _V[s] = _V[s] + _alpha*(r - _V[s]);
            }
            else
            {
                //--V(S) <-- V(S) + 𝛼[R + V(S') - V(S)]
                _V[s] = _V[s] + _alpha*(r + _gama*_V[sp] - _V[s]);
            }


            //--Update Q_Val graph files--//
            dt = (robot_primary->get_elapsed_evaluation_time() - previous_Q_save_time)/1000000.0;
            if(dt > 100.0)
            {
                save_q_val();
                save_v_val();

                previous_Q_save_time = robot_primary->get_elapsed_evaluation_time();
            }

            //--Speed of locomotion--//
            dt = (robot_primary->get_elapsed_evaluation_time() - previous_speed_calc_time)/1000000.0;
            if((int)dt % 5 == 0)
            {
                double speed = fabs(robot_primary->get_cumulative_distance_travelled_x()) / dt;
                Speed_graph << _steps << " " << speed <<  endl;

                previous_speed_calc_time = robot_primary->get_elapsed_evaluation_time();
                robot_primary->reset_cumulative_distance_travelled_x();
            }

            //--S <-- S'--//
            if(exit_state)
            {
                s = get_state_indx(start_state);
            }
            else
            {
                s = sp;
            }

            totalDistanceTravelledEuc = robot_primary->calculate_total_distance_travelled_euclidean();

            cout << endl << "    LEARNING"
                 << endl << "Step: " << _steps++ << "  R: " << r
                 << "   Euc Dist: " << totalDistanceTravelledEuc
                 << "   States_visited: " << s_visited << "   Qs_visited: " << q_visited
                 << endl;

            //}while(totalDistanceTravelledEuc < 7.25 && !exit_state);
        }while(totalDistanceTravelledEuc < BOUNDRY && !exit_state && (_steps % EVALUATION_INTERVAL) != 0);

        cout << endl << "Distance Travelled Euclidean: " << totalDistanceTravelledEuc << endl;

        robot_primary->reset_robot_position();

        if(!exit_state)
        {
            reset_robot_to_s(s);
        }

        exit_state = false;

#ifdef EVALUATE
        if((_steps % EVALUATION_INTERVAL) == 0)
        {
            evaluate(EVALUATION_PERIOD);
        }
#endif

    }while(1);

    L_curve.close();
    R_curve.close();
    Speed_graph.close();

    robot_primary->set_receive_broadcast(false);
    while(robot_primary->get_broadcast_thread());
    return;

}


//--Working copy of function learn(void const string)--//
/*void QLearning::learn(const string policy_type)
{

    if(_Q.empty())
    {
        cerr << "Morphomotion Error: QLearning class." << endl
             << "void learn(const double, const double, const double, const string) method." << endl
             << "_Q size cannot be 0: " << _Q.size()
             << endl;

        exit(0);
    }

    //initialise_Q(Q_INIT_VAL);

    //unsigned int steps=0;
    unsigned int episodes=0;
    double dt = 0;
    unsigned long previous_time = 0;
    unsigned long previous_Q_save_time = 0;
    double totalDistanceTravelledEuc = 0;
    vector<double> rot(3);

    unsigned int s_visited = 0;
    unsigned int q_visited = 0;
    unsigned int inv_act_selected = 0;
    double cumu_r = 0;
    double avg_r = 0;
    bool correct_action = false;
    bool exit_state = false;

    init_s_map();
    init_q_map();
    init_a_map();
    init_q_val();

    fstream L_curve;
    remove("Q.dat");
    L_curve.open("Q.dat",ios::out);

    L_curve << _steps << " " << episodes << endl;

    fstream R_curve;
    if(_steps == 0)
    {
        remove("R.dat");
    }
    R_curve.open("R.dat",ios::out | ios::app);

    //--A--//
    unsigned int a;

    //--R--//
    double r;

    //--Initialise S to start state--//
    vector<double> start_state;
    start_state.assign(number_of_modules, 0.0);

    unsigned int s = get_state_indx(start_state);

    cout << endl << "s: " << s << endl;

    //--S'--//
    unsigned int sp = 0;

    do
    {
        do
        {
            do
            {
                //--Choose A from S using policy derived from Q--//
                if(policy_type.compare("Epsilon_Greedy")==0)
                {
                    //--(eg: Ɛ-greedy)--//
                    a = choose_action_epsilon_greedy(s,_epsilon);
                }
                else if(policy_type.compare("Epsilon_Soft")==0)
                {
                    //--(eg: Ɛ-soft)--//
                    a = choose_action_epsilon_soft(s,_epsilon);
                }
                else
                {
                    cerr << "Morphomotion Error: QLearning class." << endl
                         << "void learn(const double, const double, const double, const string) method." << endl
                         << "Unknown policy type: " << policy_type
                         << endl;

                    exit(0);
                }

                correct_action = valid_action(s, a);

                if(!correct_action)
                {
                    //--Penalise Invalid Actions--//
                    _Q[s][a] = _Q[s][a] + INVALID_ACTION_PENALTY;

                    inv_act_selected++;

                    if(!Q_visited(s,a))
                    {
                        q_visited++;
                        update_q_map(s,a);
                    }
                }
            }while(!correct_action);

            //cout << endl << "a: " << a << endl;

            //--Take action A, observe R, S'--//
            r = act(s, a, sp);

            //--Check if the robot has entered the exit state--//
            robot_primary->get_robot_rotation(rot);
            if((rot[1] >= 50.0 ||rot[1] <= -50.0) || (rot[2] >= 100.0 ||rot[2] <= -100.0))
            {
                r = r + EXIT_STATE_PENALTY;
                exit_state = true;
            }

            //--Augmenting reward--//
            r = exp(r*3.0)-1;

            //--Record R for graph--//
            if(r > -10.0)
            {
                cumu_r = cumu_r + r;
                avg_r = avg_r + (R_ALPHA * (r - avg_r));
                //R_curve << _steps << " " << cumu_r/_steps << " " << avg_r <<  endl;
                R_curve << _steps << " " << cumu_r/_steps << " " << avg_r << " " << r <<  endl;
            }

            //cout << endl << "sp: " << sp << endl;

            //--Record S for graph--//
            if(!state_visited(s))
            {
                s_visited++;
            }
            update_s_map(s);
            plot_s_map();

            //--Record Q(s,a) for graph--//
            if(!Q_visited(s,a))
            {
                q_visited++;
            }

            update_q_map(s,a);
            update_a_map(a);

            dt = (robot_primary->get_elapsed_evaluation_time() - previous_time)/1000000.0;

            //--Update graph files--//
            if(dt > 100.0)
            {
                plot_q_map();
                plot_q_val(s,a);
                plot_a_map();
                update_q_graph();
                update_v_graph();

                create_training_dataFile();
                //create_maxA_dataFile();

                previous_time = robot_primary->get_elapsed_evaluation_time();
            }

            //--Update Learning Curve graph file--//
            L_curve << _steps << " " << s_visited << " " << q_visited << " " << inv_act_selected << endl;

            //--Estimate Q(S,A)--//
            if(exit_state)
            {
                //--Q(S,A) <-- Q(S,A) + 𝛼[R - Q(S,A)]
                _Q[s][a] = _Q[s][a] + _alpha*(r - _Q[s][a]) + IDLE_PENALTY;
            }
            else
            {
                //--Q(S,A) <-- Q(S,A) + 𝛼[R + ɣmax_aQ(S',a) - Q(S,A)]
                //_Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]);
                _Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]) + IDLE_PENALTY;
            }

            //--Estimate V(S)--//
            if(exit_state)
            {
                //--V(S) <-- V(S) + 𝛼[R - V(S)]
                _V[s] = _V[s] + _alpha*(r - _V[s]);
            }
            else
            {
                //--V(S) <-- V(S) + 𝛼[R + V(S') - V(S)]
                _V[s] = _V[s] + _alpha*(r + _gama*_V[sp] - _V[s]);
            }


            //--Update Q_Val graph files--//
            dt = (robot_primary->get_elapsed_evaluation_time() - previous_Q_save_time)/1000000.0;
            if(dt > 100.0)
            {
                save_q_val();
                save_v_val();

                previous_Q_save_time = robot_primary->get_elapsed_evaluation_time();
            }

            //--S <-- S'--//
            if(exit_state)
            {
                s = get_state_indx(start_state);
            }
            else
            {
                s = sp;
            }

            totalDistanceTravelledEuc = robot_primary->calculate_total_distance_travelled_euclidean();

            cout << endl << "Step: " << _steps++ << "  R: " << r
                 << "   Euc Dist: " << totalDistanceTravelledEuc
                 << "   States_visited: " << s_visited << "   Qs_visited: " << q_visited
                 << endl;

            //robot_primary->get_vx();

            //robot_primary->get_robot_rotation(rot);

            //char keyboard_key;
            //cout << "Press a key to continue" << endl;
            //cin.get(keyboard_key);

            //}while(_steps < STEP_LIMIT);
        }while(totalDistanceTravelledEuc < 7.25 && !exit_state);

        cout << endl << "Distance Travelled Euclidean: " << totalDistanceTravelledEuc << endl;

        robot_primary->reset_robot_position();
        reset_robot_to_s(s);

        exit_state = false;
    }while(1);

    L_curve.close();
    R_curve.close();

    robot_primary->set_receive_broadcast(false);
    while(robot_primary->get_broadcast_thread());
    return;

}*/


