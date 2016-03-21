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

#include "ApproximateQLearning.h"

ApproximateQLearning::ApproximateQLearning(void):LearningAlgorithm()
{
}


ApproximateQLearning::ApproximateQLearning(Robot* pointer_robot_primary,
                                           const vector<double> actions,
                                           const double s_min,
                                           const double s_max,
                                           const double s_res,
                                           const double alpha,
                                           const double gama,
                                           const double epsilon,
                                           const unsigned int W_size):LearningAlgorithm(pointer_robot_primary,
                                                                                        actions,
                                                                                        s_min,
                                                                                        s_max,
                                                                                        s_res,
                                                                                        alpha,
                                                                                        gama,
                                                                                        epsilon)
{

    //W.ones(W_size);
    W.randu(W_size);
    //W = W * 10.0;
    //features.ones(W_size);
}


ApproximateQLearning::ApproximateQLearning(Robot* pointer_robot_primary,
                                           Robot* pointer_robot_secondary,
                                           const vector<double> actions,
                                           const double s_min,
                                           const double s_max,
                                           const double s_res,
                                           const double alpha,
                                           const double gama,
                                           const double epsilon,
                                           const unsigned int W_size):LearningAlgorithm(pointer_robot_primary,
                                                                                        pointer_robot_secondary,
                                                                                        actions,
                                                                                        s_min,
                                                                                        s_max,
                                                                                        s_res,
                                                                                        alpha,
                                                                                        gama,
                                                                                        epsilon)
{
    //W.ones(W_size);
    W.randu(W_size);
    //W = W * 10.0;
    //features.ones(W_size);
}


// DESTRUCTOR
ApproximateQLearning::~ApproximateQLearning(void)
{
    //LearningAlgorithm::~LearningAlgorithm();
}


double ApproximateQLearning::normalise_state(const double feature) const
{
    //--Scaling feature to a value between [-1.0,1.0] or [-0.5,0.5]--//
    return (feature/(servo_max-servo_min));
}

double ApproximateQLearning::normalise_action(const double feature) const
{
    //--Scaling feature to a value between [-1.0,1.0]--//
    return (feature/(state_space_resolution));
}


vec ApproximateQLearning::get_features(const unsigned int s, const unsigned int a) const
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "double get_features(const unsigned int, const unsigned int) method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;

        exit(0);
    }

    unsigned int sp = expected_next_state(s, a);

    double joint_1 = normalise_state(_S[s][0]) * 2.0;
    double joint_2 = normalise_state(_S[s][1]) * 2.0;
    //double delta_joints = normalise(_S[sp][0] - _S[sp][1]);

    double next_joint_1 = normalise_state(_S[sp][0]) * 2.0;
    double next_joint_2 = normalise_state(_S[sp][1]) * 2.0;

    /*double action_1 = normalise_action(_A[a][0]);
    double action_2 = normalise_action(_A[a][1]);*/

    vec features(W.size());
    features(0) = 1.0;

    /*features(1) = joint_1;
    features(2) = joint_2;
    features(3) = delta_joints;

    features(4) = pow(joint_1, 2);
    features(5) = pow(joint_2, 2);
    features(6) = pow(delta_joints, 2);

    features(7) = pow(joint_1, 3);
    features(8) = pow(joint_2, 3);
    features(9) = pow(delta_joints, 3);

    features(10) = pow(joint_1, 4);
    features(11) = pow(joint_2, 4);
    features(12) = pow(delta_joints, 4);

    features(13) = pow(joint_1, 5);
    features(14) = pow(joint_2, 5);
    features(15) = pow(delta_joints, 5);*/


    features(1) = joint_1;
    features(2) = joint_2;
    //features(3) = action_1;
    //features(4) = action_2;
    features(3) = next_joint_1;
    features(4) = next_joint_2;

    /*features(5) = pow(joint_1, 2);
    features(6) = pow(joint_2, 2);
    features(7) = pow(action_1, 2);
    features(8) = pow(action_2, 2);

    features(9) = pow(joint_1, 3);
    features(10) = pow(joint_2, 3);
    features(11) = pow(action_1, 3);
    features(12) = pow(action_2, 3);*/

    return features;
}


double ApproximateQLearning::get_Q_prediction(const unsigned int s, const unsigned int a) const
{
    vec features = get_features(s, a);
    vec predicted_Q = W.t() * features;

    return predicted_Q(0);
}


unsigned int ApproximateQLearning::choose_action_epsilon_greedy(const unsigned int s, const double epsilon) const
{
    if(_A.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //--Approximate Q-Values--//
    vector<double> approx_Q(_A.size());

    //--Action index--//
    vector<unsigned int> greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Calculate the approximate Q(s,a,W) values ∀ a∈A--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        //--Check if action a is valid--//
        if(valid_action(s,a))
        {
            approx_Q[a] = get_Q_prediction(s, a);
        }
        //--If not then associate it with a large negative value--//
        else
        {
            approx_Q[a] = INVALID_ACTION_PENALTY;
        }

        //approx_Q[a] = get_Q_prediction(s, a);
    }


    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] > approx_Q[greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }

    //--Print the greedy action and its index--//
    cout << endl <<  "   Greedy Action[" << greedy_a_indx << "]: ";
    print_vector(_A[greedy_a_indx]);
    cout << "  with Q Val: " << approx_Q[greedy_a_indx];

    //--Sort greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] == approx_Q[greedy_a_indx])
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

        cout << " -- Action Chosen : " << action;
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

        cout << " -- Action Chosen : " << action;
        return action;
        //return (random_uniform * (_A.size())); //--TODO: Need to revert this back--//
    }
}


unsigned int ApproximateQLearning::choose_action_epsilon_soft(const unsigned int s, const double epsilon) const
{
    if(_A.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "unsigned int choose_action_epsilon_soft(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //--Approximate Q-Values--//
    vector<double> approx_Q(_A.size());

    //Action index--//
    vector<unsigned int> greedy;
    vector<unsigned int> non_greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Calculate the approximate Q(s,a) values ∀ a∈A--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        //--Check if action a is valid--//
        if(valid_action(s,a))
        {
            approx_Q[a] = get_Q_prediction(s, a);
        }
        //--If not then associate it with a large negative value--//
        else
        {
            approx_Q[a] = INVALID_ACTION_PENALTY;
        }

        //approx_Q[a] = get_Q_prediction(s, a);
    }

    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] > approx_Q[greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }

    //--Sort greedy and non-greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] == approx_Q[greedy_a_indx])
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


unsigned int ApproximateQLearning::choose_action_epsilon_greedy_ExpFunc(const unsigned int s, const double epsilon) const
{
    if(_A.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "unsigned int choose_action_epsilon_greedy_ExpFunc(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //--Approximate Q-Values--//
    vector<double> approx_Q(_A.size());

    //--Action index--//
    vector<unsigned int> greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Calculate the approximate Q(s,a,W) values ∀ a∈A--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        //--Check if action a is valid--//
        if(valid_action(s,a))
        {
            approx_Q[a] = N(s,a);
        }
        //--If not then associate it with a large negative value--//
        else
        {
            approx_Q[a] = INVALID_ACTION_PENALTY;
        }

        //approx_Q[a] = N(s,a);
    }


    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] > approx_Q[greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }

    //--Print the greedy action and its index--//
    cout << endl <<  "   Greedy Action[" << greedy_a_indx << "]: ";
    print_vector(_A[greedy_a_indx]);
    cout << "  with Q Val: " << approx_Q[greedy_a_indx];

    //--Sort greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] == approx_Q[greedy_a_indx])
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

        cout << " -- Action Chosen : " << action;
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

        cout << " -- Action Chosen : " << action;
        return action;
    }
}


unsigned int ApproximateQLearning::choose_action_epsilon_soft_ExpFunc(const unsigned int s, const double epsilon) const
{
    if(_A.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "unsigned int choose_action_epsilon_soft_ExpFunc(const unsigned int, const double) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //--Approximate Q-Values--//
    vector<double> approx_Q(_A.size());

    //Action index--//
    vector<unsigned int> greedy;
    vector<unsigned int> non_greedy;
    unsigned int greedy_a_indx=0;
    unsigned int action=0;

    double random_uniform;
    double p;

    //--Calculate the approximate Q(s,a) values ∀ a∈A--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        //--Check if action a is valid--//
        if(valid_action(s,a))
        {
            approx_Q[a] = N(s,a);
        }
        //--If not then associate it with a large negative value--//
        else
        {
            approx_Q[a] = INVALID_ACTION_PENALTY;
        }

        //approx_Q[a] = N(s,a);
    }

    //--Find the greedy action index--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] > approx_Q[greedy_a_indx])
        {
            greedy_a_indx = a;
        }
    }

    //--Sort greedy and non-greedy actions--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(approx_Q[a] == approx_Q[greedy_a_indx])
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


double ApproximateQLearning::get_Q_max_a(const unsigned int s) const
{
    if(_A.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "double get_Q_max_a(const unsigned int) const method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //--Approximate Q-Values--//
    vector<double> approx_Q;

    //--Calculate the approximate Q(s',a') values ∀ a'∈A--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        //--Check if action a is valid--//
        if(valid_action(s,a))
        {
            approx_Q.push_back(get_Q_prediction(s, a));
        }

        //approx_Q.push_back(get_Q_prediction(s, a));
    }

    if(approx_Q.empty())
    {
        cerr << "LearningLocomotion Error: ApproximateQLearning class." << endl
             << "double get_Q_max_a(const unsigned int) const method." << endl
             << "approx_Q size cannot be empty: " << approx_Q.size()
             << endl;

        exit(0);
    }

    double max_Q = approx_Q[0];
    for(unsigned int a=0; a<approx_Q.size(); a++)
    {
        if(approx_Q[a] > max_Q)
        {
            max_Q = approx_Q[a];
        }
    }

    return max_Q;
}


double ApproximateQLearning::get_Q_max_a_ExpFunc(const unsigned int s) const
{
    if(_A.empty())
    {
        cerr << "LearningLocomotion Error: ApproximateQLearning class." << endl
             << "double get_Q_max_a_ExpFunc(const unsigned int) const method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    //--Approximate f-Values--//
    vector<double> approx_f;

    //--Calculate the approximate f(Q(s`,a`), N(s',a')) values ∀ a'∈A--//
    for(unsigned int a=0; a<_A.size(); a++)
    {
        //--Check if action a is valid--//
        if(valid_action(s,a))
        {
            approx_f.push_back(N(s, a));
        }

        //approx_f.push_back(N(s, a));
    }

    if(approx_f.empty())
    {
        cerr << "LearningLocomotion Error: ApproximateQLearning class." << endl
             << "double get_Q_max_a_ExpFunc(const unsigned int) const method." << endl
             << "approx_f size cannot be empty: " << approx_f.size()
             << endl;

        exit(0);
    }

    double max_f = approx_f[0];
    for(unsigned int a=0; a<approx_f.size(); a++)
    {
        if(approx_f[a] > max_f)
        {
            max_f = approx_f[a];
        }
    }

    return max_f;
}


double ApproximateQLearning::N(const unsigned int s, const unsigned int a) const
{
    double u = get_Q_prediction(s,a);

    return (u + (_k/(double)_N[s][a]));
}


void ApproximateQLearning::init_controller(const double delta_time)
{
    LearningAlgorithm::init_controller(delta_time);
}


void ApproximateQLearning::set_default(void)
{
    LearningAlgorithm::set_default();
}


/*void ApproximateQLearning::init_local_variables(void)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {

  }
}*/


void ApproximateQLearning::start_learning(const string policy_type)
{
    //--Reset controller.
    reset_controller();

    robot_primary->set_receive_broadcast(true);

    std::thread ctrl(&ApproximateQLearning::learn, this, policy_type);
    std::thread read_broadcast(&ApproximateQLearning::read_servo_positions_with_time_THREAD, this);

    ctrl.join();
    read_broadcast.join();
}


void ApproximateQLearning::learn(const string policy_type)
{

    if(_Q.empty())
    {
        cerr << "Morphomotion Error: ApproximateQLearning class." << endl
             << "void learn(const double, const double, const double, const string) method." << endl
             << "_Q size cannot be 0: " << _Q.size()
             << endl;

        exit(0);
    }

    double predicted_Q = 0;
    double delta = 0;
    vec features;

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
    remove("AppQ.dat");
    L_curve.open("AppQ.dat",ios::out);
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

    //--Reward Graph--//
    fstream delta_graph;
    if(_steps == 0)
    {
        remove("delta.dat");
    }
    delta_graph.open("delta.dat",ios::out | ios::app);

    //--Weights vector Graph--//
    fstream W_graph;
    if(_steps == 0)
    {
        remove("W.dat");
    }
    W_graph.open("W.dat",ios::out | ios::app);

    //--Initialise weight vector graph--//
    W_graph << _steps;
    for(unsigned int w=0; w<W.size(); w++)
    {
        W_graph << " " << W(w);
    }
    W_graph <<  endl;

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
#ifdef EXPLORE_FUNCTION
                //--Choose A from S using policy derived from Q--//
                if(policy_type.compare("Epsilon_Greedy")==0)
                {
                    //--(eg: Ɛ-greedy)--//
                    a = choose_action_epsilon_greedy_ExpFunc(s,_epsilon);
                }
                else if(policy_type.compare("Epsilon_Soft")==0)
                {
                    //--(eg: Ɛ-soft)--//
                    a = choose_action_epsilon_soft_ExpFunc(s,_epsilon);
                }
                else
                {
                    cerr << "Morphomotion Error: ApproximateQLearning class." << endl
                         << "void learn(const double, const double, const double, const string) method." << endl
                         << "Unknown policy type: " << policy_type
                         << endl;

                    exit(0);
                }
#else
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
                    cerr << "Morphomotion Error: ApproximateQLearning class." << endl
                         << "void learn(const double, const double, const double, const string) method." << endl
                         << "Unknown policy type: " << policy_type
                         << endl;

                    exit(0);
                }
#endif


                correct_action = valid_action(s, a);

                if(!correct_action)
                {
                    //--Penalise Invalid Actions--//
                    //_Q[s][a] = _Q[s][a] + INVALID_ACTION_PENALTY;

                    inv_act_selected++;

                    if(!Q_visited(s,a))
                    {
                        _Q[s][a] = 0;
                        q_visited++;
                        update_q_map(s,a);
                    }
                }
            }while(!correct_action);

            //cout << endl << "a: " << a << endl;

            //--Take action A, observe R, S'--//
            r = act(s, a, sp);

            if(r > -5.0)
            {
                //--Augment reward linearly--//
                r = r*100.0;
            }


#ifdef EXPLORE_FUNCTION
            update_N(s,a);
#endif

            //--Check if the robot has entered the exit state--//
            robot_primary->get_robot_rotation(rot);
            if((rot[1] >= 50.0 ||rot[1] <= -50.0) || (rot[2] >= 100.0 ||rot[2] <= -100.0))
            {
                r = r + EXIT_STATE_PENALTY;
                exit_state = true;
            }

            /*cout << "S-A-S'";
            cout << endl << "Q[s,a]: " << _Q[s][a];
            cout << endl << "S: "; print_vector(_S[s]);
            cout << endl << "A: "; print_vector(_A[a]);
            cout << endl << "S': "; print_vector(_S[sp]);*/

            //--Record R for graph--//
            if(r > -10.0)
            {
                cumu_r = cumu_r + r;
                avg_r = avg_r + (R_ALPHA * (r - avg_r));
                R_curve << _steps << " " << cumu_r/_steps << " " << avg_r <<  endl;
            }

            //cout << endl << "sp: " << sp << endl;

            //--Record S for graph--//
            if(!state_visited(s))
            {
                s_visited++;

                update_s_map(s);
                plot_s_map();
            }

            //--Record Q(s,a) for graph--//
            if(!Q_visited(s,a))
            {
                _Q[s][a] = 0;
                q_visited++;
            }

            update_q_map(s,a);
            update_a_map(a);

            dt = (robot_primary->get_elapsed_evaluation_time() - previous_time)/1000000.0;

            //--Update graph files--//
            if(dt > 0.5)
            {
                plot_q_map();
                plot_q_val(s,a);
                plot_a_map();

                previous_time = robot_primary->get_elapsed_evaluation_time();
            }

            //--Update Learning Curve graph file--//
            L_curve << _steps << " " << s_visited << " " << q_visited << " " << inv_act_selected << endl;

            //--Estimate Q(S,A)--//
            //--Q(S,A) <-- Q(S,A) + 𝛼[R + ɣmax_aQ(S',a) - Q(S,A)]
            //_Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]);
            //_Q[s][a] = _Q[s][a] + _alpha*(r + _gama*get_Q_max_a(sp) - _Q[s][a]) + IDLE_PENALTY;

            //--Estimate Approximate Q(S,A,W)--//
            predicted_Q = get_Q_prediction(s, a);

            if(exit_state)
            {
                //--δ = R - Q(S,A,W)
                delta = r - predicted_Q;
            }
            else
            {
                //--δ = R + ɣmax_aQ(S',a,W) - Q(S,A,W)
                delta = r + _gama*get_Q_max_a(sp) - predicted_Q;
            }


            /*#ifdef EXPLORE_FUNCTION
            delta = r + _gama*get_Q_max_a_ExpFunc(sp) - predicted_Q + IDLE_PENALTY;
#else
            delta = r + _gama*get_Q_max_a(sp) - predicted_Q + IDLE_PENALTY;
#endif*/

            //--Feature vector--//
            features = get_features(s, a);

            //--Gradient Desent--//
            //--W <-- W + 𝛼*δ*F
            W = W + (_alpha * delta * features);


            //--Update Q_Val graph files--//
            dt = (robot_primary->get_elapsed_evaluation_time() - previous_Q_save_time)/1000000.0;
            if(dt > 100.0)
            {
                save_q_val();

                previous_Q_save_time = robot_primary->get_elapsed_evaluation_time();
            }

            //--Record delta for graph--//
            delta_graph << _steps << " " << delta <<  endl;

            //--Record weight vector for graph--//
            //W_graph << _steps+1 << " " << W(0) << " " << W(1) << " " << W(2) <<  endl;
            W_graph << _steps;
            for(unsigned int w=0; w<W.size(); w++)
            {
                W_graph << " " << W(w);
            }
            W_graph <<  endl;

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

        /*if(exit_state)
        {
            char keyboard_key;
            cout << "Press a key to continue" << endl;
            cin.get(keyboard_key);
        }*/
        exit_state = false;
    }while(1);

    L_curve.close();
    R_curve.close();
    delta_graph.close();
    W_graph.close();

    robot_primary->set_receive_broadcast(false);
    while(robot_primary->get_broadcast_thread());
    return;
}


