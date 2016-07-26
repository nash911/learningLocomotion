/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   L E A R N I N G   A L G O R I T H M   C L A S S                                                            */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "LearningAlgorithm.h"

//using namespace learning;

LearningAlgorithm::LearningAlgorithm(void)
{
    //-- Set default parameters.
    set_default();
}


LearningAlgorithm::LearningAlgorithm(Robot* pointer_robot_primary,
                                     const vector<double> actions,
                                     const double s_min,
                                     const double s_max,
                                     const double s_res,
                                     const double alpha,
                                     const double gama,
                                     const double epsilon)
{
    //-- Set default parameters.
    set_default();
    
    if(!pointer_robot_primary)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Cannot set Robot Primary to NULL pointer: " << pointer_robot_primary << "." << endl;
        exit(1);
    }
    else
    {
        //-- Robot pointer.
        robot_primary = pointer_robot_primary;
    }
    
    if(s_min > s_max)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "s_min: " << s_min << " cannot be > s_max" << s_max << endl;
        exit(1);
    }
    else
    {
        state_space_min = s_min;
        state_space_max = s_max;
    }
    
    if(s_res <= 0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "s_res: " << s_res << " cannot be <= 0." << endl;
        exit(1);
    }
    else
    {
        state_space_resolution = s_res;
    }
    
    set_S();
    set_A(actions);
    set_Q();
    set_V();
    set_N();
    set_state_1D();

    if(alpha < 0.0 || alpha > 1.0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Alpha: " << alpha << " is out of range 0 <= Alpha <= 1.0" << endl;
        exit(1);
    }
    else
    {
        _alpha = alpha;
    }

    if(gama < 0.0 || gama > 1.0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Gama: " << gama << " is out of range 0 <= Gama <= 1.0" << endl;
        exit(1);
    }
    else
    {
        _gama = gama;
    }

    if(epsilon < 0.0 || epsilon > 1.0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Epsilon: " << epsilon << " is out of range 0 <= Epsilon <= 1.0" << endl;
        exit(1);
    }
    else
    {
        _epsilon = epsilon;
    }
}


LearningAlgorithm::LearningAlgorithm(Robot* pointer_robot_primary,
                                     Robot* pointer_robot_secondary,
                                     const vector<double> actions,
                                     const double s_min,
                                     const double s_max,
                                     const double s_res,
                                     const double alpha,
                                     const double gama,
                                     const double epsilon)
{
    //-- Set default parameters.
    set_default();
    
    if(!pointer_robot_primary)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Cannot set Robot Primary to NULL pointer: " << pointer_robot_primary << "." << endl;
        exit(1);
    }
    else
    {
        //-- Robot pointer.
        robot_primary = pointer_robot_primary;
    }
    
    if(!pointer_robot_secondary)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Cannot set Robot Secondary to NULL pointer: " << pointer_robot_secondary << "." << endl;
        exit(1);
    }
    else
    {
        //-- Robot Secondary pointer.
        robot_secondary= pointer_robot_secondary;
    }
    
    if(s_min > s_max)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "s_min: " << s_min << " cannot be > s_max" << s_max << endl;
        exit(1);
    }
    else
    {
        state_space_min = s_min;
        state_space_max = s_max;
    }
    
    if(s_res <= 0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "s_res: " << s_res << " cannot be <= 0." << endl;
        exit(1);
    }
    else
    {
        state_space_resolution = s_res;
    }
    
    set_S();
    set_A(actions);
    set_Q();
    set_V();
    set_state_1D();

    if(alpha < 0.0 || alpha > 1.0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Alpha: " << alpha << " is out of range 0 <= Alpha <= 1.0" << endl;
        exit(1);
    }
    else
    {
        _alpha = alpha;
    }

    if(gama < 0.0 || gama > 1.0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Gama: " << gama << " is out of range 0 <= Gama <= 1.0" << endl;
        exit(1);
    }
    else
    {
        _gama = gama;
    }

    if(epsilon < 0.0 || epsilon > 1.0)
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void LearningAlgorithm(Robot*, Robot*, const vector<double>, const double, const double, const double, const double, const double, const double)." << endl
             << "Epsilon: " << epsilon << " is out of range 0 <= Epsilon <= 1.0" << endl;
        exit(1);
    }
    else
    {
        _epsilon = epsilon;
    }
}


// DESTRUCTOR
LearningAlgorithm::~LearningAlgorithm(void)
{
    //-- BUG: There is a problem with the below implementation. Getting a "Invalid pointer" error when freeing ServoFeeback object.
    //-- BUG FIX: Changed 'delete[]' to 'delete'
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        delete servo_feedback[module];
        std::cout << std::endl << "Deleted servo_feedback[" << module << "]" << std::endl;
    }
}


void LearningAlgorithm::reset_controller()
{
    for(unsigned int module=0; module<number_of_modules; module++) // Note: this has been moved to void init_local_variables(....)
    {
        servo_feedback[module]->reset_value();
        
        if(servo_feedback[module]->get_ExtKalmanFilter() != NULL)
        {
            servo_feedback[module]->get_ExtKalmanFilter()->reset_parameters();
        }
    }
}


void LearningAlgorithm::init_controller(const double delta_time)
{
    number_of_modules = robot_primary->get_number_of_modules();
    
    //-- Set the size of the current_servo_angle array based on the number of modules in the configuration
    servo_feedback.resize(number_of_modules);
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        servo_feedback[module] = new ServoFeedback(delta_time);
    }
}


void LearningAlgorithm::set_default(void)
{
    robot_primary =  NULL;
    robot_secondary = NULL;
    servo_max = 90.0;
    servo_min = -90.0;
    number_of_modules = 2;
    
    EKF_dt = 0.01;
    EKF_r = 0.1;
    EKF_qf = 0.0001;
    
    state_space_min = 0;
    state_space_max = 0;
    state_space_resolution = 1;
    
    _alpha = 0.1;
    _gama = 1.0;
    _epsilon = 0.1;
    _k = 100;

    zero_a = 0;
    _steps = 0;
}


void LearningAlgorithm::set_S(void)
{
    if(!_S.empty())
    {
        _S.clear();
    }
    
    //unsigned int state_space = fabs(state_space_range[0] - state_space_range[1])/state_space_resolution;
    //unsigned int q_space = pow(state_space, modules);
    //unsigned int a_size = _A.size();
    
    for(double i=state_space_min; i<=state_space_max; )
    {
        for(double j=state_space_min; j<=state_space_max; )
        {
            vector<double> sub_space_S;
            sub_space_S.push_back(i);
            sub_space_S.push_back(j);
            
            _S.push_back(sub_space_S);
            
            j = j + state_space_resolution;
        }
        
        i = i + state_space_resolution;
    }
    
    cout << endl << "_S.size(): " << _S.size() << "    _S[0].size(): " << _S[0].size() << endl;
}


void LearningAlgorithm::set_A(const vector<double> actions)
{
    if(!_A.empty())
    {
        _A.clear();
    }
    
    //unsigned int a_space = pow(actions.size(), modules);
    
    for(unsigned int i=0; i<actions.size(); i++)
    {
        for(unsigned int j=0; j<actions.size(); j++)
        {
            vector<double> a;
            a.push_back(actions[i] * state_space_resolution);
            a.push_back(actions[j] * state_space_resolution);
            _A.push_back(a);
        }
    }
    
    //--Find and initialise index of the action a ∈ {0, 0, 0, 0, ...}
    bool found = false;
    unsigned int indx = 0;
    
    for(unsigned int a=0; a<_A.size(); a++)
    {
        for(unsigned int m=0; m<number_of_modules; m++)
        {
            if(_A[a][m] == 0)
            {
                found = true;
            }
            else
            {
                found = false;
                break;
            }
        }
        
        if(found)
        {
            indx = a;
            break;
        }
    }
    zero_a = indx;
    
    cout << endl << "_A.size(): " << _A.size() << "    _A[0].size(): " << _A[0].size() << endl;
    cout << endl << "zero_a index: " << zero_a << endl;
}


void LearningAlgorithm::set_Q(void)
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void set_Q(const unsigned int, const unsigned int, const vector<double>) method." << endl
             << "_S size cannot be 0: " << _S.size()
             << endl;
        
        exit(0);
    }
    
    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void set_Q(const unsigned int, const unsigned int, const vector<double>) method." << endl
             << "_A size cannot be 0: " << _A.size()
             << endl;
        
        exit(0);
    }
    
    /*unsigned int state_space = fabs(state_space_range[0] - state_space_range[1])/state_space_resolution;
    unsigned int q_space = pow(state_space, modules);
    
    double range_min = state_space_range[0];
    double range_max = state_space_range[1];*/
    
    unsigned int state_space = _S.size();
    unsigned int action_space = _A.size();
    
    for(unsigned int i=0; i<state_space; i++)
    {
        vector<double> sub_space_Q;
        sub_space_Q.assign(action_space, Q_INIT_VAL);
        
        _Q.push_back(sub_space_Q);
    }
    
    cout << endl << "_Q.size(): " << _Q.size() << "    _Q[0].size(): " << _Q[0].size() << endl;
}


void LearningAlgorithm::set_V(void)
{
    if(!_V.empty())
    {
        _V.clear();
    }

    /*unsigned int rows;
    unsigned int cols;

    rows = cols = (abs(state_space_max - state_space_min) / state_space_resolution) + 1;

    for(double i=0; i<rows; i++)
    {
        vector<double> sub_space_V(cols, 0.0);
        _V.push_back(sub_space_V);
    }

    cout << endl << "_V.size(): " << _V.size() << "    _V[0].size(): " << _V[0].size() << endl;*/

    _V.assign(_S.size(), 0.0);

    cout << endl << "_V.size(): " << _V.size() << endl;
}


void LearningAlgorithm::set_N(void)
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void set_N(const unsigned int, const unsigned int, const vector<double>) method." << endl
             << "_Q size cannot be 0: " << _Q.size()
             << endl;

        exit(0);
    }

    if(_Q[0].empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void set_N(const unsigned int, const unsigned int, const vector<double>) method." << endl
             << "_Q[0] size cannot be 0: " << _Q[0].size()
             << endl;

        exit(0);
    }


    unsigned int rows = _Q.size();
    unsigned int cols = _Q[0].size();

    for(unsigned int i=0; i<rows; i++)
    {
        vector<unsigned int> sub_space_N;
        sub_space_N.assign(cols, 1);

        _N.push_back(sub_space_N);

    }

    cout << endl << "_N.size(): " << _N.size() << "    _N[0].size(): " << _N[0].size() << endl;
}


void LearningAlgorithm::set_state_1D(void)
{
    for(double s=state_space_min; s<=state_space_max; )
    {
        state_1D.push_back(s);
        s = s + state_space_resolution;
    }

    cout << endl << "1D_state_space.size(): " << state_1D.size() << endl;

    cout << "1D State Space" << endl;
    for(unsigned int s=0; s<state_1D.size(); s++)
    {
        cout << state_1D[s] << endl;
    }
}


void LearningAlgorithm::initialise_Q(const double init_val)
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void initialise_Q(const double) method." << endl
             << "_Q size cannot be 0: " << _Q.size()
             << endl;
        
        exit(0);
    }
    
    unsigned int A_space = _A.size();
    unsigned int Q_space = _Q.size();
    
    for(unsigned int s=0; s<Q_space; s++)
    {
        for(unsigned int a=0; a<A_space; a++)
        {
            _Q[s][a] = init_val;
        }
    }
}


void LearningAlgorithm::initialise_V(const double init_val)
{
    if(_V.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void initialise_V(const double) method." << endl
             << "_V size cannot be 0: " << _V.size()
             << endl;

        exit(0);
    }

    unsigned int S = _S.size();
    //unsigned int cols = _V[0].size();

    for(unsigned int s=0; s<S; s++)
    {
        _V[s] = init_val;

    }
}


unsigned int LearningAlgorithm::get_action_size(void) const
{
    return _A.size();
}


void LearningAlgorithm::set_epsilon(const double epsilon)
{
    if(epsilon >=0 && epsilon <= 1.0)
    {
        _epsilon = epsilon;
    }
    else
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void set_epsilon(const double method" << std::endl
             << "espilon: " << epsilon << " out of range. Should be in the range [0,1]." << endl;
        exit(1);
    }

}


//--Search and return the index of a state in the state vector _S--//
unsigned int LearningAlgorithm::get_state_indx(const vector<double> key) const
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "unsigned int get_state_indx(const vector<double>) const method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        
        exit(0);
    }
    
    bool found = false;
    unsigned int indx = 0;
    
    for(unsigned int s=0; s<_S.size(); s++)
    {
        for(unsigned int m=0; m<number_of_modules; m++)
        {
            if(_S[s][m] == key[m])
            {
                found = true;
            }
            else
            {
                found = false;
                break;
            }
        }
        
        if(found)
        {
            indx = s;
            break;
        }
    }
    
    return indx;
}


//--Simulate an action a and check if the action a in the current state s leads to a valide next state s'--//
//--Returns TRUE if action a is valid, and FALSE if not--//
bool LearningAlgorithm::valid_action(const unsigned int s, const unsigned int a) const
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "bool valid_action(const unsigned int, const unsigned int) const method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        
        exit(0);
    }
    
    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "bool valid_action(const unsigned int, const unsigned int) const method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;
        
        exit(0);
    }
    
    vector<double> sp = _S[s];
    
    bool found = false;
    unsigned int indx = 0;
    
    for(unsigned int m=0; m<number_of_modules; m++)
    {
        sp[m] = sp[m] + _A[a][m];
    }
    
    for(unsigned int s=0; s<_S.size(); s++)
    {
        for(unsigned int m=0; m<number_of_modules; m++)
        {
            if(_S[s][m] == sp[m])
            {
                found = true;
            }
            else
            {
                found = false;
                break;
            }
        }
        
        if(found)
        {
            indx = s;
            break;
        }
    }

    //--TODO: Need to be removed--//
    //--Make Zero_Action invalid--//
    /*if(a == zero_a)
    {
        return false;
    }*/
    //--TODO: Need to be removed--//
    
    if(found)
    {
        return true;
    }
    else
    {
        //char keyboard_key;
        //cout << std::endl << "Invalid Action" << endl;
        /*cout << endl << "Q[s,a]: " << _Q[s][a];
        cout << endl << "S: "; print_vector(_S[s]);
        cout << endl << "A: "; print_vector(_A[a]);
        cout << endl << "S': "; print_vector(sp);
        cout << endl;*/
        //cout << "Press a key to continue" << endl;
        //cin.get(keyboard_key);
        return false;
    }
}


//--Given current servo positions, find the closest state in state vector _S--//
vector<double> LearningAlgorithm::roundoff_to_closest_state(const vector<double> servo_pos) const
{
    vector<double> sp;

    double delta;
    unsigned int indx = 0;
    
    for(unsigned int m=0; m<servo_pos.size(); m++)
    {
        delta = fabs(state_space_min - state_space_max);
        indx = 0;
        for(unsigned int i=0; i<state_1D.size(); i++)
        {
            if(fabs(servo_pos[m] - state_1D[i]) < delta)
            {
                delta = fabs(servo_pos[m] - state_1D[i]);
                indx = i;
            }
        }
        sp.push_back(state_1D[indx]);
    }
    
    return sp;
}


//--Given a state s and action a, calculate the expected next state s'--//
//--Returns the index of the expected next state s', in state-space S--//
unsigned int LearningAlgorithm::expected_next_state(const unsigned int s, const unsigned int a) const
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "unsigned int expected_next_state(const unsigned int, const unsigned int) const method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;

        exit(0);
    }

    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "unsigned int expected_next_state(const unsigned int, const unsigned int) const method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;

        exit(0);
    }

    vector<double> sp = _S[s];

    bool found = false;
    unsigned int sp_indx = 0;

    for(unsigned int m=0; m<number_of_modules; m++)
    {
        sp[m] = sp[m] + _A[a][m];
    }

    for(unsigned int s=0; s<_S.size(); s++)
    {
        for(unsigned int m=0; m<number_of_modules; m++)
        {
            if(_S[s][m] == sp[m])
            {
                found = true;
            }
            else
            {
                found = false;
                break;
            }
        }

        if(found)
        {
            sp_indx = s;
            break;
        }
    }

    return sp_indx;
}


//--Check if a given state s was previously visited or not--//
bool LearningAlgorithm::state_visited(const unsigned int s) const
{
    bool visited = false;
    
    for(unsigned int a=0; a<_A.size(); a++)
    {
        if(_Q[s][a] != Q_INIT_VAL)
        {
            visited = true;
            break;
        }
    }
    
    return visited;
}


//--Check if a given state action pair q(a,s) was previously visited or not--//
bool LearningAlgorithm::Q_visited(const unsigned int s, const unsigned int a) const
{
    if(_Q[s][a] == Q_INIT_VAL)
    {
        return false;
    }
    else
    {
        return true;
    }
}


void LearningAlgorithm::update_N(const unsigned int s, const unsigned int a)
{
    if(s >= _N.size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_N(const unsigned int, const unsigned int) method." << endl
             << "s: " << s << " cannot be >= size of _N: " << _N.size()
             << endl;

        exit(0);
    }

    if(a >= _N[0].size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_N(const unsigned int, const unsigned int) method." << endl
             << "a: " << a << " cannot be >= size of _N[0]: " << _N[0].size()
             << endl;

        exit(0);
    }

    _N[s][a]++;
}


/*unsigned int LearningAlgorithm::choose_action_epsilon_greedy(const unsigned int s, const double epsilon) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "unsigned int choose_action_epsilon_greedy(const unsigned int, const double) method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }
    
    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
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
        //return (random_uniform * (_A.size())); //--TODO: Need to revert this back--//
    }
}


unsigned int LearningAlgorithm::choose_action_epsilon_soft(const unsigned int s, const double epsilon) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "unsigned int choose_action_epsilon_soft(const unsigned int, const double) method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }
    
    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
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


double LearningAlgorithm::get_Q_max_a(const unsigned int s) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
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
}*/


bool LearningAlgorithm::is_max_a(const unsigned int s, const unsigned int a) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "bool is_max_a(const unsigned int, const unsigned int) const method." << endl
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

    if(max_Q == _Q[s][a])
    {
        return true;
    }
    else
    {
        return false;
    }
}


double LearningAlgorithm::act(const unsigned int s, const unsigned int a, unsigned int& sp)
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "double act(const unsigned int, const unsigned int, unsigned int&) method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        
        exit(0);
    }
    
    if(a >= _A.size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "double act(const unsigned int, const unsigned int, unsigned int&) method." << endl
             << "a: " << a << " must be <= Action space size: " << _A.size()
             << endl;
        
        exit(0);
    }
    
    vector<double> servo_positions;
    servo_positions.assign(number_of_modules, 0.0);
    
    //--Reward--//
    double r = 0;

    unsigned int expected_sp = 0;
    
    unsigned long current_time = 0;
    double dt = 0;
    unsigned int cycles = 0;

#ifdef RECORD_SERVO
    double t = 0;

    //--Reference Graph--//
    fstream ref_graph;
    ref_graph.open("ref.dat",ios::out | ios::app);

    //--Servo Graph--//
    fstream servo_graph;
    servo_graph.open("servo.dat",ios::out | ios::app);
#endif

    //--Save current joint angles--//
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        previous_joint_angle[module] = servo_feedback[module]->get_servo_position();
    }

    //--Take action A--//
    vector<double> output;
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        output.push_back(_S[s][module] + _A[a][module]);
    }
    actuate_all_modules(output);

    expected_sp = expected_next_state(s,a);
    
    do
    {
        //--TODO: Simulate one step--//
        robot_primary->step("learn");

        current_time = robot_primary->get_elapsed_evaluation_time();
        
        //--Observe S'--//
        for(unsigned int module=0; module<number_of_modules; module++)
        {
            servo_positions[module] = servo_feedback[module]->get_servo_position();
        }
        sp = get_state_indx(roundoff_to_closest_state(servo_positions));

#ifdef RECORD_SERVO
        t = current_time / 1000000.0;

        ref_graph << t;
        servo_graph << t;

        for(unsigned int module=0; module<number_of_modules; module++)
        {
            ref_graph << " " << output[module];
            servo_graph << " " << servo_positions[module];
        }

        for(unsigned int module=0; module<number_of_modules; module++)
        {
            ref_graph << " " << _S[s][module];
        }

        for(unsigned int module=0; module<number_of_modules; module++)
        {
            ref_graph << " " << _A[a][module];
        }

        for(unsigned int module=0; module<number_of_modules; module++)
        {
            ref_graph << " " << _S[expected_sp][module];
        }

        for(unsigned int module=0; module<number_of_modules; module++)
        {
            ref_graph << " " << _S[sp][module];
        }

        ref_graph << endl;
        servo_graph << endl;
#endif

        dt = (current_time - robot_primary->get_previous_velocity_time()) / 1000000.0;
        cycles++;
        
        //}while(sp == s && a != zero_a && dt < CUTOFF_TIME);
    }while(sp != expected_sp && dt < CUTOFF_TIME);
    
    //--Observe R--//
    //r = robot_primary->calculate_velocity_Y();
    //r = robot_primary->calculate_distance_travelled_euclidean();
    //r = robot_primary->calculate_forward_velocity();
    r = robot_primary->get_vx();
    
    if(dt >= CUTOFF_TIME)
    {
        r = r + WRONG_STATE_PENALTY;
    }

    if(sp == s)
    {
        r = r + USELESS_ACTION_PENALTY;

        cout << endl << "Elapsed Time: " << robot_primary->get_elapsed_evaluation_time()
             << "   Previous Time: " << robot_primary->get_previous_velocity_time()
             << "   Delta Time: " << dt
             << "    Cycles: " << cycles << endl;
    }
    
    cout << endl << "   Delta Time: " << dt
         << "    Cycles: " << cycles;

#ifdef RECORD_SERVO
    ref_graph.close();
    servo_graph.close();
#endif
    
    return(r);
}


void LearningAlgorithm::reset_robot_to_s(const unsigned int s)
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void reset_robot_to_s(const unsigned int) method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;

        exit(0);
    }

    vector<double> servo_positions;
    servo_positions.assign(number_of_modules, 0.0);

    unsigned int expected_sp = 0;
    unsigned int sp = 1;

    double dt = 0;

    //--Reset Robot to State s--//
    vector<double> output;
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        output.push_back(_S[s][module]);
    }
    actuate_all_modules(output);

    expected_sp = expected_next_state(s,zero_a);

    do
    {
        //--TODO: Simulate one step--//
        robot_primary->step("learn");

        //--Observe S'--//
        for(unsigned int module=0; module<number_of_modules; module++)
        {
            servo_positions[module] = servo_feedback[module]->get_servo_position();
        }
        sp = get_state_indx(roundoff_to_closest_state(servo_positions));

        dt = (robot_primary->get_elapsed_evaluation_time() - robot_primary->get_previous_velocity_time()) / 1000000.0;

        //}while(sp == s && a != zero_a && dt < CUTOFF_TIME);
    }while(sp != expected_sp && dt < CUTOFF_TIME*2.0);


    if(dt >= CUTOFF_TIME*2.0)
    {
        //char keyboard_key;
        cout << endl << "Elapsed Time: " << robot_primary->get_elapsed_evaluation_time()
             << "   Previous Time: " << robot_primary->get_previous_velocity_time()
             << "   Delta Time: " << dt << endl;
        //cout << "Press a key to continue" << endl;
    }
}


void LearningAlgorithm::print_vector(const vector<double> v) const
{
    for(unsigned int i=0; i<v.size(); i++)
    {
        cout << "[" << i << "]: " << v[i] << "  ";
    }
}


void LearningAlgorithm::init_s_map(void)
{
    if(s_map.size())
    {
        for(unsigned int i=0; i<s_map.size(); i++)
        {
            s_map[i].clear();
        }
        
        s_map.clear();
    }
    
    unsigned int s_size = 1 + fabs(state_space_min - state_space_max)/state_space_resolution;
    
    for(unsigned int i=0; i<s_size; i++)
    {
        vector<unsigned int> sub_s_map;
        sub_s_map.assign(s_size, 0);
        
        s_map.push_back(sub_s_map);
    }
    cout << endl << "S_Map Size: " << s_map.size() << " x " << s_map[0].size() << endl;
}


void LearningAlgorithm::update_s_map(const unsigned int s)
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_s_map(void) method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        
        exit(0);
    }
    
    unsigned int row = (_S[s][0] + state_space_max)/state_space_resolution;
    unsigned int col = (_S[s][1] + state_space_max)/state_space_resolution;
    
    if(row >= s_map.size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_s_map(void) method." << endl
             << "row: " << row << " is out of bound of s_map row size: " << s_map.size()
             << endl;
        
        exit(0);
    }
    
    if(col >= s_map[0].size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_s_map(void) method." << endl
             << "col: " << col << " is out of bound of s_map col size: " << s_map[0].size()
             << endl;
        
        exit(0);
    }
    
    s_map[row][col]++;
}


void LearningAlgorithm::plot_s_map(void) const
{
    fstream State_map;
    State_map.open("S_Map.dat",ios::out | ios::trunc);
    
    unsigned int s_map_size = s_map.size();
    
    for(unsigned int i=0; i<s_map_size; i++)
    {
        for(unsigned int j=0; j<s_map_size; j++)
        {
            State_map << s_map[i][j] << " ";
        }
        State_map << endl;
    }
    
    State_map.close();
}


void LearningAlgorithm::init_q_map(void)
{
    
    if(q_map.size())
    {
        for(unsigned int i=0; i<q_map.size(); i++)
        {
            q_map[i].clear();
        }
        
        q_map.clear();
    }
    
    unsigned int action_size = sqrt(_A.size());
    unsigned int q_size = (1 + fabs(state_space_min - state_space_max)/state_space_resolution) * action_size;
    
    for(unsigned int i=0; i<q_size; i++)
    {
        vector<unsigned int> sub_q_map;
        sub_q_map.assign(q_size, 0);
        
        q_map.push_back(sub_q_map);
    }
    cout << endl << "Q_Map Size: " << q_map.size() << " x " << q_map[0].size() << endl;
}


void LearningAlgorithm::update_q_map(const unsigned int s, const unsigned int a)
{
    if(_S.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_q_map(void) method." << endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        
        exit(0);
    }
    
    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_q_map(void) method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;
        
        exit(0);
    }
    
    unsigned int action_size = sqrt(_A.size());
    
    unsigned int row = (((_S[s][0] + state_space_max)/state_space_resolution) * action_size) + (a/action_size);
    unsigned int col = (((_S[s][1] + state_space_max)/state_space_resolution) * action_size) + (a/action_size);
    
    if(row >= q_map.size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_q_map(void) method." << endl
             << "row: " << row << " is out of bound of q_map row size: " << q_map.size()
             << endl;
        
        exit(0);
    }
    
    if(col >= q_map[0].size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_q_map(void) method." << endl
             << "col: " << col << " is out of bound of q_map col size: " << q_map[0].size()
             << endl;
        
        exit(0);
    }
    
    q_map[row][col]++;
}


void LearningAlgorithm::plot_q_map(void) const
{
    fstream Q_map;
    Q_map.open("Q_Map.dat",ios::out | ios::trunc);
    
    unsigned int q_map_size = q_map.size();
    
    for(unsigned int i=0; i<q_map_size; i++)
    {
        for(unsigned int j=0; j<q_map_size; j++)
        {
            Q_map << q_map[i][j] << " ";
        }
        Q_map << endl;
    }
    
    Q_map.close();
}


void LearningAlgorithm::init_a_map(void)
{
    
    if(a_map.size())
    {
        a_map.clear();
    }
    
    a_map.assign(_A.size(), 0);
    
    cout << endl << "A_Map Size: " << a_map.size() << endl;
}


void LearningAlgorithm::update_a_map(const unsigned int a)
{
    if(a >= a_map.size())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void update_a_map(const unsigned int) method." << endl
             << "a: " << a << " is out of bound of a_map size: " << a_map.size()
             << endl;
        
        exit(0);
    }
    
    a_map[a]++;
}


void LearningAlgorithm::plot_a_map(void) const
{
    if(_A.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void plot_a_map(void) const method." << endl
             << "_A size cannot be empty: " << _A.size()
             << endl;
        
        exit(0);
    }
    
    fstream A_map;
    A_map.open("A_Map.dat",ios::out | ios::trunc);
    
    unsigned int a_map_size = a_map.size();
    
    //A_map << "0" << 1 << endl;
    
    for(unsigned int i=0; i<a_map_size; i++)
    {
        A_map << i << " [" << _A[i][0] << "," << _A[i][1] << "] " << a_map[i] << endl;
    }
    //A_map << "10" << 1 << endl;
    
    A_map.close();
}


void LearningAlgorithm::init_q_val(void)
{
    
    if(q_val.size())
    {
        for(unsigned int i=0; i<q_val.size(); i++)
        {
            q_val[i].clear();
        }
        
        q_val.clear();
    }
    
    unsigned int action_size = sqrt(_A.size());
    unsigned int q_size = (1 + fabs(state_space_min - state_space_max)/state_space_resolution) * action_size;
    
    for(unsigned int i=0; i<q_size; i++)
    {
        vector<double> sub_q_val;
        sub_q_val.assign(q_size, 0.0);
        
        q_val.push_back(sub_q_val);
    }
    cout << endl << "Q_Val Size: " << q_val.size() << " x " << q_val[0].size() << endl;
}


void LearningAlgorithm::plot_q_val(const unsigned int S, const unsigned int A)
{
    fstream Q_val;
    Q_val.open("Q_Val.dat",ios::out | ios::trunc);
    
    unsigned int s_size = _S.size(); // 441
    unsigned int a_size = _A.size(); // 9
    
    unsigned int state_size = sqrt(_S.size()); // 21
    unsigned int action_size = sqrt(_A.size()); // 3
    unsigned int q_val_size = q_val.size(); // 63
    
    unsigned int row = 0;
    unsigned int col = 0;
    
    for(unsigned int s=0; s<s_size; s++)
    {
        for(unsigned int a=0; a<a_size; a++)
        {
            row = ((s / state_size) * action_size) + (a / action_size);
            col = ((s % state_size) * action_size) + (a % action_size);
            
            q_val[row][col]=_Q[s][a];
        }
    }
    
    //--To show the curent state of the robot on the plot--//
    for(unsigned int a=0; a<a_size; a++)
    {
        row = ((S / state_size) * action_size) + (a / action_size);
        col = ((S % state_size) * action_size) + (a % action_size);
        
        if(a == A)
        {
            q_val[row][col] = -10.0;
        }
        else
        {
            q_val[row][col] = Q_INIT_VAL * 2.0;
        }
    }
    
    for(unsigned int row=0; row<q_val_size; row++)
    {
        for(unsigned int col=0; col<q_val_size; col++)
        {
            Q_val << q_val[row][col] << " ";
        }
        Q_val << endl;
    }
    
    Q_val.close();
}


void LearningAlgorithm::save_q_val(void) const
{
    if(_Q.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void save_q_val(void) const method." << endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;

        exit(0);
    }

    fstream Q_val;
    Q_val.open("Q_Pi.dat",ios::out | ios::trunc);

    for(unsigned int s=0; s<_S.size(); s++)
    {
        for(unsigned int a=0; a<_A.size(); a++)
        {
            Q_val << _Q[s][a] << " ";
        }
        Q_val << endl;
    }

    Q_val.close();
}


void LearningAlgorithm::update_q_graph(void) const
{
    if(_Q.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void update_q_graph(void) method" << std::endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;
        exit(0);
    }

    if(_A.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void update_q_graph(void) method" << std::endl
             << "_A size cannot be empty: " << _A.size()
             << endl;
        exit(0);
    }

    unsigned int q_size = _Q.size();
    unsigned int a_size = _A.size();

    //--Generating 2D graph file--//
    fstream Q_graph_2d;
    Q_graph_2d.open("Q_Graph_2D.dat",ios::out | ios::trunc);

    unsigned int n = 0;

    for(unsigned int s = 0; s < q_size; s++)
    {
        for(unsigned int a = 0; a < a_size; a++)
        {
            if(valid_action(s, a))
            {
                Q_graph_2d << n << " " << _Q[s][a] << endl;
                n++;
            }
        }
    }

    //--Generating 3D graph file--//
    fstream Q_graph_3d;
    Q_graph_3d.open("Q_Graph_3D.dat",ios::out | ios::trunc);

    vector<double> q_vals;

    //--Calculate the mean of Q(s,a) ∀ s ∈ S, a ∈ A
    for(unsigned int s = 0; s < q_size; s++)
    {
        for(unsigned int a = 0; a < a_size; a++)
        {
            if(valid_action(s, a))
            {
                q_vals.push_back(_Q[s][a]);
            }
        }
    }
    double q_vals_sum = std::accumulate(q_vals.begin(), q_vals.end(), 0.0);
    double q_vals_mean = q_vals_sum / q_vals.size();

    for(unsigned int s = 0; s < q_size; s++)
    {
        for(unsigned int a = 0; a < a_size; a++)
        {
            if(valid_action(s, a))
            {
                Q_graph_3d << s << " " << a << " " << _Q[s][a] << endl;
            }
            else
            {
                Q_graph_3d << s << " " << a << " " << q_vals_mean << endl;
            }
        }
    }

    Q_graph_2d.close();
    Q_graph_3d.close();
}


void LearningAlgorithm::update_v_graph(void) const
{
    if(_S.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void update_v_graph(void) method" << std::endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        exit(0);
    }

    if(_A.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void update_v_graph(void) method" << std::endl
             << "_A size cannot be empty: " << _A.size()
             << endl;
        exit(0);
    }

    if(_Q.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void update_v_graph(void) method" << std::endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;
        exit(0);
    }

    if(_V.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void update_v_graph(void) method" << std::endl
             << "_V size cannot be empty: " << _V.size()
             << endl;
        exit(0);
    }

    fstream V_graph;
    V_graph.open("V_Graph.dat",ios::out | ios::trunc);

    unsigned int S = _S.size();

    for(unsigned int s=0; s<S; s++)
    {
        V_graph << _S[s][0] << " " << _S[s][1] << " " << _V[s] << endl;
    }

    V_graph.close();
}


void LearningAlgorithm::init_q_val(char* q_filename)
{
    if(_Q.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_q_val(char*) method" << std::endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;
        exit(0);
    }

    std::fstream q_file;
    std::string line;

    vector<vector<double> > _q;

    q_file.open(q_filename, std::ios::in);
    if(!q_file.is_open())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_q_val(char*) method" << std::endl
             << "Cannot open Q_val file: "<< q_filename  << std::endl;
        exit(1);
    }


    while(getline(q_file, line))
    {
        istringstream ss(line);
        double num;
        vector<double> q_sub;

        while(ss >> num)
        {
            q_sub.push_back(num);
        }
        _q.push_back(q_sub);
    }

    cout << endl << "Q from file Rows: " << _q.size() << "   Col: " << _q[0].size() << endl;
    q_file.close();

    if(_Q.size() != _q.size())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_q_val(char*) method" << std::endl
             << "_Q rows: "<< _Q.size() << " != _q rows: " << _q.size() << " on file." << endl;
        exit(1);
    }

    if(_Q[0].size() != _q[0].size())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_q_val(char*) method" << std::endl
             << "_Q cols: "<< _Q[0].size() << " != _q cols: " << _q[0].size() << " on file." << endl;
        exit(1);
    }

    for(unsigned int s=0; s<_Q.size(); s++)
    {
        for(unsigned int a=0; a<_Q[0].size(); a++)
        {
            _Q[s][a] = _q[s][a];
        }
    }
}


/*void LearningAlgorithm::init_v_val(char* v_filename)
{
    if(_V.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "_V size cannot be empty: " << _V.size()
             << endl;
        exit(0);
    }

    std::fstream v_file;
    std::string line;

    vector<vector<double> > _v;

    v_file.open(v_filename, std::ios::in);
    if(!v_file.is_open())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "Cannot open V_val file: "<< v_filename  << std::endl;
        exit(1);
    }


    while(getline(v_file, line))
    {
        istringstream ss(line);
        double num;
        vector<double> v_sub;

        while(ss >> num)
        {
            v_sub.push_back(num);
        }
        _v.push_back(v_sub);
    }

    cout << endl << "V from file Rows: " << _v.size() << "   Col: " << _v[0].size() << endl;
    v_file.close();

    if(_V.size() != _v.size())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "_V rows: "<< _V.size() << " != _v rows: " << _v.size() << " on file." << endl;
        exit(1);
    }

    if(_V[0].size() != _v[0].size())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "_V cols: "<< _V[0].size() << " != _v cols: " << _v[0].size() << " on file." << endl;
        exit(1);
    }

    unsigned int rows = _V.size();
    unsigned int cols = _V[0].size();

    for(unsigned int i=0; i<rows; i++)
    {
        for(unsigned int j=0; j<cols; j++)
        {
            _V[i][j] = _v[i][j];
        }
    }
}*/


void LearningAlgorithm::init_v_val(char* v_filename)
{
    if(_V.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "_V size cannot be empty: " << _V.size()
             << endl;
        exit(0);
    }

    std::fstream v_file;
    std::string line;

    vector<double> _v;

    v_file.open(v_filename, std::ios::in);
    if(!v_file.is_open())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "Cannot open V_val file: "<< v_filename  << std::endl;
        exit(1);
    }


    while(getline(v_file, line))
    {
        istringstream ss(line);
        double num;

        while(ss >> num)
        {
            _v.push_back(num);
        }
    }

    cout << endl << "V from file Rows: " << _v.size() << endl;
    v_file.close();

    if(_V.size() != _v.size())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_v_val(char*) method" << std::endl
             << "_V rows: "<< _V.size() << " != _v rows: " << _v.size() << " on file." << endl;
        exit(1);
    }

    unsigned int S = _S.size();

    for(unsigned int s=0; s<S; s++)
    {
        _V[s] = _v[s];
    }
}


void LearningAlgorithm::save_v_val(void) const
{
    if(_V.empty())
    {
        cerr << "Morphomotion Error: LearningAlgorithm class." << endl
             << "void save_v_val(void) const method." << endl
             << "_V size cannot be empty: " << _V.size()
             << endl;

        exit(0);
    }

    fstream V_val;
    V_val.open("V.dat",ios::out | ios::trunc);

    unsigned int S = _S.size();

    for(unsigned int s=0; s<S; s++)
    {
        V_val << _V[s]<< endl;
    }

    V_val.close();
}


void LearningAlgorithm::create_training_dataFile(void) const
{
    if(_S.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void create_training_dataFile(void) const method" << std::endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        exit(0);
    }

    if(_Q.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void create_training_dataFile(void) const method" << std::endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;
        exit(0);
    }

    //--Generating Q Regression Training data file--//
    fstream training_file_Reg;
    remove("/home/nash/Dropbox/ProjectArchive/linearRegression/Data/Q_Training_Reg.dat");
    training_file_Reg.open("/home/nash/Dropbox/ProjectArchive/linearRegression/Data/Q_Training_Reg.dat",ios::out | ios::trunc);
    training_file_Reg << "#S[s][0]   #S[s][1]   #S[sp][0]   #S[sp][1]   #Q[s][a]" << endl;

    //--Generating Q Classification Training data file--//
    fstream training_file_Class;
    remove("/home/nash/Dropbox/ProjectArchive/linearRegression/Data/Q_Training_Class.dat");
    training_file_Class.open("/home/nash/Dropbox/ProjectArchive/linearRegression/Data/Q_Training_Class.dat",ios::out | ios::trunc);

    unsigned int S = _S.size();
    unsigned int A = _A.size();

    unsigned int sp = 0;

    double s_0;
    double s_1;
    double sp_0;
    double sp_1;

    unsigned int degrees = 7;

    for(unsigned int s=0; s<S; s++)
    {
        for(unsigned int a=0; a<A; a++)
        {
            if(valid_action(s,a))
            {
                sp = expected_next_state(s, a);

                s_0 = _S[s][0];
                s_1 = _S[s][1];
                sp_0 = _S[sp][0];
                sp_1 = _S[sp][1];

                //--X_1, X_2, X_3, X_4--//
                /*training_file_Reg << s_0 << " "
                              << s_1 << " "
                              << sp_0 << " "
                              << sp_1 << " "

                                 //--X_i * X_j, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << s_0 * s_0 << " "
                              << s_0 * s_1 << " "
                              << s_0 * sp_0 << " "
                              << s_0 * sp_1 << " "

                              << s_1 * s_0 << " "
                              << s_1 * s_1 << " "
                              << s_1 * sp_0 << " "
                              << s_1 * sp_1 << " "

                              << sp_0 * s_0 << " "
                              << sp_0 * s_1 << " "
                              << sp_0 * sp_0 << " "
                              << sp_0 * sp_1 << " "

                              << sp_1 * s_0 << " "
                              << sp_1 * s_1 << " "
                              << sp_1 * sp_0 << " "
                              << sp_1 * sp_1 << " "

                                 //--(X_i)^2 * X_j, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 2) * s_0 << " "
                              << pow(s_0, 2) * s_1 << " "
                              << pow(s_0, 2) * sp_0 << " "
                              << pow(s_0, 2) * sp_1 << " "

                              << pow(s_1, 2) * s_0 << " "
                              << pow(s_1, 2) * s_1 << " "
                              << pow(s_1, 2) * sp_0 << " "
                              << pow(s_1, 2) * sp_1 << " "

                              << pow(sp_0, 2) * s_0 << " "
                              << pow(sp_0, 2) * s_1 << " "
                              << pow(sp_0, 2) * sp_0 << " "
                              << pow(sp_0, 2) * sp_1 << " "

                              << pow(sp_1, 2) * s_0 << " "
                              << pow(sp_1, 2) * s_1 << " "
                              << pow(sp_1, 2) * sp_0 << " "
                              << pow(sp_1, 2) * sp_1 << " "

                                 //--(X_i)^2 * (X_j)^2, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 2) * pow(s_0, 2) << " "
                              << pow(s_0, 2) * pow(s_1, 2) << " "
                              << pow(s_0, 2) * pow(sp_0, 2) << " "
                              << pow(s_0, 2) * pow(sp_1, 2) << " "

                              << pow(s_1, 2) * pow(s_0, 2) << " "
                              << pow(s_1, 2) * pow(s_1, 2) << " "
                              << pow(s_1, 2) * pow(sp_0, 2) << " "
                              << pow(s_1, 2) * pow(sp_1, 2) << " "

                              << pow(sp_0, 2) * pow(s_0, 2) << " "
                              << pow(sp_0, 2) * pow(s_1, 2) << " "
                              << pow(sp_0, 2) * pow(sp_0, 2) << " "
                              << pow(sp_0, 2) * pow(sp_1, 2) << " "

                              << pow(sp_1, 2) * pow(s_0, 2) << " "
                              << pow(sp_1, 2) * pow(s_1, 2) << " "
                              << pow(sp_1, 2) * pow(sp_0, 2) << " "
                              << pow(sp_1, 2) * pow(sp_1, 2) << " "

                                 //--(X_i)^3 * X_j, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 3) * s_0 << " "
                              << pow(s_0, 3) * s_1 << " "
                              << pow(s_0, 3) * sp_0 << " "
                              << pow(s_0, 3) * sp_1 << " "

                              << pow(s_1, 3) * s_0 << " "
                              << pow(s_1, 3) * s_1 << " "
                              << pow(s_1, 3) * sp_0 << " "
                              << pow(s_1, 3) * sp_1 << " "

                              << pow(sp_0, 3) * s_0 << " "
                              << pow(sp_0, 3) * s_1 << " "
                              << pow(sp_0, 3) * sp_0 << " "
                              << pow(sp_0, 3) * sp_1 << " "

                              << pow(sp_1, 3) * s_0 << " "
                              << pow(sp_1, 3) * s_1 << " "
                              << pow(sp_1, 3) * sp_0 << " "
                              << pow(sp_1, 3) * sp_1 << " "

                                 //--(X_i)^3 * (X_j)^2, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 3) * pow(s_0, 2) << " "
                              << pow(s_0, 3) * pow(s_1, 2) << " "
                              << pow(s_0, 3) * pow(sp_0, 2) << " "
                              << pow(s_0, 3) * pow(sp_1, 2) << " "

                              << pow(s_1, 3) * pow(s_0, 2) << " "
                              << pow(s_1, 3) * pow(s_1, 2) << " "
                              << pow(s_1, 3) * pow(sp_0, 2) << " "
                              << pow(s_1, 3) * pow(sp_1, 2) << " "

                              << pow(sp_0, 3) * pow(s_0, 2) << " "
                              << pow(sp_0, 3) * pow(s_1, 2) << " "
                              << pow(sp_0, 3) * pow(sp_0, 2) << " "
                              << pow(sp_0, 3) * pow(sp_1, 2) << " "

                              << pow(sp_1, 3) * pow(s_0, 2) << " "
                              << pow(sp_1, 3) * pow(s_1, 2) << " "
                              << pow(sp_1, 3) * pow(sp_0, 2) << " "
                              << pow(sp_1, 3) * pow(sp_1, 2) << " "

                                 //--(X_i)^3 * (X_j)^3, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 3) * pow(s_0, 3) << " "
                              << pow(s_0, 3) * pow(s_1, 3) << " "
                              << pow(s_0, 3) * pow(sp_0, 3) << " "
                              << pow(s_0, 3) * pow(sp_1, 3) << " "

                              << pow(s_1, 3) * pow(s_0, 3) << " "
                              << pow(s_1, 3) * pow(s_1, 3) << " "
                              << pow(s_1, 3) * pow(sp_0, 3) << " "
                              << pow(s_1, 3) * pow(sp_1, 3) << " "

                              << pow(sp_0, 3) * pow(s_0, 3) << " "
                              << pow(sp_0, 3) * pow(s_1, 3) << " "
                              << pow(sp_0, 3) * pow(sp_0, 3) << " "
                              << pow(sp_0, 3) * pow(sp_1, 3) << " "

                              << pow(sp_1, 3) * pow(s_0, 3) << " "
                              << pow(sp_1, 3) * pow(s_1, 3) << " "
                              << pow(sp_1, 3) * pow(sp_0, 3) << " "
                              << pow(sp_1, 3) * pow(sp_1, 3) << " "

                                 //--(X_i)^4 * X_j, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 4) * s_0 << " "
                              << pow(s_0, 4) * s_1 << " "
                              << pow(s_0, 4) * sp_0 << " "
                              << pow(s_0, 4) * sp_1 << " "

                              << pow(s_1, 4) * s_0 << " "
                              << pow(s_1, 4) * s_1 << " "
                              << pow(s_1, 4) * sp_0 << " "
                              << pow(s_1, 4) * sp_1 << " "

                              << pow(sp_0, 4) * s_0 << " "
                              << pow(sp_0, 4) * s_1 << " "
                              << pow(sp_0, 4) * sp_0 << " "
                              << pow(sp_0, 4) * sp_1 << " "

                              << pow(sp_1, 4) * s_0 << " "
                              << pow(sp_1, 4) * s_1 << " "
                              << pow(sp_1, 4) * sp_0 << " "
                              << pow(sp_1, 4) * sp_1 << " "

                                 //--(X_i)^4 * (X_j)^2, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 4) * pow(s_0, 2) << " "
                              << pow(s_0, 4) * pow(s_1, 2) << " "
                              << pow(s_0, 4) * pow(sp_0, 2) << " "
                              << pow(s_0, 4) * pow(sp_1, 2) << " "

                              << pow(s_1, 4) * pow(s_0, 2) << " "
                              << pow(s_1, 4) * pow(s_1, 2) << " "
                              << pow(s_1, 4) * pow(sp_0, 2) << " "
                              << pow(s_1, 4) * pow(sp_1, 2) << " "

                              << pow(sp_0, 4) * pow(s_0, 2) << " "
                              << pow(sp_0, 4) * pow(s_1, 2) << " "
                              << pow(sp_0, 4) * pow(sp_0, 2) << " "
                              << pow(sp_0, 4) * pow(sp_1, 2) << " "

                              << pow(sp_1, 4) * pow(s_0, 2) << " "
                              << pow(sp_1, 4) * pow(s_1, 2) << " "
                              << pow(sp_1, 4) * pow(sp_0, 2) << " "
                              << pow(sp_1, 4) * pow(sp_1, 2) << " "

                                 //--(X_i)^4 * (X_j)^3, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 4) * pow(s_0, 3) << " "
                              << pow(s_0, 4) * pow(s_1, 3) << " "
                              << pow(s_0, 4) * pow(sp_0, 3) << " "
                              << pow(s_0, 4) * pow(sp_1, 3) << " "

                              << pow(s_1, 4) * pow(s_0, 3) << " "
                              << pow(s_1, 4) * pow(s_1, 3) << " "
                              << pow(s_1, 4) * pow(sp_0, 3) << " "
                              << pow(s_1, 4) * pow(sp_1, 3) << " "

                              << pow(sp_0, 4) * pow(s_0, 3) << " "
                              << pow(sp_0, 4) * pow(s_1, 3) << " "
                              << pow(sp_0, 4) * pow(sp_0, 3) << " "
                              << pow(sp_0, 4) * pow(sp_1, 3) << " "

                              << pow(sp_1, 4) * pow(s_0, 3) << " "
                              << pow(sp_1, 4) * pow(s_1, 3) << " "
                              << pow(sp_1, 4) * pow(sp_0, 3) << " "
                              << pow(sp_1, 4) * pow(sp_1, 3) << " "

                                 //--(X_i)^4 * (X_j)^4, ∀ i ∈ {1,2,3,4} ,j ∈ {1,2,3,4}--//
                              << pow(s_0, 4) * pow(s_0, 4) << " "
                              << pow(s_0, 4) * pow(s_1, 4) << " "
                              << pow(s_0, 4) * pow(sp_0, 4) << " "
                              << pow(s_0, 4) * pow(sp_1, 4) << " "

                              << pow(s_1, 4) * pow(s_0, 4) << " "
                              << pow(s_1, 4) * pow(s_1, 4) << " "
                              << pow(s_1, 4) * pow(sp_0, 4) << " "
                              << pow(s_1, 4) * pow(sp_1, 4) << " "

                              << pow(sp_0, 4) * pow(s_0, 4) << " "
                              << pow(sp_0, 4) * pow(s_1, 4) << " "
                              << pow(sp_0, 4) * pow(sp_0, 4) << " "
                              << pow(sp_0, 4) * pow(sp_1, 4) << " "

                              << pow(sp_1, 4) * pow(s_0, 4) << " "
                              << pow(sp_1, 4) * pow(s_1, 4) << " "
                              << pow(sp_1, 4) * pow(sp_0, 4) << " "
                              << pow(sp_1, 4) * pow(sp_1, 4) << " "

                              << _Q[s][a] << endl;*/

                vec feat(4);

                feat(0) = _S[s][0];
                feat(1) = _S[s][1];
                feat(2) = _S[sp][0];
                feat(3) = _S[sp][1];

                //--X_1, X_2, X_3, X_4--//
                training_file_Reg << feat(0) << " "
                                  << feat(1) << " "
                                  << feat(2) << " "
                                  << feat(3) << " ";

                for(unsigned int k=1; k<=degrees; k++)
                {
                    for(unsigned int l=1; l<=k; l++)
                    {
                        for(unsigned int i=0; i<feat.n_rows; i++)
                        {
                            for(unsigned int j=0; j<feat.n_rows; j++)
                            {
                                training_file_Reg << pow(feat(i), k) * pow(feat(j), l) << " ";
                                training_file_Class << pow(feat(i), k) * pow(feat(j), l) << " ";
                            }
                        }
                    }
                }

                //--X_i + X_j--//
                /*training_file_Reg << feat(0) + feat(0) << " "
                              << feat(0) + feat(1) << " "
                              << feat(0) + feat(2) << " "
                              << feat(0) + feat(3) << " "

                              << feat(1) + feat(0) << " "
                              << feat(1) + feat(1) << " "
                              << feat(1) + feat(2) << " "
                              << feat(1) + feat(3) << " "

                              << feat(2) + feat(0) << " "
                              << feat(2) + feat(1) << " "
                              << feat(2) + feat(2) << " "
                              << feat(2) + feat(3) << " "

                              << feat(3) + feat(0) << " "
                              << feat(3) + feat(1) << " "
                              << feat(3) + feat(2) << " "
                              << feat(3) + feat(3) << " ";*/

                //--X_i - X_j--//
                /*training_file_Reg << feat(0) - feat(0) << " "
                              << feat(0) - feat(1) << " "
                              << feat(0) - feat(2) << " "
                              << feat(0) - feat(3) << " "

                              << feat(1) - feat(0) << " "
                              << feat(1) - feat(1) << " "
                              << feat(1) - feat(2) << " "
                              << feat(1) - feat(3) << " "

                              << feat(2) - feat(0) << " "
                              << feat(2) - feat(1) << " "
                              << feat(2) - feat(2) << " "
                              << feat(2) - feat(3) << " "

                              << feat(3) - feat(0) << " "
                              << feat(3) - feat(1) << " "
                              << feat(3) - feat(2) << " "
                              << feat(3) - feat(3) << " ";*/

                training_file_Reg << _Q[s][a] << endl;
                training_file_Class << is_max_a(s,a) << endl;
            }
        }
    }

    training_file_Reg.close();
    training_file_Class.close();

}


void LearningAlgorithm::create_maxA_dataFile(void) const
{
    if(_S.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void create_training_dataFile(void) const method" << std::endl
             << "_S size cannot be empty: " << _S.size()
             << endl;
        exit(0);
    }

    if(_Q.empty())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void create_training_dataFile(void) const method" << std::endl
             << "_Q size cannot be empty: " << _Q.size()
             << endl;
        exit(0);
    }

    //--Generating Q Training data file--//
    fstream maxA_file;
    //remove("/home/nash/Dropbox/ProjectArchive/linearRegression/Data/MaxA.dat");
    maxA_file.open("/home/nash/Dropbox/ProjectArchive/linearRegression/Data/MaxA.dat",ios::out | ios::trunc);
    maxA_file << "#M   #Max_A(Bool)" << endl;

    unsigned int S = _S.size();
    unsigned int A = _A.size();

    unsigned int m = 0;
    //bool max;

    for(unsigned int s=0; s<S; s++)
    {
        for(unsigned int a=0; a<A; a++)
        {
            if(valid_action(s,a))
            {
                /*if(get_Q_max_a(s) == _Q[s][a])
                {
                    max = true;
                }
                else
                {
                    max = false;
                }*/

                maxA_file << m++ << " "
                          << is_max_a(s,a) << endl;
            }
        }
    }

    maxA_file.close();
}


void LearningAlgorithm::init_steps(char* r_filename)
{
    std::fstream r_file;
    std::string line;

    unsigned int steps = 0;

    r_file.open(r_filename, std::ios::in);
    if(!r_file.is_open())
    {
        cerr << "LearningLocomotion Error: LearningAlgorithm class." << endl
             << "void init_steps(char*) method" << std::endl
             << "Cannot open R file: "<< r_filename  << std::endl;
        exit(1);
    }

    while(getline(r_file, line))
    {
        steps++;
    }

    r_file.close();

    _steps = steps;
}


void LearningAlgorithm::actuate_module(const unsigned int module, double output)
{
    //-- Send actuation command to the corrsponding module in the robot configuration.
    robot_primary->set_moduleServo_position(module, output);
    
    if(robot_secondary != NULL)
    {
        robot_secondary->set_moduleServo_position(module, output);
    }
}


void LearningAlgorithm::actuate_all_modules(const vector<double>& output)
{
    std::vector<double> servo_angle;
    
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        servo_angle.push_back(output[module]);
    }
    //-- Send actuation command to the corrsponding module in the robot configuration.
    robot_primary->set_all_moduleServo_position(servo_angle);
    
    if(robot_secondary != NULL)
    {
        robot_secondary->set_all_moduleServo_position(servo_angle);
    }
}


bool LearningAlgorithm::read_servo_positions_with_time() // TODO: This should be implemented as a seperate thread.
{
    bool got_position;
    
    got_position = robot_primary->get_all_moduleServo_position(servo_feedback);
    
    return got_position;
}


void LearningAlgorithm::read_servo_positions_with_time_THREAD()
{
    robot_primary->get_all_moduleServo_position(servo_feedback);
    //std::cout << "Returning from read_servo_positions_with_time_THREAD()." << std::endl; //--TODO: Debugger to be removed.
}


void LearningAlgorithm::set_robot_secondary(Robot* pointer_robot_secondary)
{
    if(pointer_robot_secondary != NULL)
    {
        robot_secondary = pointer_robot_secondary;
    }
    else
    {
        std::cerr << "Morphomotion Error: LearningAlgorithm class." << std::endl
                  << "void set_robot_secondary(Robot*) method." << std::endl
                  << "Cannot set Robot Secondary to NULL pointer: " << pointer_robot_secondary << "." <<std::endl;
        exit(1);
    }
}


void LearningAlgorithm::set_robot_primary(Robot* pointer_robot_primary)
{
    if(pointer_robot_primary != NULL)
    {
        robot_primary = pointer_robot_primary;
    }
    else
    {
        std::cerr << "Morphomotion Error: LearningAlgorithm class." << std::endl
                  << "void set_robot_primary(Robot*) method." << std::endl
                  << "Cannot set Robot Primary to NULL pointer: " << pointer_robot_primary << "." <<std::endl;
        exit(1);
    }
}


Robot* LearningAlgorithm::get_robot_primary(void)
{
    return(robot_primary);
}


Robot* LearningAlgorithm::get_robot_secondary(void)
{
    return(robot_secondary);
}


void LearningAlgorithm::set_servo_max(double new_servo_max)
{
    if(new_servo_max <= 90.0 && new_servo_max >= -90.0)
    {
        servo_max = new_servo_max;
    }
    else
    {
        std::cerr << "Morphomotion Error: LearningAlgorithm class." << std::endl
                  << "void set_servo_max(double) method."
                  << "New servo angle is out of range." << std::endl;
        exit(1);
    }
}


double LearningAlgorithm::get_servo_max(void)
{
    return(servo_max);
}


void LearningAlgorithm::set_servo_min(double new_servo_min)
{
    if(new_servo_min <= 90.0 && new_servo_min >= -90.0)
    {
        servo_min = new_servo_min;
    }
    else
    {
        std::cerr << "Morphomotion Error: LearningAlgorithm class." << std::endl
                  << "void set_servo_min(double) method."
                  << "New servo angle is out of range." << std::endl;
        exit(1);
    }
}


double LearningAlgorithm::get_servo_min(void)
{
    return(servo_min);
}


void LearningAlgorithm::set_EKF_dt(const double delta_time)
{
    if(delta_time <= 0)
    {
        std::cerr << "Morphomotion Error: LearningAlgorithm class." << std::endl
                  << "void set_EKF_dt(const double) method." << std::endl
                  << "Invalide delta_time: " << delta_time << std::endl;
        exit(1);
    }
    else
    {
        EKF_dt = delta_time;
    }
}


double LearningAlgorithm::get_EKF_dt(void)
{
    return(EKF_dt);
}


void LearningAlgorithm::set_EKF_r(const double new_r)
{
    EKF_r = new_r;
}


double LearningAlgorithm::get_EKF_r(void)
{
    return(EKF_r);
}


void LearningAlgorithm::set_EKF_qf(const double new_qf)
{
    EKF_qf = new_qf;
}


double LearningAlgorithm::get_EKF_qf(void)
{
    return(EKF_qf);
}


double LearningAlgorithm::calculate_random_uniform(double minimum, double maximum) const
{
    double random = (double)rand()/(RAND_MAX+1.0);
    
    double random_uniform = minimum + (maximum-minimum)*random;
    
    return(random_uniform);
}


double LearningAlgorithm::calculate_random_normal(double mean, double standard_deviation) const
{
    double random_normal = 0.0;
    
    const double pi = 4.0*atan(1.0);
    
    double random_uniform_1;
    double random_uniform_2;
    
    do
    {
        random_uniform_1 = (double)rand()/(RAND_MAX+1.0);
        
    }while(random_uniform_1 == 0.0);
    
    random_uniform_2 = (double)rand()/(RAND_MAX+1.0);
    
    // Box-Muller transformation
    
    random_normal = mean + sqrt(-2.0*log(random_uniform_1))*sin(2.0*pi*random_uniform_2)*standard_deviation;
    
    return(random_normal);
}


void LearningAlgorithm::changemode(int dir)
{
    static struct termios oldt, newt;
    
    if ( dir == 1 )
    {
        tcgetattr( STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}


int LearningAlgorithm::kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;
    
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);
    
    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);
    
}
