/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S I M U L A T I O N O P E N R A V E   C L A S S                                                            */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "SimulationOpenRave.h"

using namespace OpenRAVE;

void SimulationOpenRave::SetViewer()
{
    viewer = RaveCreateViewer(penv,"qtcoin");
    BOOST_ASSERT(!!viewer);
    
    cout << "Viewer!!!!" << endl;
    
    // attach it to the environment:
    penv->Add(viewer);
    
    //-- Set the camera
    //SetCamera(0.427, 0.285, 0.47, 0.718, 0.338, 0.18, 0.166);
    
    //--Finally you call the viewer's infinite loop (This is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);
    
    return;
}

//-- DEFAULT CONSTRUCTOR
SimulationOpenRave::SimulationOpenRave(void):Robot()
{
    //-- Set default parameters.
    set_default_parameters();
}


//-- COPY CONSTRUCTOR
SimulationOpenRave::SimulationOpenRave(const Robot* source_object):Robot()
{
    if(source_object == NULL)
    {
        std::cerr << "MorphoMotion Error: SimulationOpenRave class." << std::endl
                  << "SimulationOpenRave(const Robot*) method." << std::endl
                  << "Source object pointer = NULL" <<std::endl;
        
        exit(1);
    }
    
    //-- Set default parameters.
    set_default_parameters();
    
    this->set_robot_type(source_object->get_robot_type());
    this->set_evaluation_method(source_object->get_evaluation_method());
    this->set_number_of_modules(source_object->get_number_of_modules());
    
    //-- Set robot priority
    this->set_robot_priority("Robot_Secondary");
}


//-- DESTRUCTOR
SimulationOpenRave::~SimulationOpenRave(void)
{
    if(showgui)
    {
        //std::cout << "Destroyed Simulation" << std::endl;
        if(pthviewer)
        {
            pthviewer->join();
            delete pthviewer;
        }
    }
    
    //-- Destroy the environment
    if(penv)
    {
        penv->Destroy();
    }
}


void SimulationOpenRave::copy(const Robot* source_object)
{
    if(source_object == NULL)
    {
        std::cerr << "MorphoMotion Error: SimulationOpenRave class." << std::endl
                  << "void copy(const Robot*) method." << std::endl
                  << "Source object pointer = NULL" <<std::endl;
        
        exit(1);
    }
    
    this->set_robot_type(source_object->get_robot_type());
    this->set_evaluation_method(source_object->get_evaluation_method());
    this->set_number_of_modules(source_object->get_number_of_modules());
    
    //-- Set robot priority
    this->set_robot_priority("Robot_Secondary");
}

void SimulationOpenRave::init_simu_env(std::string controller)
{
    //-------------------------------//
    //-- OPENRAVE INITIZALIZATION -- //
    //-------------------------------//
    RaveInitialize(true); //-- start openrave core.
    penv = RaveCreateEnvironment(); //-- create the main environment.
    RaveSetDebugLevel(Level_Debug);
    
    penv->StopSimulation();
    
    if (showgui)
    {
        //boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
        pthviewer = new boost::thread(boost::bind(&SimulationOpenRave::SetViewer, this));
        usleep(100000); //-- Wait for the viewer to setup.
    }
    
    // load the scene
    if(!penv->Load(scene_file_name))
    {
        penv->Destroy();
        return;
    }
    usleep(100000); // wait for the viewer to init
    
    //-- Get the robot.
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);
    probot = robots[0];
    //cout << "Robot: " << probot->GetName() << endl;
    
    //-- create the controllers, make sure to lock environment!
    EnvironmentMutex::scoped_lock EvnLock(penv->GetMutex()); //-- lock environment
    
    //-- Get number of modules in the configuration
    number_of_modules = probot->GetDOF();
    
    //-- Load the controller.
    pcontroller=RaveCreateController(penv,"servocontroller");
    
    vector<int> dofindices(probot->GetDOF());
    for(int i = 0; i < probot->GetDOF(); ++i)
    {
        dofindices[i] = i;
    }
    probot->SetController(pcontroller,dofindices,1);
    //load_controller("servocontroller");
    
    //-- Record the initial position of the robot.
    t0=probot->GetTransform();
    
    // Capture initial position of the robot
    //robot_pos_initial = get_robot_XY(); // TODO: May be not needed here.
}


void SimulationOpenRave::set_default_parameters(void)
{
    //penv = NULL;
    //viewer = NULL;
    pthviewer = NULL;
    //probot = NULL;
    //pcontroller = NULL;
    
    this->set_robot_environment("SimulationOpenRave");
    this->set_robot_type("CubeN_ServoFeedBack");
    number_of_modules = 2;
    this->set_robot_priority("Robot_Primary");
    scene_file_name = "../models/Minicube-I.env.xml";
    simu_resolution_microseconds = 0.001;
    
    showgui = true;
}


void SimulationOpenRave::SetCamera(dReal q0, dReal q1, dReal q2, dReal q3, dReal tx, dReal ty, dReal tz)
{
    RaveVector<float> rotation(q0,q1,q2,q3);
    RaveVector<float> translation(tx,ty,tz);
    RaveTransform<float> T(rotation,translation);
    viewer->SetCamera(T);
}


void SimulationOpenRave::load_controller(std::string controller)
{
    EnvironmentMutex::scoped_lock EvnLock(penv->GetMutex()); //-- lock environment
    pcontroller=RaveCreateController(penv,controller);
    vector<int> dofindices(probot->GetDOF());
    for(int i = 0; i < probot->GetDOF(); ++i)
    {
        dofindices[i] = i;
    }
    probot->SetController(pcontroller,dofindices,1);
}

void SimulationOpenRave::set_scene_file_name(const std::string new_scene_file_name)
{
    scene_file_name = new_scene_file_name;
}


std::string SimulationOpenRave::get_scene_file_name(void)
{
    return(scene_file_name);
}


void SimulationOpenRave::set_simu_resolution_microseconds(double new_simu_resolution_microseconds)
{
    simu_resolution_microseconds = new_simu_resolution_microseconds;
}


double SimulationOpenRave::get_simu_resolution_microseconds(void)
{
    return(simu_resolution_microseconds);
}


Vector SimulationOpenRave::get_robot_XY()
{
    return(probot->GetCenterOfMass());
}


bool SimulationOpenRave::get_showgui()
{
    return(showgui);
}


void SimulationOpenRave::set_showgui(bool new_showgui)
{
    showgui = new_showgui;
}


void SimulationOpenRave::reset_robot(void)
{
    //-- Stop Simulation
    penv->StopSimulation();
    
    if(this->get_robot_type() == "Hoap3")
    {
        //-- Set the translation of the robot to the initial position.
        probot->SetTransform(t0);
        
        //-- Turn off gravity
        penv->GetPhysicsEngine()->SetGravity({0,0,0});
        
        //-- Record previous Physics Engine Option
        int physicsOption = penv->GetPhysicsEngine()->GetPhysicsOptions();
        
        //-- Turn off Physics Engine Option
        penv->GetPhysicsEngine()->SetPhysicsOptions(0);
        
        //-- Reset Modules
        reset_modules();
        
        //-- Set the translation of the robot to the initial position.
        probot->SetTransform(t0);
        
        //-- Turn on gravity
        penv->GetPhysicsEngine()->SetGravity({0,0,-9.81});
        
        //-- Revert to the previous Physics Engine Option
        penv->GetPhysicsEngine()->SetPhysicsOptions(physicsOption);
    }
    else
    {
        reset_modules();
        
        //-- Set the translation of the robot to the initial position.
        probot->SetTransform(t0);
    }
    
    
    //-- Capture initial position and orientation of the robot
    robot_pos_initial = get_robot_XY();
    robot_pos_previous = get_robot_XY();
    _roll = M_PI/2.0;
    
    //-- Initialize the controller evaluation time counter to 0;
    init_elapsed_evaluation_time();
    
    distance_travelled = 0;
}


void SimulationOpenRave::reset_robot_position(void)
{
    //-- Stop Simulation
    penv->StopSimulation();
    
    reset_modules();
    
    //-- Set the translation of the robot to the initial position.
    probot->SetTransform(t0);
    
    //-- Capture initial position and orientation of the robot
    robot_pos_initial = get_robot_XY();
    robot_pos_previous = get_robot_XY();
    _roll = M_PI/2.0;
    
    distance_travelled = 0;
}

void SimulationOpenRave::reset_modules(void)
{
    //-- Set the servo of each module to zero.
    for(unsigned int i=0; i<number_of_modules; i++)
    {
        set_moduleServo_position(i, 0);
    }
    
    // Wait for two seconds.
    int two_seconds = (1/simu_resolution_microseconds)*2;
    for (int ss=0; ss<two_seconds; ss++)
    {
        penv->StepSimulation(simu_resolution_microseconds);
    }
}

void SimulationOpenRave::set_sinusoidal_controller_parameters(const vector<double>& sinusoidal_amplitude, const vector<double>& sinusoidal_offset, const vector<double>& sinusoidal_phase, const double sinusoidal_frequency)
{
    stringstream os,is;
    
    is << "setamplitude ";
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        is << sinusoidal_amplitude[module] << " ";
    }
    pcontroller->SendCommand(os,is);
    
    is << "setinitialphase ";
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        is << sinusoidal_phase[module] << " ";
    }
    pcontroller->SendCommand(os,is);
    
    is << "setoffset ";
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        is << sinusoidal_offset[module] << " ";
    }
    pcontroller->SendCommand(os,is);
    
    is << "setperiod " << sinusoidal_frequency << " ";
    pcontroller->SendCommand(os,is);
    
    is << "oscillation on ";
    pcontroller->SendCommand(os,is);
}


void SimulationOpenRave::stop_sinusoidal_controller(void)
{
    stringstream os,is;
    
    is << "reset_controller ";
    pcontroller->SendCommand(os,is);
    
    is << "oscillation off ";
    pcontroller->SendCommand(os,is);
}


void SimulationOpenRave::set_moduleServo_position(unsigned int module, double servo_angle)
{
    stringstream os,is;
    is << "setpos1" << " " << module << " " << servo_angle << " ";
    pcontroller->SendCommand(os,is);
}


void SimulationOpenRave::set_all_moduleServo_position(const vector<double>& servo_angle)
{
    stringstream os,is;
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        is << "setpos1" << " " << module << " " << servo_angle[module] << " ";
        pcontroller->SendCommand(os,is);
    }
}


double SimulationOpenRave::get_moduleServo_position(unsigned int module)
{
    stringstream os,is;
    double servo_angle;
    
    is << "getpos1" << " " << module << " ";
    pcontroller->SendCommand(os,is);
    os >> servo_angle;
    
    return servo_angle;
}


bool SimulationOpenRave::get_all_moduleServo_position(vector<ServoFeedback*>& servo_feedback)
{
    return(get_all_moduleServo_position_with_time_THREAD(servo_feedback));
}


bool SimulationOpenRave::get_all_moduleServo_position_with_time(vector<ServoFeedback*>& servo_feedback)
{
    stringstream os,is;
    double angle = 12.3456; //-- This value set to 12.3456 as a way of detecting when a failuer to read module position occurs.
    
    for(unsigned int module=0; module<number_of_modules; module++)
    {
        is << "getpos1" << " " << module << " ";
        pcontroller->SendCommand(os,is);
        os >> angle;
        servo_feedback[module]->set_new_value(elapsed_evaluation_time, angle);
        //servo_feedback[module]->add_to_history(elapsed_evaluation_time, angle);
    }
    
    return true;
}


bool SimulationOpenRave::get_all_moduleServo_position_with_time_THREAD(vector<ServoFeedback*>& servo_feedback)
{
    while(receive_broadcast)
    {
        set_broadcast_thread(true);
        stringstream os,is;
        double angle = 12.3456; //-- This value set to 12.3456 as a way of detecting when a failuer to read module position occurs.
        
        for(unsigned int module=0; module<number_of_modules; module++)
        {
            is << "getpos1" << " " << module << " ";
            pcontroller->SendCommand(os,is);
            os >> angle;
            //angle = angle + calculate_random_uniform(-10.0, 10.0); // TODO: To be removed
            //servo_feedback[module]->add_to_history(elapsed_evaluation_time, angle);
            servo_feedback[module]->set_new_value(elapsed_evaluation_time, angle);
        }
        //std::cout << std::endl << elapsed_evaluation_time; // TODO: Debugger to be removed.
    }
    
    set_broadcast_thread(false);
    return true;
}


void SimulationOpenRave::init_elapsed_evaluation_time(void)
{
    previous_read_evaluation_time = 0;
    elapsed_evaluation_time = 0;
}


void SimulationOpenRave::update_elapsed_evaluation_time(void)
{
    previous_read_evaluation_time = elapsed_evaluation_time;
    elapsed_evaluation_time = elapsed_evaluation_time + (simu_resolution_microseconds * 1000000);
}


unsigned long SimulationOpenRave::get_previous_read_evaluation_time(void)
{
    return(previous_read_evaluation_time);
}


unsigned long SimulationOpenRave::get_elapsed_evaluation_time(void)
{
    return(elapsed_evaluation_time);
}


double SimulationOpenRave::calculate_total_distance_travelled_euclidean(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_current - robot_pos_initial;
    double totalDistanceTravelled = sqrt((distance3D.x*distance3D.x)+(distance3D.y*distance3D.y));
    
    return(totalDistanceTravelled);
}


double SimulationOpenRave::calculate_total_distance_travelled_forward(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    
    Vector distance3D = robot_pos_current - robot_pos_initial;
    double distanceTravelledFwd = distance3D.y;
    //probot->
    
    return(distanceTravelledFwd);
}


double SimulationOpenRave::calculate_distance_travelled_euclidean(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_current - robot_pos_previous;
    
    double distanceTravelledEuc = sqrt((distance3D.x*distance3D.x)+(distance3D.y*distance3D.y));
    
    previous_velocity_time = elapsed_evaluation_time;
    robot_pos_previous = robot_pos_current;
    
    return(distanceTravelledEuc);
}


double SimulationOpenRave::calculate_distance_travelled_forward(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_previous - robot_pos_current;
    double distanceTravelled = distance3D.y;
    robot_pos_previous = robot_pos_current;
    return(distanceTravelled);
}


double SimulationOpenRave::calculate_velocity(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_current - robot_pos_previous;
    
    double dt = (elapsed_evaluation_time - previous_velocity_time) / 1000000.0;
    double velocity = sqrt((distance3D.x*distance3D.x)+(distance3D.y*distance3D.y))/dt;
    
    previous_velocity_time = elapsed_evaluation_time;
    robot_pos_previous = robot_pos_current;
    
    return(velocity);
}


double SimulationOpenRave::calculate_forward_velocity(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_current - robot_pos_previous;
    
    double dt = (elapsed_evaluation_time - previous_velocity_time) / 1000000.0;
    double velocity_x = distance3D.x/dt;
    double velocity_y = distance3D.y/dt;
    double velocity_fwd = velocity_y;
    
    if(fabs(velocity_x) > fabs(velocity_y))
    {
        velocity_fwd = velocity_x;
    }
    else
    {
        velocity_fwd = velocity_y;
    }
    
    previous_velocity_time = elapsed_evaluation_time;
    robot_pos_previous = robot_pos_current;
    
    return(velocity_fwd);
}


double SimulationOpenRave::calculate_velocity_X(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_current - robot_pos_previous;
    
    double dt = (elapsed_evaluation_time - previous_velocity_time) / 1000000.0;
    double velocity_x = distance3D.x/dt;
    
    previous_velocity_time = elapsed_evaluation_time;
    robot_pos_previous = robot_pos_current;
    
    return(velocity_x);
}


double SimulationOpenRave::calculate_velocity_Y(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    Vector distance3D = robot_pos_current - robot_pos_previous;
    
    double dt = (elapsed_evaluation_time - previous_velocity_time) / 1000000.0;
    double velocity_y = distance3D.y/dt;
    
    previous_velocity_time = elapsed_evaluation_time;
    robot_pos_previous = robot_pos_current;
    
    return(velocity_y);
}


void SimulationOpenRave::measure_cumulative_distance(void)
{
    distance_travelled = distance_travelled + calculate_total_distance_travelled_euclidean();
}


double SimulationOpenRave::get_vx(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();

    double cosT = cos(_roll);
    double sinT = sin(_roll);

    vec t(2);
    t(0) = robot_pos_previous.x;
    t(1) = robot_pos_previous.y;

    mat R(2,2);
    R(0,0) = cosT;
    R(0,1) = -sinT;

    R(1,0) = sinT;
    R(1,1) = cosT;

    t = -R.t() * t;

    //mat X_inv(3,3);
    mat X_inv = eye<mat>(3,3);
    X_inv(0,0) = cosT;
    X_inv(0,1) = sinT;
    X_inv(0,2) = t(0);

    X_inv(1,0) = -sinT;
    X_inv(1,1) = cosT;
    X_inv(1,2) = t(1);

    vec P_glo(3);
    P_glo(0) = robot_pos_current.x;
    P_glo(1) = robot_pos_current.y;
    P_glo(2) = 1.0;

    vec P_loc;
    P_loc = X_inv * P_glo;

    double dt = (elapsed_evaluation_time - previous_velocity_time) / 1000000.0;
    double vx = P_loc(0)/dt;

    //--TODO: To be removed--//
    Vector distance3D = robot_pos_current - robot_pos_previous;
    double velocity_y = distance3D.y/dt;

    previous_velocity_time = elapsed_evaluation_time;
    robot_pos_previous = robot_pos_current;

    Transform tn;
    tn = probot->GetTransform();

    double q0 = tn.rot.w;
    double q1 = tn.rot.x;
    double q2 = tn.rot.y;
    double q3 = tn.rot.z;

    _roll = atan2(2*((q0*q1) + (q2*q3)) , 1 - (2 * (pow(q1,2) + pow(q2,2))));

    if(_roll < 0.0)
    {
        _roll = (2.0 * M_PI) + _roll;
    }
    _roll = (1.5 * M_PI) - _roll;

    //--TODO: To be removed--//
    double pitch = asin(2 * ((q0*q2) - (q3*q1)));
    double yaw = atan2(2*((q0*q3) + (q1*q2)) , 1 - (2 * (pow(q2,2) + pow(q3,2))));

    /*cout << endl
         << "   X: " << tn.trans.x <<  "  Y: " << tn.trans.y
         << "   Roll[X]: " << _roll * (180.0/M_PI) <<  "º  Pitch[Y]: " << pitch * (180.0/M_PI)
         << "º  Yaw[Z]: " << yaw * (180.0/M_PI) << "º   P_loc_X: " << P_loc(0)
         << "  vx: " << vx << "  Delta_Y: " << velocity_y;*/
    
    return(vx);
}


void SimulationOpenRave::get_robot_rotation(vector<double>& rot)
{
    Vector robot_pos_current;
    robot_pos_current = probot->GetCenterOfMass();

    Transform tn;
    tn = probot->GetTransform();
    
    double q0 = tn.rot.w;
    double q1 = tn.rot.x;
    double q2 = tn.rot.y;
    double q3 = tn.rot.z;
    
    /*double roll = atan((2*((q0*q1) + (q2*q3))) / (1 - (2 * (pow(q1,2) + pow(q2,2)))));
    double pitch = asin(2 * ((q0*q2) - (q3*q1)));
    double yaw = atan((2*((q0*q3) + (q1*q2))) / (1 - (2 * (pow(q2,2) + pow(q3,2)))));*/
    
    double roll = atan2(2*((q0*q1) + (q2*q3)) , 1 - (2 * (pow(q1,2) + pow(q2,2))));
    double pitch = asin(2 * ((q0*q2) - (q3*q1)));
    double yaw = atan2(2*((q0*q3) + (q1*q2)) , 1 - (2 * (pow(q2,2) + pow(q3,2))));

    if(roll < 0.0)
    {
        //roll = M_PI + (M_PI - (roll * -1.0));
        roll = (2.0 * M_PI) + roll;
    }
    roll = (1.5 * M_PI) - roll;
    
    rot[0] = roll * (180.0/M_PI);
    rot[1] = pitch * (180.0/M_PI);
    rot[2] = yaw * (180.0/M_PI);
    
    /*cout << endl << "    Trans-X: " << tn.trans.x <<  "  Trans-Y: " << tn.trans.y
         << "     X: " << robot_pos_current.x <<  "   Y: " << robot_pos_current.y
         << "     Roll[X]: " << roll * (180.0/M_PI) <<  "º    Pitch[Y]: " << pitch * (180.0/M_PI)
         << "º    Yaw[Z]: " << yaw * (180.0/M_PI) << "º";*/
}


double SimulationOpenRave::get_robot_X(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    double robot_X = robot_pos_current.x;
    return(robot_X);
}


double SimulationOpenRave::get_robot_Y(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();  //-- TODO: To be changed to get_robot_XY();
    double robot_Y = robot_pos_current.y;
    return(robot_Y);
}

double SimulationOpenRave::get_robot_Z(void)
{
    Vector robot_pos_current = probot->GetCenterOfMass();
    double robot_Z = robot_pos_current.z;
    return(robot_Z);
}


double SimulationOpenRave::get_robot_feet_X(const std::string feet)
{
    KinBody::LinkPtr lPtr_feet;
    if(feet == "Left")
    {
        lPtr_feet = probot->GetLink("left_ankle2");
    }
    else if(feet == "Right")
    {
        lPtr_feet = probot->GetLink("right_ankle2");
    }
    else
    {
        std::cerr << "Morphomotion Error: Robot class." << std::endl
                  << "double get_robot_feet_X(const std::string) method." << std::endl
                  << "Incorrect feet type: " << feet << std::endl;
        exit(1);
    }
    
    Vector feet_pos_current = lPtr_feet->GetGlobalCOM();
    double feet_X = feet_pos_current.x;
    
    return(feet_X);
}


double SimulationOpenRave::get_robot_feet_Y(const std::string feet)
{
    KinBody::LinkPtr lPtr_feet;
    if(feet == "Left")
    {
        lPtr_feet = probot->GetLink("left_ankle2");
    }
    else if(feet == "Right")
    {
        lPtr_feet = probot->GetLink("right_ankle2");
    }
    else
    {
        std::cerr << "Morphomotion Error: Robot class." << std::endl
                  << "double get_robot_feet_Y(const std::string) method." << std::endl
                  << "Incorrect feet type: " << feet << std::endl;
        exit(1);
    }
    
    Vector feet_pos_current = lPtr_feet->GetGlobalCOM();
    double feet_Y = feet_pos_current.y;
    
    return(feet_Y);
}


double SimulationOpenRave::get_robot_feet_Z(const std::string feet)
{
    KinBody::LinkPtr lPtr_feet;
    if(feet == "Left")
    {
        lPtr_feet = probot->GetLink("left_ankle2");
    }
    else if(feet == "Right")
    {
        lPtr_feet = probot->GetLink("right_ankle2");
    }
    else
    {
        std::cerr << "Morphomotion Error: Robot class." << std::endl
                  << "double get_robot_feet_Z(const std::string) method." << std::endl
                  << "Incorrect feet type: " << feet << std::endl;
        exit(1);
    }
    
    Vector feet_pos_current = lPtr_feet->GetGlobalCOM();
    double feet_Z = feet_pos_current.z;
    
    return(feet_Z);
    //return(feet_Z-0.0375);
}


void SimulationOpenRave::record_robot_foot_com(const double time)
{
    vector<double> left_feet_com(4);
    vector<double> right_feet_com(4);
    
    left_feet_com[0] = time;
    left_feet_com[1] = get_robot_feet_X("Left");
    left_feet_com[2] = get_robot_feet_Y("Left");
    left_feet_com[3] = get_robot_feet_Z("Left");
    
    right_feet_com[0] = time;
    right_feet_com[1] = get_robot_feet_X("Right");
    right_feet_com[2] = get_robot_feet_Y("Right");
    right_feet_com[3] = get_robot_feet_Z("Right");
    
    d_robotfeet_left_com.push_back(left_feet_com);
    d_robotfeet_right_com.push_back(right_feet_com);
}


void SimulationOpenRave::calculate_step_length(void)
{
    if(!d_robotfeet_left_com.size() || !d_robotfeet_right_com.size())
    {
        std::cerr << "Morphomotion Error: Robot class." << std::endl
                  << "void calculate_step_length(void) method." << std::endl
                  << "Robot feet COM vector cannot be emoty." << std::endl;
        exit(1);
    }
    
    vector<double> left_feet_xy;
    vector<double> right_feet_xy;
    
    //for()
    
    fstream foot_com_graph_file;
    remove("../Evaluation_Files/foot_step.dat");
    foot_com_graph_file.open("../Evaluation_Files/foot_step.dat", fstream::out);
    
    if(!foot_com_graph_file.is_open())
    {
        std::cerr << "Morphomotion Error: Robot class." << std::endl
                  << "void calculate_step_length(void) method." << std::endl
                  << "Cannot open file:"  << std::endl;
        exit(1);
    }
    
    for(unsigned int i=0; i<d_robotfeet_left_com.size(); i++)
    {
        for(unsigned int j=0; j<d_robotfeet_left_com[i].size(); j++)
        {
            foot_com_graph_file << d_robotfeet_left_com[i][j] << " ";
        }
        
        for(unsigned int j=1; j<d_robotfeet_right_com[i].size(); j++)
        {
            foot_com_graph_file << d_robotfeet_right_com[i][j] << " ";
        }
        
        foot_com_graph_file << std::endl;
    }
    
    foot_com_graph_file.close();
}


void SimulationOpenRave::step(const std::string& type)
{
    penv->StepSimulation(simu_resolution_microseconds);
    
    if(this->robot_priority == Robot_Primary)
    {
        if(type == "evaluation")
        {
            //usleep(get_simu_resolution_microseconds() * 1000000);  //-- Real-Time
            //usleep(get_simu_resolution_microseconds() * 2000000);  //-- 2x
        }
        
        update_elapsed_evaluation_time();
        
        if(evaluation_method == Euclidean_Distance_Cumulative)
        {
            if(elapsed_evaluation_time % (CUMULATIVE_DISTENCE_MEASUREMENT_RESOLUTION * 1000000) == 0)
            {
                measure_cumulative_distance();
            }
        }
    }
    else if(this->robot_priority == Robot_Secondary)
    {
        update_elapsed_evaluation_time();
    }
}

