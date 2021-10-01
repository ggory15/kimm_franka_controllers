#include "kimm_franka_controllers/basic_franka_controller_simul.h"

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace RobotController;

int main(int argc, char **argv)
{   
    //Ros setting
    ros::init(argc, argv, "franka_controller");
    ros::NodeHandle n_node;

    ros::Rate loop_rate(1000);

    // Robot Wapper
    ctrl_ = new RobotController::FrankaWrapper("ns0", true, n_node);
    ctrl_->initialize();

    // mujoco sub
    ros::Subscriber jointState = n_node.subscribe("/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_command_sub = n_node.subscribe("/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_time_sub = n_node.subscribe("/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber ctrl_type_sub = n_node.subscribe("/mujoco_ros_interface/ctrl_type", 1, &ctrltypeCallback, ros::TransportHints().tcpNoDelay(true));

    // mujoco pub
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 5);

    // robot pub
    ee_state_pub_ = n_node.advertise<geometry_msgs::Transform>("ns0/mujoco_ros_interface/ee_state", 5);

    // msg 
    robot_command_msg_.torque.resize(9); // gripper(2) + wheels(4) + robot (7)
    ee_state_msg_ = geometry_msgs::Transform();

    isgrasp_ = false;
    InitMob();
    
    while (ros::ok()){
        keyboard_event();

        // ctrl computation
        ctrl_->compute(time_);
        
        // get output
        ctrl_->mass(robot_mass_);
        ctrl_->nle(robot_nle_);
        ctrl_->g(robot_g_);
        ctrl_->J(robot_J_);
        ctrl_->state(state_);
        
        ctrl_->franka_output(franka_qacc_); // this is only for simulation mode
        franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;

        if (ctrl_->ctrltype() != 0)
            UpdateMob();
        else
            InitMob();
        setGripperCommand();
        setRobotCommand();
        robot_command_pub_.publish(robot_command_msg_);

        getEEState();
        
        ros::spinOnce();
        loop_rate.sleep();
    }//while

    return 0;
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    ctrl_->franka_update(msg);
}

void ctrltypeCallback(const std_msgs::Int16ConstPtr &msg){
    ROS_WARN("%d", msg->data);
    
    if (msg->data != 899){
        int data = msg->data;
        ctrl_->ctrl_update(data);
    }
    else{
        if (isgrasp_)
            isgrasp_=false;
        else
            isgrasp_=true;
    }
}

void setRobotCommand(){
    robot_command_msg_.MODE = 1;
    robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    
    // cout << franka_qacc_ << endl;

    for (int i=0; i<7; i++)
        robot_command_msg_.torque[i] = franka_torque_(i);    
}

void setGripperCommand(){
    if (isgrasp_){
        robot_command_msg_.torque[7] = -200.0;
        robot_command_msg_.torque[8] = -200.0;
    }
    else{
        robot_command_msg_.torque[7] = 100.0;
        robot_command_msg_.torque[8] = 100.0;
    }
}

void getEEState(){
    Vector3d pos;
    Quaterniond q;
    ctrl_->ee_state(pos, q);

    ee_state_msg_.translation.x = pos(0);
    ee_state_msg_.translation.y = pos(1);
    ee_state_msg_.translation.z = pos(2);

    ee_state_msg_.rotation.x = q.x();
    ee_state_msg_.rotation.y = q.y();
    ee_state_msg_.rotation.z = q.z();
    ee_state_msg_.rotation.w = q.w();
    ee_state_pub_.publish(ee_state_msg_);
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'g': //gravity
                msg = 0;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Gravity mode" << endl;
                cout << " " << endl;
                break;
            case 'h': //home
                msg = 1;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Initialization" << endl;
                cout << " " << endl;
                break;
            case 'a': //home
                msg = 2;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;    
            case 's': //home
                msg = 3;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;    
            case 'd': //home
                msg = 4;
                ctrl_->ctrl_update(msg);
                cout << " " << endl;
                cout << "Move EE with wholebody Motion" << endl;
                cout << " " << endl;
                break;   
            case 'z': //grasp
                if (isgrasp_){
                    cout << "Release hand" << endl;
                    isgrasp_ = false;
                }
                else{
                    cout << "Grasp object" << endl;
                    isgrasp_ = true; 
                }
                break;
        }
    }
}

