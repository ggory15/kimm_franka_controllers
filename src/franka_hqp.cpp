#include "kimm_franka_controllers/franka_hqp.h"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace kimmhqp;
using namespace kimmhqp::trajectory;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace kimmhqp::robot;
using namespace kimmhqp::contacts;

namespace RobotController{
    FrankaWrapper::FrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), n_node_(node)
    {
        time_ = 0.;
        mode_change_ = false;
        ctrl_mode_ = 0;
        node_index_ = 0;
    }

    void FrankaWrapper::initialize(){
        // Robot
        string model_path, urdf_name;
        n_node_.getParam("/" + robot_node_ +"/robot_urdf_path", model_path);    
        n_node_.getParam("/" + robot_node_ +"/robot_urdf", urdf_name);  
        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;
        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false, false);
        model_ = robot_->model();
        na_ = robot_->na();
        nv_ = robot_->nv();
        nq_ = robot_->nq();

        // State
        state_.q_.setZero(nq_);
        state_.v_.setZero(nv_);
        state_.dv_.setZero(nv_);
        state_.torque_.setZero(na_);

        // tsid
        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q_, state_.v_);
        data_ = tsid_->data();

        // tasks
        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        postureTask_->Kp(500.0*Vector::Ones(na_));
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());

        Vector3d ee_offset(0.0, 0, 0.0);
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset);
        eeTask_->Kp(500.0*Vector::Ones(6));
        eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());
        
        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        Vector dq_max = 500000.0*Vector::Ones(na_);    
        Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);

        // trajecotries
        sampleEE_.resize(12, 6);
        samplePosture_.resize(na_);

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");
        
        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "qpoases");           

        // service
        joint_action_client_ = n_node_.serviceClient<kimm_joint_planner_ros_interface::action_joint_path>("/ns0/kimm_joint_planner_ros_interface_server/action_joint_path");
        se3_action_client_ = n_node_.serviceClient<kimm_se3_planner_ros_interface::action_se3_path>("/ns0/kimm_se3_planner_ros_interface_server/action_se3_path");    
    }
    
    void FrankaWrapper::franka_update(const sensor_msgs::JointState::ConstPtr& msg){
        assert(issimulation_);
        for (int i=0; i< 7; i++){ 
            state_.q_(i) = msg->position[i];
            state_.v_(i) = msg->velocity[i];
        }
    }
    void FrankaWrapper::franka_update(const Vector7d& q, const Vector7d& qdot){
        assert(!issimulation_);
        state_.q_.tail(na_) = q;
        state_.v_.tail(na_) = qdot;
    }

    void FrankaWrapper::ctrl_update(const int& msg){
        ctrl_mode_ = msg;
        mode_change_ = true;
    }

    void FrankaWrapper::compute(const double& time){
        time_ = time;

        robot_->computeAllTerms(data_, state_.q_, state_.v_);

        if (ctrl_mode_ == 0){ // gravity mode
            state_.torque_.setZero();
        }
        if (ctrl_mode_ == 1){
            if (mode_change_){
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");
                
                tsid_->addMotionTask(*postureTask_, 1e-2, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                q_ref_.setZero(7);
                q_ref_(0) =  0;//M_PI /4.0;
                q_ref_(1) = 0.0 * M_PI / 180.0;
                q_ref_(3) = -M_PI / 2.0;
                q_ref_(5) = M_PI/ 2.0;
                q_ref_(6) = M_PI/ 4.0;

                trajPosture_Cubic_->setInitSample(state_.q_.tail(na_));
                trajPosture_Cubic_->setDuration(1.0);
                trajPosture_Cubic_->setStartTime(time_);
                trajPosture_Cubic_->setGoalSample(q_ref_);      
                            
                mode_change_ = false;                
            }

            trajPosture_Cubic_->setCurrentTime(time_);
            samplePosture_ = trajPosture_Cubic_->computeNext();
            postureTask_->setReference(samplePosture_);
           
            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       

            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }
        if (ctrl_mode_ == 2){
            if (mode_change_){
                tsid_->removeTask("task-se3");
                tsid_->removeTask("task-posture");
                tsid_->removeTask("task-torque-bounds");

                tsid_->addMotionTask(*postureTask_, 1e-2, 1);
                // tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1, 0);

                trajEE_Timeopt_->clearWaypoints();
                trajEE_Timeopt_->setStartTime(time_);                
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));               
                trajEE_Timeopt_->addWaypoint(H_ee_ref_);
                H_ee_ref_.translation()(0) += 0.10;
                trajEE_Timeopt_->addWaypoint(H_ee_ref_);
               
                q_ref_ = state_.q_;       

                mode_change_ = false;     
            }
            trajEE_Timeopt_->setCurrentTime(time_);
            sampleEE_ = trajEE_Timeopt_->computeNext();
            eeTask_->setReference(sampleEE_);

            samplePosture_.pos = q_ref_;
            postureTask_->setReference(samplePosture_);

            // Momentum-based Observer

            const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
            state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));
        }



        // ///////////////////////// Predefined CTRL for GUI //////////////////////////////////////////////////
        // if (ctrl_mode_ == 900){ // joint ctrl
        //     if (mode_change_){
        //         action_joint_srv_.request.test.data = true;
        //         joint_action_client_.call(action_joint_srv_);
                
        //         node_num_ = action_joint_srv_.response.res_traj.size() -1;
                
        //         tsid_->removeTask("task-mobile");
        //         tsid_->removeTask("task-mobile2");
        //         tsid_->removeTask("task-se3");
        //         tsid_->removeTask("task-posture");
        //         tsid_->removeTask("task-torque-bounds");

        //         tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        //         tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

        //         mode_change_ = false;
        //         update_weight_ = false;
        //         stime_ = time_;

        //         prev_node_ = -1;
        //         node_index_ = 0;
        //         q_ref_ = state_.q_.tail(na_-2); 
        //         VectorXd Kp, Kd;
        //         Kp.setOnes(na_-2);
        //         Kd.setOnes(na_-2);
        //         for (int i=0; i<na_-2; i++){
        //             Kp(i) = action_joint_srv_.response.kp[i];
        //             Kd(i) = action_joint_srv_.response.kv[i];
        //         }
        //         postureTask_->Kp(Kp);
        //         postureTask_->Kd(Kd);
        //     }           

        //     samplePosture_.pos.setZero(na_-2);
        //     samplePosture_.vel.setZero(na_-2);
        //     samplePosture_.acc.setZero(na_-2);
           
        //     if (node_num_ == -1){
        //         samplePosture_.pos = q_ref_;
        //     }
        //     else{
        //         if (node_index_ < node_num_){
        //             if (prev_node_ != node_index_){      
        //                 for (int i=0; i<na_-2; i++)
        //                     samplePosture_.pos(i) = action_joint_srv_.response.res_traj[node_index_].position[i];
        //                 node_index_ += 1;      
        //             }           
        //         }
        //         else{
        //             for (int i=0; i<na_-2; i++)
        //                 samplePosture_.pos(i) = action_joint_srv_.response.res_traj[node_num_].position[i];
        //         }
        //     }
        //     postureTask_->setReference(samplePosture_);

        //     const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
        //     const HQPOutput & sol = solver_->solve(HQPData);

        //     state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        //     state_.torque_.head(2).setZero();
        // }
        // if (ctrl_mode_ == 901){ // base ctrl
        //     if (mode_change_){
        //         action_mobile_srv_.request.test.data = true;
        //         mobile_action_client_.call(action_mobile_srv_);
                
        //         node_num_ = action_mobile_srv_.response.mobile_path.points.size() -1;
        //         tsid_->removeTask("task-posture");
        //         tsid_->removeTask("task-torque-bounds");
        //         tsid_->removeTask("task-mobile");
        //         tsid_->removeTask("task-mobile2");
        //         tsid_->removeTask("task-se3");

        //         H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
        //         double theta = atan2(-H_mobile_ref_.rotation()(0, 1), H_mobile_ref_.rotation()(0,0));
                
        //         tsid_->addMotionTask(*mobileTask_, 0.1, 1);
        //         tsid_->addMotionTask(*mobileTask2_, 1, 1);
        //         tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        //         tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

        //         trajMobile_Cubic_->setStartTime(time_);
        //         trajMobile_Cubic_->setDuration(0.001);
        //         trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
        //         trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));

        //         mode_change_ = false;
        //         update_weight_ = false;
        //         stime_ = time_;

        //         prev_node_ = -1;
        //         node_index_ = 0;

        //         trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
        //         trajPosture_Cubic_->setDuration(1.0);
        //         trajPosture_Cubic_->setStartTime(time_);
        //         trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));    
        //     }           

        //     Vector3d goal_path; 
        //     if (node_num_ != -1){
        //         goal_path << action_mobile_srv_.response.mobile_path.points[node_index_].x, action_mobile_srv_.response.mobile_path.points[node_index_].y, action_mobile_srv_.response.mobile_path.points[node_index_].theta; 
        //         H_mobile_ref_.translation().head(2) = goal_path.head(2);

        //         H_mobile_ref_.rotation().setIdentity();
        //         H_mobile_ref_.rotation()(0, 0) = cos(goal_path(2));
        //         H_mobile_ref_.rotation()(0, 1) = -sin(goal_path(2));
        //         H_mobile_ref_.rotation()(1, 0) = sin(goal_path(2));
        //         H_mobile_ref_.rotation()(1, 1) = cos(goal_path(2));
        //     }            
           
        //     if (node_index_ < node_num_){
        //         if (prev_node_ != node_index_){                   
        //             trajMobile_Cubic_->setStartTime(time_);
        //             trajMobile_Cubic_->setDuration(0.001);
        //             trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
        //             trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));

        //             prev_node_ = node_index_;
        //             stime_ = time_;
        //         }

        //         if (stime_ + 0.1 <= time_)
        //             node_index_ += 1;              
        //     }
        //     else{
        //         if (!update_weight_){
        //             tsid_->updateTaskWeight("task-mobile", 1.);
        //             tsid_->updateTaskWeight("task-mobile2", 0.01);

        //             trajMobile_Cubic_->setStartTime(time_);
        //             trajMobile_Cubic_->setDuration(1.5);
        //             trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
        //             trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));

        //             update_weight_ = true;
        //         }         
        //     }

        //     trajMobile_Cubic_->setCurrentTime(time_);
        //     sampleMobile_ = trajMobile_Cubic_->computeNext();

        //     mobileTask2_-> setReference(sampleMobile_);
        //     mobileTask_ -> setReference(sampleMobile_);
           
        //     trajPosture_Cubic_->setCurrentTime(time_);
        //     samplePosture_ = trajPosture_Cubic_->computeNext();
        //     postureTask_->setReference(samplePosture_);

        //     const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
        //     const HQPOutput & sol = solver_->solve(HQPData);

        //     state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        //     state_.torque_.head(2) = tsid_->getAccelerations(solver_->solve(HQPData)).head(2);
        // }
        // if (ctrl_mode_ == 902){ //base forward
        //     if (mode_change_){
        //         stime_ = time_;
        //         tsid_->removeTask("task-posture");
        //         tsid_->removeTask("task-torque-bounds");
        //         tsid_->removeTask("task-mobile");
        //         tsid_->removeTask("task-mobile2");
        //         tsid_->removeTask("task-se3");

        //         tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        //         tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

        //         trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
        //         trajPosture_Cubic_->setDuration(1.0);
        //         trajPosture_Cubic_->setStartTime(time_);
        //         trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));  

        //         mode_change_ = false;
        //     }
        //     if (stime_ + 2.0 > time_){
        //         state_.torque_(0) = 5.0;
        //         state_.torque_(1) = 5.0;
        //     }
        //     else{
        //         state_.torque_(0) = 0.0;
        //         state_.torque_(1) = 0.0;
        //     }
        //     trajPosture_Cubic_->setCurrentTime(time_);
        //     samplePosture_ = trajPosture_Cubic_->computeNext();
        //     postureTask_->setReference(samplePosture_);

        //     const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
        //     const HQPOutput & sol = solver_->solve(HQPData);

        //     state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2); 
        // }
        // if (ctrl_mode_ == 903){ //base backward
        //     if (mode_change_){
        //         stime_ = time_;
        //         tsid_->removeTask("task-posture");
        //         tsid_->removeTask("task-torque-bounds");
        //         tsid_->removeTask("task-mobile");
        //         tsid_->removeTask("task-mobile2");
        //         tsid_->removeTask("task-se3");

        //         tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        //         tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

        //         trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
        //         trajPosture_Cubic_->setDuration(1.0);
        //         trajPosture_Cubic_->setStartTime(time_);
        //         trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));  

        //         mode_change_ = false;
        //     }
        //     if (stime_ + 2.0 > time_){
        //         state_.torque_(0) = -5.0;
        //         state_.torque_(1) = -5.0;
        //     }
        //     else{
        //         state_.torque_(0) = 0.0;
        //         state_.torque_(1) = 0.0;
        //     }
        //     trajPosture_Cubic_->setCurrentTime(time_);
        //     samplePosture_ = trajPosture_Cubic_->computeNext();
        //     postureTask_->setReference(samplePosture_);

        //     const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
        //     const HQPOutput & sol = solver_->solve(HQPData);

        //     state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2); 
        // }
        // if (ctrl_mode_ == 904){ // se3 ctrl
        //     if (mode_change_){
        //         action_se3_srv_.request.test.data = true;
        //         se3_action_client_.call(action_se3_srv_);
               
        //         node_num_ = action_se3_srv_.response.res_traj.size() -1;

        //         tsid_->removeTask("task-mobile");
        //         tsid_->removeTask("task-mobile2");
        //         tsid_->removeTask("task-se3");
        //         tsid_->removeTask("task-posture");
        //         tsid_->removeTask("task-torque-bounds");

        //         tsid_->addMotionTask(*eeTask_, 1, 1);
        //         tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        //         tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

        //         if (!action_se3_srv_.response.iswholebody.data){
        //             tsid_->addMotionTask(*mobileTask_, 1, 0);
        //             trajMobile_Cubic_->setStartTime(time_);
        //             trajMobile_Cubic_->setDuration(0.001);
        //             trajMobile_Cubic_->setGoalSample(robot_->getMobilePosition(data_, 5));
        //             trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
        //         }

        //         mode_change_ = false;
        //         update_weight_ = false;
        //         stime_ = time_;

        //         prev_node_ = -1;
        //         node_index_ = 0;

        //         H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
        //         VectorXd Kp, Kd;
        //         Kp.setOnes(6);
        //         Kd.setOnes(6);
        //         for (int i=0; i<6; i++){
        //             Kp(i) = action_se3_srv_.response.kp[i];
        //             Kd(i) = action_se3_srv_.response.kv[i];
        //         }

        //         trajPosture_Cubic_->setInitSample(state_.q_.tail(na_-2));
        //         trajPosture_Cubic_->setDuration(1.0);
        //         trajPosture_Cubic_->setStartTime(time_);
        //         trajPosture_Cubic_->setGoalSample(state_.q_.tail(na_-2));  

        //     }           

        //     sampleEE_.pos.setZero(12);
        //     sampleEE_.vel.setZero(6);
        //     sampleEE_.acc.setZero(6);
        //     Eigen::Matrix3d rot;
        //     Eigen::Quaterniond quat;

        //     if (node_num_ == -1){
        //         sampleEE_.pos.head(3) = H_ee_ref_.translation();
        //         sampleEE_.pos.segment(3,3) = H_ee_ref_.rotation().col(0);
        //         sampleEE_.pos.segment(6,3) = H_ee_ref_.rotation().col(1);
        //         sampleEE_.pos.tail(3) = H_ee_ref_.rotation().col(2);
        //     }
        //     else{
        //         if (node_index_ < node_num_){
        //             if (prev_node_ != node_index_){   
        //                 sampleEE_.pos(0) = action_se3_srv_.response.res_traj[node_index_].translation.x;
        //                 sampleEE_.pos(1) = action_se3_srv_.response.res_traj[node_index_].translation.y;
        //                 sampleEE_.pos(2) = action_se3_srv_.response.res_traj[node_index_].translation.z;
        //                 quat.x() = action_se3_srv_.response.res_traj[node_index_].rotation.x;
        //                 quat.y() = action_se3_srv_.response.res_traj[node_index_].rotation.y;
        //                 quat.z() = action_se3_srv_.response.res_traj[node_index_].rotation.z;
        //                 quat.w() = action_se3_srv_.response.res_traj[node_index_].rotation.w;
        //                 rot = quat.toRotationMatrix();

        //                 sampleEE_.pos.segment(3,3) = rot.col(0);
        //                 sampleEE_.pos.segment(6,3) = rot.col(1);
        //                 sampleEE_.pos.tail(3) = rot.col(2);     
                        
        //                 sampleEE_.pos.segment(3,3) = H_ee_ref_.rotation().col(0);
        //                 sampleEE_.pos.segment(6,3) = H_ee_ref_.rotation().col(1);
        //                 sampleEE_.pos.tail(3) = H_ee_ref_.rotation().col(2);
        //                 node_index_ += 1;      
        //             }           
        //         }
        //         else{
        //             sampleEE_.pos(0) = action_se3_srv_.response.res_traj[node_num_].translation.x;
        //             sampleEE_.pos(1) = action_se3_srv_.response.res_traj[node_num_].translation.y;
        //             sampleEE_.pos(2) = action_se3_srv_.response.res_traj[node_num_].translation.z;
        //             quat.x() = action_se3_srv_.response.res_traj[node_num_].rotation.x;
        //             quat.y() = action_se3_srv_.response.res_traj[node_num_].rotation.y;
        //             quat.z() = action_se3_srv_.response.res_traj[node_num_].rotation.z;
        //             quat.w() = action_se3_srv_.response.res_traj[node_num_].rotation.w;
        //             rot = quat.toRotationMatrix();

        //             sampleEE_.pos.segment(3,3) = rot.col(0);
        //             sampleEE_.pos.segment(6,3) = rot.col(1);
        //             sampleEE_.pos.tail(3) = rot.col(2);      

        //             sampleEE_.pos.segment(3,3) = H_ee_ref_.rotation().col(0);
        //             sampleEE_.pos.segment(6,3) = H_ee_ref_.rotation().col(1);
        //             sampleEE_.pos.tail(3) = H_ee_ref_.rotation().col(2);  
        //         }
        //     }
        //     eeTask_->setReference(sampleEE_);
            
        //     trajPosture_Cubic_->setCurrentTime(time_);
        //     samplePosture_ = trajPosture_Cubic_->computeNext();
        //     postureTask_->setReference(samplePosture_);
            
        //     if (!action_se3_srv_.response.iswholebody.data){
        //         trajMobile_Cubic_->setCurrentTime(time_);
        //         sampleMobile_ = trajMobile_Cubic_->computeNext();
        //         mobileTask_->setReference(sampleMobile_);
        //     }
        //     const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);       
        //     const HQPOutput & sol = solver_->solve(HQPData);

        //     state_.torque_.tail(na_ -2) = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        //     state_.torque_.head(2) = tsid_->getAccelerations(solver_->solve(HQPData)).head(2);
        // }

        
    }
    
    void FrankaWrapper::franka_output(VectorXd & qacc) {
        qacc = state_.torque_.tail(na_);
    }

    void FrankaWrapper::mass(MatrixXd & mass_mat){
        mass_mat = robot_->mass(data_).bottomRightCorner(na_, na_);
    }

    void FrankaWrapper::nle(VectorXd & nle_vec){
        nle_vec = robot_->nonLinearEffects(data_).tail(na_);
    }
    void FrankaWrapper::g(VectorXd & g_vec){
        g_vec = data_.g;
    }
    
    void FrankaWrapper::J(MatrixXd & Jo){
        Data::Matrix6x Jo2;
        Jo2.resize(6, 7);
        robot_->jacobianWorld(data_, robot_->model().getJointId("panda_joint7"), Jo2);
        Jo = Jo2;
    }

    void FrankaWrapper::ee_state(Vector3d & pos, Eigen::Quaterniond & quat){
        for (int i=0; i<3; i++)
            pos(i) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation()(i);

        Quaternion<double> q(robot_->position(data_, robot_->model().getJointId("panda_joint7")).rotation());
    }


}// namespace
