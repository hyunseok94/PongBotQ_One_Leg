#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "Eigen/Dense"
#include <stdlib.h>


//develope : adh 18.8.31

#define PI      3.141592
#define D2R     PI/180
#define R2D     180/PI
#define MAX_DEBUG_INDEX 60000

using Eigen::MatrixXd;

namespace gazebo
{

    class PongBotLeg2_plugin : public ModelPlugin
    {
        double hip_pos_error, knee_pos_error;
        double HR_err, HP_err, KN_err;
        double time = 0;
        double dt;
        double save[MAX_DEBUG_INDEX][2];
        unsigned int debug_index = 0;

        double q[3], q_dot[3];
        double g = 9.81;
        double ref_hip_pos;
        double ref_knee_pos;
        double ref_RHR_deg;
        double x, z;
        int cnt = 0;
        double theta1, d_theta1, d2_theta1;
        double theta2, d_theta2, d2_theta2;
        double old_theta2, old_d_theta2;
        double thigh = 0.305, calf = 0.305;
        double P_, D_, I_;
        double target_pos[3] = {0, 0, 0};

        enum
        {
            flight_phase = 0,
            stance_phase,
            take_off,
            landing,
            continuous
        };


        //  Pointers
        physics::JointPtr HR_JOINT;
        physics::JointPtr HP_JOINT;
        physics::JointPtr KN_JOINT;

        physics::JointWrench HR_JOINT_W;
        physics::JointWrench HP_JOINT_W;
        physics::JointWrench KN_JOINT_W;

        physics::LinkPtr FOOT;

        //  PID
        common::PID pid_HIP_ROLL;
        common::PID pid_HIP_PITCH;
        common::PID pid_KNEE;

        // ** Pointers for each joints

        physics::ModelPtr model_;
        event::ConnectionPtr update_connection_;
        common::Time last_update_time_;

        // ============= node handler define ==============
        ros::NodeHandle n;

        // ============= subscriber define ==============
        ros::Subscriber SUB; // (ros --> gazebo)

        ros::Subscriber S_Data_Save_Flag;

        // ============= publisher define ==============
        ros::Publisher P_Times;

        // ============= topic define ==============
        std_msgs::Float64 m_Times;

        // ============= position vector define ==============
        math::Vector3 FOOT_POS;

    public:

        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
        {
            this->model_ = _model;
            // initialize a PID class

            printf("[!! Hopping Robot was Loaded !!!]\n");

            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "PongBotLeg2_plugin");

            ROS_INFO("==== PLUGIN_LOADED ====");

            S_Data_Save_Flag = n.subscribe("Data_save_flag", 1, &PongBotLeg2_plugin::S_Data_Save_Flag_func, this);
            P_Times = n.advertise<std_msgs::Float64>("times", 1);

            ros::Rate loop_rate(1000);

            printf("ADH\n");

            this->model_ = _model;

            // =================== PID GAIN TUNNING ==================== //

            this->pid_HIP_ROLL.Init(150, 0.1, 1, 200, -200, 1000, -1000);
            this->HR_JOINT = this->model_->GetJoint("HIP_ROLL_JOINT");
            this->pid_HIP_PITCH.Init(200, 0.2, 1, 200, -200, 1000, -1000);
            this->HP_JOINT = this->model_->GetJoint("HIP_PITCH_JOINT");
            this->pid_KNEE.Init(200, 0.2, 1, 200, -200, 1000, -1000);
            this->KN_JOINT = this->model_->GetJoint("KNEE_JOINT");

            // =================== Get Link ==================== //
            this->FOOT = this->model_->GetLink("FOOT");

            // =================== Connect world ==================== //
            this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBotLeg2_plugin::UpdateAlgorithm, this));
        }

        // ============= Callback function setting ==============
    public:
        void S_Data_Save_Flag_func(const std_msgs::Float64Ptr &msg)
        {
            //        data_save_flag = msg->data;
            //        printf("data_save_flag = %f\n",data_save_flag);
        }

        void UpdateAlgorithm();

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PongBotLeg2_plugin)
}

void gazebo::PongBotLeg2_plugin::UpdateAlgorithm()
{
    common::Time current_time = this->model_->GetWorld()->GetSimTime();
    HR_JOINT_W = this->HR_JOINT->GetForceTorque(0);
    HP_JOINT_W = this->HP_JOINT->GetForceTorque(0);
    KN_JOINT_W = this->KN_JOINT->GetForceTorque(0);

    dt = current_time.Double() - this->last_update_time_.Double();

    //        printf("dt = %f\n",dt);

    FOOT_POS = this->FOOT->GetWorldPose().pos;

    // ============== control code start ================ //

    target_pos[0] = 0; // deg
    target_pos[1] = -45;
    target_pos[2] = 90;


    // =============== control code end ================ //


    HR_err = this->HR_JOINT->GetAngle(0).Radian() - (target_pos[0] * D2R);
    this->pid_HIP_ROLL.Update(HR_err, dt);
    this->HR_JOINT->SetForce(1, this->pid_HIP_ROLL.GetCmd());
    if (this->pid_HIP_ROLL.GetCmd() >= 1000 || this->pid_HIP_ROLL.GetCmd() <= -1000) {
        printf("======= this->pid_HIP_ROLL.GetCmd() = %f\n", this->pid_HIP_ROLL.GetCmd());
    }
    HP_err = this->HP_JOINT->GetAngle(0).Radian() - (target_pos[1] * D2R);
    this->pid_HIP_PITCH.Update(HP_err, dt);
    this->HP_JOINT->SetForce(1, this->pid_HIP_PITCH.GetCmd());
    if (this->pid_HIP_PITCH.GetCmd() >= 1000 || this->pid_HIP_PITCH.GetCmd() <= -1000) {
        printf("======= this->pid_HIP_PITCH.GetCmd() = %f\n", this->pid_HIP_PITCH.GetCmd());
    }

    KN_err = this->KN_JOINT->GetAngle(0).Radian() - (target_pos[2] * D2R);
    this->pid_KNEE.Update(KN_err, dt);
    this->KN_JOINT->SetForce(1, this->pid_KNEE.GetCmd());
    if (this->pid_KNEE.GetCmd() >= 1000 || this->pid_KNEE.GetCmd() <= -1000) {
        printf("======= this->pid_KNEE.GetCmd() = %f\n", this->pid_KNEE.GetCmd());
    }

    this->last_update_time_ = current_time;

    // =============== publish data setting ================ //

    m_Times.data = 0;

    P_Times.publish(m_Times);

}
