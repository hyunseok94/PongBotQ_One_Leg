/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : Run Elmo driver
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 1000
 *
 * This is a redundancy test.
 *
 * (c)Arthur Kim Hyeon Seok 2020
 */

//#include "main.h"

// **********Basic libraries*********//
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>
#include <inttypes.h>
#include <stdlib.h>

// **********ROS libraries*********//
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

// **********SOEM library*********//
#include "ethercat.h"
#include "pdo_def.h"
#include "ecat_dc.h"
#include "slave_info.h"
#include "osal.h"

// **********Eigen library*********//
#include "Eigen/Dense"

// **********RBDL library*********//
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

// **********CRobot library*********//
#include "CRobot.h"

// **********Xenomai libraries*********//
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>

#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>
//#include <rtdm/serial.h>

// *********** RVIZ *************//
#include <sensor_msgs/JointState.h>             //for rviz
#include <geometry_msgs/WrenchStamped.h>        //for rviz
#include <tf/transform_broadcaster.h>           //for rviz
//#include <ignition/math/Vector3.hh>

//JoyStick
#include <sensor_msgs/Joy.h>

// IMU CoM
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
int fd_MIC;


#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500


//#define NUM_OF_ELMO 3
//#define _USE_DC

//#define R2D 180/PI
//#define D2R PI/180

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;

//*************** 1. Variables ****************/
// Ethercat
//char ecat_ifname[32] = "enp5s0";
char ecat_ifname[32] = "rteth0";
ELMO_Drive_pt ELMO_drive_pt[JOINT_NUM];
bool needlf;
bool inOP;
uint32_t ob;
uint16_t ob2;
uint8_t ob3;
char IOmap[4096];
int oloop, iloop, wkc_count;
int expectedWKC;
volatile int wkc;
unsigned int cycle_ns = 1000000; // Control Cycle 1[ms]
int recv_fail_cnt = 0;

unsigned int n_fail = 0;

// loop (main & thread)
int motion_run_flag = 1;
int print_run_flag = 1;
int imu_run_flag = 0;
int QP_run_flag = 1;
int sys_ready_flag = 0;
//bool nmpc_run_flag = false;

// Servo
int ServoState = 0;
int servo_ready = 0;

VectorXd started = VectorXd::Zero(JOINT_NUM);

VectorXd Max_Value_Save(VectorXd Value_set);
VectorXd roll_set = VectorXd::Zero(2);
VectorXd pitch_set = VectorXd::Zero(2);
VectorXd yaw_set = VectorXd::Zero(2);

VectorXd roll_vel_set = VectorXd::Zero(2);
VectorXd pitch_vel_set = VectorXd::Zero(2);
VectorXd yaw_vel_set = VectorXd::Zero(2);

VectorXd x_acc_set = VectorXd::Zero(2);
VectorXd y_acc_set = VectorXd::Zero(2);
VectorXd z_acc_set = VectorXd::Zero(2);

VectorXd motion_time_set = VectorXd::Zero(2);
VectorXd IMU_time_set = VectorXd::Zero(2);
VectorXd QP_time_set = VectorXd::Zero(2);

uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;

// ============== DH Parameters ================ //

VectorNd Low_ABS_actual_pos = VectorNd::Zero(JOINT_NUM);
VectorNd Low_Incre_actual_pos = VectorNd::Zero(JOINT_NUM);
VectorNd Low_actual_vel = VectorNd::Zero(JOINT_NUM);

VectorNd ABS_actual_pos = VectorNd::Zero(JOINT_NUM);
VectorNd Incre_actual_pos = VectorNd::Zero(JOINT_NUM);
VectorNd tmp_Incre_actual_joint_pos = VectorNd::Zero(JOINT_NUM);

int Low_Gear[JOINT_NUM] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
double Low_ratedCur[JOINT_NUM] = {2.85, 2.85, 8.9, 8.9, 2.85, 2.85, 2.85, 2.85, 8.9, 8.9, 2.85, 2.85};
double Low_Kt[JOINT_NUM] = {0.159, 0.159, 0.156, 0.156, 0.159, 0.159, 0.159, 0.159, 0.156, 0.156, 0.159, 0.159};
int32_t Low_Resolution[JOINT_NUM] = {262144, 262144, 16384, 16384, 262144, 262144,  262144, 262144, 16384, 16384 , 262144, 262144};


// ============================================= //

//// Elmo setting
UINT16 maxTorque = 3500;
INT16 Low_TargetTor[JOINT_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Save
//#define SAVE_LENGTH 13    //The number of data
#define SAVE_LENGTH 20    //The number of data
//#define SAVE_COUNT 3600000 //Save Time = 3600000[ms]=3600[s]
//#define SAVE_COUNT 600000 //Save Time = 3600000[ms]=3600[s]
#define SAVE_COUNT 10000 //Save Time = 300000[ms]=300[s]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];
//double save_array[SAVE_COUNT][SAVE_LENGTH];

// ROS Param
ros::Subscriber S_Joystick;
ros::Subscriber S_IMU;
ros::Publisher P_data;
int ros_exit = 0;
std_msgs::Float64MultiArray m_data;

double Thread_time = 0.0;

//Xenomai
RT_TASK PongBot_task;
RT_TASK Print_task;
RT_TASK NMPC_task;
RT_TASK IMU_task;
RT_TASK QP_task;

//RT_TASK RT_task3;
//RT_MUTEX mutex_desc; //mutex
RTIME check_now_motion_time, check_previous_motion_time; // Ethercat time
RTIME now_motion_time, previous_motion_time; // Thread 1 cycle time
RTIME now_print_time, previous_print_time; // Thread 2 cycle time
RTIME now_nmpc_time, previous_nmpc_time; // Thread 2 cycle time
RTIME now_imu_time, previous_imu_time; // Thread 2 cycle time
RTIME check_now_imu_time, check_previous_imu_time; // Thread 2 cycle time
RTIME now_QP_time, previous_QP_time; // Thread 2 cycle time
RTIME check_now_QP_time, check_previous_QP_time; // Thread 2 cycle time
RTIME check_now_nmpc_time, check_previous_nmpc_time;

double check_motion_time = 0.0;
double motion_time = 0.0;
double print_time = 0.0;
double nmpc_time = 0.0;
double check_imu_time = 0.0;
double imu_time = 0.0;
double QP_time = 0.0;
double check_QP_time = 0.0;
double check_nmpc_time = 0;

double m_time = 0;

//RBDL
int version_test;
Model* PB_model = new Model();

//CRobot
CRobot PongBot;

//Rviz
ros::Publisher P_joint_states;
sensor_msgs::JointState m_joint_states;
geometry_msgs::TransformStamped odom_trans;


//**********************************************//
//*************** 2. Functions ****************//
static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);
void ServoOn(void); // : Servo ON
void ServoOff(void); // : Servo Off
//void Torque_Off(void); // : Torque Off
void motion_run(void* arg); //  TMotor Control task
void print_run(void* arg); //  Print task
void imu_run(void* arg); //  Imu Task
void nmpc_run(void* arg); //  Imu Task
void QP_run(void* arg); //  QP Task
void signal_handler(int sig); //  Catch "Ctrl + C signal"
bool ecat_init(void);

void Load(void); // : parameter initial setting
int IMU_COM_Setting();
void EncoderRead(void); // : q, q_dot Read
void TargetTor_Gen(void);
void DataSave(void);
void FileSave(void);
void Max_Time_Save(double now_time);
double Count2Rad_ABS(int _Resolution, INT32 Count);
double Count2RadDot(int Gear_Ratio, INT32 CountPerSec);
INT16 Tor2Cur(double OutputTorque, double _Kt, int _Gear, double _ratedCur);

// ROS function
void JoystickCallback(const sensor_msgs::Joy& msg);
void ROSMsgPublish(void);

int main(int argc, char* argv[]) {
//    rtser_config_t ABC;
//    ABC.baud_rate = 115200;

    printf(" ROS Setting ...\n");
    //ros::init(argc, argv, "elmo_pkgs");
    ros::init(argc, argv, "pongbot_pkgs");
    ros::NodeHandle nh;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    S_Joystick = nh.subscribe("joy", 1, &JoystickCallback);

    P_data = nh.advertise<std_msgs::Float64MultiArray>("tmp_data", 1000);
    P_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    m_data.data.resize(20);
    m_joint_states.name.resize(12);
    m_joint_states.position.resize(12);
    sleep(1);

    printf(" Init main ...\n");
    sleep(1);

    // Display Adapter name
    printf(" Use default adapter %s ...\n", ecat_ifname);
    sleep(1);
    printf("\n");

    // Initial Setting
    printf(" Loading...\n");
    Load();
    printf("\n");

    // Thread Setting Start
    printf("Create Thread ...\n");
    for (int i = 1; i < 4; ++i) {
        printf("%d...\n", i);
        sleep(1);
    }
    
    rt_task_create(&PongBot_task, "Motion_task", 0, 99, 0);
    rt_task_create(&Print_task, "Print_task", 0, 60, 0);
    rt_task_create(&NMPC_task, "NMPC_task", 0, 80, 0);
    //rt_task_create(&IMU_task, "Imu_task", 0, 95, 0);
    //rt_task_create(&IMU_task, "Imu_task", 0, 99, 0);
    //rt_task_create(&QP_task, "QP_task", 0, 90, 0);

    rt_task_start(&PongBot_task, &motion_run, NULL);
    rt_task_start(&Print_task, &print_run, NULL);
    rt_task_start(&NMPC_task, &nmpc_run, NULL);
    //rt_task_start(&IMU_task, &imu_run, NULL);
    //rt_task_start(&QP_task, &QP_run, NULL);
    printf("\n");

    // Thread Setting End
    ros::Rate loop_rate(1000);
    while (ros::ok()) {

        ROSMsgPublish();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void motion_run(void* arg) {
    //printf("Motion Thread Start \n");
    if (ecat_init() == false) {
        motion_run_flag = 0;
    }
    rt_task_sleep(1e6);
    
    ec_send_processdata();
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

    
    cout << "!!!!!!!!!!!!!!!!!!!! motion_run_flag = " << motion_run_flag << endl;
    while (motion_run_flag) {
 
        
        
        previous_motion_time = now_motion_time;
        check_previous_motion_time = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3 * (JOINT_NUM)) {
            recv_fail_cnt++;
        }


        if (sys_ready_flag == 0) {
            ServoOn();
            if (ServoState == (1 << JOINT_NUM) - 1) //all servos are in ON state
            {
                if (servo_ready == 0) {
                    servo_ready = 1;
                }
            }
            if (servo_ready) {
                ready_cnt++;
            }
            if (ready_cnt >= 5000) {
                sys_ready_flag = 1;
            }///
        } else { // realtime action...
            EncoderRead();
            //GetActualData();

            switch (PongBot.ControlMode) {
                case CTRLMODE_NONE:
                    //cout << "============= [CTRLMODE_NONE] ==========" << endl;
                    //PongBot.CommandFlag = TORQUE_OFF;
                    //PongBot.CommandFlag = NO_ACT;
                    break;

                case CTRLMODE_WALK_READY:
                    cout << "============= [CTRLMODE_WALK_READY] ==========" << endl;

                    PongBot.wr_cnt = 0;
                    PongBot.com.des_pos = PongBot.com.init_pos;
                    PongBot.com.des_vel << 0, 0, 0;
                    PongBot.base.des_pos = PongBot.com.des_pos - PongBot.com.offset;
                    PongBot.base.des_vel << 0, 0, 0;
                    PongBot.base.des_Euler_Ang << 0, 0, 0;
                    PongBot.base.des_Ang_Vel << 0, 0, 0;
                    
//                    nmpc_run_flag = true;
                    
                    PongBot.CommandFlag = GOTO_WALK_READY_POS;
                    PongBot.ControlMode = CTRLMODE_NONE;

                    break;

               

                case CTRLMODE_PRONK_JUMP:
                    //cout << "============= [CTRLMODE_PRONK_JUMP] ==========" << endl;
                    //                    if (PongBot.walk_ready_moving_done_flag == true) {
                    //                        PongBot.pr_cnt = 0;
                    //                        PongBot.JUMP_PHASE = 0;
                    //                        PongBot.jump_num = 0;
                    //                        PongBot.move_cnt = 0;
                    //                        PongBot.moving_done_flag = false;
                    //                        PongBot.Robot_para_init();
                    //                        //
                    //                        PongBot.CommandFlag = PRONK_JUMP;
                    //                    } else {
                    //                        cout << " ======== not yet walk ready ======== " << endl;
                    //                    }
                    PongBot.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_TEST:


                    PongBot.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_TORQUE_OFF:
                    //cout << "============= [CTRLMODE_TORQUE_OFF] ==========" << endl;
                    PongBot.CommandFlag = TORQUE_OFF;
                    PongBot.ControlMode = CTRLMODE_NONE;
                    break;

            }

            switch (PongBot.CommandFlag) {
                case NO_ACT:
                    // No action
                    break;

                case TORQUE_OFF:
                    PongBot.Torque_off();
                    break;

                case GOTO_HOME_POS:
                    PongBot.Torque_off();
                    //PongBot.Init_Pos_Traj(); // Only simulation
                    break;

                case GOTO_WALK_READY_POS:
                    PongBot.StateUpdate();
                    PongBot.WalkReady_Pos_Traj();
                    PongBot.ComputeTorqueControl();
                    
                    DataSave();
                    
                    break;

                case PRONK_JUMP:
                    
                    break;

                case TEST_FLAG:
                   
                    break;

            }
            TargetTor_Gen();
        }

        
        //============   Thread Time Checking   ============//
        now_motion_time = rt_timer_read();
        check_now_motion_time = rt_timer_read();

        check_motion_time = (double) (check_now_motion_time - check_previous_motion_time) / 1000000;
        motion_time = (double) (now_motion_time - previous_motion_time) / 1000000;

        motion_time_set(1) = check_motion_time;
        motion_time_set = Max_Value_Save(motion_time_set);
        //==================================================//
        
        
        
        PongBot.NMPC_thread_cnt++;
        
        rt_task_wait_period(NULL);
    }
    
    
    for (int i = 0; i < JOINT_NUM; ++i) {
        ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    rt_task_sleep(cycle_ns);

    //rt_printf("End simple test, close socket\n");
    /* stop SOEM, close socket */
    printf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    ec_close();
}

void print_run(void* arg) {

    //rt_task_set_periodic(NULL, TM_NOW, 100000000);
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 100);

    //rt_printf("Print Thread Start \n");

    while (print_run_flag) {
        rt_task_wait_period(NULL);
        previous_print_time = now_print_time;
        //        rt_mutex_acquire(&mutex_desc, TM_INFINITE);
//        inOP = TRUE;     //ddddddddd
//        sys_ready_flag = 1;
        if (inOP == TRUE) {
            if (!sys_ready_flag) {
                if (stick == 0)
                    rt_printf("waiting for system ready...\n");
                if (stick % 10 == 0)
                    rt_printf("%i\n", stick / 10);
                stick++;
            } else {
//                rt_printf("=============================[Thread Time]=================================\n");
//                rt_printf("Thread_time : (%f [ms] / %f [ms]) / (%f [ms]) / (%f [ms] / %f [ms]) / (%f [ms] / %f [ms])\n", check_motion_time, motion_time, print_time, check_imu_time, imu_time, check_QP_time, QP_time);
//                rt_printf("Motion Time(Max) : %f [ms] / IMU Time(Max) : %f [ms]\n", motion_time_set(0), IMU_time_set(0));
                //                rt_printf("=============================[Elmo Status]=================================\n");
               //rt_printf("Status word = (0x%X / 0x%X / 0x%X),(0x%X / 0x%X / 0x%X),(0x%X / 0x%X / 0x%X),(0x%X / 0x%X / 0x%X) \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord, ELMO_drive_pt[5].ptInParam->StatusWord, ELMO_drive_pt[4].ptInParam->StatusWord, ELMO_drive_pt[3].ptInParam->StatusWord, ELMO_drive_pt[6].ptInParam->StatusWord, ELMO_drive_pt[7].ptInParam->StatusWord, ELMO_drive_pt[8].ptInParam->StatusWord, ELMO_drive_pt[11].ptInParam->StatusWord, ELMO_drive_pt[10].ptInParam->StatusWord, ELMO_drive_pt[9].ptInParam->StatusWord);
                //rt_printf("Status word = (0x%X)", ELMO_drive_pt[0].ptInParam->StatusWord);
                //rt_printf("Actual Torque = (%d / %d / %d),(%d / %d / %d),(%d / %d / %d),(%d / %d / %d)\n", ELMO_drive_pt[0].ptInParam->TorqueActualValue, ELMO_drive_pt[1].ptInParam->TorqueActualValue, ELMO_drive_pt[2].ptInParam->TorqueActualValue, ELMO_drive_pt[5].ptInParam->TorqueActualValue, ELMO_drive_pt[4].ptInParam->TorqueActualValue, ELMO_drive_pt[3].ptInParam->TorqueActualValue, ELMO_drive_pt[6].ptInParam->TorqueActualValue, ELMO_drive_pt[7].ptInParam->TorqueActualValue, ELMO_drive_pt[8].ptInParam->TorqueActualValue, ELMO_drive_pt[11].ptInParam->TorqueActualValue, ELMO_drive_pt[10].ptInParam->TorqueActualValue, ELMO_drive_pt[9].ptInParam->TorqueActualValue);
//                rt_printf("=============================[Sensor]=================================\n");
                //                ////
//                rt_printf("----------------------------------------------------\n");
            }
        }
        // rt_mutex_release(&mutex_desc);
        now_print_time = rt_timer_read();
        print_time = (double) (now_print_time - previous_print_time) / 1000000;
    }
}

void nmpc_run(void* arg) {

    //rt_task_set_periodic(NULL, TM_NOW, 100000000);
//    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * PongBot.Ns);
    
    
    
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 100);

//    cout << "nmpc_run_flag = " << nmpc_run_flag << endl;
    //rt_printf("Print Thread Start \n");

    while (1) {
        
//        cout << "nmpc run" << endl;

        check_previous_nmpc_time = rt_timer_read();
        previous_nmpc_time = now_nmpc_time;
        
        if(PongBot.nmpc_run_flag == true){

            PongBot.NMPC_Process();

            PongBot.NMPC_thread_cnt = 0;
        }
//        //============   Thread Time Checking   ============//
//        now_motion_time = rt_timer_read();
//        check_now_motion_time = rt_timer_read();
//
//        check_motion_time = (double) (check_now_motion_time - check_previous_motion_time) / 1000000;
//        motion_time = (double) (now_motion_time - previous_motion_time) / 1000000;
//
//        motion_time_set(1) = check_motion_time;
//        motion_time_set = Max_Value_Save(motion_time_set);
//        //==================================================//
        
        
        
        now_nmpc_time = rt_timer_read();
        check_now_nmpc_time = now_nmpc_time;
        
        check_nmpc_time = (double) (check_now_nmpc_time - check_previous_nmpc_time) / 1000000;
        nmpc_time = (double) (now_nmpc_time - previous_nmpc_time) / 1000000;
        
//        cout << "nmpc_time = " << nmpc_time << endl;
        
        rt_task_wait_period(NULL);
    }
}

void signal_handler(int sig) {

    printf("Program END...\n");

//    cout << "[1]" << endl;
    rt_task_delete(&IMU_task);
    close(fd_MIC);

//    cout << "[2]" << endl;
    rt_task_delete(&PongBot_task);

//    cout << "[3]" << endl;
    rt_task_delete(&Print_task);

//    cout << "[4]" << endl;
    rt_task_delete(&QP_task);
    
    rt_task_delete(&NMPC_task);

    FileSave();

    ros::shutdown();
    exit(0);
}

void ServoOn(void) {
    //servo-on
    for (int i = 0; i < JOINT_NUM; i++) {
        controlword = 0;
        started[i] = ServoOn_GetCtrlWrd(ELMO_drive_pt[i].ptInParam->StatusWord, &controlword);
        ELMO_drive_pt[i].ptOutParam->ControlWord = controlword;
        if (started[i]) ServoState |= (1 << i);
    }
}

void ServoOff(void) {
    //printf("function is Servo_off");
    //Servo OFF
    for (int i = 0; i < JOINT_NUM; i++) {
        ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
    }
}

double Count2Rad_ABS(int _Resolution, INT32 Count) {
    double th = (double) Count * 2 * PI / (_Resolution);
    return th;
}

double Count2RadDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 2 * PI / (2048 * Gear_Ratio);
    //double th_dot = (double) CountPerSec * 2 * PI / (36 * Gear_Ratio);
    return th_dot; // [rad]
}

void EncoderRead(void) {

    for (int i = 0; i < JOINT_NUM; i++) {
        Low_ABS_actual_pos[i] = Count2Rad_ABS(Low_Resolution[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue);
        //PongBot.Low_Incre_actual_pos[i] = PongBot.Count2Rad(PongBot.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
        Low_actual_vel[i] = Count2RadDot(Low_Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
    }
    //    for (int i = 0; i <3; ++i){
    //        PongBot.Incre_actual_joint_pos[i] = PongBot.Raw_Incre_actual_joint_pos[i];
    //    }
    //    for (int i = 3; i <6; ++i){
    //        PongBot.Incre_actual_joint_pos[i] = PongBot.Raw_Incre_actual_joint_pos[8-i];
    //    }
    //    for (int i = 6; i <9; ++i){
    //        PongBot.Incre_actual_joint_pos[i] = PongBot.Raw_Incre_actual_joint_pos[i];
    //    }
    //    for (int i = 9; i <12; ++i){
    //        PongBot.Incre_actual_joint_pos[i] = PongBot.Raw_Incre_actual_joint_pos[20-i];
    //    }

    for (int i = 0; i < 3; ++i) {
        PongBot.joint->act_vel[i] = Low_actual_vel[i];
        if (i == 2) {
            PongBot.joint->act_vel[i] = Low_actual_vel[i]*1.5;
        }
    }
    for (int i = 3; i < 6; ++i) {
        PongBot.joint->act_vel[i] = Low_actual_vel[8 - i];
        if (i == 5) {
            PongBot.joint->act_vel[i] = Low_actual_vel[8 - i]*1.5;
        }
    }
    for (int i = 6; i < 9; ++i) {
        PongBot.joint->act_vel[i] = Low_actual_vel[i];
        if (i == 8) {
            PongBot.joint->act_vel[i] = Low_actual_vel[i]*1.5;
        }
    }
    for (int i = 9; i < 12; ++i) {
        PongBot.joint->act_vel[i] = Low_actual_vel[20 - i];
        if (i == 11) {
            PongBot.joint->act_vel[i] = Low_actual_vel[20 - i]*1.5;
        }
    }

    // if (PongBot.Encoder_Reset_Flag == true) {

    for (int i = 0; i < 3; ++i) {
        ABS_actual_pos[i] = Low_ABS_actual_pos[i];
    }
    for (int i = 3; i < 6; ++i) {
        ABS_actual_pos[i] = Low_ABS_actual_pos[8 - i];
    }
    for (int i = 6; i < 9; ++i) {
        ABS_actual_pos[i] = Low_ABS_actual_pos[i];
    }
    for (int i = 9; i < 12; ++i) {
        ABS_actual_pos[i] = Low_ABS_actual_pos[20 - i];
    }

    for (int i = 0; i < JOINT_NUM; ++i) {
        PongBot.joint->act_pos[i] = ABS_actual_pos[i];
    }
}

void Load(void) {
    rbdl_check_api_version(RBDL_API_VERSION);
    version_test = rbdl_get_api_version();

    printf("rbdl api version = %d\n", version_test);
    Addons::URDFReadFromFile("/home/rclab/catkin_ws/src/RcLab-PongBot_V1/model/PONGBOT_Q_V6/urdf/PONGBOT_Q_V6.urdf", PB_model, true, false);
    PongBot.setRobotModel(PB_model);

    //IMU_COM_Setting();

    PongBot.ControlMode = CTRLMODE_NONE; //=0
    //PongBot.CommandFlag = TORQUE_OFF;
    PongBot.CommandFlag = NO_ACT;

}

INT16 Tor2Cur(double OutputTorque, double _Kt, int _Gear, double _ratedCur) {
    INT16 inputCurrent = (OutputTorque / _Gear) / _Kt / _ratedCur * 1000;

    return inputCurrent;
}

void TargetTor_Gen(void) {

    static int limit_tor = 3000; //3000;

    //*********** Arrange Target Torque in Elmo's order *************//
    Low_TargetTor[0] = PongBot.joint->torque[0]; //HR joint
    Low_TargetTor[1] = PongBot.joint->torque[1]; //HP joint
    Low_TargetTor[2] = PongBot.joint->torque[2] * 1.5; //Knee joint

    Low_TargetTor[5] = PongBot.joint->torque[3]; //HR joint
    Low_TargetTor[4] = PongBot.joint->torque[4]; //HP joint
    Low_TargetTor[3] = PongBot.joint->torque[5] * 1.5; //Knee joint

    Low_TargetTor[6] = PongBot.joint->torque[6]; //HR joint
    Low_TargetTor[7] = PongBot.joint->torque[7]; //HP joint
    Low_TargetTor[8] = PongBot.joint->torque[8] * 1.5; //Knee joint

    Low_TargetTor[11] = PongBot.joint->torque[9]; //HR joint
    Low_TargetTor[10] = PongBot.joint->torque[10]; //HP joint
    Low_TargetTor[9]  = PongBot.joint->torque[11] * 1.5; //Knee joint
    //****************************************************************//

    for (int i = 0; i < JOINT_NUM; i++) {
        if (Low_TargetTor[i] >= limit_tor) {
            Low_TargetTor[i] = limit_tor;
            printf("%d 's torque > %d\n", i, limit_tor);
        } else if (Low_TargetTor[i] <= -limit_tor) {
            Low_TargetTor[i] = -limit_tor;
            printf("%d 's torque <= -%d\n", i, limit_tor);
        }
    }


    for (int i = 0; i < JOINT_NUM; i++) {
        ELMO_drive_pt[i].ptOutParam->TargetTorque = Tor2Cur(Low_TargetTor[i], Low_Kt[i], Low_Gear[i], Low_ratedCur[i]);
    }
    
//    cout<<ELMO_drive_pt[5].ptInParam->PositionActualValue<<endl;
//    cout <<"TorqueActualValue = "<< ELMO_drive_pt[5].ptInParam->TorqueActualValue<<endl;
//    cout << Low_TargetTor[5] << "/" << Low_TargetTor[4] << "/" << Low_TargetTor[3] << endl;
//    cout << ELMO_drive_pt[5].ptOutParam->TargetTorque << "/" << ELMO_drive_pt[4].ptOutParam->TargetTorque << "/" << ELMO_drive_pt[3].ptOutParam->TargetTorque << endl;
//    cout << "-------" << endl;
}

void JoystickCallback(const sensor_msgs::Joy& msg) {
    const double max_x_vel = 0.3; //1.0;
    const double max_y_vel = 0.2;
    const double max_yaw_ori = 5 * D2R; // rad
    static double tmp_y_vel = 0;

    if ((int) msg.buttons[9] == 1) {
        printf("======================== [Torque Off] ==========================\n");
        PongBot.ControlMode = CTRLMODE_TORQUE_OFF;
    } else if ((int) msg.buttons[13] == 1) {
        if (PongBot.move_done_flag == true) {
            printf("======================== [Walk Ready] ==========================\n");
            PongBot.ControlMode = CTRLMODE_WALK_READY;
        }
    } 
//    else if ((int) msg.buttons[2] == 1) {
//        printf("======================== [Move Stop] ==========================\n");
//        PongBot.move_stop_flag = true;
//        if (PongBot.pre_sub_ctrl_flag_HS == true) {
//            PongBot.move_stop_flag = false;
//        }
//    } else if ((int) msg.buttons[12] == 1) {
//        printf("======================== [HS's Walk Mode] ==========================\n");
//        if (PongBot.moving_done_flag == true) {
//            PongBot.ControlMode = CTRLMODE_SLOW_WALK_HS;
//            PongBot.move_stop_flag = false;
//        }
//    }

//    tmp_y_vel = -(double) msg.axes[0] / 32767.0 * max_y_vel;
//    if (tmp_y_vel < 0.1 && tmp_y_vel > -0.1) {
//        PongBot.tmp_y_moving_speed = 0;
//    } else {
//        PongBot.tmp_y_moving_speed = tmp_y_vel;
//    }
//    PongBot.tmp_x_moving_speed = -(double) msg.axes[1] / 32767.0 * max_x_vel;
//    PongBot.tmp_base_ori(2) = -(double) msg.axes[2] / 32767.0 * max_yaw_ori;

//    PongBot.speed_x_HS = -(double) msg.axes[1] / 32767.0 * 0.06;
//    PongBot.speed_y_HS = -(double) msg.axes[0] / 32767.0 * 0.04;
//    PongBot.speed_yaw_HS = -(double) msg.axes[2] / 32767.0 * 4.0 * (PI / 180);
}

void ROSMsgPublish(void) {
    tf::TransformBroadcaster broadcaster;

    m_data.data[0] = m_time;
    m_data.data[1] = PongBot.IMURoll;
    m_data.data[2] = PongBot.IMUPitch;
    m_data.data[3] = PongBot.IMUYaw;
    m_data.data[4] = imu_time;
    m_data.data[5] = check_imu_time;
    m_data.data[6] = PongBot.RL.des_pos_global_now[0];
    m_data.data[7] = PongBot.RL.des_pos_global_now[1];
    m_data.data[8] = PongBot.RL.des_pos_global_now[2];
    
    P_data.publish(m_data);

}

void DataSave(void) {
    
    m_time = m_time + PongBot.dt;
    
    save_array[save_cnt][0] = m_time;
    save_array[save_cnt][1] = check_motion_time;
    save_array[save_cnt][2] = motion_time;
    save_array[save_cnt][3] = check_nmpc_time;
    save_array[save_cnt][4] = nmpc_time;
    save_array[save_cnt][5] = PongBot.RL.des_pos_global_now[0];
    save_array[save_cnt][6] = PongBot.RL.des_pos_global_now[1];
    save_array[save_cnt][7] = PongBot.RL.des_pos_global_now[2];
    
    //    save_array[save_cnt][2] = PongBot.actual_joint_pos_HS[0] * R2D;
    //    save_array[save_cnt][3] = PongBot.actual_joint_pos_HS[1] * R2D;
    //    save_array[save_cnt][4] = PongBot.actual_joint_pos_HS[2] * R2D;
    //    save_array[save_cnt][5] = PongBot.target_joint_pos_HS[0] * R2D;
    //    save_array[save_cnt][6] = PongBot.target_joint_pos_HS[1] * R2D;
    //    save_array[save_cnt][7] = PongBot.target_joint_pos_HS[2] * R2D;
    //    for (int i=0;i<48;i++){
    //     save_array[save_cnt][i] = Receive_MIC_buf[i];
    //    }


    //    save_array[save_cnt][3] = ELMO_drive_pt[0].ptInParam->StatusWord;
    //    save_array[save_cnt][4] = ELMO_drive_pt[1].ptInParam->StatusWord;
    //    save_array[save_cnt][5] = ELMO_drive_pt[2].ptInParam->StatusWord;
    //    save_array[save_cnt][6] = ELMO_drive_pt[0].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][7] = ELMO_drive_pt[1].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][8] = ELMO_drive_pt[2].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][3] = PongBot.actual_joint_pos[0] * R2D;
    //    save_array[save_cnt][4] = PongBot.actual_joint_pos[1] * R2D;
    //    save_array[save_cnt][5] = PongBot.actual_joint_pos[2] * R2D;

    if (save_cnt < SAVE_COUNT - 1) {
        save_cnt++;
    }
}

void FileSave(void) {
    FILE *fp;

    fp = fopen("Data.txt", "w");

    for (int j = 0; j < SAVE_COUNT; ++j) {
        for (int i = 0; i < SAVE_LENGTH-1; ++i) {
            fprintf(fp, "%f \t", (double) save_array[j][i]);
            //            fprintf(fp, "%c\t", save_array[j][i]);
        }
        fprintf(fp, "%f \n", (double) save_array[j][SAVE_LENGTH-1]);
//        fprintf(fp, "%f\n", (double) save_array[j][SAVE_LENGTH - 1]);
        //fprintf(fp, "%c\n", save_array[j][SAVE_LENGTH - 1]);
    }
    
    //		for(unsigned int i = 0; i < SAVE_LENGTH-1 ; ++i){
//			fprintf(fp,"%f\t",save[j][i]);
//		}
//		fprintf(fp,"%f\n",save[j][SAVE_LENGTH-1]);

    fclose(fp);
    
    cout << "=========== Data Save Done ===========" << endl << endl;
}


bool ecat_init(void) {

    needlf = FALSE;
    inOP = FALSE;

//    printf("Starting simple test\n");

    if (ec_init(ecat_ifname)) {

        printf("ec_init on %s succeeded.\n", ecat_ifname);

        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);
            for (int i = 0; i < JOINT_NUM; ++i) {
                printf("Has CA? %d %s\n", i + 1, ec_slave[i + 1].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
            }

            for (int i = 1; i < JOINT_NUM; ++i) {
                ec_slave[i + 1].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            // PDO re-mapping //
            for (int k = 0; k < JOINT_NUM; ++k) {
                if (ec_slavecount >= 1) {
                    wkc += RS3_write8(k + 1, 0x1c12, 0x0000, 0x00); //(slave, index, subindex, value)
                    wkc += RS3_write8(k + 1, 0x1608, 0x0000, 0x00);

                    wkc += RS3_write32(k + 1, 0x1608, 0x0001, 0x607A0020);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0002, 0x60FF0020);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0003, 0x60710010);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0004, 0x60720010);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0005, 0x60400010);
                    wkc += RS3_write32(k + 1, 0x1608, 0x0006, 0x60600008);
                    wkc += RS3_write8(k + 1, 0x1608, 0x0000, 0x06);


                    wkc += RS3_write16(k + 1, 0x1c12, 0x0001, 0x1608); //  (row,PDO)
                    wkc += RS3_write8(k + 1, 0x1c12, 0x0000, 0x01); //  (index,Row)

                    ////////////////////////////////////////////////////////////////////////////

                    wkc += RS3_write8(k + 1, 0x1c13, 0x0000, 0x00); //(slave, index, subindex, value)
                    wkc += RS3_write8(k + 1, 0x1a07, 0x0000, 0x00);

                    wkc += RS3_write32(k + 1, 0x1a07, 0x0001, 0x60640020);
                    //wkc += RS3_write32(k + 1, 0x1a07, 0x0002, 0x60FD0020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0002, 0x606C0020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0003, 0x60410010);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0004, 0x60770010);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0005, 0x20a00020);
                    wkc += RS3_write8(k + 1, 0x1a07, 0x0000, 0x05);

                    wkc += RS3_write16(k + 1, 0x1c13, 0x0001, 0x1a07); //  (row,PDO)
                    wkc += RS3_write8(k + 1, 0x1c13, 0x0000, 0x01); //  (index,Row)
                }
            }

            ec_config_map(&IOmap);

            printf("Slaves mapped, state to SAFE_OP.\n");
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            slaveinfo(ecat_ifname);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            ec_writestate(0);
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP

            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;

                for (int k = 0; k < JOINT_NUM; ++k) {
                    ELMO_drive_pt[k].ptOutParam = (ELMO_DRIVE_RxPDO_t*) ec_slave[k + 1].outputs;
                    ELMO_drive_pt[k].ptInParam = (ELMO_DRIVE_TxPDO_t*) ec_slave[k + 1].inputs;
                    ELMO_drive_pt[k].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
                    ELMO_drive_pt[k].ptOutParam->MaxTorque = maxTorque;

                }
                inOP = TRUE;
            } else {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; ++i) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
                for (int i = 0; i < JOINT_NUM; ++i)
                    ec_dcsync01(i + 1, FALSE, 0, 0, 0); // SYNC0,1
            }

        } else {
            printf("No slaves found!\n");
            inOP = FALSE;
        }

    } else {
        printf("No socket connection on %s\nExcecute as root\n", ecat_ifname);
        return FALSE;
    }

    return inOP;
}

int IMU_COM_Setting() {
    printf(" IMU Setting ... \n");
    struct termios oldtio, newtio;

    //fd_MIC = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    fd_MIC = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
    //fd_MIC = open("/dev/IMU_COM", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
    //fd_MIC = open("/dev/IMU_COM", O_RDWR | O_NOCTTY);

    //fd_MIC = open("/dev/IMU_COM", O_RDWR);
    // dev/ttyACM0
    // O_RDWF : Use write/read mode
    // O_NOCCTY : No Controlling tty
    // O_NONBLOCK :
    if (fd_MIC < 0) {
        perror("/dev/ttyACM0");
        exit(-1);
    }
    tcgetattr(fd_MIC, &oldtio); // get current port's setting.
    bzero(&newtio, sizeof (newtio)); // clear new port's setting.
    //memset(&newtio, 0, sizeof (newtio)); // clear new port's setting.

    //newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    //newtio.c_cflag = B230400 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    //newtio.c_cflag = B460800 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    // Control mode Setting(speed = 921600, letter size = 8bits, neglect modem line, allow to receive)
    newtio.c_cflag = B921600 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능

    // Input mode Setting(neglect partiy error, transform CR to NL)
    newtio.c_iflag = IGNPAR | ICRNL; //Parity 오류가 있는 문자 무시

    // Output mode Setting(No setting)
    newtio.c_oflag = 0; //출력처리 설정 0이면 아무것도 안함

    // Local mode Setting(No setting)
    newtio.c_lflag = 0; //Local Mode 설정, ICANON이면 널 문자가 들어올때까지 수신(non-canonical mode)

    //Control Characters
    newtio.c_cc[VINTR] = 0;
    newtio.c_cc[VQUIT] = 0;
    newtio.c_cc[VERASE] = 0;
    newtio.c_cc[VKILL] = 0;
    newtio.c_cc[VEOF] = 4;
    newtio.c_cc[VTIME] = 0; //time-out 값으로 사용, time-out 값은 TIME*0.1초
    //newtio.c_cc[VTIME] = 100; //time-out 값으로 사용, time-out 값은 TIME*0.1초

    newtio.c_cc[VMIN] = 1; //read가 리턴되기 위한 최소한의 문자 개수
    //newtio.c_cc[VMIN] = 0; //read가 리턴되기 위한 최소한의 문자 개수

    newtio.c_cc[VSWTC] = 0;
    newtio.c_cc[VSTART] = 0;
    newtio.c_cc[VSTOP] = 0;
    newtio.c_cc[VSUSP] = 0;
    newtio.c_cc[VEOL] = 0;
    newtio.c_cc[VREPRINT] = 0;
    newtio.c_cc[VDISCARD] = 0;
    newtio.c_cc[VWERASE] = 0;
    newtio.c_cc[VLNEXT] = 0;
    newtio.c_cc[VEOL2] = 0;

    tcflush(fd_MIC, TCIFLUSH); //설정을 초기화
    tcsetattr(fd_MIC, TCSANOW, &newtio); //newtio 터미널 구조체의 속정으로 설정을 적용

    imu_run_flag = 1;
    //    fd_MIC=rt_dev_open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    //cout << "fd_MIC =" << fd_MIC << endl;
    return 1;
}

void imu_run(void *arg) {
    //unsigned int MIC_length = 20;
    unsigned int MIC_length = 20;
    //unsigned char Receive_MIC_buf[MIC_length];

//    unsigned char Send_MIC_buf[13];
//    unsigned char Read_MIC_buf[30];

    unsigned char tmp_Receive_MIC_buf[1];
    unsigned char Receive_MIC_buf[MIC_length];
    //unsigned char tmp_Receive_MIC_buf[5];
    //rt_task_set_periodic(NULL, TM_NOW, 0.5*cycle_ns);
    //    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns);
    //rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns);

    //rt_task_set_periodic(NULL, TM_NOW, 2 * cycle_ns);
    rt_task_set_periodic(NULL, TM_NOW, 1 * cycle_ns);
    //rt_task_set_periodic(NULL, TM_NOW, 5*cycle_ns); //소
    //rt_task_set_periodic(NULL, TM_NOW, 10*cycle_ns);
    //rt_task_set_periodic(NULL, TM_NOW, 20*cycle_ns);

    uint32_t roll_MIC_f = 0;
    uint32_t pitch_MIC_f = 0;
    uint32_t yaw_MIC_f = 0;

    uint32_t Gyro_X_MIC_f = 0;
    uint32_t Gyro_Y_MIC_f = 0;
    uint32_t Gyro_Z_MIC_f = 0;

    uint32_t Acc_X_MIC_f = 0;
    uint32_t Acc_Y_MIC_f = 0;
    uint32_t Acc_Z_MIC_f = 0;

    float roll_MIC = 0, pitch_MIC = 0, yaw_MIC = 0;
    float Gyro_X_MIC = 0, Gyro_Y_MIC = 0, Gyro_Z_MIC = 0;
    float Acc_X_MIC = 0, Acc_Y_MIC = 0, Acc_Z_MIC = 0;

    const double PI_IMU = 3.1415927f;

    int mic_flag1 = 0;
    int data_start = 0;

    IMU_COM_Setting();

    memset(Receive_MIC_buf, 0, MIC_length);

    while (imu_run_flag) {
        rt_task_wait_period(NULL);
        uint8_t checksum_byte1 = 0, checksum_byte2 = 0;

        previous_imu_time = now_imu_time;
        check_previous_imu_time = rt_timer_read();

//        while(read(fd_MIC, tmp_Receive_MIC_buf, 1)>0){
//
//        }
        //cout << "fd_MIC=" << read(fd_MIC, tmp_Receive_MIC_buf, 1) << endl;
        //        while ((double) read(fd_MIC, tmp_Read_MIC_buf, 1) > 0) {
        //            if (tmp_Read_MIC_buf[0] == 0x75) {
        //                read(fd_MIC, Read_MIC_buf + 1, 30 - 1);
        //                Read_MIC_buf[0] = tmp_Read_MIC_buf[0];
        //                for (int i = 0; i < 30; i++) {
        //                    rt_printf("0x%X /", Read_MIC_buf[i]);
        //                }
        //                rt_printf("\n");
        //            }
        //        }
        //tcflush(fd_MIC, TCIFLUSH);
        cout <<"flush="<< tcflush(fd_MIC, TCIOFLUSH) << endl;

        now_imu_time = rt_timer_read();
        check_now_imu_time = rt_timer_read();
        //
        imu_time = (double) (now_imu_time - previous_imu_time) / 1000000;
        check_imu_time = (double) (check_now_imu_time - check_previous_imu_time) / 1000000;

        //tcflush(fd_MIC,TCIFLUSH);
    }
    //tcsetattr(fd_MIC,TCSANOW,&oldtio);
}

//void imu_run(void *arg) {
//    unsigned int MIC_length = 48;
//    unsigned char Receive_MIC_buf[MIC_length];
//
//    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns);
//    //rt_task_set_periodic(NULL, TM_NOW, 2 * cycle_ns);
//    //rt_task_set_periodic(NULL, TM_NOW, 4*cycle_ns);
//    //rt_task_set_periodic(NULL, TM_NOW, 5*cycle_ns); //소
//    //rt_task_set_periodic(NULL, TM_NOW, 10*cycle_ns);
//    //rt_task_set_periodic(NULL, TM_NOW, 20*cycle_ns);
//
//    uint32_t roll_MIC_f = 0;
//    uint32_t pitch_MIC_f = 0;
//    uint32_t yaw_MIC_f = 0;
//
//    uint32_t Gyro_X_MIC_f = 0;
//    uint32_t Gyro_Y_MIC_f = 0;
//    uint32_t Gyro_Z_MIC_f = 0;
//
//    uint32_t Acc_X_MIC_f = 0;
//    uint32_t Acc_Y_MIC_f = 0;
//    uint32_t Acc_Z_MIC_f = 0;
//
//    float roll_MIC = 0, pitch_MIC = 0, yaw_MIC = 0;
//    float Gyro_X_MIC = 0, Gyro_Y_MIC = 0, Gyro_Z_MIC = 0;
//    float Acc_X_MIC = 0, Acc_Y_MIC = 0, Acc_Z_MIC = 0;
//
//    const double PI_IMU = 3.1415927f;
//    int cnt_over1 = 0;
//    int mic_flag1 = 0;
//    int buf_index = 0;
//    int data_start = 0;
//
//    memset(Receive_MIC_buf, 0, MIC_length);
//
//    while (imu_run_flag) {
//        rt_task_wait_period(NULL);
//        previous_imu_time = now_imu_time;
//        check_previous_imu_time = rt_timer_read();
//        //         memset(Receive_MIC_buf, 0, MIC_length);
//
//        uint8_t checksum_byte1 = 0, checksum_byte2 = 0;
//
//        if (mic_flag1 == 0) {
//            for (int j = 0; j < MIC_length; ++j) {
//                read(fd_MIC, Receive_MIC_buf + j, 1);
//                if (j < MIC_length - 2) {
//                    checksum_byte1 += Receive_MIC_buf[j];
//                    checksum_byte2 += checksum_byte1;
//                }
//            } //rt_printf("\n");
//        }
//
//        //
//        //        uint16_t checksum = 0;
//        //        for (int i = 0; i < MIC_length - 2; i++) {
//        //
//        //        }
//
//        tcflush(fd_MIC, TCIFLUSH);
//        //tcflush(fd_MIC, TCIOFLUSH);
//        //rt_printf("\n");
//        //rt_printf("flush = %d\n",flush_check);
//        //        if(flush_check!=0){
//        //            rt_printf("Flush Error!\n");
//        //        }
//
//        //        for(int i=0;i<MIC_length;i++){
//        //            rt_printf("0x%X /", Receive_MIC_buf[i]);
//        //            if (i % 10 == 9) {
//        //                rt_printf("\n");
//        //            }
//        //        }
//
//        if (Receive_MIC_buf[MIC_length - 2] == checksum_byte1 && Receive_MIC_buf[MIC_length - 1] == checksum_byte2) {
//            if (Receive_MIC_buf[0] == 0x75 && Receive_MIC_buf[1] == 0x65 && Receive_MIC_buf[2] == 0x80 && Receive_MIC_buf[3] == 0x2a && Receive_MIC_buf[4] == 0x0E && Receive_MIC_buf[5] == 0x0C && Receive_MIC_buf[18] == 0x0E && Receive_MIC_buf[19] == 0x05 && Receive_MIC_buf[32] == 0x0E && Receive_MIC_buf[33] == 0x04) {
//                data_start = 6;
//                mic_flag1 = 1;
//                //                                rt_printf("\n");
//                //                                for (int i = 0; i < MIC_length; i++) {
//                //                                    rt_printf("0x%X /", Receive_MIC_buf[i]);
//                //                                    if (i % 10 == 9) {
//                //                                        rt_printf("\n");
//                //                                    }
//                //                                }
//                //                                rt_printf("\n");
//            }
//        } else {
//            n_fail++;
//        }
//        //rt_printf("fail_num=%d\n", n_fail);
//
//        if (mic_flag1 == 1) {
//            roll_MIC_f = Receive_MIC_buf[data_start] << 24 | Receive_MIC_buf[data_start + 1] << 16 | Receive_MIC_buf[data_start + 2] << 8 | Receive_MIC_buf[data_start + 3];
//            pitch_MIC_f = Receive_MIC_buf[data_start + 4] << 24 | Receive_MIC_buf[data_start + 5] << 16 | Receive_MIC_buf[data_start + 6] << 8 | Receive_MIC_buf[data_start + 7];
//            yaw_MIC_f = Receive_MIC_buf[data_start + 8] << 24 | Receive_MIC_buf[data_start + 9] << 16 | Receive_MIC_buf[data_start + 10] << 8 | Receive_MIC_buf[data_start + 11];
//            roll_MIC = *(float*) &roll_MIC_f;
//            roll_MIC = -roll_MIC;
//            pitch_MIC = *(float*) &pitch_MIC_f;
//            pitch_MIC = pitch_MIC;
//            yaw_MIC = *(float*) &yaw_MIC_f;
//            yaw_MIC = -yaw_MIC;
//
//            Gyro_X_MIC_f = Receive_MIC_buf[data_start + 14] << 24 | Receive_MIC_buf[data_start + 15] << 16 | Receive_MIC_buf[data_start + 16] << 8 | Receive_MIC_buf[data_start + 17];
//            Gyro_Y_MIC_f = Receive_MIC_buf[data_start + 18] << 24 | Receive_MIC_buf[data_start + 19] << 16 | Receive_MIC_buf[data_start + 20] << 8 | Receive_MIC_buf[data_start + 21];
//            Gyro_Z_MIC_f = Receive_MIC_buf[data_start + 22] << 24 | Receive_MIC_buf[data_start + 23] << 16 | Receive_MIC_buf[data_start + 24] << 8 | Receive_MIC_buf[data_start + 25];
//            Gyro_X_MIC = *(float*) &Gyro_X_MIC_f;
//            Gyro_X_MIC = -Gyro_X_MIC;
//            Gyro_Y_MIC = *(float*) &Gyro_Y_MIC_f;
//            Gyro_Y_MIC = -Gyro_Y_MIC;
//            Gyro_Z_MIC = *(float*) &Gyro_Z_MIC_f;
//            Gyro_Z_MIC = Gyro_Z_MIC;
//
//            Acc_X_MIC_f = Receive_MIC_buf[data_start + 28] << 24 | Receive_MIC_buf[data_start + 29] << 16 | Receive_MIC_buf[data_start + 30] << 8 | Receive_MIC_buf[data_start + 31];
//            Acc_Y_MIC_f = Receive_MIC_buf[data_start + 32] << 24 | Receive_MIC_buf[data_start + 33] << 16 | Receive_MIC_buf[data_start + 34] << 8 | Receive_MIC_buf[data_start + 35];
//            Acc_Z_MIC_f = Receive_MIC_buf[data_start + 36] << 24 | Receive_MIC_buf[data_start + 37] << 16 | Receive_MIC_buf[data_start + 38] << 8 | Receive_MIC_buf[data_start + 39];
//            Acc_X_MIC = *(float*) &Acc_X_MIC_f;
//            Acc_X_MIC = Acc_X_MIC;
//            Acc_Y_MIC = *(float*) &Acc_Y_MIC_f;
//            Acc_Y_MIC = Acc_Y_MIC;
//            Acc_Z_MIC = *(float*) &Acc_Z_MIC_f;
//            Acc_Z_MIC = Acc_Z_MIC;
//
//            if (roll_MIC >= -180 * D2R && roll_MIC <= 0) {
//                roll_MIC = roll_MIC + 180.0 * D2R;
//            } else if (roll_MIC <= 180 * D2R && roll_MIC >= 0) {
//                roll_MIC = roll_MIC - 180.0 * D2R;
//            }
//
//            PongBot.IMURoll = roll_MIC;
//            PongBot.IMUPitch = pitch_MIC;
//            PongBot.IMUYaw = yaw_MIC;
//            PongBot.IMURoll_dot = Gyro_X_MIC;
//            PongBot.IMUPitch_dot = Gyro_Y_MIC;
//            PongBot.IMUYaw_dot = Gyro_Z_MIC;
//            PongBot.IMUAccX = Acc_X_MIC;
//            PongBot.IMUAccY = Acc_Y_MIC;
//            PongBot.IMUAccZ = Acc_Z_MIC;
//
//            const double max_IMU_Roll = 40 * D2R;
//            const double max_IMU_Pitch = 40 * D2R;
//            const double max_IMU_Roll_dot = 100 * D2R;
//            const double max_IMU_Pitch_dot = 100 * D2R;
//            const double max_IMU_Yaw_dot = 100 * D2R;
//            const double max_IMU_AccX = 15.0;
//            const double max_IMU_AccY = 15.0;
//            const double max_IMU_AccZ = 15.0;
//
//            //=================== [Limits of Angle] =============================//
//            if (PongBot.IMURoll > max_IMU_Roll) {
//                PongBot.IMURoll = max_IMU_Roll;
//            } else if (PongBot.IMURoll < -max_IMU_Roll) {
//                PongBot.IMURoll = -max_IMU_Roll;
//            }
//            if (PongBot.IMUPitch > max_IMU_Pitch) {
//                PongBot.IMUPitch = max_IMU_Pitch;
//            } else if (PongBot.IMUPitch < -max_IMU_Pitch) {
//                PongBot.IMUPitch = -max_IMU_Pitch;
//            }
//
//            //================== [Limits of Angular Velocity] =============================//
//            if (PongBot.IMURoll_dot > max_IMU_Roll_dot) {
//                PongBot.IMURoll_dot = max_IMU_Roll_dot;
//            } else if (PongBot.IMURoll_dot < -max_IMU_Roll_dot) {
//                PongBot.IMURoll_dot = -max_IMU_Roll_dot;
//            }
//            if (PongBot.IMUPitch_dot > max_IMU_Pitch_dot) {
//                PongBot.IMUPitch_dot = max_IMU_Pitch_dot;
//            } else if (PongBot.IMUPitch_dot < -max_IMU_Pitch_dot) {
//                PongBot.IMUPitch_dot = -max_IMU_Pitch_dot;
//            }
//            if (PongBot.IMUYaw_dot > max_IMU_Yaw_dot) {
//                PongBot.IMUYaw_dot = max_IMU_Yaw_dot;
//            } else if (PongBot.IMUYaw_dot < -max_IMU_Yaw_dot) {
//                PongBot.IMUYaw_dot = -max_IMU_Yaw_dot;
//            }
//
//            //================== [Limits of Linear Acceleration ] =============================//
//            if (PongBot.IMUAccX > max_IMU_AccX) {
//                PongBot.IMUAccX = max_IMU_AccX;
//            } else if (PongBot.IMUAccX < -max_IMU_AccX) {
//                PongBot.IMUAccX = -max_IMU_AccX;
//            }
//            if (PongBot.IMUAccY > max_IMU_AccY) {
//                PongBot.IMUAccY = max_IMU_AccY;
//            } else if (PongBot.IMUAccY < -max_IMU_AccY) {
//                PongBot.IMUAccY = -max_IMU_AccY;
//            }
//            if (PongBot.IMUAccZ > max_IMU_AccZ) {
//                PongBot.IMUAccZ = max_IMU_AccZ;
//            } else if (PongBot.IMUAccZ < -max_IMU_AccZ) {
//                PongBot.IMUAccZ = -max_IMU_AccZ;
//            }
//
//
//            roll_set(1) = PongBot.IMURoll;
//            pitch_set(1) = PongBot.IMUPitch;
//            yaw_set(1) = PongBot.IMUYaw;
//            roll_set = Max_Value_Save(roll_set);
//            pitch_set = Max_Value_Save(pitch_set);
//            yaw_set = Max_Value_Save(yaw_set);
//
//            roll_vel_set(1) = PongBot.IMURoll_dot;
//            pitch_vel_set(1) = PongBot.IMUPitch_dot;
//            yaw_vel_set(1) = PongBot.IMUYaw_dot;
//            roll_vel_set = Max_Value_Save(roll_vel_set);
//            pitch_vel_set = Max_Value_Save(pitch_vel_set);
//            yaw_vel_set = Max_Value_Save(yaw_vel_set);
//
//            x_acc_set(1) = PongBot.IMUAccX;
//            y_acc_set(1) = PongBot.IMUAccY;
//            z_acc_set(1) = PongBot.IMUAccZ;
//            x_acc_set = Max_Value_Save(x_acc_set);
//            y_acc_set = Max_Value_Save(y_acc_set);
//            z_acc_set = Max_Value_Save(z_acc_set);
//
////            if (abs(roll_set(1)) > 100000 || abs(pitch_set(1)) > 100000 || abs(yaw_set(1)) > 100000 || abs(roll_vel_set(1)) > 100000 || abs(pitch_vel_set(1)) > 100000 || abs(yaw_vel_set(1)) > 100000 || abs(x_acc_set(1)) > 100000 || abs(y_acc_set(1)) > 100000 || abs(z_acc_set(1)) > 100000) {
////                rt_printf("EEEEE\n");
////                for (int i = 0; i < MIC_length; i++) {
////                    rt_printf("0x%X /", Receive_MIC_buf[i]);
////                    if (i % 10 == 9) {
////                        rt_printf("\n");
////                    }
////                }
////                // rt_printf("0x%4X \n", checksum);
////                rt_printf("\n");
////            }
//
//            mic_flag1 = 0;
//            data_start = 0;
//
//            memset(Receive_MIC_buf, 0, MIC_length);
//        }
//
//        check_now_imu_time = rt_timer_read();
//        now_imu_time = rt_timer_read();
//
//        check_imu_time = (double) (check_now_imu_time - check_previous_imu_time) / 1000000;
//        imu_time = (double) (now_imu_time - previous_imu_time) / 1000000;
//
//        IMU_time_set(1) = check_imu_time;
//        IMU_time_set = Max_Value_Save(IMU_time_set);
//
//        //rt_printf("\n -----Data End--------\n");
//    }
//    //tcsetattr(fd_MIC,TCSANOW,&oldtio);
//}



    void QP_run(void *arg) {
        rt_task_set_periodic(NULL, TM_NOW, 2 * cycle_ns);

        while (QP_run_flag) {
            rt_task_wait_period(NULL);
            check_previous_QP_time = rt_timer_read();
            previous_QP_time = now_QP_time;

            if (sys_ready_flag) {
                if (PongBot.CommandFlag != NO_ACT && PongBot.CommandFlag != TORQUE_OFF) {
                    if (PongBot.CommandFlag == GOTO_SLOW_WALK_POS_HS) {
                        //PongBot.Get_Opt_F_HS3();
                    } else {
//                        PongBot.QP_process();
                    }
                }
            }

            check_now_QP_time = rt_timer_read();
            now_QP_time = rt_timer_read();

            check_QP_time = (double) (check_now_QP_time - check_previous_QP_time) / 1000000;
            QP_time = (double) (now_QP_time - previous_QP_time) / 1000000;

            QP_time_set(1) = check_QP_time;
            QP_time_set = Max_Value_Save(QP_time_set);
        }
    }

    VectorXd Max_Value_Save(VectorXd Value_set) {
        VectorNd new_Value_set(2);
        if (abs(Value_set(0)) < abs(Value_set(1))) {
            Value_set(0) = Value_set(1);
        }
        new_Value_set = Value_set;
        return new_Value_set;
    }

    static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value) {
        int wkc;
        wkc += ec_SDOwrite(slave, index, subindex, FALSE, sizeof (value), &value, EC_TIMEOUTRXM);
        return wkc;
    }

    static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value) {
        int wkc;
        wkc += ec_SDOwrite(slave, index, subindex, FALSE, sizeof (value), &value, EC_TIMEOUTRXM);
        return wkc;
    }

    static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value) {
        int wkc;
        wkc += ec_SDOwrite(slave, index, subindex, FALSE, sizeof (value), &value, EC_TIMEOUTRXM);
        return wkc;
    }
