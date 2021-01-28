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
//#include "osal.h"

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

// *********** RVIZ *************//
#include <sensor_msgs/JointState.h>             //for rviz
#include <geometry_msgs/WrenchStamped.h>        //for rviz
//#include <tf/transform_broadcaster.h>           //for rviz
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

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//*************** 1. Variables ****************/
// Ethercat
//char ecat_ifname[32] = "enp5s0";
char ecat_ifname[32] = "rteth0";
ELMO_Drive_pt ELMO_drive_pt[NUM_OF_ELMO];
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

unsigned int n_fail=0;

// loop (main & thread)
int main_run = 1;
int motion_run = 1;
int print_run = 1;
int ros_run = 1;
int imu_run = 0;

// Servo
int ServoState = 0;
int servo_ready = 0;
int sys_ready = 0;
VectorXd started = VectorXd::Zero(NUM_OF_ELMO);

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

uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;

//// Elmo setting
UINT16 maxTorque = 3500;

//Save
//#define SAVE_LENGTH 13    //The number of data
#define SAVE_LENGTH 48    //The number of data
//#define SAVE_COUNT 3600000 //Save Time = 3600000[ms]=3600[s]
//#define SAVE_COUNT 600000 //Save Time = 3600000[ms]=3600[s]
#define SAVE_COUNT 60000 //Save Time = 3600000[ms]=3600[s]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];
//double save_array[SAVE_COUNT][SAVE_LENGTH];

// ROS Param
ros::Subscriber S_Mode;
ros::Subscriber S_Stop;
ros::Subscriber S_Joystick;
ros::Publisher P_data;
int ros_exit = 0;
std_msgs::Float64MultiArray m_data;

double Thread_time = 0.0;

//Xenomai
RT_TASK RT_task1;
RT_TASK RT_task2;
RT_TASK RT_task3;
//RT_TASK RT_task3;
//RT_MUTEX mutex_desc; //mutex
RTIME now_time_realcheck, previous_time_realcheck; // Ethercat time
RTIME now_time_motion, previous_time_motion; // Thread 1 cycle time
RTIME now_time_print, previous_time_print; // Thread 2 cycle time
RTIME now_time_imu, previous_time_imu; // Thread 2 cycle time
RTIME now_time_imu_check, previous_time_imu_check; // Thread 2 cycle time
double del_time_realcheck = 0.0; // Ethercat time
double del_time_motion = 0.0; // Thread 1 cycle time
double del_time_print = 0.0; // Thread 1 cycle time
double del_time_imu_check = 0.0; // Thread 1 cycle time
double del_time_imu = 0.0; // Thread 1 cycle time

double max_time = 0.0;
//RBDL
int version_test;
Model* pongbot_q_model = new Model();

//CRobot
CRobot PongBotQ;

//Rviz
ros::Publisher P_joint_states;
sensor_msgs::JointState m_joint_states;
//geometry_msgs::TransformStamped odom_trans;


//**********************************************//
//*************** 2. Functions ****************//
void ServoOn(void); // : Servo ON
void ServoOff(void); // : Servo Off
void Torque_Off(void);
void motion_task(void* arg); // : Thread 1
void print_task(void* arg); // : Thread 2
void imu_task(void* arg);
void catch_signal(int sig); // : Catch "Ctrl + C signal"
bool ecat_init(void);

int IMU_initialize();

void Load(void); // : parameter initial setting
void EncoderRead(void); // : q, q_dot Read
void GetActualData(void);
void jointController(void);
void DataSave(void);
void FileSave(void);
void Max_Time_Save(double now_time);

// ROS function
void Callback1(const std_msgs::Int32 &msg);
void Callback2(const std_msgs::Int32 &msg);
void Callback3(const sensor_msgs::Joy& msg);

void ROSMsgPublish(void);
static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);

int main(int argc, char* argv[]) {

    printf(" ROS Setting ...\n");
    ros::init(argc, argv, "elmo_pkgs");
    ros::NodeHandle nh;

    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    S_Mode = nh.subscribe("Mode", 1, Callback1);
    S_Stop = nh.subscribe("STOP", 1, Callback2);
    S_Joystick = nh.subscribe("joy", 1, &Callback3);
    P_data = nh.advertise<std_msgs::Float64MultiArray>("tmp_data", 1);
    P_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    m_data.data.resize(10);
    m_joint_states.name.resize(3);
    m_joint_states.position.resize(3);
    sleep(1);

    printf(" Init main ...\n");
    sleep(1);

    // Display Adapter name
    printf(" Use default adapter %s ...\n", ecat_ifname);
    sleep(1);

    // Initial Setting
    printf(" Load Prams...\n");
    Load();

    // Thread Setting Start
    printf("Create Thread ...\n");
    for (int i = 1; i < 4; ++i) {
        printf("%d...\n", i);
        sleep(1);
    }

    rt_task_create(&RT_task1, "Motion_task", 0, 99, 0);
    rt_task_create(&RT_task2, "Print_task", 0, 85, 0);
    rt_task_create(&RT_task3, "Imu_task", 0, 95, 0);
    
//    rt_task_create(&RT_task1, "Motion_task", 0, 80, 0);
//    rt_task_create(&RT_task2, "Print_task", 0, 99, 0);
//    rt_task_create(&RT_task3, "Imu_task", 0, 85, 0);
    

    rt_task_start(&RT_task1, &motion_task, NULL);
    rt_task_start(&RT_task2, &print_task, NULL);
    rt_task_start(&RT_task3, &imu_task, NULL);

    // Thread Setting End

    ros::Rate loop_rate(1000);
    while (ros::ok()) {

        ROSMsgPublish();

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

void motion_task(void* arg) {
    //printf("Motion Thread Start \n");
    if (ecat_init() == false) {
        motion_run = 0;
    }
    rt_task_sleep(1e6);

#ifdef _USE_DC
    //for dc computation
    long long toff;
    long long cur_DCtime = 0, max_DCtime = 0;
    unsigned long long cur_dc32 = 0, pre_dc32 = 0;
    int32_t shift_time = 380000; //dc event shifted compared to master reference clock
    long long diff_dc32;

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        ec_dcsync0(1 + i, TRUE, cycle_ns, 0); // SYNC0,1 on slave 1
    }

    RTIME cycletime = cycle_ns, cur_time = 0;
    RTIME cur_cycle_cnt = 0, cycle_time;
    RTIME remain_time, dc_remain_time;
    toff = 0;
    RTIME rt_ts;

    //get DC time for first time
    ec_send_processdata();

    cur_time = rt_timer_read(); //get current master time
    cur_cycle_cnt = cur_time / cycle_ns; //calcualte number of cycles has passed
    cycle_time = cur_cycle_cnt * cycle_ns;
    remain_time = cur_time % cycle_ns; //remain time to next cycle, test only

    rt_printf("cycle_cnt=%lld\n", cur_cycle_cnt);
    rt_printf("remain_time=%lld\n", remain_time);

    wkc = ec_receive_processdata(EC_TIMEOUTRET); //get reference DC time

    cur_dc32 = (uint32_t) (ec_DCtime & 0xffffffff); //only consider first 32-bit
    dc_remain_time = cur_dc32 % cycletime; //remain time to next cycle of REF clock, update to master

    //remain time to next cycle of REF clock, update to master
    rt_ts = cycle_time + dc_remain_time; //update master time to REF clock
    rt_printf("dc remain_time=%lld\n", dc_remain_time);
    rt_task_sleep_until(rt_ts);

#else  //nonDC
    ec_send_processdata();
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    //rt_task_set_periodic(NULL, TM_NOW, 100*cycle_ns);

#endif

    while (motion_run) {
        //       rt_mutex_acquire(&mutex_desc, TM_INFINITE);
#ifdef _USE_DC     
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);
#else  
        rt_task_wait_period(NULL);
#endif
        previous_time_motion = now_time_motion;
        previous_time_realcheck = rt_timer_read();

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3 * (NUM_OF_ELMO)) {
            recv_fail_cnt++;
        }

#ifdef _USE_DC    
        cur_dc32 = (uint32_t) (ec_DCtime & 0xffffffff); //use 32-bit only
        if (cur_dc32 > pre_dc32) { //normal case
            diff_dc32 = cur_dc32 - pre_dc32;
        } else { //32-bit data overflow
            diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
        }
        pre_dc32 = cur_dc32;
        cur_DCtime += diff_dc32;
        toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);
        if (cur_DCtime > max_DCtime) {
            max_DCtime = cur_DCtime;
        }
#endif                   
        ready_cnt++;
        if (ready_cnt >= 1000) {
            ready_cnt = 6000;
            sys_ready = 1;
        }

        if (sys_ready == 0) {
            ServoOn();
            if (ServoState == (1 << NUM_OF_ELMO) - 1) //all servos are in ON state
            {
                if (servo_ready == 0) {
                    servo_ready = 1;
                }
            }
            if (servo_ready) {
                ready_cnt++;
            }
            if (ready_cnt >= 5000) {
                sys_ready = 1;
            }
        } else { // realtime action...           
            EncoderRead();
            GetActualData();
            //PongBotQ.ComputeTorqueControl();
            //            PongBotQ.Mode_Change();
            //            switch (PongBotQ.ControlMode) {
            //                case CTRLMODE_NONE: // 0
            //                    //cout << "============= [CTRLMODE NONE] ==========" << endl;
            //                    //PongBotQ.CommandFlag = TORQUE_OFF;
            //                    break;
            //                case CTRLMODE_INITIALIZE: //1
            //                    //cout << "============= [[CTRLMODE INITIALIZES HS] ==========" << endl;  // = Joint Control
            //                    PongBotQ.cnt_HS = 0;
            //                    PongBotQ.ControlMode_print = 1;
            //                    PongBotQ.CommandFlag = GOTO_INIT_POS_HS;
            //                    PongBotQ.ControlMode = CTRLMODE_NONE;
            //                    break;
            //
            //                case CTRLMODE_WALK_READY_HS: // 2
            //                    //cout << "============= [[CTRLMODE WALK READY HS] ==========" << endl;    // = Cartesian Control
            //                    PongBotQ.cnt_HS = 0;
            //                    PongBotQ.ControlMode_print = 2;
            //                    PongBotQ.CommandFlag = GOTO_WALK_READY_POS_HS;
            //                    PongBotQ.ControlMode = CTRLMODE_NONE;
            //                    break;
            //
            //                case CTRLMODE_CYCLE_TEST_HS: // 3
            //                    //cout << "============= [[CTRLMODE CYCLE TEST HS] ==========" << endl;    // = Cartesian Control
            //                    PongBotQ.cnt_HS = 0;
            //                    PongBotQ.ControlMode_print = 3;
            //                    PongBotQ.CommandFlag = GOTO_CYCLE_POS_HS;
            //                    PongBotQ.ControlMode = CTRLMODE_NONE;
            //                    break;
            //                case CTRLMODE_JOYSTICK_HS: // 4
            //                    //cout << "============= [[CTRLMODE JOYSITCK HS] ==========" << endl;    // = Cartesian Control
            //                    PongBotQ.cnt_HS = 0;
            //                    PongBotQ.ControlMode_print = 4;
            //                    PongBotQ.CommandFlag = GOTO_JOYSTICK_POS_HS;
            //                    PongBotQ.ControlMode = CTRLMODE_NONE;
            //                    PongBotQ.JoyMode = JOYMODE_HOME;
            //                    break;
            //
            //            }

            //            switch (PongBotQ.CommandFlag) {
            //
            //                case NO_ACT:
            //                    PongBotQ.target_joint_pos_HS = PongBotQ.actual_joint_pos;
            //                    //                    PongBotQ.target_joint_vel_HS << 0, 0, 0;
            //                    PongBotQ.target_EP_pos_HS = PongBotQ.actual_EP_pos_local;
            //                    //                    PongBotQ.target_EP_vel_HS << 0, 0, 0;
            //                    // PongBotQ.ComputeTorqueControl();
            //
            //                    break;
            //
            //                case GOTO_INIT_POS_HS:
            //                    if (PongBotQ.Mode_Change_flag == true) {
            //                        //                        PongBotQ.Init_Pos_Traj_HS();
            //                    }
            //                    //PongBotQ.ComputeTorqueControl();
            //
            //                    break;
            //
            //                case GOTO_WALK_READY_POS_HS:
            //                    if (PongBotQ.Mode_Change_flag == true) {
            //                        //                        PongBotQ.Home_Pos_Traj_HS();
            //                    }
            //                    //PongBotQ.ComputeTorqueControl();
            //
            //                    break;
            //
            //                case GOTO_CYCLE_POS_HS:
            //                    if (PongBotQ.Mode_Change_flag == true) {
            //                        //                        PongBotQ.Cycle_Test_Pos_Traj_HS();
            //                    }
            //                    //PongBotQ.ComputeTorqueControl();
            //
            //                    break;
            //                case GOTO_JOYSTICK_POS_HS:
            //                    if (PongBotQ.Mode_Change_flag == true) {
            //                        //                        PongBotQ.Joystick_Pos_Traj_HS();
            //                    }
            //                    //PongBotQ.ComputeTorqueControl();
            //
            //                    break;
            //
            //            }
            //jointController();
        }

        now_time_motion = rt_timer_read();
        now_time_realcheck = rt_timer_read();

        del_time_realcheck = (double) (now_time_realcheck - previous_time_realcheck) / 1000000;
        del_time_motion = (double) (now_time_motion - previous_time_motion) / 1000000;
        
        motion_time_set(1) = del_time_realcheck;
        motion_time_set = Max_Value_Save(motion_time_set);
       // DataSave();
    }
    //    rt_task_sleep(cycle_ns);
#ifdef _USE_DC
    for (int i = 0; i < NUM_OF_ELMO; ++i)
        ec_dcsync0(i + 1, FALSE, 0, 0); // SYNC0,1 on slave 1
#endif 
    for (int i = 0; i < NUM_OF_ELMO; ++i) {
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

void print_task(void* arg) {

    //rt_task_set_periodic(NULL, TM_NOW, 100000000);
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 100);

    //rt_printf("Print Thread Start \n");

    while (print_run) {
        rt_task_wait_period(NULL);
        previous_time_print = now_time_print;
        //        rt_mutex_acquire(&mutex_desc, TM_INFINITE);
        inOP = TRUE;
        sys_ready = 1;
        if (inOP == TRUE) {
            if (!sys_ready) {
                if (stick == 0)
                    rt_printf("waiting for system ready...\n");
                if (stick % 10 == 0)
                    rt_printf("%i\n", stick / 10);
                stick++;
            } else {

                rt_printf("______________________________________________________________________________\n");
                rt_printf("Thread_time : %f [ms] / %f [ms] / %f [ms] / %f [ms] / %f [ms]\n", del_time_realcheck, del_time_motion, del_time_print, del_time_imu_check, del_time_imu);
                rt_printf("Motion Time(Max) : %f [ms] / IMU Time(Max) : %f [ms]\n", motion_time_set(0), IMU_time_set(0));


                //                //                //                rt_printf("tmp_flag = %d \n", PongBotQ.tmp_Mode_Change_flag);
                //                //                //                rt_printf("cnt  = %d \n", PongBotQ.cnt_mode_change);
                //                //                //                rt_printf("flag = %d \n", PongBotQ.Mode_Change_flag);
                //                //                //                rt_printf("Mode = %d \n", PongBotQ.ControlMode_print);
                //                //                //                rt_printf("Status word = 0x%X / 0x%X / 0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord);
                //                //                //                rt_printf("Actual Torque = %d / %d / %d \n", ELMO_drive_pt[0].ptInParam->TorqueActualValue, ELMO_drive_pt[1].ptInParam->TorqueActualValue, ELMO_drive_pt[2].ptInParam->TorqueActualValue);
                //                //                rt_printf("Status word = 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord, ELMO_drive_pt[3].ptInParam->StatusWord, ELMO_drive_pt[4].ptInParam->StatusWord, ELMO_drive_pt[5].ptInParam->StatusWord, ELMO_drive_pt[6].ptInParam->StatusWord, ELMO_drive_pt[7].ptInParam->StatusWord, ELMO_drive_pt[8].ptInParam->StatusWord, ELMO_drive_pt[9].ptInParam->StatusWord, ELMO_drive_pt[10].ptInParam->StatusWord, ELMO_drive_pt[11].ptInParam->StatusWord);
                rt_printf("-----------------------------------------------------------------------------\n");

                rt_printf("Base_acc(m/s^2) = ( %3f / %3f/ %3f )  ---  Max=(%3f / %3f / %3f)  \n", PongBotQ.actual_base_acc_local[0], PongBotQ.actual_base_acc_local[1], PongBotQ.actual_base_acc_local[2], x_acc_set(0), y_acc_set(0), z_acc_set(0));
                rt_printf("Base_ori(degree) = ( %3f / %3f/ %3f )  ---  Max=(%3f / %3f / %3f) \n", PongBotQ.actual_base_ori_local[0] * R2D, PongBotQ.actual_base_ori_local[1] * R2D, PongBotQ.actual_base_ori_local[2] * R2D, roll_set(0) * R2D, pitch_set(0) * R2D, yaw_set(0) * R2D);
                rt_printf("Base_ori_vel(rad/s) = ( %3f / %3f/ %3f )  ---  Max=(%3f / %3f / %3f)  \n", PongBotQ.actual_base_ori_vel_local[0], PongBotQ.actual_base_ori_vel_local[1], PongBotQ.actual_base_ori_vel_local[2], roll_vel_set(0), pitch_vel_set(0), yaw_vel_set(0));
                rt_printf("actual_q(degree) = ( %3f / %3f/ %3f ), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.actual_joint_pos[0] * R2D, PongBotQ.actual_joint_pos[1] * R2D, PongBotQ.actual_joint_pos[2] * R2D, PongBotQ.actual_joint_pos[3] * R2D, PongBotQ.actual_joint_pos[4] * R2D, PongBotQ.actual_joint_pos[5] * R2D, PongBotQ.actual_joint_pos[6] * R2D, PongBotQ.actual_joint_pos[7] * R2D, PongBotQ.actual_joint_pos[8] * R2D, PongBotQ.actual_joint_pos[9] * R2D, PongBotQ.actual_joint_pos[10] * R2D, PongBotQ.actual_joint_pos[11] * R2D);
                //                //                //rt_printf("Incre_q(degree) = %3f / %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f \n", PongBotQ.Incre_actual_joint_pos[0] * R2D, PongBotQ.Incre_actual_joint_pos[1] * R2D, PongBotQ.Incre_actual_joint_pos[2] * R2D, PongBotQ.Incre_actual_joint_pos[3] * R2D, PongBotQ.Incre_actual_joint_pos[4] * R2D, PongBotQ.Incre_actual_joint_pos[5] * R2D, PongBotQ.Incre_actual_joint_pos[6] * R2D, PongBotQ.Incre_actual_joint_pos[7] * R2D, PongBotQ.Incre_actual_joint_pos[8] * R2D, PongBotQ.Incre_actual_joint_pos[9] * R2D, PongBotQ.Incre_actual_joint_pos[10] * R2D, PongBotQ.Incre_actual_joint_pos[11] * R2D);
                rt_printf("actual_joint_vel(rad/s) = (%3f / %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.actual_joint_vel[0], PongBotQ.actual_joint_vel[1], PongBotQ.actual_joint_vel[2], PongBotQ.actual_joint_vel[3], PongBotQ.actual_joint_vel[4], PongBotQ.actual_joint_vel[5], PongBotQ.actual_joint_vel[6], PongBotQ.actual_joint_vel[7], PongBotQ.actual_joint_vel[8], PongBotQ.actual_joint_vel[9], PongBotQ.actual_joint_vel[10], PongBotQ.actual_joint_vel[11]);
                rt_printf("actual_EP_pos_local(FK) = (%3f / %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.actual_EP_pos_local[0], PongBotQ.actual_EP_pos_local[1], PongBotQ.actual_EP_pos_local[2], PongBotQ.actual_EP_pos_local[3], PongBotQ.actual_EP_pos_local[4], PongBotQ.actual_EP_pos_local[5], PongBotQ.actual_EP_pos_local[6], PongBotQ.actual_EP_pos_local[7], PongBotQ.actual_EP_pos_local[8], PongBotQ.actual_EP_pos_local[9], PongBotQ.actual_EP_pos_local[10], PongBotQ.actual_EP_pos_local[11]);
                //                //                rt_printf("cal_actual_joint_pos(IK) = %3f / %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f \n",PongBotQ.cal_actual_joint_pos[0] * R2D,PongBotQ.cal_actual_joint_pos[1] * R2D,PongBotQ.cal_actual_joint_pos[2] * R2D,PongBotQ.cal_actual_joint_pos[3] * R2D,PongBotQ.cal_actual_joint_pos[4] * R2D,PongBotQ.cal_actual_joint_pos[5] * R2D,PongBotQ.cal_actual_joint_pos[6] * R2D,PongBotQ.cal_actual_joint_pos[7] * R2D,PongBotQ.cal_actual_joint_pos[8] * R2D,PongBotQ.cal_actual_joint_pos[9] * R2D,PongBotQ.cal_actual_joint_pos[10] * R2D, PongBotQ.cal_actual_joint_pos[11] * R2D);
                //
                //                rt_printf("angle=%3f / %3f / %3f \n", PongBotQ.actual_base_ori_local(0) * R2D, PongBotQ.actual_base_ori_local(1) * R2D, PongBotQ.actual_base_ori_local(2) * R2D);
                //  rt_printf("angle=%3f / %3f / %3f \n", PongBotQ.actual_base_ori_local(0), PongBotQ.actual_base_ori_local(1), PongBotQ.actual_base_ori_local(2));
                //  rt_printf("angle_vel=%3f / %3f / %3f \n", PongBotQ.actual_base_ori_vel_local(0), PongBotQ.actual_base_ori_vel_local(1), PongBotQ.actual_base_ori_vel_local(2));
                //  rt_printf("tmp_angle_vel=%3f / %3f / %3f \n", PongBotQ.tmp_actual_base_ori_vel_local(0), PongBotQ.tmp_actual_base_ori_vel_local(1), PongBotQ.tmp_actual_base_ori_vel_local(2));
                //  rt_printf("acc=%3f / %3f / %3f \n", PongBotQ.actual_base_acc_local(0), PongBotQ.actual_base_acc_local(1), PongBotQ.actual_base_acc_local(2));

                //                cout<<"actual_EP_pos_local = "<<endl<<(PongBotQ.actual_EP_pos_local).transpose()<<endl;
                //                   cout<<"---------------------------------------"<<endl;
                //                cout<<"cal_actual_joint_pos = "<<endl<<(PongBotQ.cal_actual_joint_pos).transpose() * R2D<<endl;
                //                 cout<<"---------------------------------------"<<endl;

                //                rt_printf("Abs_q(degree) = %3f / %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f \n", PongBotQ.ABS_actual_joint_pos[0] * R2D, PongBotQ.ABS_actual_joint_pos[1] * R2D, PongBotQ.ABS_actual_joint_pos[2] * R2D, PongBotQ.ABS_actual_joint_pos[3] * R2D, PongBotQ.ABS_actual_joint_pos[4] * R2D, PongBotQ.ABS_actual_joint_pos[5] * R2D, PongBotQ.ABS_actual_joint_pos[6] * R2D, PongBotQ.ABS_actual_joint_pos[7] * R2D, PongBotQ.ABS_actual_joint_pos[8] * R2D, PongBotQ.ABS_actual_joint_pos[9] * R2D, PongBotQ.ABS_actual_joint_pos[10] * R2D, PongBotQ.ABS_actual_joint_pos[11] * R2D);
                //                //  rt_printf("actual_q_dot = %3f / %3f / %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f\n", PongBotQ.actual_joint_vel_HS[0], PongBotQ.actual_joint_vel_HS[1], PongBotQ.actual_joint_vel_HS[2],PongBotQ.actual_joint_vel_HS[3],PongBotQ.actual_joint_vel_HS[4],PongBotQ.actual_joint_vel_HS[5],PongBotQ.actual_joint_vel_HS[6],PongBotQ.actual_joint_vel_HS[7],PongBotQ.actual_joint_vel_HS[8],PongBotQ.actual_joint_vel_HS[9],PongBotQ.actual_joint_vel_HS[10],PongBotQ.actual_joint_vel_HS[11],PongBotQ.actual_joint_vel_HS[12]);
                ////
                //                //rt_printf("Incre_actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.Incre_actual_joint_pos_HS[0] * R2D, PongBotQ.Incre_actual_joint_pos_HS[1] * R2D, PongBotQ.Incre_actual_joint_pos_HS[2] * R2D);
                ////                rt_printf("abs_actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.ABS_actual_joint_pos_HS2[0] * R2D, PongBotQ.ABS_actual_joint_pos_HS2[1] * R2D, PongBotQ.ABS_actual_joint_pos_HS2[2] * R2D);
                ////                rt_printf("actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.actual_joint_pos_HS[0] * R2D, PongBotQ.actual_joint_pos_HS[1] * R2D, PongBotQ.actual_joint_pos_HS[2] * R2D);
                ////                rt_printf("actual_q_dot = %3f / %3f / %3f\n", PongBotQ.actual_joint_vel_HS[0], PongBotQ.actual_joint_vel_HS[1], PongBotQ.actual_joint_vel_HS[2]);
                ////                rt_printf("init_q(degree) = %3f / %3f / %3f\n", PongBotQ.init_joint_pos_HS[0] * R2D, PongBotQ.init_joint_pos_HS[1] * R2D, PongBotQ.init_joint_pos_HS[2] * R2D);
                ////                rt_printf("goal_q(degree) = %3f / %3f / %3f \n", PongBotQ.goal_joint_pos_HS[0] * R2D, PongBotQ.goal_joint_pos_HS[1] * R2D, PongBotQ.goal_joint_pos_HS[2] * R2D);
                ////                rt_printf("target_q(degree) = %3f / %3f / %3f \n", PongBotQ.target_joint_pos_HS[0] * R2D, PongBotQ.target_joint_pos_HS[1] * R2D, PongBotQ.target_joint_pos_HS[2] * R2D);
                ////                rt_printf("target_vel = %3f / %3f / %3f \n", PongBotQ.target_joint_vel_HS[0], PongBotQ.target_joint_vel_HS[1], PongBotQ.target_joint_vel_HS[2]);
                //                rt_printf("_________________________________________\n");
                ////                rt_printf("actual_EP(m) = %3f / %3f / %3f\n", PongBotQ.actual_EP_pos_local_HS[0], PongBotQ.actual_EP_pos_local_HS[1], PongBotQ.actual_EP_pos_local_HS[2]);
                ////                rt_printf("actual_EP_vel(m/s) = %3f / %3f / %3f\n", PongBotQ.actual_EP_vel_local_HS[0], PongBotQ.actual_EP_vel_local_HS[1], PongBotQ.actual_EP_vel_local_HS[2]);
                ////                rt_printf("init_EP(m) = %3f / %3f / %3f\n", PongBotQ.init_EP_pos_HS[0], PongBotQ.init_EP_pos_HS[1], PongBotQ.init_EP_pos_HS[2]);
                ////                rt_printf("goal_EP(m) = %3f / %3f / %3f \n", PongBotQ.goal_EP_pos_HS[0], PongBotQ.goal_EP_pos_HS[1], PongBotQ.goal_EP_pos_HS[2]);
                ////                rt_printf("target_EP(m) = %3f / %3f / %3f \n", PongBotQ.target_EP_pos_HS[0], PongBotQ.target_EP_pos_HS[1], PongBotQ.target_EP_pos_HS[2]);
                ////                rt_printf("target_EP_vel(m/s) = %3f / %3f / %3f \n", PongBotQ.target_EP_vel_HS[0], PongBotQ.target_EP_vel_HS[1], PongBotQ.target_EP_vel_HS[2]);
                ////                rt_printf("_________________________________________\n");
                ////                rt_printf("CTC_Torque = %3f / %3f / %3f \n", PongBotQ.joint[0].torque, PongBotQ.joint[1].torque, PongBotQ.joint[2].torque);
                ////                rt_printf("Joint_Torque = %3f / %3f / %3f \n", PongBotQ.Joint_Controller_HS[6], PongBotQ.Joint_Controller_HS[7], PongBotQ.Joint_Controller_HS[8]);
                ////                rt_printf("Cart_Torque = %3f / %3f / %3f \n", PongBotQ.Cart_Controller_HS[6], PongBotQ.Cart_Controller_HS[7], PongBotQ.Cart_Controller_HS[8]);
                ////                rt_printf("Gravity = %3f / %3f / %3f \n", PongBotQ.G_term[6], PongBotQ.G_term[7], PongBotQ.G_term[8]);
                ////                rt_printf("Coriolis = %3f / %3f / %3f \n", PongBotQ.C_term[6], PongBotQ.C_term[7], PongBotQ.C_term[8]);
                ////                rt_printf("_________________________________________\n");
                ////                rt_printf("Joint_P_gain = %3f / %3f / %3f \n", PongBotQ.kp_joint_HS[0], PongBotQ.kp_joint_HS[1], PongBotQ.kp_joint_HS[2]);
                ////                rt_printf("Joint_D_gain = %3f / %3f / %3f \n", PongBotQ.kd_joint_HS[0], PongBotQ.kd_joint_HS[1], PongBotQ.kd_joint_HS[2]);
                ////                rt_printf("Cart_P_gain = %3f / %3f / %3f \n", PongBotQ.kp_EP_HS[0], PongBotQ.kp_EP_HS[1], PongBotQ.kp_EP_HS[2]);
                ////                rt_printf("Cart_D_gain = %3f / %3f / %3f \n", PongBotQ.kd_EP_HS[0], PongBotQ.kd_EP_HS[1], PongBotQ.kd_EP_HS[2]);
                ////                rt_printf("_________________________________________\n");
                ////                rt_printf("Joy_mode = %d \n", PongBotQ.JoyMode);
                ////                rt_printf("stop_flag= %d \n", PongBotQ.walk_stop_flag);
                ////                rt_printf("Joy_vel = %3f / %3f / %3f \n ", PongBotQ.joy_vel_x, PongBotQ.joy_vel_y, PongBotQ.joy_vel_z);
                ////                rt_printf("Joy_tmp_EP1 = %3f / %3f / %3f \n ", PongBotQ.tmp1_target_EP_pos_HS[0], PongBotQ.tmp1_target_EP_pos_HS[1], PongBotQ.tmp1_target_EP_pos_HS[2]);
                ////                rt_printf("Joy_tmp_EP2 = %3f / %3f / %3f \n ", PongBotQ.tmp2_target_EP_pos_HS[0], PongBotQ.tmp2_target_EP_pos_HS[1], PongBotQ.tmp2_target_EP_pos_HS[2]);
                ////                rt_printf("_________________________________________\n");
                ////
                // rt_printf("----------------------------------------------------\n");
                //                //rt_printf("(1)=%3f/(2)=%3f/(3)=%3f/(4)=%3f/(5)=%3f/(6)=%3f/(7)=%3f/(8)=%3f",joy_info[0],joy_info[1],joy_info[2],joy_info[3],joy_info[4],joy_info[5],joy_info[6],joy_info[7],joy_info[8]);
            }
        }
        // rt_mutex_release(&mutex_desc);
        now_time_print = rt_timer_read();
        del_time_print = (double) (now_time_print - previous_time_print) / 1000000;
    }
}

void catch_signal(int sig) {

    printf("Program END...\n");

    FileSave();

    rt_task_delete(&RT_task1);
    rt_task_delete(&RT_task2);
    rt_task_delete(&RT_task3);
    ros::shutdown();
    exit(0);
}

void ServoOn(void) {
    //servo-on	
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        controlword = 0;
        started[i] = ServoOn_GetCtrlWrd(ELMO_drive_pt[i].ptInParam->StatusWord, &controlword);
        ELMO_drive_pt[i].ptOutParam->ControlWord = controlword;
        if (started[i]) ServoState |= (1 << i);
    }
}

void ServoOff(void) {
    //printf("function is Servo_off");
    //Servo OFF
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
    }
}

void EncoderRead(void) {

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        PongBotQ.Raw_ABS_actual_joint_pos[i] = PongBotQ.Count2Rad_ABS(PongBotQ.Resolution[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue);
        PongBotQ.Raw_Incre_actual_joint_pos[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
        PongBotQ.Raw_actual_joint_vel[i] = PongBotQ.Count2RadDot(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
    }
    //    for (int i = 0; i <3; ++i){
    //        PongBotQ.Incre_actual_joint_pos[i] = PongBotQ.Raw_Incre_actual_joint_pos[i];
    //    }
    //    for (int i = 3; i <6; ++i){
    //        PongBotQ.Incre_actual_joint_pos[i] = PongBotQ.Raw_Incre_actual_joint_pos[8-i];
    //    }    
    //    for (int i = 6; i <9; ++i){
    //        PongBotQ.Incre_actual_joint_pos[i] = PongBotQ.Raw_Incre_actual_joint_pos[i];
    //    }
    //    for (int i = 9; i <12; ++i){
    //        PongBotQ.Incre_actual_joint_pos[i] = PongBotQ.Raw_Incre_actual_joint_pos[20-i];
    //    }

    for (int i = 0; i < 3; ++i) {
        PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[i];
        if (i == 2) {
            PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[i]*1.5;
        }
    }
    for (int i = 3; i < 6; ++i) {
        PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[8 - i];
        if (i == 5) {
            PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[8 - i]*1.5;
        }
    }
    for (int i = 6; i < 9; ++i) {
        PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[i];
        if (i == 8) {
            PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[i]*1.5;
        }
    }
    for (int i = 9; i < 12; ++i) {
        PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[20 - i];
        if (i == 11) {
            PongBotQ.actual_joint_vel[i] = PongBotQ.Raw_actual_joint_vel[20 - i]*1.5;
        }
    }

    // if (PongBotQ.Encoder_Reset_Flag == true) {

    for (int i = 0; i < 3; ++i) {
        PongBotQ.ABS_actual_joint_pos[i] = PongBotQ.Raw_ABS_actual_joint_pos[i];
    }
    for (int i = 3; i < 6; ++i) {
        PongBotQ.ABS_actual_joint_pos[i] = PongBotQ.Raw_ABS_actual_joint_pos[8 - i];
    }
    for (int i = 6; i < 9; ++i) {
        PongBotQ.ABS_actual_joint_pos[i] = PongBotQ.Raw_ABS_actual_joint_pos[i];
    }
    for (int i = 9; i < 12; ++i) {
        PongBotQ.ABS_actual_joint_pos[i] = PongBotQ.Raw_ABS_actual_joint_pos[20 - i];
    }

    //        for (int i = 0; i < NUM_OF_ELMO ; ++i) {
    //            PongBotQ.tmp_Incre_actual_joint_pos[i] = PongBotQ.Incre_actual_joint_pos[i];
    //        }
    // PongBotQ.Encoder_Reset_Flag = false;
    //}

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        //PongBotQ.actual_joint_pos[i] = PongBotQ.Incre_actual_joint_pos[i] + PongBotQ.ABS_actual_joint_pos[i] - PongBotQ.tmp_Incre_actual_joint_pos[i];
        PongBotQ.actual_joint_pos[i] = PongBotQ.ABS_actual_joint_pos[i];
    }

    //    if (PongBotQ.Encoder_Reset_Flag == true) {
    //        for (int i = 0; i < NUM_OF_MOTOR; i++) {
    //            PongBotQ.ABS_actual_joint_pos[i] = PongBotQ.Count2Rad_ABS(PongBotQ.Resolution[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue);
    ////            PongBotQ.Incre_actual_joint_pos_offset[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
    //            PongBotQ.actual_joint_pos[i] = PongBotQ.ABS_actual_joint_pos[i] + PongBotQ.Incre_actual_joint_pos[i];
    //            //PongBotQ.actual_joint_pos_HS[i] = PongBotQ.Incre_actual_joint_pos_HS[i];
    //            //            if (i == 2) {
    //            //                PongBotQ.actual_joint_pos_HS[i] = PongBotQ.Incre_actual_joint_pos_HS[i] - PI / 2;
    //            //            }
    //        }
    //        //        PongBotQ.actual_EP_pos_local_HS = PongBotQ.FK_HS(PongBotQ.actual_joint_pos_HS);
    //        //        PongBotQ.pre_actual_EP_pos_local_HS = PongBotQ.actual_EP_pos_local_HS;
    //
    //        PongBotQ.Encoder_Reset_Flag = false;
    //    }




    //    for (int i = 0; i < NUM_OF_MOTOR; i++) {
    //        PongBotQ.online_ABS_actual_joint_pos[i] = PongBotQ.Count2Rad_ABS(PongBotQ.Resolution[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue);
    ////        PongBotQ.Incre_actual_joint_pos[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue) - PongBotQ.Incre_actual_joint_pos_offset[i];
    //        PongBotQ.actual_joint_pos[i] = PongBotQ.ABS_actual_joint_pos[i] + PongBotQ.Incre_actual_joint_pos[i];
    //        PongBotQ.actual_joint_vel[i] = PongBotQ.Count2RadDot(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
    //    }
}

void GetActualData(void) {
    PongBotQ.actual_EP_pos_local = PongBotQ.FK(PongBotQ.actual_joint_pos);
    //    cout << "J_SJ=" << endl << PongBotQ.Jac(PongBotQ.actual_joint_pos) << endl;
    //    cout << "------------------------" << endl;
    //    PongBotQ.tmp_actual_joint_vel_HS << 0, 0, 0, 0, 0, 0, PongBotQ.actual_joint_vel_HS;
    //    PongBotQ.tmp_actual_EP_vel_local_HS = PongBotQ.J_A * PongBotQ.tmp_actual_joint_vel_HS;
    //    PongBotQ.actual_EP_vel_local_HS = PongBotQ.tmp_actual_EP_vel_local_HS.tail(3);
    //PongBotQ.pre_actual_EP_pos_local_HS = PongBotQ.actual_EP_pos_local_HS;

    //PongBotQ.J_HS = PongBotQ.Jacobian_HS(PongBotQ.actual_joint_pos_HS);
}

void Load(void) {
    rbdl_check_api_version(RBDL_API_VERSION);
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);
    Addons::URDFReadFromFile("/home/rclab/catkin_ws/src/PongBotQ_V6/model/PONGBOT_Q_V6/urdf/PONGBOT_Q_V6.urdf", pongbot_q_model, true, false);
    PongBotQ.setRobotModel(pongbot_q_model);
    cout << "IMU Setting" << endl;
    IMU_initialize();
    //    PongBotQ.abs_kp_joint_HS << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 0.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0;
    //    PongBotQ.abs_kd_joint_HS << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
    //    PongBotQ.abs_kp_EP_HS << 1000.0, 1000.0, 150.0, 1000.0, 1000.0, 150.0, 1000.0, 1000.0, 150.0, 1000.0, 1000.0, 150.0;
    //    PongBotQ.abs_kd_EP_HS << 50.0, 50.0, 10.0, 50.0, 50.0, 10.0, 50.0, 50.0, 10.0, 50.0, 50.0, 10.0;

    //    PongBotQ.kp_joint_HS = PongBotQ.abs_kp_joint_HS;
    //    PongBotQ.kd_joint_HS = PongBotQ.abs_kd_joint_HS;
    //PongBotQ.kp_EP_HS = PongBotQ.abs_kp_EP_HS;
    //PongBotQ.kd_EP_HS = PongBotQ.abs_kd_EP_HS;
    //    PongBotQ.foot_height_HS = 0.15;

    //    PongBotQ.init_kp_joint_HS = PongBotQ.kp_joint_HS;
    //    PongBotQ.init_kd_joint_HS = PongBotQ.kd_joint_HS;
    //    PongBotQ.init_kp_EP_HS = PongBotQ.kp_EP_HS;
    //    PongBotQ.init_kd_EP_HS = PongBotQ.kd_EP_HS;

    PongBotQ.ControlMode = CTRLMODE_NONE; //=0
    PongBotQ.CommandFlag = NO_ACT; //=0
}

void jointController(void) {

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        if (PongBotQ.joint[i].torque >= 3000) {
            PongBotQ.joint[i].torque = 3000;
        } else if (PongBotQ.joint[i].torque <= -3000) {
            PongBotQ.joint[i].torque = -3000;
        }
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->TargetTorque = PongBotQ.Tor2Cur(PongBotQ.joint[i].torque, PongBotQ.Kt[i], PongBotQ.Gear[i], PongBotQ.ratedCur[i]);
    }
}

void Callback1(const std_msgs::Int32 &msg) {
    PongBotQ.ControlMode = msg.data;
    PongBotQ.tmp_Mode_Change_flag = true;
    PongBotQ.cnt_Control_change = 0;
}

void Callback2(const std_msgs::Int32 &msg) {
    PongBotQ.stop_flag = true;
}

void Callback3(const sensor_msgs::Joy& msg) {
    PongBotQ.joy_vel_x = (float) msg.axes[1]; //pitch
    PongBotQ.joy_vel_y = (float) msg.axes[0];
    PongBotQ.joy_vel_z = (float) msg.axes[2];

    if ((int) msg.buttons[2] == 1) { //mani
        PongBotQ.JoyMode = JOYMODE_HOME;
        PongBotQ.cnt_HS = 0;
    }

    if ((int) msg.buttons[3] == 1) { //takeoff
        PongBotQ.JoyMode = JOYMODE_MOVE;
    }

    if ((int) msg.buttons[1] == 1) { //takeoff
        PongBotQ.JoyMode = JOYMODE_WALK;
        PongBotQ.cnt_HS = 0;
    }

    if ((int) msg.buttons[5] == 1) {
        PongBotQ.walk_stop_flag = true;
    }

    if ((int) msg.buttons[4] == 1) {
        PongBotQ.walk_stop_flag = false;
    }
}

void ROSMsgPublish(void) {
    //tf::TransformBroadcaster broadcaster;

    m_data.data[0] = PongBotQ.actual_base_ori_local[0];
    m_data.data[1] = PongBotQ.actual_base_ori_local[1];
    m_data.data[2] = PongBotQ.actual_base_ori_local[2];
    
//    m_data.data[2] = PongBotQ.target_joint_vel_HS[2];
//    m_data.data[1] = PongBotQ.actual_joint_pos_HS[1] * R2D;
//    m_data.data[2] = PongBotQ.actual_joint_pos_HS[2] * R2D;
//    
//    m_data.data[3] = PongBotQ.target_joint_pos_HS[0] * R2D;
//    m_data.data[4] = PongBotQ.target_joint_pos_HS[1] * R2D;
//    m_data.data[5] = PongBotQ.target_joint_pos_HS[2] * R2D;

    m_joint_states.header.stamp = ros::Time::now();
    m_joint_states.name[0] = "HR_JOINT";
    m_joint_states.name[1] = "HP_JOINT";
    m_joint_states.name[2] = "KN_JOINT";
    //m_joint_states.position[0] = PongBotQ.target_joint_pos_HS[0];
    //m_joint_states.position[1] = PongBotQ.target_joint_pos_HS[1];
    //  m_joint_states.position[2] = PongBotQ.actual_joint_pos_HS[2];
    //m_joint_states.position[0] = PongBotQ.target_joint_pos_HS[0];
    //m_joint_states.position[1] = PongBotQ.target_joint_pos_HS[1];
    //m_joint_states.position[2] = PongBotQ.target_joint_pos_HS[2];

    //    odom_trans.header.stamp = ros::Time::now();
    //    odom_trans.header.frame_id = "odom";
    //    odom_trans.child_frame_id = "BASE";
    //    
    //    odom_trans.transform.translation.x = 0;
    //    odom_trans.transform.translation.y = 0;
    //    odom_trans.transform.translation.z = 1;
    //    odom_trans.transform.rotation.x = 0;
    //    odom_trans.transform.rotation.y = 0;
    //    odom_trans.transform.rotation.z = 0;
    //    odom_trans.transform.rotation.w = 1;z


    P_data.publish(m_data);

    P_joint_states.publish(m_joint_states);
    //broadcaster.sendTransform(odom_trans);

}

void DataSave(void) {
    //save_array[save_cnt][0] = PongBotQ.cnt_HS / 1000.0;
    //save_array[save_cnt][1] = (int) PongBotQ.ControlMode;
    //    save_array[save_cnt][2] = PongBotQ.actual_joint_pos_HS[0] * R2D;
    //    save_array[save_cnt][3] = PongBotQ.actual_joint_pos_HS[1] * R2D;
    //    save_array[save_cnt][4] = PongBotQ.actual_joint_pos_HS[2] * R2D;
    //    save_array[save_cnt][5] = PongBotQ.target_joint_pos_HS[0] * R2D;
    //    save_array[save_cnt][6] = PongBotQ.target_joint_pos_HS[1] * R2D;
    //    save_array[save_cnt][7] = PongBotQ.target_joint_pos_HS[2] * R2D;
//    for (int i=0;i<48;i++){
//     save_array[save_cnt][i] = Receive_MIC_buf[i];    
//    }
    
    
    //    save_array[save_cnt][3] = ELMO_drive_pt[0].ptInParam->StatusWord;
    //    save_array[save_cnt][4] = ELMO_drive_pt[1].ptInParam->StatusWord;
    //    save_array[save_cnt][5] = ELMO_drive_pt[2].ptInParam->StatusWord;
    //    save_array[save_cnt][6] = ELMO_drive_pt[0].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][7] = ELMO_drive_pt[1].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][8] = ELMO_drive_pt[2].ptInParam->TorqueActualValue;
//    save_array[save_cnt][3] = PongBotQ.actual_joint_pos[0] * R2D;
//    save_array[save_cnt][4] = PongBotQ.actual_joint_pos[1] * R2D;
//    save_array[save_cnt][5] = PongBotQ.actual_joint_pos[2] * R2D;

    if (save_cnt < SAVE_COUNT - 1) {
        save_cnt++;
    }
}

void FileSave(void) {
    FILE *fp;

    fp = fopen("HSData.txt", "w");

    for (int j = 0; j <= SAVE_COUNT - 1; ++j) {
        for (int i = 0; i <= SAVE_LENGTH - 1; ++i) {
            fprintf(fp, "%d ",(int)save_array[j][i]);
//            fprintf(fp, "%c\t", save_array[j][i]);
        }
        fprintf(fp, "%d\n", (int)save_array[j][SAVE_LENGTH - 1]);
        //fprintf(fp, "%c\n", save_array[j][SAVE_LENGTH - 1]);
    }

    fclose(fp);
}

void Torque_Off(void) {

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        PongBotQ.joint[i].torque = 0.0;
    }
}

bool ecat_init(void) {

    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");

    if (ec_init(ecat_ifname)) {

        printf("ec_init on %s succeeded.\n", ecat_ifname);

        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);
            for (int i = 0; i < NUM_OF_ELMO; ++i) {
                printf("Has CA? %d %s\n", i + 1, ec_slave[i + 1].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
            }

            for (int i = 1; i < NUM_OF_ELMO; ++i) {
                ec_slave[i + 1].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            // PDO re-mapping //
            for (int k = 0; k < NUM_OF_ELMO; ++k) {
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
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0002, 0x60FD0020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0003, 0x606C0020);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0004, 0x60410010);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0005, 0x60770010);
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0006, 0x20a00020);
                    wkc += RS3_write8(k + 1, 0x1a07, 0x0000, 0x06);

                    wkc += RS3_write16(k + 1, 0x1c13, 0x0001, 0x1a07); //  (row,PDO) 
                    wkc += RS3_write8(k + 1, 0x1c13, 0x0000, 0x01); //  (index,Row)
                }
            }

            ec_config_map(&IOmap);

#ifdef _USE_DC
            ec_configdc();
#endif  
            printf("Slaves mapped, state to SAFE_OP.\n");
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

#ifdef _USE_DC
            printf("DC capable : %d\n", ec_configdc());
#endif

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

                for (int k = 0; k < NUM_OF_ELMO; ++k) {
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
                for (int i = 0; i < NUM_OF_ELMO; ++i)
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

int IMU_initialize() {
    
    struct termios oldtio, newtio;
    
    fd_MIC = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    // dev/ttyACM0 
    // O_RDWF : Use write/read mode with fd(file descriptor) 
    // O_NOCCTY : 
    if (fd_MIC < 0) {
        perror("/dev/ttyACM0");
        exit(-1);
    }

    tcgetattr(fd_MIC, &oldtio);
    bzero(&newtio, sizeof (newtio));

    //newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    //newtio.c_cflag = B230400 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    //newtio.c_cflag = B460800 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    newtio.c_cflag = B921600 | CS8 | CLOCAL | CREAD; //baud 통신 속도 | CS8 (8bit, No Parity, 1 Stop Bit)설정 | 내부통신 포트 | 읽기가능
    newtio.c_iflag = IGNPAR | ICRNL; //Parity 오류가 있는 문자 무시
    newtio.c_oflag = 0; //출력처리 설정 0이면 아무것도 안함
    newtio.c_lflag = 0; //Local Mode 설정, ICANON이면 널 문자가 들어올때까지 수신
    newtio.c_cc[VINTR] = 0;
    newtio.c_cc[VQUIT] = 0;
    newtio.c_cc[VERASE] = 0;
    newtio.c_cc[VKILL] = 0;
    newtio.c_cc[VEOF] = 4;
    newtio.c_cc[VTIME] = 0; //time-out 값으로 사용, time-out 값은 TIME*0.1초
    
    //newtio.c_cc[VMIN] = 1; //read가 리턴되기 위한 최소한의 문자 개수
    newtio.c_cc[VMIN] = 0; //read가 리턴되기 위한 최소한의 문자 개수
    
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

    imu_run = 1;

    //cout << "fd_MIC =" << fd_MIC << endl;
    return 1;
}

void imu_task(void *arg) {
    unsigned int MIC_length = 48;
    unsigned char Receive_MIC_buf[MIC_length];

    //rt_task_set_periodic(NULL, TM_NOW, 20*cycle_ns);
//    rt_task_set_periodic(NULL, TM_NOW, 4*cycle_ns);
//    rt_task_set_periodic(NULL, TM_NOW, 2*cycle_ns);
    rt_task_set_periodic(NULL, TM_NOW, 1*cycle_ns);
    //rt_task_set_periodic(NULL, TM_NOW, 5*cycle_ns); //소
    //rt_task_set_periodic(NULL, TM_NOW, 10*cycle_ns);

    //unsigned char Receive_MIC_buf[MIC_length];

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
    int cnt_over1 = 0;
    int mic_flag1 = 0;
    int buf_index = 0;
    int data_start = 0;
       memset(Receive_MIC_buf, 0, MIC_length);
       
    while (imu_run == 1) {
        
        rt_task_wait_period(NULL);
        previous_time_imu = now_time_imu;
        previous_time_imu_check = rt_timer_read();
        // Sensor Read
        // -----> Upload Sensor data-> shared meomry
        // Sensor Read by Hyun Do & kyung min
        //MIC_IMU sensor
        // Clear Receive_MIC_buf bytes as 0
        
       // rt_printf("\n---Data start---------\n-");
        //Data Read
//        rt_printf("%d \n",sizeof(fd_MIC));

        //        rt_printf("%s \n",fd_MIC);
//         memset(Receive_MIC_buf, 0, MIC_length);

        if (mic_flag1 == 0) {
            for (int j = 0; j < MIC_length; ++j) {
                read(fd_MIC, Receive_MIC_buf + j, 1);
            } //rt_printf("\n");    
        }
        
        
        tcflush(fd_MIC, TCIFLUSH);
        //tcflush(fd_MIC, TCIOFLUSH);
        //rt_printf("\n");
        //rt_printf("flush = %d\n",flush_check);
//        if(flush_check!=0){
//            rt_printf("Flush Error!\n");
//        }
        
        now_time_imu_check = rt_timer_read();
//        for(int i=0;i<MIC_length;i++){
//            rt_printf("0x%X /", Receive_MIC_buf[i]);
//            if (i % 10 == 9) {
//                rt_printf("\n");
//            }
//        }
//        rt_printf("\n-------------------------\n-");
//        
//        if (Receive_MIC_buf[0] == 0x75 && Receive_MIC_buf[1] == 0x65 && Receive_MIC_buf[2] == 0x80 && Receive_MIC_buf[3] == 0x2a && Receive_MIC_buf[4] == 0x0E && Receive_MIC_buf[5] == 0x0C) {            
//            data_start = 6;
//            mic_flag1 = 1;
//        }else{
//            n_fail++;
//        }
        
        
        if (Receive_MIC_buf[0] == 0x75 && Receive_MIC_buf[1] == 0x65 && Receive_MIC_buf[2] == 0x80 && Receive_MIC_buf[3] == 0x2a && Receive_MIC_buf[4] == 0x0E && Receive_MIC_buf[5] == 0x0C && Receive_MIC_buf[18] == 0x0E && Receive_MIC_buf[19] == 0x05 && Receive_MIC_buf[32] == 0x0E && Receive_MIC_buf[33] == 0x04) {            
            data_start = 6;
            mic_flag1 = 1;
        }else{
            n_fail++;
        }
         //rt_printf("fail_num=%d\n", n_fail);
       
        //Data Save (42=Pay load's length)
        if (mic_flag1 == 1) {
//            //MIC IMU
            roll_MIC_f = Receive_MIC_buf[data_start] << 24 | Receive_MIC_buf[data_start + 1] << 16 | Receive_MIC_buf[data_start + 2] << 8 | Receive_MIC_buf[data_start + 3];
            pitch_MIC_f = Receive_MIC_buf[data_start + 4] << 24 | Receive_MIC_buf[data_start + 5] << 16 | Receive_MIC_buf[data_start + 6] << 8 | Receive_MIC_buf[data_start + 7];
            yaw_MIC_f = Receive_MIC_buf[data_start + 8] << 24 | Receive_MIC_buf[data_start + 9] << 16 | Receive_MIC_buf[data_start + 10] << 8 | Receive_MIC_buf[data_start + 11];
            roll_MIC = *(float*) &roll_MIC_f;
            roll_MIC = -roll_MIC;
            pitch_MIC = *(float*) &pitch_MIC_f;
            pitch_MIC = pitch_MIC;
            yaw_MIC = *(float*) &yaw_MIC_f;
            yaw_MIC = -yaw_MIC;

            Gyro_X_MIC_f = Receive_MIC_buf[data_start + 14] << 24 | Receive_MIC_buf[data_start + 15] << 16 | Receive_MIC_buf[data_start + 16] << 8 | Receive_MIC_buf[data_start + 17];
            Gyro_Y_MIC_f = Receive_MIC_buf[data_start + 18] << 24 | Receive_MIC_buf[data_start + 19] << 16 | Receive_MIC_buf[data_start + 20] << 8 | Receive_MIC_buf[data_start + 21];
            Gyro_Z_MIC_f = Receive_MIC_buf[data_start + 22] << 24 | Receive_MIC_buf[data_start + 23] << 16 | Receive_MIC_buf[data_start + 24] << 8 | Receive_MIC_buf[data_start + 25];
            Gyro_X_MIC = *(float*) &Gyro_X_MIC_f;
            Gyro_X_MIC = -Gyro_X_MIC;
            Gyro_Y_MIC = *(float*) &Gyro_Y_MIC_f;
            Gyro_Y_MIC = -Gyro_Y_MIC;
            Gyro_Z_MIC = *(float*) &Gyro_Z_MIC_f;
            Gyro_Z_MIC = Gyro_Z_MIC;

            Acc_X_MIC_f = Receive_MIC_buf[data_start + 28] << 24 | Receive_MIC_buf[data_start + 29] << 16 | Receive_MIC_buf[data_start + 30] << 8 | Receive_MIC_buf[data_start + 31];
            Acc_Y_MIC_f = Receive_MIC_buf[data_start + 32] << 24 | Receive_MIC_buf[data_start + 33] << 16 | Receive_MIC_buf[data_start + 34] << 8 | Receive_MIC_buf[data_start + 35];
            Acc_Z_MIC_f = Receive_MIC_buf[data_start + 36] << 24 | Receive_MIC_buf[data_start + 37] << 16 | Receive_MIC_buf[data_start + 38] << 8 | Receive_MIC_buf[data_start + 39];
            Acc_X_MIC = *(float*) &Acc_X_MIC_f;
            Acc_X_MIC = Acc_X_MIC;
            Acc_Y_MIC = *(float*) &Acc_Y_MIC_f;
            Acc_Y_MIC = Acc_Y_MIC;
            Acc_Z_MIC = *(float*) &Acc_Z_MIC_f;
            Acc_Z_MIC = Acc_Z_MIC;

            if (roll_MIC <= 180 * D2R && roll_MIC >= 90 * D2R) {
                roll_MIC = roll_MIC - 180.0 * D2R;
            } else if (roll_MIC >= -180 * D2R && roll_MIC <= -90 * D2R) {
                roll_MIC = roll_MIC + 180.0 * D2R;
            }

            PongBotQ.actual_base_ori_local << roll_MIC, pitch_MIC, yaw_MIC;
            PongBotQ.actual_base_ori_vel_local << Gyro_X_MIC, Gyro_Y_MIC, Gyro_Z_MIC;
            PongBotQ.actual_base_acc_local << Acc_X_MIC, Acc_Y_MIC, Acc_Z_MIC;

            roll_set(1) = PongBotQ.actual_base_ori_local(0);
            pitch_set(1) = PongBotQ.actual_base_ori_local(1);
            yaw_set(1) = PongBotQ.actual_base_ori_local(2);
            roll_set = Max_Value_Save(roll_set);
            pitch_set = Max_Value_Save(pitch_set);
            yaw_set = Max_Value_Save(yaw_set);

            roll_vel_set(1) = PongBotQ.actual_base_ori_vel_local(0);
            pitch_vel_set(1) = PongBotQ.actual_base_ori_vel_local(1);
            yaw_vel_set(1) = PongBotQ.actual_base_ori_vel_local(2);
            roll_vel_set = Max_Value_Save(roll_vel_set);
            pitch_vel_set = Max_Value_Save(pitch_vel_set);
            yaw_vel_set = Max_Value_Save(yaw_vel_set);

            x_acc_set(1) = PongBotQ.actual_base_acc_local(0);
            y_acc_set(1) = PongBotQ.actual_base_acc_local(1);
            z_acc_set(1) = PongBotQ.actual_base_acc_local(2);
            x_acc_set = Max_Value_Save(x_acc_set);
            y_acc_set = Max_Value_Save(y_acc_set);
            z_acc_set = Max_Value_Save(z_acc_set);

            if (abs(x_acc_set(1)) > 100000 || abs(y_acc_set(1)) > 100000 || abs(z_acc_set(1)) > 100000) {
                rt_printf("EEEEE\n");
                for (int i = 0; i < MIC_length; i++) {
                    rt_printf("0x%X /", Receive_MIC_buf[i]);
                    if (i % 10 == 9) {
                        rt_printf("\n");
                    }
                }
            }

//            rt_printf("Roll(max)=%f , Roll(now)=%f \n", roll_set(0) * R2D, roll_set(1) * R2D);
//            rt_printf("Pitch(max)=%f , Pitch(now)=%f \n", pitch_set(0) * R2D, pitch_set(1) * R2D);
//            rt_printf("Yaw(max)=%f , Yaw(now)=%f \n", yaw_set(0) * R2D, yaw_set(1) * R2D);
//            rt_printf("------- \n");
//            rt_printf("Roll_vel(max)=%f , Roll_vel(now)=%f \n", roll_vel_set(0), roll_vel_set(1));
//            rt_printf("Pitch_vel(max)=%f , Pitch_vel(now)=%f \n", pitch_vel_set(0), pitch_vel_set(1));
//            rt_printf("Yaw_vel(max)=%f , Yaw_vel(now)=%f \n", yaw_vel_set(0), yaw_vel_set(1));
//            rt_printf("------- \n");
//            rt_printf("Xacc(max)=%f , Xacc(now)=%f \n", x_acc_set(0), x_acc_set(1));
//            rt_printf("Yacc(max)=%f , Yacc(now)=%f \n", y_acc_set(0), y_acc_set(1));
//            rt_printf("Zacc(max)=%f , Zacc(now)=%f \n", z_acc_set(0), z_acc_set(1));
//            rt_printf("------- \n");

            mic_flag1 = 0;
            data_start = 0;
            memset(Receive_MIC_buf, 0, MIC_length);
        } //else {
//            cnt_over1++;
//            printf("MIC_ = %5.5d\n", cnt_over1);
//        }
         //now_time_imu = rt_timer_read();
        now_time_imu_check = rt_timer_read();
        now_time_imu = rt_timer_read();
        
        del_time_imu_check = (double) (now_time_imu_check - previous_time_imu_check) / 1000000;
        del_time_imu = (double) (now_time_imu - previous_time_imu) / 1000000;
        
        IMU_time_set(1) = del_time_imu_check;
        IMU_time_set = Max_Value_Save(IMU_time_set);
        
        //rt_printf("\n -----Data End--------\n");
    }
    //    tcsetattr(fd_MIC,TCSANOW,&oldtio);

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

