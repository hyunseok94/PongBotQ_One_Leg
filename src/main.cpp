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
#include <rtdm/serial.h>

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

unsigned int n_fail = 0;

// loop (main & thread)
int motion_run_flag = 1;
int print_run_flag = 1;
int imu_run_flag = 0;
int QP_run_flag = 1;
int sys_ready_flag = 0;

// Servo
int ServoState = 0;
int servo_ready = 0;

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
VectorXd QP_time_set = VectorXd::Zero(2);

uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;

//// Elmo setting
UINT16 maxTorque = 3500;
INT16 Low_TargetTor[NUM_OF_ELMO] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
ros::Subscriber S_Joystick;
ros::Subscriber S_IMU;
ros::Publisher P_data;
int ros_exit = 0;
std_msgs::Float64MultiArray m_data;

double Thread_time = 0.0;

//Xenomai
RT_TASK PongBotQ_task;
RT_TASK Print_task;
RT_TASK IMU_task;
RT_TASK QP_task;

//RT_TASK RT_task3;
//RT_MUTEX mutex_desc; //mutex
RTIME check_now_motion_time, check_previous_motion_time; // Ethercat time
RTIME now_motion_time, previous_motion_time; // Thread 1 cycle time
RTIME now_print_time, previous_print_time; // Thread 2 cycle time
RTIME now_imu_time, previous_imu_time; // Thread 2 cycle time
RTIME check_now_imu_time, check_previous_imu_time; // Thread 2 cycle time
RTIME now_QP_time, previous_QP_time; // Thread 2 cycle time
RTIME check_now_QP_time, check_previous_QP_time; // Thread 2 cycle time

double check_motion_time = 0.0;
double motion_time = 0.0;
double print_time = 0.0;
double check_imu_time = 0.0;
double imu_time = 0.0;
double QP_time = 0.0;
double check_QP_time = 0.0;

//RBDL
int version_test;
Model* pongbot_q_model = new Model();

//CRobot
CRobot PongBotQ;

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
void Torque_Off(void); // : Torque Off
void motion_run(void* arg); //  TMotor Control task
void print_run(void* arg); //  Print task
void imu_run(void* arg); //  Imu Task
void QP_run(void* arg); //  QP Task
void signal_handler(int sig); //  Catch "Ctrl + C signal"
bool ecat_init(void);

void Load(void); // : parameter initial setting
int IMU_COM_Setting();
void EncoderRead(void); // : q, q_dot Read
void GetActualData(void);
void TargetTor_Gen(void);
void DataSave(void);
void FileSave(void);
void Max_Time_Save(double now_time);

// ROS function
void JoystickCallback(const sensor_msgs::Joy& msg);
void ROSMsgPublish(void);

int main(int argc, char* argv[]) {
    rtser_config_t ABC;
    ABC.baud_rate = 115200;

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

    m_data.data.resize(10);
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

    //rt_task_create(&PongBotQ_task, "Motion_task", 0, 99, 0);
    //rt_task_create(&Print_task, "Print_task", 0, 60, 0);
    //rt_task_create(&IMU_task, "Imu_task", 0, 95, 0);
    rt_task_create(&IMU_task, "Imu_task", 0, 99, 0);
    //rt_task_create(&QP_task, "QP_task", 0, 90, 0);

    //rt_task_start(&PongBotQ_task, &motion_run, NULL);
    //rt_task_start(&Print_task, &print_run, NULL);
    rt_task_start(&IMU_task, &imu_run, NULL);
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

    while (motion_run_flag) {
        //       rt_mutex_acquire(&mutex_desc, TM_INFINITE);
#ifdef _USE_DC     
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);
#else  
        rt_task_wait_period(NULL);
#endif
        previous_motion_time = now_motion_time;
        check_previous_motion_time = rt_timer_read();

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
            sys_ready_flag = 1;
        }

        if (sys_ready_flag == 0) {
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
                sys_ready_flag = 1;
            }///
        } else { // realtime action...           
            EncoderRead();
            GetActualData();

            switch (PongBotQ.ControlMode) {
                case CTRLMODE_NONE:
                    //cout << "============= [CTRLMODE_NONE] ==========" << endl;
                    //PongBotQ.CommandFlag = TORQUE_OFF;
                    //PongBotQ.CommandFlag = NO_ACT;
                    break;

                case CTRLMODE_WALK_READY:
                    cout << "============= [CTRLMODE_WALK_READY] ==========" << endl;
                    PongBotQ.wr_cnt = 0;

                    PongBotQ.com_pos = PongBotQ.init_com_pos;
                    PongBotQ.com_vel << 0, 0, 0;
                    PongBotQ.base_pos = PongBotQ.com_pos + PongBotQ.base_offset;
                    PongBotQ.base_vel << 0, 0, 0;
                    PongBotQ.base_ori << 0, 0, 0;
                    PongBotQ.base_ori_dot << 0, 0, 0;
                    PongBotQ.tmp_com_pos << 0, 0, 0;

                    PongBotQ.CommandFlag = GOTO_WALK_READY_POS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    //PongBotQ.target_pos = PongBotQ.actual_pos;
                    break;

                case CTRLMODE_TROT_WALKING:
                    //cout << "============= [CTRLMODE_TROT_WALKING] ==========" << endl;
                    //                    if (PongBotQ.walk_ready_moving_done_flag == true) {
                    //                        PongBotQ.tw_cnt = 0;
                    //                        PongBotQ.CommandFlag = NOMAL_TROT_WALKING;
                    //                        PongBotQ.move_cnt = 0;
                    //                        PongBotQ.moving_done_flag = false;
                    //                        PongBotQ.Robot_para_init();
                    //                        //					PongBotQ.lpf_tar_pitch_ang = 0;
                    //                    } else {
                    //                        cout << " ======== not yet walk ready ======== " << endl;
                    //                    }
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_FLYING_TROT:
                    //cout << "============= [CTRLMODE_FLYING_TROT] ==========" << endl;
                    //                    if (PongBotQ.walk_ready_moving_done_flag == true) {
                    //                        PongBotQ.ft_cnt = 0;
                    //                        PongBotQ.CommandFlag = FLYING_TROT_RUNNING;
                    //                        PongBotQ.move_cnt = 0;
                    //                        PongBotQ.moving_done_flag = false;
                    //                        PongBotQ.Robot_para_init();
                    //                    } else {
                    //                        cout << " ======== not yet walk ready ======== " << endl;
                    //                    }
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_PRONK_JUMP:
                    //cout << "============= [CTRLMODE_PRONK_JUMP] ==========" << endl;
                    //                    if (PongBotQ.walk_ready_moving_done_flag == true) {
                    //                        PongBotQ.pr_cnt = 0;
                    //                        PongBotQ.JUMP_PHASE = 0;
                    //                        PongBotQ.jump_num = 0;
                    //                        PongBotQ.move_cnt = 0;
                    //                        PongBotQ.moving_done_flag = false;
                    //                        PongBotQ.Robot_para_init();
                    //                        //
                    //                        PongBotQ.CommandFlag = PRONK_JUMP;
                    //                    } else {
                    //                        cout << " ======== not yet walk ready ======== " << endl;
                    //                    }
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_TEST:
                    //                    cout << "============= [CTRLMODE_TEST] ==========" << endl;
                    //                    if (PongBotQ.walk_ready_moving_done_flag == true) {
                    //                        PongBotQ.test_cnt = 0;
                    //                        PongBotQ.CommandFlag = TEST_FLAG;
                    //                        PongBotQ.move_cnt = 0;
                    //                        PongBotQ.moving_done_flag = false;
                    //                        PongBotQ.Robot_para_init();
                    //                    } else {
                    //                        cout << " ======== not yet walk ready ======== " << endl;
                    //                    }
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_TORQUE_OFF:
                    //cout << "============= [CTRLMODE_TORQUE_OFF] ==========" << endl;
                    PongBotQ.CommandFlag = TORQUE_OFF;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_STAIR:
                    //cout << "============= [CTRLMODE_STAIR] ==========" << endl;
                    //                    if (PongBotQ.walk_ready_moving_done_flag == true) {
                    //                        PongBotQ.tw_cnt = 0;
                    //                        PongBotQ.CommandFlag = STAIR_WALKING;
                    //                        PongBotQ.move_cnt = 0;
                    //                        PongBotQ.moving_done_flag = false;
                    //                        PongBotQ.Robot_para_init();
                    //                    } else {
                    //                        cout << " ======== not yet walk ready ======== " << endl;
                    //                    }
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;
                case CTRLMODE_SLOW_WALK_HS:
                    //cout << "============= [CTRLMODE_Walk] ==========" << endl;
                    //PongBotQ.cnt_HS = 0;
                    //                    PongBotQ.now_vel_HS << 0.0, 0.0, 0.0;
                    //                    PongBotQ.tar_vel_HS << 0.0, 0.0, 0.0;
                    //                    PongBotQ.zmp_ref_x_array_HS = VectorNd::Zero(
                    //                            PongBotQ.preview_cnt_HS);
                    //                    PongBotQ.zmp_ref_y_array_HS = VectorNd::Zero(
                    //                            PongBotQ.preview_cnt_HS);
                    //
                    //                    PongBotQ.X_new_x_HS << 0.0, 0.0, 0.0;
                    //                    PongBotQ.X_new_y_HS << 0.0, 0.0, 0.0;
                    //
                    //                    PongBotQ.sum_e_x_HS = 0.0;
                    //                    PongBotQ.sum_e_y_HS = 0.0;
                    //
                    //                    PongBotQ.WalkReady_flag_HS = true;
                    //                    PongBotQ.move_stop_flag = false;
                    //                    PongBotQ.walk_stop_flag_HS = false;
                    //                    PongBotQ.com_stop_flag_HS = false;
                    //                    PongBotQ.foot_height_HS = PongBotQ.tmp_foot_height_HS;
                    //                    PongBotQ.init_Force_flag_HS = true;
                    //
                    //                    PongBotQ.target_base_ori_local_HS = VectorNd::Zero(3);
                    //
                    //                    PongBotQ.Contact_Info_HS << 1, 1, 1, 1;
                    PongBotQ.CommandFlag = GOTO_SLOW_WALK_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;
            }

            switch (PongBotQ.CommandFlag) {
                case NO_ACT:
                    // No action
                    break;

                case TORQUE_OFF:
                    PongBotQ.Torque_off();

                    break;

                case GOTO_HOME_POS:
                    PongBotQ.Torque_off(); // Temp
                    //PongBotQ.Init_Pos_Traj(); // Only simulation
                    break;

                case GOTO_WALK_READY_POS:
                    PongBotQ.StateUpdate();
                    if (PongBotQ.wr_cnt == 0) {
                        if (PongBotQ.Mode == MODE_SIMULATION) {
                            PongBotQ.set_simul_para();
                        } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                            PongBotQ.set_act_robot_para();
                        }

                        PongBotQ.Kp_q = PongBotQ.tar_Kp_q_low;
                        PongBotQ.Kd_q = PongBotQ.tar_Kd_q_low;
                        PongBotQ.Kp_t = PongBotQ.tar_Kp_t_low;
                        PongBotQ.Kd_t = PongBotQ.tar_Kd_t_low;
                        PongBotQ.Kp_x = PongBotQ.tar_Kp_x;
                        PongBotQ.Kd_x = PongBotQ.tar_Kd_x;
                        PongBotQ.Kp_w = PongBotQ.tar_Kp_w;
                        PongBotQ.Kd_w = PongBotQ.tar_Kd_w;
                    }
                    PongBotQ.WalkReady_Pos_Traj();
                    PongBotQ.ComputeTorqueControl();
                    break;

                case NOMAL_TROT_WALKING:
                    //printf("===========================================\n");
                    //dh_save();
                    PongBotQ.StateUpdate();
                    //                    if (PongBotQ.move_cnt == 0) {
                    //
                    //                        if (PongBotQ.Mode == MODE_SIMULATION) {
                    //                            PongBotQ.set_simul_para();
                    //                        } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                    //                            PongBotQ.set_act_robot_para();
                    //                        }
                    //
                    //                        PongBotQ.init_Kp_q = PongBotQ.Kp_q;
                    //                        PongBotQ.init_Kd_q = PongBotQ.Kd_q;
                    //                        PongBotQ.init_Kp_t = PongBotQ.Kp_t;
                    //                        PongBotQ.init_Kd_t = PongBotQ.Kd_t;
                    //                        PongBotQ.init_Kp_x = PongBotQ.Kp_x;
                    //                        PongBotQ.init_Kd_x = PongBotQ.Kd_x;
                    //                        PongBotQ.init_Kp_w = PongBotQ.Kp_w;
                    //                        PongBotQ.init_Kd_w = PongBotQ.Kd_w;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else if (PongBotQ.move_cnt < 1000) {
                    //                        PongBotQ.tmp_weight = 1 / 2.0
                    //                                * (1
                    //                                - cos(
                    //                                PI2 / (2) * (double) (PongBotQ.move_cnt)
                    //                                * PongBotQ.dt));
                    //                        PongBotQ.Kp_q = PongBotQ.init_Kp_q
                    //                                + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_q = PongBotQ.init_Kd_q
                    //                                + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_t = PongBotQ.init_Kp_t
                    //                                + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_t = PongBotQ.init_Kd_t
                    //                                + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_x = PongBotQ.init_Kp_x
                    //                                + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_x = PongBotQ.init_Kd_x
                    //                                + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_w = PongBotQ.init_Kp_w
                    //                                + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_w = PongBotQ.init_Kd_w
                    //                                + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)
                    //                                * PongBotQ.tmp_weight;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else {
                    //                        PongBotQ.Trot_Walking4();
                    //                    }

                    //PongBotQ.ComputeTorqueControl();
                    break;

                case FLYING_TROT_RUNNING:
                    //printf("===========================================\n");
                    //dh_save();
                    PongBotQ.StateUpdate();

                    //                    if (PongBotQ.move_cnt == 0) {
                    //
                    //                        if (PongBotQ.Mode == MODE_SIMULATION) {
                    //                            PongBotQ.set_simul_para();
                    //                        } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                    //                            PongBotQ.set_act_robot_para();
                    //                        }
                    //
                    //                        PongBotQ.init_Kp_q = PongBotQ.Kp_q;
                    //                        PongBotQ.init_Kd_q = PongBotQ.Kd_q;
                    //                        PongBotQ.init_Kp_t = PongBotQ.Kp_t;
                    //                        PongBotQ.init_Kd_t = PongBotQ.Kd_t;
                    //                        PongBotQ.init_Kp_x = PongBotQ.Kp_x;
                    //                        PongBotQ.init_Kd_x = PongBotQ.Kd_x;
                    //                        PongBotQ.init_Kp_w = PongBotQ.Kp_w;
                    //                        PongBotQ.init_Kd_w = PongBotQ.Kd_w;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else if (PongBotQ.move_cnt < 1000) {
                    //                        PongBotQ.tmp_weight = 1 / 2.0
                    //                                * (1
                    //                                - cos(
                    //                                PI2 / (2) * (double) (PongBotQ.move_cnt)
                    //                                * PongBotQ.dt));
                    //                        PongBotQ.Kp_q = PongBotQ.init_Kp_q
                    //                                + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_q = PongBotQ.init_Kd_q
                    //                                + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_t = PongBotQ.init_Kp_t
                    //                                + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_t = PongBotQ.init_Kd_t
                    //                                + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_x = PongBotQ.init_Kp_x
                    //                                + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_x = PongBotQ.init_Kd_x
                    //                                + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_w = PongBotQ.init_Kp_w
                    //                                + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_w = PongBotQ.init_Kd_w
                    //                                + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)
                    //                                * PongBotQ.tmp_weight;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else {
                    //                        PongBotQ.Flying_Trot_Running3();
                    //                    }

                    //PongBotQ.ComputeTorqueControl();

                    break;

                case PRONK_JUMP:
                    PongBotQ.StateUpdate();
                    //                    if (PongBotQ.move_cnt == 0) {
                    //
                    //                        if (PongBotQ.Mode == MODE_SIMULATION) {
                    //                            PongBotQ.set_simul_para();
                    //                        } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                    //                            PongBotQ.set_act_robot_para();
                    //                        }
                    //
                    //                        PongBotQ.init_Kp_q = PongBotQ.Kp_q;
                    //                        PongBotQ.init_Kd_q = PongBotQ.Kd_q;
                    //                        PongBotQ.init_Kp_t = PongBotQ.Kp_t;
                    //                        PongBotQ.init_Kd_t = PongBotQ.Kd_t;
                    //                        PongBotQ.init_Kp_x = PongBotQ.Kp_x;
                    //                        PongBotQ.init_Kd_x = PongBotQ.Kd_x;
                    //                        PongBotQ.init_Kp_w = PongBotQ.Kp_w;
                    //                        PongBotQ.init_Kd_w = PongBotQ.Kd_w;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else if (PongBotQ.move_cnt < 1000) {
                    //                        PongBotQ.tmp_weight = 1 / 2.0
                    //                                * (1
                    //                                - cos(
                    //                                PI2 / (2) * (double) (PongBotQ.move_cnt)
                    //                                * PongBotQ.dt));
                    //                        PongBotQ.Kp_q = PongBotQ.init_Kp_q
                    //                                + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_q = PongBotQ.init_Kd_q
                    //                                + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_t = PongBotQ.init_Kp_t
                    //                                + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_t = PongBotQ.init_Kd_t
                    //                                + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_x = PongBotQ.init_Kp_x
                    //                                + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_x = PongBotQ.init_Kd_x
                    //                                + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_w = PongBotQ.init_Kp_w
                    //                                + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_w = PongBotQ.init_Kd_w
                    //                                + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)
                    //                                * PongBotQ.tmp_weight;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else {
                    //                        PongBotQ.Pronk_Jump();
                    //                    }
                    //PongBotQ.ComputeTorqueControl();

                    break;

                case TEST_FLAG:
                    //        printf("===========================================\n");
                    PongBotQ.StateUpdate();
                    //                    if (PongBotQ.move_cnt == 0) {
                    //
                    //                        if (PongBotQ.Mode == MODE_SIMULATION) {
                    //                            PongBotQ.set_simul_para();
                    //                        } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                    //                            PongBotQ.set_act_robot_para();
                    //                        }
                    //
                    //                        PongBotQ.init_Kp_q = PongBotQ.Kp_q;
                    //                        PongBotQ.init_Kd_q = PongBotQ.Kd_q;
                    //                        PongBotQ.init_Kp_t = PongBotQ.Kp_t;
                    //                        PongBotQ.init_Kd_t = PongBotQ.Kd_t;
                    //                        PongBotQ.init_Kp_x = PongBotQ.Kp_x;
                    //                        PongBotQ.init_Kd_x = PongBotQ.Kd_x;
                    //                        PongBotQ.init_Kp_w = PongBotQ.Kp_w;
                    //                        PongBotQ.init_Kd_w = PongBotQ.Kd_w;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else if (PongBotQ.move_cnt < 1000) {
                    //                        PongBotQ.tmp_weight = 1 / 2.0
                    //                                * (1
                    //                                - cos(
                    //                                PI2 / (2) * (double) (PongBotQ.move_cnt)
                    //                                * PongBotQ.dt));
                    //                        PongBotQ.Kp_q = PongBotQ.init_Kp_q
                    //                                + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_q = PongBotQ.init_Kd_q
                    //                                + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_t = PongBotQ.init_Kp_t
                    //                                + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_t = PongBotQ.init_Kd_t
                    //                                + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_x = PongBotQ.init_Kp_x
                    //                                + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_x = PongBotQ.init_Kd_x
                    //                                + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_w = PongBotQ.init_Kp_w
                    //                                + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_w = PongBotQ.init_Kd_w
                    //                                + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)
                    //                                * PongBotQ.tmp_weight;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else {
                    //                        PongBotQ.Test_Function();
                    //                    }
                    //PongBotQ.Test_Function();
                    //PongBotQ.ComputeTorqueControl();
                    break;

                case STAIR_WALKING:
                    //        printf("===========================================\n");
                    //dh_save();
                    PongBotQ.StateUpdate();
                    //                    if (PongBotQ.move_cnt == 0) {
                    //
                    //                        if (PongBotQ.Mode == MODE_SIMULATION) {
                    //                            PongBotQ.set_simul_para();
                    //                        } else if (PongBotQ.Mode == MODE_ACTUAL_ROBOT) {
                    //                            PongBotQ.set_act_robot_para();
                    //                        }
                    //
                    //                        PongBotQ.init_Kp_q = PongBotQ.Kp_q;
                    //                        PongBotQ.init_Kd_q = PongBotQ.Kd_q;
                    //                        PongBotQ.init_Kp_t = PongBotQ.Kp_t;
                    //                        PongBotQ.init_Kd_t = PongBotQ.Kd_t;
                    //                        PongBotQ.init_Kp_x = PongBotQ.Kp_x;
                    //                        PongBotQ.init_Kd_x = PongBotQ.Kd_x;
                    //                        PongBotQ.init_Kp_w = PongBotQ.Kp_w;
                    //                        PongBotQ.init_Kd_w = PongBotQ.Kd_w;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else if (PongBotQ.move_cnt < 1000) {
                    //                        PongBotQ.tmp_weight = 1 / 2.0
                    //                                * (1
                    //                                - cos(
                    //                                PI2 / (2) * (double) (PongBotQ.move_cnt)
                    //                                * PongBotQ.dt));
                    //                        PongBotQ.Kp_q = PongBotQ.init_Kp_q
                    //                                + (PongBotQ.tar_Kp_q_low - PongBotQ.init_Kp_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_q = PongBotQ.init_Kd_q
                    //                                + (PongBotQ.tar_Kd_q_low - PongBotQ.init_Kd_q)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_t = PongBotQ.init_Kp_t
                    //                                + (PongBotQ.tar_Kp_t_low - PongBotQ.init_Kp_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_t = PongBotQ.init_Kd_t
                    //                                + (PongBotQ.tar_Kd_t_low - PongBotQ.init_Kd_t)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_x = PongBotQ.init_Kp_x
                    //                                + (PongBotQ.tar_Kp_x - PongBotQ.init_Kp_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_x = PongBotQ.init_Kd_x
                    //                                + (PongBotQ.tar_Kd_x - PongBotQ.init_Kd_x)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kp_w = PongBotQ.init_Kp_w
                    //                                + (PongBotQ.tar_Kp_w - PongBotQ.init_Kp_w)
                    //                                * PongBotQ.tmp_weight;
                    //                        PongBotQ.Kd_w = PongBotQ.init_Kd_w
                    //                                + (PongBotQ.tar_Kd_w - PongBotQ.init_Kd_w)
                    //                                * PongBotQ.tmp_weight;
                    //
                    //                        PongBotQ.move_cnt++;
                    //                    } else {
                    //                        PongBotQ.Stair_Walking();
                    //                    }
                    //PongBotQ.ComputeTorqueControl();
                    break;

                case GOTO_SLOW_WALK_POS_HS:
                    PongBotQ.StateUpdate();
                    //                    PongBotQ.Global_Transform_Z_HS();
                    //                    PongBotQ.Slope_Controller3();
                    //
                    //                    if (PongBotQ.WalkReady_flag_HS == true) {
                    //                        PongBotQ.WalkReady_Pos_Traj_HS();
                    //                    } else {
                    //                        dh_save();
                    //                        PongBotQ.Walking_Gait_Traj_HS();
                    //                    }
                    // PongBotQ.ComputeTorqueControl_HS();
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

void print_run(void* arg) {

    //rt_task_set_periodic(NULL, TM_NOW, 100000000);
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 100);

    //rt_printf("Print Thread Start \n");

    while (print_run_flag) {
        rt_task_wait_period(NULL);
        previous_print_time = now_print_time;
        //        rt_mutex_acquire(&mutex_desc, TM_INFINITE);
        inOP = TRUE;
        sys_ready_flag = 1;
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
                //                //                //rt_printf("Status word = (0x%X / 0x%X / 0x%X),(0x%X / 0x%X / 0x%X),(0x%X / 0x%X / 0x%X),(0x%X / 0x%X / 0x%X) \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord, ELMO_drive_pt[5].ptInParam->StatusWord, ELMO_drive_pt[4].ptInParam->StatusWord, ELMO_drive_pt[3].ptInParam->StatusWord, ELMO_drive_pt[6].ptInParam->StatusWord, ELMO_drive_pt[7].ptInParam->StatusWord, ELMO_drive_pt[8].ptInParam->StatusWord, ELMO_drive_pt[11].ptInParam->StatusWord, ELMO_drive_pt[10].ptInParam->StatusWord, ELMO_drive_pt[9].ptInParam->StatusWord);
                //                //                //rt_printf("Actual Torque = (%d / %d / %d),(%d / %d / %d),(%d / %d / %d),(%d / %d / %d)\n", ELMO_drive_pt[0].ptInParam->TorqueActualValue, ELMO_drive_pt[1].ptInParam->TorqueActualValue, ELMO_drive_pt[2].ptInParam->TorqueActualValue, ELMO_drive_pt[5].ptInParam->TorqueActualValue, ELMO_drive_pt[4].ptInParam->TorqueActualValue, ELMO_drive_pt[3].ptInParam->TorqueActualValue, ELMO_drive_pt[6].ptInParam->TorqueActualValue, ELMO_drive_pt[7].ptInParam->TorqueActualValue, ELMO_drive_pt[8].ptInParam->TorqueActualValue, ELMO_drive_pt[11].ptInParam->TorqueActualValue, ELMO_drive_pt[10].ptInParam->TorqueActualValue, ELMO_drive_pt[9].ptInParam->TorqueActualValue);
                //                //                rt_printf("=============================[Sensor]=================================\n");
                //                //                rt_printf("Base_acc(m/s^2) = ( %3f / %3f/ %3f )  ---  Max=(%3f / %3f / %3f)  \n", PongBotQ.IMUAccX, PongBotQ.IMUAccY, PongBotQ.IMUAccZ, x_acc_set(0), y_acc_set(0), z_acc_set(0));
                //                rt_printf("Base_ori(degree) = ( %3f / %3f/ %3f )  ---  Max=(%3f / %3f / %3f) \n", PongBotQ.IMURoll*R2D, PongBotQ.IMUPitch*R2D, PongBotQ.IMUYaw * R2D, roll_set(0) * R2D, pitch_set(0) * R2D, yaw_set(0) * R2D);
                //                rt_printf("Base_ori_vel(rad/s) = ( %3f / %3f/ %3f )  ---  Max=(%3f / %3f / %3f)  \n", PongBotQ.IMURoll_dot, PongBotQ.IMUPitch_dot, PongBotQ.IMUYaw_dot, roll_vel_set(0), pitch_vel_set(0), yaw_vel_set(0));
                //                rt_printf("actual_q(degree) = ( %3f / %3f/ %3f ), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.actual_pos[0] * R2D, PongBotQ.actual_pos[1] * R2D, PongBotQ.actual_pos[2] * R2D, PongBotQ.actual_pos[3] * R2D, PongBotQ.actual_pos[4] * R2D, PongBotQ.actual_pos[5] * R2D, PongBotQ.actual_pos[6] * R2D, PongBotQ.actual_pos[7] * R2D, PongBotQ.actual_pos[8] * R2D, PongBotQ.actual_pos[9] * R2D, PongBotQ.actual_pos[10] * R2D, PongBotQ.actual_pos[11] * R2D);
                //                rt_printf("actual_joint_vel(rad/s) = (%3f / %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.actual_vel[0], PongBotQ.actual_vel[1], PongBotQ.actual_vel[2], PongBotQ.actual_vel[3], PongBotQ.actual_vel[4], PongBotQ.actual_vel[5], PongBotQ.actual_vel[6], PongBotQ.actual_vel[7], PongBotQ.actual_vel[8], PongBotQ.actual_vel[9], PongBotQ.actual_vel[10], PongBotQ.actual_vel[11]);
                //                rt_printf("=============================[Kinematics]=================================\n");
                //rt_printf("actual_EP_pos_local = (%3f / %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.actual_EP_local[0], PongBotQ.actual_EP_local[1], PongBotQ.actual_EP_local[2], PongBotQ.actual_EP_local[3], PongBotQ.actual_EP_local[4], PongBotQ.actual_EP_local[5], PongBotQ.actual_EP_local[6], PongBotQ.actual_EP_local[7], PongBotQ.actual_EP_local[8], PongBotQ.actual_EP_local[9], PongBotQ.actual_EP_local[10], PongBotQ.actual_EP_local[11]);
                //                rt_printf("--> com_pos= ( %3f / %3f/ %3f ) \n", PongBotQ.com_pos(0), PongBotQ.com_pos(1), PongBotQ.com_pos(2));
                //                rt_printf("--> com_vel= ( %3f / %3f/ %3f ) \n", PongBotQ.com_vel(0), PongBotQ.com_vel(1), PongBotQ.com_vel(2));
                //                rt_printf("--> target foot pos= ( %3f / %3f/ %3f ) / ( %3f / %3f/ %3f ) /( %3f / %3f/ %3f ) / ( %3f / %3f/ %3f ) \n", PongBotQ.RL_foot_pos(0), PongBotQ.RL_foot_pos(1), PongBotQ.RL_foot_pos(2), PongBotQ.RR_foot_pos(0), PongBotQ.RR_foot_pos(1), PongBotQ.RR_foot_pos(2), PongBotQ.FL_foot_pos(0), PongBotQ.FL_foot_pos(1), PongBotQ.FL_foot_pos(2), PongBotQ.FR_foot_pos(0), PongBotQ.FR_foot_pos(1), PongBotQ.FR_foot_pos(2));
                //                rt_printf("--> target foot vel= ( %3f / %3f/ %3f ) / ( %3f / %3f/ %3f ) /( %3f / %3f/ %3f ) / ( %3f / %3f/ %3f ) \n", PongBotQ.RL_foot_vel(0), PongBotQ.RL_foot_vel(1), PongBotQ.RL_foot_vel(2), PongBotQ.RR_foot_vel(0), PongBotQ.RR_foot_vel(1), PongBotQ.RR_foot_vel(2), PongBotQ.FL_foot_vel(0), PongBotQ.FL_foot_vel(1), PongBotQ.FL_foot_vel(2), PongBotQ.FR_foot_vel(0), PongBotQ.FR_foot_vel(1), PongBotQ.FR_foot_vel(2));
                //                rt_printf("--> Force = ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) \n", PongBotQ.Fc(6), PongBotQ.Fc(7), PongBotQ.Fc(8), PongBotQ.Fc(9), PongBotQ.Fc(10), PongBotQ.Fc(11), PongBotQ.Fc(12), PongBotQ.Fc(13), PongBotQ.Fc(14), PongBotQ.Fc(15), PongBotQ.Fc(16), PongBotQ.Fc(17));
                //                rt_printf("--> C_Force = ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) \n", PongBotQ.C_term(6), PongBotQ.C_term(7), PongBotQ.C_term(8), PongBotQ.C_term(9), PongBotQ.C_term(10), PongBotQ.C_term(11), PongBotQ.C_term(12), PongBotQ.C_term(13), PongBotQ.C_term(14), PongBotQ.C_term(15), PongBotQ.C_term(16), PongBotQ.C_term(17));
                //                rt_printf("--> G_Force = ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) / ( %3f / %3f / %3f ) \n", PongBotQ.G_term(6), PongBotQ.G_term(7), PongBotQ.G_term(8), PongBotQ.G_term(9), PongBotQ.G_term(10), PongBotQ.G_term(11), PongBotQ.G_term(12), PongBotQ.G_term(13), PongBotQ.G_term(14), PongBotQ.G_term(15), PongBotQ.G_term(16), PongBotQ.G_term(17));

                //                rt_printf("--> RL(L)= ( %3f / %3f/ %3f ) \n", PongBotQ.act_RL_foot_pos_local(0), PongBotQ.act_RL_foot_pos_local(1), PongBotQ.act_RL_foot_pos_local(2));
                //                rt_printf("--> RR(L)= ( %3f / %3f/ %3f ) \n", PongBotQ.act_RR_foot_pos_local(0), PongBotQ.act_RR_foot_pos_local(1), PongBotQ.act_RR_foot_pos_local(2));
                //                rt_printf("--> FL(L)= ( %3f / %3f/ %3f ) \n", PongBotQ.act_FL_foot_pos_local(0), PongBotQ.act_FL_foot_pos_local(1), PongBotQ.act_FL_foot_pos_local(2));
                //                rt_printf("--> FR(L)= ( %3f / %3f/ %3f ) \n", PongBotQ.act_FR_foot_pos_local(0), PongBotQ.act_FR_foot_pos_local(1), PongBotQ.act_FR_foot_pos_local(2));
                //                rt_printf("=============================[Joint's values]=================================\n");
                //                rt_printf("Gravity = (%3f / %3f / %3f),(%3f / %3f / %3f),(%3f / %3f / %3f),(%3f / %3f / %3f) \n", PongBotQ.G_term[6], PongBotQ.G_term[7], PongBotQ.G_term[8], PongBotQ.G_term[9], PongBotQ.G_term[10], PongBotQ.G_term[11], PongBotQ.G_term[12], PongBotQ.G_term[13], PongBotQ.G_term[14], PongBotQ.G_term[15], PongBotQ.G_term[16], PongBotQ.G_term[17]);
                //                rt_printf("Coriolis = (%3f / %3f / %3f),(%3f / %3f / %3f),(%3f / %3f / %3f),(%3f / %3f / %3f) \n", PongBotQ.C_term[6], PongBotQ.C_term[7], PongBotQ.C_term[8], PongBotQ.C_term[9], PongBotQ.C_term[10], PongBotQ.C_term[11], PongBotQ.C_term[12], PongBotQ.C_term[13], PongBotQ.C_term[14], PongBotQ.C_term[15], PongBotQ.C_term[16], PongBotQ.C_term[17]);
                //                rt_printf("TargetTorque = (%3f / %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f), (%3f/ %3f/ %3f) \n", PongBotQ.joint[0].torque, PongBotQ.joint[1].torque, PongBotQ.joint[2].torque, PongBotQ.joint[3].torque, PongBotQ.joint[4].torque, PongBotQ.joint[5].torque, PongBotQ.joint[6].torque, PongBotQ.joint[7].torque, PongBotQ.joint[8].torque, PongBotQ.joint[9].torque, PongBotQ.joint[10].torque, PongBotQ.joint[11].torque);
                ////                
                //                ////                rt_printf("_________________________________________\n");
                //                ////                rt_printf("Joint_P_gain = %3f / %3f / %3f \n", PongBotQ.kp_joint_HS[0], PongBotQ.kp_joint_HS[1], PongBotQ.kp_joint_HS[2]);
                //                ////                rt_printf("Joint_D_gain = %3f / %3f / %3f \n", PongBotQ.kd_joint_HS[0], PongBotQ.kd_joint_HS[1], PongBotQ.kd_joint_HS[2]);
                //                ////                rt_printf("Cart_P_gain = %3f / %3f / %3f \n", PongBotQ.kp_EP_HS[0], PongBotQ.kp_EP_HS[1], PongBotQ.kp_EP_HS[2]);
                //                ////                rt_printf("Cart_D_gain = %3f / %3f / %3f \n", PongBotQ.kd_EP_HS[0], PongBotQ.kd_EP_HS[1], PongBotQ.kd_EP_HS[2]);
                //                ////                rt_printf("_________________________________________\n");
                //                
                //                ////
                //rt_printf("----------------------------------------------------\n");
            }
        }
        // rt_mutex_release(&mutex_desc);
        now_print_time = rt_timer_read();
        print_time = (double) (now_print_time - previous_print_time) / 1000000;
    }
}

void signal_handler(int sig) {

    printf("Program END...\n");

    cout << "[1]" << endl;
    rt_task_delete(&IMU_task);
    close(fd_MIC);
    
    cout << "[2]" << endl;
    rt_task_delete(&PongBotQ_task);

    cout << "[3]" << endl;
    rt_task_delete(&Print_task);

    cout << "[4]" << endl;
    rt_task_delete(&QP_task);

    FileSave();

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
        PongBotQ.Low_ABS_actual_pos[i] = PongBotQ.Count2Rad_ABS(PongBotQ.Low_Resolution[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue);
        //PongBotQ.Low_Incre_actual_pos[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
        PongBotQ.Low_actual_vel[i] = PongBotQ.Count2RadDot(PongBotQ.Low_Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
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
        PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[i];
        if (i == 2) {
            PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[i]*1.5;
        }
    }
    for (int i = 3; i < 6; ++i) {
        PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[8 - i];
        if (i == 5) {
            PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[8 - i]*1.5;
        }
    }
    for (int i = 6; i < 9; ++i) {
        PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[i];
        if (i == 8) {
            PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[i]*1.5;
        }
    }
    for (int i = 9; i < 12; ++i) {
        PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[20 - i];
        if (i == 11) {
            PongBotQ.actual_vel[i] = PongBotQ.Low_actual_vel[20 - i]*1.5;
        }
    }

    // if (PongBotQ.Encoder_Reset_Flag == true) {

    for (int i = 0; i < 3; ++i) {
        PongBotQ.ABS_actual_pos[i] = PongBotQ.Low_ABS_actual_pos[i];
    }
    for (int i = 3; i < 6; ++i) {
        PongBotQ.ABS_actual_pos[i] = PongBotQ.Low_ABS_actual_pos[8 - i];
    }
    for (int i = 6; i < 9; ++i) {
        PongBotQ.ABS_actual_pos[i] = PongBotQ.Low_ABS_actual_pos[i];
    }
    for (int i = 9; i < 12; ++i) {
        PongBotQ.ABS_actual_pos[i] = PongBotQ.Low_ABS_actual_pos[20 - i];
    }

    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        PongBotQ.actual_pos[i] = PongBotQ.ABS_actual_pos[i];
    }
}

void GetActualData(void) {
    PongBotQ.actual_EP_local = PongBotQ.FK(PongBotQ.actual_pos);
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

    printf("  rbdl api version = %d\n", version_test);
    Addons::URDFReadFromFile("/home/rclab/catkin_ws/src/PongBotQ_V6/model/PONGBOT_Q_V6/urdf/PONGBOT_Q_V6.urdf", pongbot_q_model, true, false);
    PongBotQ.setRobotModel(pongbot_q_model);

    //IMU_COM_Setting();

    PongBotQ.ControlMode = CTRLMODE_NONE; //=0
    //PongBotQ.CommandFlag = TORQUE_OFF;
    PongBotQ.CommandFlag = NO_ACT;

}

void TargetTor_Gen(void) {

    static int limit_tor = 3000; //3000;

    //*********** Arrange Target Torque in Elmo's order *************//
    Low_TargetTor[0] = PongBotQ.joint[0].torque; //HR joint
    Low_TargetTor[1] = PongBotQ.joint[1].torque; //HP joint
    Low_TargetTor[2] = PongBotQ.joint[2].torque * 1.5; //Knee joint

    Low_TargetTor[5] = PongBotQ.joint[3].torque; //HR joint
    Low_TargetTor[4] = PongBotQ.joint[4].torque; //HP joint
    Low_TargetTor[3] = PongBotQ.joint[5].torque * 1.5; //Knee joint

    Low_TargetTor[6] = PongBotQ.joint[6].torque; //HR joint
    Low_TargetTor[7] = PongBotQ.joint[7].torque; //HP joint
    Low_TargetTor[8] = PongBotQ.joint[8].torque * 1.5; //Knee joint

    Low_TargetTor[11] = PongBotQ.joint[9].torque; //HR joint
    Low_TargetTor[10] = PongBotQ.joint[10].torque; //HP joint
    Low_TargetTor[9] = PongBotQ.joint[11].torque * 1.5; //Knee joint
    //****************************************************************//

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        if (Low_TargetTor[i] >= limit_tor) {
            Low_TargetTor[i] = limit_tor;
            printf("%d 's torque > %d\n", i, limit_tor);
        } else if (Low_TargetTor[i] <= -limit_tor) {
            Low_TargetTor[i] = -limit_tor;
            printf("%d 's torque <= -%d\n", i, limit_tor);
        }
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->TargetTorque = PongBotQ.Tor2Cur(Low_TargetTor[i], PongBotQ.Low_Kt[i], PongBotQ.Low_Gear[i], PongBotQ.Low_ratedCur[i]);
    }
}

void JoystickCallback(const sensor_msgs::Joy& msg) {
    const double max_x_vel = 0.3; //1.0;
    const double max_y_vel = 0.2;
    const double max_yaw_ori = 5 * D2R; // rad
    static double tmp_y_vel = 0;

    if ((int) msg.buttons[9] == 1) {
        printf("======================== [Torque Off] ==========================\n");
        PongBotQ.ControlMode = CTRLMODE_TORQUE_OFF;
    } else if ((int) msg.buttons[13] == 1) {
        printf("======================== [Walk Ready] ==========================\n");
        if (PongBotQ.moving_done_flag == true) {
            PongBotQ.ControlMode = CTRLMODE_WALK_READY;
        }
    } else if ((int) msg.buttons[2] == 1) {
        printf("======================== [Move Stop] ==========================\n");
        PongBotQ.move_stop_flag = true;
        if (PongBotQ.pre_sub_ctrl_flag_HS == true) {
            PongBotQ.move_stop_flag = false;
        }
    } else if ((int) msg.buttons[12] == 1) {
        printf("======================== [HS's Walk Mode] ==========================\n");
        if (PongBotQ.moving_done_flag == true) {
            PongBotQ.ControlMode = CTRLMODE_SLOW_WALK_HS;
            PongBotQ.move_stop_flag = false;
        }
    }

    tmp_y_vel = -(double) msg.axes[0] / 32767.0 * max_y_vel;
    if (tmp_y_vel < 0.1 && tmp_y_vel > -0.1) {
        PongBotQ.tmp_y_moving_speed = 0;
    } else {
        PongBotQ.tmp_y_moving_speed = tmp_y_vel;
    }
    PongBotQ.tmp_x_moving_speed = -(double) msg.axes[1] / 32767.0 * max_x_vel;
    PongBotQ.tmp_base_ori(2) = -(double) msg.axes[2] / 32767.0 * max_yaw_ori;

    PongBotQ.speed_x_HS = -(double) msg.axes[1] / 32767.0 * 0.06;
    PongBotQ.speed_y_HS = -(double) msg.axes[0] / 32767.0 * 0.04;
    PongBotQ.speed_yaw_HS = -(double) msg.axes[2] / 32767.0 * 4.0 * (PI / 180);
}

void ROSMsgPublish(void) {
    tf::TransformBroadcaster broadcaster;

    m_data.data[0] = PongBotQ.IMURoll;
    m_data.data[1] = PongBotQ.IMUPitch;
    m_data.data[2] = PongBotQ.IMUYaw;
    m_data.data[3] = imu_time;
    m_data.data[4] = check_imu_time;


    //    m_data.data[2] = PongBotQ.target_joint_vel_HS[2];
    //    m_data.data[1] = PongBotQ.actual_joint_pos_HS[1] * R2D;
    //    m_data.data[2] = PongBotQ.actual_joint_pos_HS[2] * R2D;
    //    
    //    m_data.data[3] = PongBotQ.target_joint_pos_HS[0] * R2D;
    //    m_data.data[4] = PongBotQ.target_joint_pos_HS[1] * R2D;
    //    m_data.data[5] = PongBotQ.target_joint_pos_HS[2] * R2D;

    //m_joint_states.header.stamp = ros::Time::now();
    //    m_joint_states.name[0] = "1_RL_HR_JOINT";
    //    m_joint_states.name[1] = "RL_HP_JOINT";
    //    m_joint_states.name[2] = "RL_KN_JOINT";
    //    m_joint_states.name[3] = "2_RR_HR_JOINT";
    //    m_joint_states.name[4] = "RR_HP_JOINT";
    //    m_joint_states.name[5] = "RR_KN_JOINT";
    //    m_joint_states.name[6] = "3_FL_HR_JOINT";
    //    m_joint_states.name[7] = "FL_HP_JOINT";
    //    m_joint_states.name[8] = "FL_KN_JOINT";
    //    m_joint_states.name[9] = "4_FR_HR_JOINT";
    //    m_joint_states.name[10] = "FR_HP_JOINT";
    //    m_joint_states.name[11] = "FR_KN_JOINT";
    //    
    //    m_joint_states.position[0] = PongBotQ.actual_pos[0];
    //    m_joint_states.position[1] = PongBotQ.actual_pos[1];
    //    m_joint_states.position[2] = PongBotQ.actual_pos[2];
    //    m_joint_states.position[3] = PongBotQ.actual_pos[3];
    //    m_joint_states.position[4] = PongBotQ.actual_pos[4];
    //    m_joint_states.position[5] = PongBotQ.actual_pos[5];
    //    m_joint_states.position[6] = PongBotQ.actual_pos[6];
    //    m_joint_states.position[7] = PongBotQ.actual_pos[7];
    //    m_joint_states.position[8] = PongBotQ.actual_pos[8];
    //    m_joint_states.position[9] = PongBotQ.actual_pos[9];
    //    m_joint_states.position[10] = PongBotQ.actual_pos[10];
    //    m_joint_states.position[11] = PongBotQ.actual_pos[11];

    //    m_joint_states.position[] = PongBotQ.target_joint_pos_HS[0];
    //    m_joint_states.position[1] = PongBotQ.target_joint_pos_HS[1];
    //    m_joint_states.position[2] = PongBotQ.target_joint_pos_HS[2];

    //    odom_trans.header.stamp = ros::Time::now();
    //    odom_trans.header.frame_id = "odom";
    //    odom_trans.child_frame_id = "BASE";
    //    //    
    //    odom_trans.transform.translation.x = PongBotQ.init_base_pos(0);
    //    odom_trans.transform.translation.y = PongBotQ.init_base_pos(1);
    //    odom_trans.transform.translation.z = PongBotQ.init_base_pos(2);
    //    odom_trans.transform.rotation.x = 0;
    //    odom_trans.transform.rotation.y = 0;
    //    odom_trans.transform.rotation.z = 0;
    //    odom_trans.transform.rotation.w = 1;

    P_data.publish(m_data);

    //    P_joint_states.publish(m_joint_states);
    //    broadcaster.sendTransform(odom_trans);

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
            fprintf(fp, "%d ", (int) save_array[j][i]);
            //            fprintf(fp, "%c\t", save_array[j][i]);
        }
        fprintf(fp, "%d\n", (int) save_array[j][SAVE_LENGTH - 1]);
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

            //slaveinfo(ecat_ifname);

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

int IMU_COM_Setting() {
    printf(" IMU Setting ... \n");
    struct termios oldtio, newtio;

    //fd_MIC = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    //    fd_MIC = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    fd_MIC = open("/dev/IMU_COM", O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
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
//            PongBotQ.IMURoll = roll_MIC;
//            PongBotQ.IMUPitch = pitch_MIC;
//            PongBotQ.IMUYaw = yaw_MIC;
//            PongBotQ.IMURoll_dot = Gyro_X_MIC;
//            PongBotQ.IMUPitch_dot = Gyro_Y_MIC;
//            PongBotQ.IMUYaw_dot = Gyro_Z_MIC;
//            PongBotQ.IMUAccX = Acc_X_MIC;
//            PongBotQ.IMUAccY = Acc_Y_MIC;
//            PongBotQ.IMUAccZ = Acc_Z_MIC;
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
//            if (PongBotQ.IMURoll > max_IMU_Roll) {
//                PongBotQ.IMURoll = max_IMU_Roll;
//            } else if (PongBotQ.IMURoll < -max_IMU_Roll) {
//                PongBotQ.IMURoll = -max_IMU_Roll;
//            }
//            if (PongBotQ.IMUPitch > max_IMU_Pitch) {
//                PongBotQ.IMUPitch = max_IMU_Pitch;
//            } else if (PongBotQ.IMUPitch < -max_IMU_Pitch) {
//                PongBotQ.IMUPitch = -max_IMU_Pitch;
//            }
//
//            //================== [Limits of Angular Velocity] =============================//
//            if (PongBotQ.IMURoll_dot > max_IMU_Roll_dot) {
//                PongBotQ.IMURoll_dot = max_IMU_Roll_dot;
//            } else if (PongBotQ.IMURoll_dot < -max_IMU_Roll_dot) {
//                PongBotQ.IMURoll_dot = -max_IMU_Roll_dot;
//            }
//            if (PongBotQ.IMUPitch_dot > max_IMU_Pitch_dot) {
//                PongBotQ.IMUPitch_dot = max_IMU_Pitch_dot;
//            } else if (PongBotQ.IMUPitch_dot < -max_IMU_Pitch_dot) {
//                PongBotQ.IMUPitch_dot = -max_IMU_Pitch_dot;
//            }
//            if (PongBotQ.IMUYaw_dot > max_IMU_Yaw_dot) {
//                PongBotQ.IMUYaw_dot = max_IMU_Yaw_dot;
//            } else if (PongBotQ.IMUYaw_dot < -max_IMU_Yaw_dot) {
//                PongBotQ.IMUYaw_dot = -max_IMU_Yaw_dot;
//            }
//            
//            //================== [Limits of Linear Acceleration ] =============================//
//            if (PongBotQ.IMUAccX > max_IMU_AccX) {
//                PongBotQ.IMUAccX = max_IMU_AccX;
//            } else if (PongBotQ.IMUAccX < -max_IMU_AccX) {
//                PongBotQ.IMUAccX = -max_IMU_AccX;
//            }
//            if (PongBotQ.IMUAccY > max_IMU_AccY) {
//                PongBotQ.IMUAccY = max_IMU_AccY;
//            } else if (PongBotQ.IMUAccY < -max_IMU_AccY) {
//                PongBotQ.IMUAccY = -max_IMU_AccY;
//            }
//            if (PongBotQ.IMUAccZ > max_IMU_AccZ) {
//                PongBotQ.IMUAccZ = max_IMU_AccZ;
//            } else if (PongBotQ.IMUAccZ < -max_IMU_AccZ) {
//                PongBotQ.IMUAccZ = -max_IMU_AccZ;
//            }
//            
//         
//            roll_set(1) = PongBotQ.IMURoll;
//            pitch_set(1) = PongBotQ.IMUPitch;
//            yaw_set(1) = PongBotQ.IMUYaw;
//            roll_set = Max_Value_Save(roll_set);
//            pitch_set = Max_Value_Save(pitch_set);
//            yaw_set = Max_Value_Save(yaw_set);
//
//            roll_vel_set(1) = PongBotQ.IMURoll_dot;
//            pitch_vel_set(1) = PongBotQ.IMUPitch_dot;
//            yaw_vel_set(1) = PongBotQ.IMUYaw_dot;
//            roll_vel_set = Max_Value_Save(roll_vel_set);
//            pitch_vel_set = Max_Value_Save(pitch_vel_set);
//            yaw_vel_set = Max_Value_Save(yaw_vel_set);
//
//            x_acc_set(1) = PongBotQ.IMUAccX;
//            y_acc_set(1) = PongBotQ.IMUAccY;
//            z_acc_set(1) = PongBotQ.IMUAccZ;
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
                if (PongBotQ.CommandFlag != NO_ACT && PongBotQ.CommandFlag != TORQUE_OFF) {
                    if (PongBotQ.CommandFlag == GOTO_SLOW_WALK_POS_HS) {
                        //PongBotQ.Get_Opt_F_HS3();
                    } else {
                        PongBotQ.QP_process();
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
