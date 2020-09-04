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

#include "main.h"

// **********Basic libraries*********//
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
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

//PCAN Driver
#include <fcntl.h>      // for pcan     O_RDWR
#include <libpcan.h>    // for pcan library.

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

// loop (main & thread)
int main_run = 1;
int motion_run = 1;
int print_run = 1;
int can_run = 1;
int ros_run = 1;

// Servo
int ServoState = 0;
int servo_ready = 0;
int sys_ready = 0;
VectorXd started = VectorXd::Zero(NUM_OF_ELMO);
uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;

//// Elmo setting
UINT16 maxTorque = 3500;

//Save
#define SAVE_LENGTH 13    //The number of data
//#define SAVE_COUNT 3600000 //Save Time = 3600000[ms]=3600[s]
#define SAVE_COUNT 600000 //Save Time = 3600000[ms]=3600[s]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];

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
//RT_MUTEX mutex_desc; //mutex
RTIME now1, previous1; // Ethercat time
RTIME now2, previous2; // Thread 1 cycle time
RTIME now3, previous3; // Thread 2 cycle time
double del_time1 = 0.0; // Ethercat time
double del_time2 = 0.0; // Thread 1 cycle time
double del_time3 = 0.0; // Thread 1 cycle time
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

//CAN
HANDLE can_handle = nullptr;

//**********************************************//
//*************** 2. Functions ****************//
void ServoOn(void); // : Servo ON
void ServoOff(void); // : Servo Off
void Torque_Off(void);
void motion_task(void* arg); // : Thread 1
void print_task(void* arg); // : Thread 2
void can_task(void* arg); // : Thread 3
void catch_signal(int sig); // : Catch "Ctrl + C signal"
bool ecat_init(void);

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

    // CAN Setting
    printf(" Init CAN ...\n");
    can_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
    CAN_Init(can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);

    // Thread Setting Start
    printf("Create Thread ...\n");
    for (int i = 1; i < 4; ++i) {
        printf("%d...\n", i);
        sleep(1);
    }

    //rt_task_create(&RT_task1, "Motion_task", 0, 99, 0);
    rt_task_create(&RT_task2, "Print_task", 0, 80, 0);
    rt_task_create(&RT_task3, "CAN_task", 0, 95, 0);

    //rt_task_start(&RT_task1, &motion_task, NULL);
    rt_task_start(&RT_task2, &print_task, NULL);
    rt_task_start(&RT_task3, &can_task, NULL);
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
        previous2 = now2;
        previous1 = rt_timer_read();

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
            PongBotQ.Mode_Change();
            switch (PongBotQ.ControlMode) {
                case CTRLMODE_NONE: // 0
                    //cout << "============= [CTRLMODE NONE] ==========" << endl;
                    //PongBotQ.CommandFlag = TORQUE_OFF;
                    break;
                case CTRLMODE_INITIALIZE: //1
                    //cout << "============= [[CTRLMODE INITIALIZES HS] ==========" << endl;  // = Joint Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.ControlMode_print = 1;
                    PongBotQ.CommandFlag = GOTO_INIT_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_WALK_READY_HS: // 2
                    //cout << "============= [[CTRLMODE WALK READY HS] ==========" << endl;    // = Cartesian Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.ControlMode_print = 2;
                    PongBotQ.CommandFlag = GOTO_WALK_READY_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_CYCLE_TEST_HS: // 3
                    //cout << "============= [[CTRLMODE CYCLE TEST HS] ==========" << endl;    // = Cartesian Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.ControlMode_print = 3;
                    PongBotQ.CommandFlag = GOTO_CYCLE_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;
                case CTRLMODE_JOYSTICK_HS: // 4
                    //cout << "============= [[CTRLMODE JOYSITCK HS] ==========" << endl;    // = Cartesian Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.ControlMode_print = 4;
                    PongBotQ.CommandFlag = GOTO_JOYSTICK_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    PongBotQ.JoyMode = JOYMODE_HOME;
                    break;

            }

            switch (PongBotQ.CommandFlag) {

                case NO_ACT:
                    PongBotQ.target_joint_pos_HS = PongBotQ.actual_joint_pos_HS;
                    PongBotQ.target_joint_vel_HS << 0, 0, 0;
                    PongBotQ.target_EP_pos_HS = PongBotQ.actual_EP_pos_local_HS;
                    PongBotQ.target_EP_vel_HS << 0, 0, 0;
                    PongBotQ.ComputeTorqueControl();

                    break;

                case GOTO_INIT_POS_HS:
                    if (PongBotQ.Mode_Change_flag == true) {
                        PongBotQ.Init_Pos_Traj_HS();
                    }
                    PongBotQ.ComputeTorqueControl();

                    break;

                case GOTO_WALK_READY_POS_HS:
                    if (PongBotQ.Mode_Change_flag == true) {
                        PongBotQ.Home_Pos_Traj_HS();
                    }
                    PongBotQ.ComputeTorqueControl();

                    break;

                case GOTO_CYCLE_POS_HS:
                    if (PongBotQ.Mode_Change_flag == true) {
                        PongBotQ.Cycle_Test_Pos_Traj_HS();
                    }
                    PongBotQ.ComputeTorqueControl();

                    break;
                case GOTO_JOYSTICK_POS_HS:
                    if (PongBotQ.Mode_Change_flag == true) {
                        PongBotQ.Joystick_Pos_Traj_HS();
                    }
                    PongBotQ.ComputeTorqueControl();

                    break;

            }
            jointController();
        }

        now2 = rt_timer_read();
        now1 = rt_timer_read();

        del_time1 = (double) (now1 - previous1) / 1000000;
        del_time2 = (double) (now2 - previous2) / 1000000;
        Max_Time_Save(del_time1);
        DataSave();
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

        previous3 = now3;
        //rt_mutex_acquire(&mutex_desc, TM_INFINITE);
        rt_task_wait_period(NULL);

        if (inOP == TRUE) {
            if (!sys_ready) {
                if (stick == 0)
                    //rt_printf("waiting for system ready...\n");
                    if (stick % 10 == 0)
                        // rt_printf("%i\n", stick / 10);
                        stick++;
            } else {

                rt_printf("_______________________________\n");
                rt_printf("Thread_time : %f [ms] / %f [ms] / %f [ms] \n", del_time1, del_time2, del_time3);
                rt_printf("max_time : %f [ms]\n", max_time);
                rt_printf("tmp_flag = %d \n", PongBotQ.tmp_Mode_Change_flag);
                rt_printf("cnt  = %d \n", PongBotQ.cnt_mode_change);
                rt_printf("flag = %d \n", PongBotQ.Mode_Change_flag);
                rt_printf("Mode = %d \n", PongBotQ.ControlMode_print);
                rt_printf("Status word = 0x%X / 0x%X / 0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord);
                rt_printf("Actual Torque = %d / %d / %d \n", ELMO_drive_pt[0].ptInParam->TorqueActualValue, ELMO_drive_pt[1].ptInParam->TorqueActualValue, ELMO_drive_pt[2].ptInParam->TorqueActualValue);
                //rt_printf("Status word = 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X / 0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord, ELMO_drive_pt[3].ptInParam->StatusWord, ELMO_drive_pt[4].ptInParam->StatusWord, ELMO_drive_pt[5].ptInParam->StatusWord, ELMO_drive_pt[6].ptInParam->StatusWord, ELMO_drive_pt[7].ptInParam->StatusWord, ELMO_drive_pt[8].ptInParam->StatusWord, ELMO_drive_pt[9].ptInParam->StatusWord, ELMO_drive_pt[10].ptInParam->StatusWord, ELMO_drive_pt[11].ptInParam->StatusWord, ELMO_drive_pt[12].ptInParam->StatusWord);
                rt_printf("_________________________________________\n");

                //rt_printf("actual_q(degree) = %3f / %3f / %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f \n", PongBotQ.actual_joint_pos_HS[0] * R2D, PongBotQ.actual_joint_pos_HS[1] * R2D, PongBotQ.actual_joint_pos_HS[2] * R2D, PongBotQ.actual_joint_pos_HS[3] * R2D, PongBotQ.actual_joint_pos_HS[4] * R2D, PongBotQ.actual_joint_pos_HS[5] * R2D, PongBotQ.actual_joint_pos_HS[6] * R2D, PongBotQ.actual_joint_pos_HS[7] * R2D, PongBotQ.actual_joint_pos_HS[8] * R2D, PongBotQ.actual_joint_pos_HS[9] * R2D, PongBotQ.actual_joint_pos_HS[10] * R2D, PongBotQ.actual_joint_pos_HS[11] * R2D, PongBotQ.actual_joint_pos_HS[12] * R2D);
                //rt_printf("actual_q_dot = %3f / %3f / %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f/ %3f\n", PongBotQ.actual_joint_vel_HS[0], PongBotQ.actual_joint_vel_HS[1], PongBotQ.actual_joint_vel_HS[2],PongBotQ.actual_joint_vel_HS[3],PongBotQ.actual_joint_vel_HS[4],PongBotQ.actual_joint_vel_HS[5],PongBotQ.actual_joint_vel_HS[6],PongBotQ.actual_joint_vel_HS[7],PongBotQ.actual_joint_vel_HS[8],PongBotQ.actual_joint_vel_HS[9],PongBotQ.actual_joint_vel_HS[10],PongBotQ.actual_joint_vel_HS[11],PongBotQ.actual_joint_vel_HS[12]);

                rt_printf("Incre_actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.Incre_actual_joint_pos_HS[0] * R2D, PongBotQ.Incre_actual_joint_pos_HS[1] * R2D, PongBotQ.Incre_actual_joint_pos_HS[2] * R2D);
                rt_printf("abs_actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.ABS_actual_joint_pos_HS2[0] * R2D, PongBotQ.ABS_actual_joint_pos_HS2[1] * R2D, PongBotQ.ABS_actual_joint_pos_HS2[2] * R2D);
                rt_printf("actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.actual_joint_pos_HS[0] * R2D, PongBotQ.actual_joint_pos_HS[1] * R2D, PongBotQ.actual_joint_pos_HS[2] * R2D);
                rt_printf("actual_q_dot = %3f / %3f / %3f\n", PongBotQ.actual_joint_vel_HS[0], PongBotQ.actual_joint_vel_HS[1], PongBotQ.actual_joint_vel_HS[2]);
                rt_printf("init_q(degree) = %3f / %3f / %3f\n", PongBotQ.init_joint_pos_HS[0] * R2D, PongBotQ.init_joint_pos_HS[1] * R2D, PongBotQ.init_joint_pos_HS[2] * R2D);
                rt_printf("goal_q(degree) = %3f / %3f / %3f \n", PongBotQ.goal_joint_pos_HS[0] * R2D, PongBotQ.goal_joint_pos_HS[1] * R2D, PongBotQ.goal_joint_pos_HS[2] * R2D);
                rt_printf("target_q(degree) = %3f / %3f / %3f \n", PongBotQ.target_joint_pos_HS[0] * R2D, PongBotQ.target_joint_pos_HS[1] * R2D, PongBotQ.target_joint_pos_HS[2] * R2D);
                rt_printf("target_vel = %3f / %3f / %3f \n", PongBotQ.target_joint_vel_HS[0], PongBotQ.target_joint_vel_HS[1], PongBotQ.target_joint_vel_HS[2]);
                rt_printf("_________________________________________\n");
                rt_printf("actual_EP(m) = %3f / %3f / %3f\n", PongBotQ.actual_EP_pos_local_HS[0], PongBotQ.actual_EP_pos_local_HS[1], PongBotQ.actual_EP_pos_local_HS[2]);
                rt_printf("actual_EP_vel(m/s) = %3f / %3f / %3f\n", PongBotQ.actual_EP_vel_local_HS[0], PongBotQ.actual_EP_vel_local_HS[1], PongBotQ.actual_EP_vel_local_HS[2]);
                rt_printf("init_EP(m) = %3f / %3f / %3f\n", PongBotQ.init_EP_pos_HS[0], PongBotQ.init_EP_pos_HS[1], PongBotQ.init_EP_pos_HS[2]);
                rt_printf("goal_EP(m) = %3f / %3f / %3f \n", PongBotQ.goal_EP_pos_HS[0], PongBotQ.goal_EP_pos_HS[1], PongBotQ.goal_EP_pos_HS[2]);
                rt_printf("target_EP(m) = %3f / %3f / %3f \n", PongBotQ.target_EP_pos_HS[0], PongBotQ.target_EP_pos_HS[1], PongBotQ.target_EP_pos_HS[2]);
                rt_printf("target_EP_vel(m/s) = %3f / %3f / %3f \n", PongBotQ.target_EP_vel_HS[0], PongBotQ.target_EP_vel_HS[1], PongBotQ.target_EP_vel_HS[2]);
                rt_printf("_________________________________________\n");
                rt_printf("CTC_Torque = %3f / %3f / %3f \n", PongBotQ.joint[0].torque, PongBotQ.joint[1].torque, PongBotQ.joint[2].torque);
                rt_printf("Joint_Torque = %3f / %3f / %3f \n", PongBotQ.Joint_Controller_HS[6], PongBotQ.Joint_Controller_HS[7], PongBotQ.Joint_Controller_HS[8]);
                rt_printf("Cart_Torque = %3f / %3f / %3f \n", PongBotQ.Cart_Controller_HS[6], PongBotQ.Cart_Controller_HS[7], PongBotQ.Cart_Controller_HS[8]);
                rt_printf("Gravity = %3f / %3f / %3f \n", PongBotQ.G_term[6], PongBotQ.G_term[7], PongBotQ.G_term[8]);
                rt_printf("Coriolis = %3f / %3f / %3f \n", PongBotQ.C_term[6], PongBotQ.C_term[7], PongBotQ.C_term[8]);
                rt_printf("_________________________________________\n");
                rt_printf("Joint_P_gain = %3f / %3f / %3f \n", PongBotQ.kp_joint_HS[0], PongBotQ.kp_joint_HS[1], PongBotQ.kp_joint_HS[2]);
                rt_printf("Joint_D_gain = %3f / %3f / %3f \n", PongBotQ.kd_joint_HS[0], PongBotQ.kd_joint_HS[1], PongBotQ.kd_joint_HS[2]);
                rt_printf("Cart_P_gain = %3f / %3f / %3f \n", PongBotQ.kp_EP_HS[0], PongBotQ.kp_EP_HS[1], PongBotQ.kp_EP_HS[2]);
                rt_printf("Cart_D_gain = %3f / %3f / %3f \n", PongBotQ.kd_EP_HS[0], PongBotQ.kd_EP_HS[1], PongBotQ.kd_EP_HS[2]);
                rt_printf("_________________________________________\n");
                rt_printf("Joy_mode = %d \n", PongBotQ.JoyMode);
                rt_printf("stop_flag= %d \n", PongBotQ.walk_stop_flag);
                rt_printf("Joy_vel = %3f / %3f / %3f \n ", PongBotQ.joy_vel_x, PongBotQ.joy_vel_y, PongBotQ.joy_vel_z);
                rt_printf("Joy_tmp_EP1 = %3f / %3f / %3f \n ", PongBotQ.tmp1_target_EP_pos_HS[0], PongBotQ.tmp1_target_EP_pos_HS[1], PongBotQ.tmp1_target_EP_pos_HS[2]);
                rt_printf("Joy_tmp_EP2 = %3f / %3f / %3f \n ", PongBotQ.tmp2_target_EP_pos_HS[0], PongBotQ.tmp2_target_EP_pos_HS[1], PongBotQ.tmp2_target_EP_pos_HS[2]);
                rt_printf("_________________________________________\n");

                rt_printf("----------------------------------------------------\n");
                //rt_printf("(1)=%3f/(2)=%3f/(3)=%3f/(4)=%3f/(5)=%3f/(6)=%3f/(7)=%3f/(8)=%3f",joy_info[0],joy_info[1],joy_info[2],joy_info[3],joy_info[4],joy_info[5],joy_info[6],joy_info[7],joy_info[8]);
            }
        }
        // rt_mutex_release(&mutex_desc);
        now3 = rt_timer_read();
        del_time3 = (double) (now3 - previous3) / 1000000;
    }
}

void can_task(void* arg) {
    TPCANMsg can_QP_msg[1]; //msgType for Can-Send
    TPCANRdMsg can_recv_msg[1]; //msgType for Can-Read

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

    while (can_run) {
        rt_task_wait_period(NULL);

        LINUX_CAN_Read(can_handle, &can_recv_msg[0]);
        usleep(100);
        printf("canstat:%d\n", CAN_Status(can_handle));
        printf("Frame(Read) = %08lx %02x %02x %02x %02x %02x %02x %02x %02x," "Time Stamp = %u ms\n",
                (unsigned long) can_recv_msg[0].Msg.ID,
                can_recv_msg[0].Msg.DATA[0],
                can_recv_msg[0].Msg.DATA[1],
                can_recv_msg[0].Msg.DATA[2],
                can_recv_msg[0].Msg.DATA[3],
                can_recv_msg[0].Msg.DATA[4],
                can_recv_msg[0].Msg.DATA[5],
                can_recv_msg[0].Msg.DATA[6],
                can_recv_msg[0].Msg.DATA[7],
                can_recv_msg[0].dwTime
                );
        CAN_Write(can_handle, &can_QP_msg[0]);
        can_QP_msg[0].MSGTYPE = MSGTYPE_STANDARD;
        can_QP_msg[0].LEN = 8; //QP;
        can_QP_msg[0].ID = 1; //
        can_QP_msg[0].DATA[0] = 0x33;
        can_QP_msg[0].DATA[1] = 0x33;
        can_QP_msg[0].DATA[2] = 0x22;
        can_QP_msg[0].DATA[3] = 0x22;
        can_QP_msg[0].DATA[4] = 0x11;
        can_QP_msg[0].DATA[5] = 0x11;
        can_QP_msg[0].DATA[6] = 0x00;
        can_QP_msg[0].DATA[7] = 0x00;

        printf("Frame(Write) = %08lx %02x %02x %02x %02x %02x %02x %02x %02x \n",
                (unsigned long) can_recv_msg[0].Msg.ID,
                can_QP_msg[0].DATA[0],
                can_QP_msg[0].DATA[1],
                can_QP_msg[0].DATA[2],
                can_QP_msg[0].DATA[3],
                can_QP_msg[0].DATA[4],
                can_QP_msg[0].DATA[5],
                can_QP_msg[0].DATA[6],
                can_QP_msg[0].DATA[7]
                );
        std::cout << "________________________" << std::endl;
    }
}

void catch_signal(int sig) {

    printf("Program END...\n");

    FileSave();
    CAN_Close(can_handle);
    
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
    if (PongBotQ.Encoder_Reset_Flag == true) {
        for (int i = 0; i < NUM_OF_ELMO; i++) {
            PongBotQ.ABS_actual_joint_pos_HS[i] = PongBotQ.Count2Rad_ABS(PongBotQ.Resolution[i], PongBotQ.Count_tf(PongBotQ.Ratio[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
            PongBotQ.Incre_actual_joint_pos_offset_HS[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
            PongBotQ.actual_joint_pos_HS[i] = PongBotQ.ABS_actual_joint_pos_HS[i] + PongBotQ.Incre_actual_joint_pos_HS[i];
            //PongBotQ.actual_joint_pos_HS[i] = PongBotQ.Incre_actual_joint_pos_HS[i];
            //            if (i == 2) {
            //                PongBotQ.actual_joint_pos_HS[i] = PongBotQ.Incre_actual_joint_pos_HS[i] - PI / 2;
            //            }
        }
        PongBotQ.actual_EP_pos_local_HS = PongBotQ.FK_HS(PongBotQ.actual_joint_pos_HS);
        PongBotQ.pre_actual_EP_pos_local_HS = PongBotQ.actual_EP_pos_local_HS;
        PongBotQ.Encoder_Reset_Flag = false;
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        PongBotQ.ABS_actual_joint_pos_HS2[i] = PongBotQ.Count2Rad_ABS(PongBotQ.Resolution[i], PongBotQ.Count_tf(PongBotQ.Ratio[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
        PongBotQ.Incre_actual_joint_pos_HS[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue) - PongBotQ.Incre_actual_joint_pos_offset_HS[i];
        PongBotQ.actual_joint_pos_HS[i] = PongBotQ.ABS_actual_joint_pos_HS[i] + PongBotQ.Incre_actual_joint_pos_HS[i];
        //PongBotQ.actual_joint_pos_HS[i] = PongBotQ.Incre_actual_joint_pos_HS[i];
        PongBotQ.actual_joint_vel_HS[i] = PongBotQ.Count2RadDot(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
        //        if (i == 2) {
        //            PongBotQ.actual_joint_pos_HS[i] = PongBotQ.Incre_actual_joint_pos_HS[i] - PI / 2;
        //        }
    }
}

void GetActualData(void) {
    PongBotQ.actual_EP_pos_local_HS = PongBotQ.FK_HS(PongBotQ.actual_joint_pos_HS);
    PongBotQ.tmp_actual_joint_vel_HS << 0, 0, 0, 0, 0, 0, PongBotQ.actual_joint_vel_HS;
    PongBotQ.tmp_actual_EP_vel_local_HS = PongBotQ.J_A * PongBotQ.tmp_actual_joint_vel_HS;
    PongBotQ.actual_EP_vel_local_HS = PongBotQ.tmp_actual_EP_vel_local_HS.tail(3);
    //PongBotQ.pre_actual_EP_pos_local_HS = PongBotQ.actual_EP_pos_local_HS;

    PongBotQ.actual_EP_pos_global_HS << PongBotQ.actual_EP_pos_local_HS(0), PongBotQ.actual_EP_pos_local_HS(1), 0;
    PongBotQ.actual_EP_vel_global_HS = (PongBotQ.actual_EP_pos_global_HS - PongBotQ.pre_actual_EP_pos_global_HS) / PongBotQ.dt;
    PongBotQ.pre_actual_EP_pos_global_HS = PongBotQ.actual_EP_pos_global_HS;
    PongBotQ.actual_Base_pos_global_HS << 0, 0, -PongBotQ.actual_EP_pos_local_HS(2);
    PongBotQ.J_HS = PongBotQ.Jacobian_HS(PongBotQ.actual_joint_pos_HS);
}

void Load(void) {
    rbdl_check_api_version(RBDL_API_VERSION);
    version_test = rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);
    Addons::URDFReadFromFile("/home/rclab/catkin_ws/src/PongBotQ_One_Leg/model/PONGBOT_ONE_LEG/urdf/PONGBOT_ONE_LEG.urdf", pongbot_q_model, true, true);
    PongBotQ.setRobotModel(pongbot_q_model);

    PongBotQ.abs_kp_joint_HS << 100.0, 100.0, 100.0;
    PongBotQ.abs_kd_joint_HS << 5.0, 5.0, 5.0;
    PongBotQ.abs_kp_EP_HS << 1000.0, 1000.0, 150.0;
    PongBotQ.abs_kd_EP_HS << 50.0, 50.0, 10.0;

    PongBotQ.kp_joint_HS = PongBotQ.abs_kp_joint_HS;
    PongBotQ.kd_joint_HS = PongBotQ.abs_kd_joint_HS;
    PongBotQ.kp_EP_HS = PongBotQ.abs_kp_EP_HS;
    PongBotQ.kd_EP_HS = PongBotQ.abs_kd_EP_HS;
    PongBotQ.foot_height_HS = 0.15;
    //PongBotQ.foot_height_HS = 0.10;
    //PongBotQ.foot_height_HS = 0.05;

    PongBotQ.init_kp_joint_HS = PongBotQ.kp_joint_HS;
    PongBotQ.init_kd_joint_HS = PongBotQ.kd_joint_HS;
    PongBotQ.init_kp_EP_HS = PongBotQ.kp_EP_HS;
    PongBotQ.init_kd_EP_HS = PongBotQ.kd_EP_HS;

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

    m_data.data[0] = PongBotQ.target_EP_pos_HS[2];
    m_data.data[1] = PongBotQ.target_joint_pos_HS[2];
    //m_data.data[1] = PongBotQ.target_joint_pos_HS[2] * R2D;
    //m_data.data[2] = PongBotQ.target_joint_vel_HS[2];
    //    m_data.data[1] = PongBotQ.actual_joint_pos_HS[1] * R2D;
    //    m_data.data[2] = PongBotQ.actual_joint_pos_HS[2] * R2D;
    //    m_data.data[3] = PongBotQ.target_joint_pos_HS[0] * R2D;
    //    m_data.data[4] = PongBotQ.target_joint_pos_HS[1] * R2D;
    //    m_data.data[5] = PongBotQ.target_joint_pos_HS[2] * R2D;

    m_joint_states.header.stamp = ros::Time::now();
    m_joint_states.name[0] = "HR_JOINT";
    m_joint_states.name[1] = "HP_JOINT";
    m_joint_states.name[2] = "KN_JOINT";
    m_joint_states.position[0] = PongBotQ.target_joint_pos_HS[0];
    m_joint_states.position[1] = PongBotQ.target_joint_pos_HS[1];
    //  m_joint_states.position[2] = PongBotQ.actual_joint_pos_HS[2];
    //m_joint_states.position[0] = PongBotQ.target_joint_pos_HS[0];
    //m_joint_states.position[1] = PongBotQ.target_joint_pos_HS[1];
    m_joint_states.position[2] = PongBotQ.target_joint_pos_HS[2];

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
    save_array[save_cnt][0] = del_time1;
    save_array[save_cnt][1] = del_time2;
    save_array[save_cnt][2] = max_time;
    //    save_array[save_cnt][3] = ELMO_drive_pt[0].ptInParam->StatusWord;
    //    save_array[save_cnt][4] = ELMO_drive_pt[1].ptInParam->StatusWord;
    //    save_array[save_cnt][5] = ELMO_drive_pt[2].ptInParam->StatusWord;
    //    save_array[save_cnt][6] = ELMO_drive_pt[0].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][7] = ELMO_drive_pt[1].ptInParam->TorqueActualValue;
    //    save_array[save_cnt][8] = ELMO_drive_pt[2].ptInParam->TorqueActualValue;
    save_array[save_cnt][3] = PongBotQ.actual_joint_pos_HS[0] * R2D;
    save_array[save_cnt][4] = PongBotQ.actual_joint_pos_HS[1] * R2D;
    save_array[save_cnt][5] = PongBotQ.actual_joint_pos_HS[2] * R2D;





    if (save_cnt < SAVE_COUNT - 1) {
        save_cnt++;
    }
}

void FileSave(void) {
    FILE *fp;

    fp = fopen("HSData.txt", "w");

    for (int j = 0; j <= SAVE_COUNT - 1; ++j) {
        for (int i = 0; i <= SAVE_LENGTH - 1; ++i) {
            fprintf(fp, "%f\t", save_array[j][i]);
        }
        fprintf(fp, "%f\n", save_array[j][SAVE_LENGTH - 1]);
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

void Max_Time_Save(double now_time) {

    if (max_time < now_time) {
        max_time = now_time;
    }
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

