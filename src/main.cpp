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

// **********ROS libraries*********//
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

// **********SOEM library*********//
#include "ethercat.h"
#include "pdo_def.h"
#include "ecat_dc.h"
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

// *********** RVIZ *************//
#include <sensor_msgs/JointState.h>             //for rviz
#include <geometry_msgs/WrenchStamped.h>        //for rviz
//#include <tf/transform_broadcaster.h>           //for rviz
//#include <ignition/math/Vector3.hh>


#include <inttypes.h>

//#include <stdlib.h>
//#include <unistd.h>
//#include <sched.h>
//#include <string.h>
//#include <sys/time.h>
//#include <math.h>
//#include <sys/socket.h>
//#include <netinet/in.h>

//#include <native/task.h>   //can't use
//#include <native/timer.h>   //can't use
//#include <native/mutex.h>   
//#include <alchemy/sem.h>
//#include <boilerplate/trace.h>
//#include <xenomai/init.h>


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
#define SAVE_LENGTH 8    //The number of data
//#define SAVE_COUNT 3600000 //Save Time = 3600000[ms]=3600[s]
#define SAVE_COUNT 3600 //Save Time = 3600000[ms]=3600[s]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];

// ROS Param
ros::Subscriber S_Mode;
ros::Subscriber S_Stop;
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


//**********************************************//
//*************** 2. Functions ****************//
void ServoOn(void); // : Servo ON
void ServoOff(void); // : Servo Off
void Torque_Off(void);
void motion_task(void* arg); // : Thread 1
void print_task(void* arg); // : Thread 2
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

void ROSMsgPublish(void);
static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);

void slaveinfo(char *ifname);
int si_map_sdo(int slave);
int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
int si_map_sii(int slave);
void si_sdo(int cnt);
boolean printSDO = FALSE;
boolean printMAP = TRUE;
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
char* dtype2string(uint16 dtype);
ec_ODlistt ODlist;
ec_OElistt OElist;
char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype);
char usdo[128];
char hstr[1024];

int main(int argc, char* argv[]) {

    printf(" ROS Setting ...\n");
    ros::init(argc, argv, "elmo_pkgs");
    ros::NodeHandle nh;
    
    signal(SIGINT, catch_signal);
    signal(SIGTERM, catch_signal); 
    mlockall(MCL_CURRENT | MCL_FUTURE);
 
    S_Mode = nh.subscribe("Mode", 1, Callback1);
    S_Stop = nh.subscribe("STOP", 1, Callback2);
    P_data = nh.advertise<std_msgs::Float64MultiArray>("ROS_DATA", 1);
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
    rt_task_create(&RT_task2, "Print_task", 0, 80, 0);

    rt_task_start(&RT_task1, &motion_task, NULL);
    rt_task_start(&RT_task2, &print_task, NULL);
    // Thread Setting End
    
    ros::Rate loop_rate(1000);
    while(ros::ok()){
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
                    //cout << "============= [CTRLMODE_NONE] ==========" << endl;
                    //PongBotQ.CommandFlag = TORQUE_OFF;
                    break;
                case CTRLMODE_INITIALIZE: //1
                    //cout << "============= [[CTRLMODE_INITIALIZES HS] ==========" << endl;  // = Joint Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.CommandFlag = GOTO_INIT_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;

                case CTRLMODE_WALK_READY_HS: // 2
                    //cout << "============= [[CTRLMODE_WALK READY HS] ==========" << endl;    // = Cartesian Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.CommandFlag = GOTO_WALK_READY_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
                    break;
                
                case CTRLMODE_CYCLE_TEST_HS: // 3
                    //cout << "============= [[CTRLMODE_WALK READY HS] ==========" << endl;    // = Cartesian Control
                    PongBotQ.cnt_HS = 0;
                    PongBotQ.CommandFlag = GOTO_CYCLE_POS_HS;
                    PongBotQ.ControlMode = CTRLMODE_NONE;
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
                rt_printf("_________________________________________\n");
                rt_printf("Status word = 0x%X / 0x%X / 0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord);
                rt_printf("actual_q(degree) = %3f / %3f / %3f \n", PongBotQ.actual_joint_pos_HS[0] * R2D, PongBotQ.actual_joint_pos_HS[1] * R2D, PongBotQ.actual_joint_pos_HS[2] * R2D);
                rt_printf("actual_q_dot=%3f / %3f / %3f\n", PongBotQ.actual_joint_vel_HS[0], PongBotQ.actual_joint_vel_HS[1], PongBotQ.actual_joint_vel_HS[2]);
                rt_printf("init_q(degree)=%3f / %3f / %3f\n", PongBotQ.init_joint_pos_HS[0] * R2D, PongBotQ.init_joint_pos_HS[1] * R2D, PongBotQ.init_joint_pos_HS[2] * R2D);
                rt_printf("goal_q(degree)=%3f / %3f / %3f \n", PongBotQ.goal_joint_pos_HS[0] * R2D, PongBotQ.goal_joint_pos_HS[1] * R2D, PongBotQ.goal_joint_pos_HS[2] * R2D);
                rt_printf("target_q(degree)=%3f / %3f / %3f \n", PongBotQ.target_joint_pos_HS[0] * R2D, PongBotQ.target_joint_pos_HS[1] * R2D, PongBotQ.target_joint_pos_HS[2] * R2D);
                rt_printf("target_vel=%3f / %3f / %3f \n", PongBotQ.target_joint_vel_HS[0], PongBotQ.target_joint_vel_HS[1], PongBotQ.target_joint_vel_HS[2]);
                rt_printf("_________________________________________\n");
                rt_printf("actual_EP(m)=%3f / %3f / %3f\n", PongBotQ.actual_EP_pos_local_HS[0], PongBotQ.actual_EP_pos_local_HS[1], PongBotQ.actual_EP_pos_local_HS[2]);
                rt_printf("actual_EP_vel(m/s)=%3f / %3f / %3f\n", PongBotQ.actual_EP_vel_local_HS[0], PongBotQ.actual_EP_vel_local_HS[1], PongBotQ.actual_EP_vel_local_HS[2]);
                rt_printf("init_EP(m)=%3f / %3f / %3f\n", PongBotQ.init_EP_pos_HS[0], PongBotQ.init_EP_pos_HS[1], PongBotQ.init_EP_pos_HS[2]);
                rt_printf("goal_EP(m)=%3f / %3f / %3f \n", PongBotQ.goal_EP_pos_HS[0], PongBotQ.goal_EP_pos_HS[1], PongBotQ.goal_EP_pos_HS[2]);
                rt_printf("target_EP(m)=%3f / %3f / %3f \n", PongBotQ.target_EP_pos_HS[0], PongBotQ.target_EP_pos_HS[1], PongBotQ.target_EP_pos_HS[2]);
                rt_printf("target_EP_vel(m/s)=%3f / %3f / %3f \n", PongBotQ.target_EP_vel_HS[0], PongBotQ.target_EP_vel_HS[1], PongBotQ.target_EP_vel_HS[2]);
                rt_printf("_________________________________________\n");
                rt_printf("CTC_Torque=%3f / %3f / %3f \n", PongBotQ.joint[0].torque, PongBotQ.joint[1].torque, PongBotQ.joint[2].torque);
                rt_printf("Joint_Torque=%3f / %3f / %3f \n", PongBotQ.Joint_Controller_HS[6], PongBotQ.Joint_Controller_HS[7], PongBotQ.Joint_Controller_HS[8]);
                rt_printf("Cart_Torque=%3f / %3f / %3f \n", PongBotQ.Cart_Controller_HS[6], PongBotQ.Cart_Controller_HS[7], PongBotQ.Cart_Controller_HS[8]);
                rt_printf("Gravity=%3f / %3f / %3f \n", PongBotQ.G_term[6], PongBotQ.G_term[7], PongBotQ.G_term[8]);
                rt_printf("Coriolis=%3f / %3f / %3f \n", PongBotQ.C_term[6], PongBotQ.C_term[7], PongBotQ.C_term[8]);
                rt_printf("_________________________________________\n");
                rt_printf("Joint_P_gain=%3f / %3f / %3f \n", PongBotQ.kp_joint_HS[0], PongBotQ.kp_joint_HS[1], PongBotQ.kp_joint_HS[2]);
                rt_printf("Joint_D_gain=%3f / %3f / %3f \n", PongBotQ.kd_joint_HS[0], PongBotQ.kd_joint_HS[1], PongBotQ.kd_joint_HS[2]);
                rt_printf("Cart_P_gain=%3f / %3f / %3f \n", PongBotQ.kp_EP_HS[0], PongBotQ.kp_EP_HS[1], PongBotQ.kp_EP_HS[2]);
                rt_printf("Cart_D_gain=%3f / %3f / %3f \n", PongBotQ.kd_EP_HS[0], PongBotQ.kd_EP_HS[1], PongBotQ.kd_EP_HS[2]);
                rt_printf("_________________________________________\n");
                rt_printf("----------------------------------------------------\n");
            }
        }
        // rt_mutex_release(&mutex_desc);
        now3 = rt_timer_read();
        del_time3 = (double) (now3 - previous3) / 1000000;
    }
}

void catch_signal(int sig) {
    
    printf("Program END...\n");
    
    FileSave();
    
    rt_task_delete(&RT_task1);
    rt_task_delete(&RT_task2);
    
    //sleep(2); //after 2[s] Program end
    //exit(1);
    
    ros::shutdown();
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
            PongBotQ.actual_EP_pos_local_HS = PongBotQ.FK_HS(PongBotQ.actual_joint_pos_HS);
            PongBotQ.pre_actual_EP_pos_local_HS = PongBotQ.actual_EP_pos_local_HS;
        }
        PongBotQ.Encoder_Reset_Flag = false;
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        PongBotQ.Incre_actual_joint_pos_HS[i] = PongBotQ.Count2Rad(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue) - PongBotQ.Incre_actual_joint_pos_offset_HS[i];
        PongBotQ.actual_joint_pos_HS[i] = PongBotQ.ABS_actual_joint_pos_HS[i] + PongBotQ.Incre_actual_joint_pos_HS[i];
        PongBotQ.actual_joint_vel_HS[i] = PongBotQ.Count2RadDot(PongBotQ.Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
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

void ROSMsgPublish(void) {
    //tf::TransformBroadcaster broadcaster;
    
    m_data.data[0] = del_time1;
    m_data.data[1] = PongBotQ.target_joint_pos_HS[2] * R2D;
    m_data.data[2] = PongBotQ.target_joint_vel_HS[2];
    //    m_data.data[1] = PongBotQ.actual_joint_pos_HS[1] * R2D;
    //    m_data.data[2] = PongBotQ.actual_joint_pos_HS[2] * R2D;
    //    m_data.data[3] = PongBotQ.target_joint_pos_HS[0] * R2D;
    //    m_data.data[4] = PongBotQ.target_joint_pos_HS[1] * R2D;
    //    m_data.data[5] = PongBotQ.target_joint_pos_HS[2] * R2D;
    
    m_joint_states.header.stamp = ros::Time::now();
    m_joint_states.name[0] = "HR_JOINT";
    m_joint_states.name[1] = "HP_JOINT";
    m_joint_states.name[2] = "KN_JOINT";
//    m_joint_states.position[0] = PongBotQ.actual_joint_pos_HS[0];
//    m_joint_states.position[1] = PongBotQ.actual_joint_pos_HS[1];
//    m_joint_states.position[2] = PongBotQ.actual_joint_pos_HS[2];
    m_joint_states.position[0] = PongBotQ.actual_joint_pos_HS[0];
    m_joint_states.position[1] = PongBotQ.actual_joint_pos_HS[1];
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
//    odom_trans.transform.rotation.w = 1;
    
    
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
   

    if (save_cnt < SAVE_COUNT - 1){
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

            //ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            ec_slave[0].state = EC_STATE_PRE_OP;
            ec_writestate(0);

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
                    wkc += RS3_write32(k + 1, 0x1a07, 0x0005, 0x20a00020);

                    wkc += RS3_write8(k + 1, 0x1a07, 0x0000, 0x05);
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

void slaveinfo(char *ifname)
{
    int cnt, i, j, nSM;
    uint16 ssigen;
    int expectedWKC;

    printf("Starting slaveinfo\n");

    while(EcatError) printf("%s", ec_elist2string());
    printf("%d slaves found and configured.\n",ec_slavecount);
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);
    /* wait for all slaves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
    if (ec_slave[0].state != EC_STATE_SAFE_OP )
    {
        printf("Not all slaves reached safe operational state.\n");
        ec_readstate();
        for(i = 1; i<=ec_slavecount ; i++)
        {
            if(ec_slave[i].state != EC_STATE_SAFE_OP)
            {
                printf("Slave %d State=%2x StatusCode=%4x : %s\n",
                       i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }


    ec_readstate();
    for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
        printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
               cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
               ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
        if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
        printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
               (ec_slave[cnt].activeports & 0x02) > 0 ,
               (ec_slave[cnt].activeports & 0x04) > 0 ,
               (ec_slave[cnt].activeports & 0x08) > 0 );
        printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
        printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
        for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
        {
            if(ec_slave[cnt].SM[nSM].StartAddr > 0)
                printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, ec_slave[cnt].SM[nSM].StartAddr, ec_slave[cnt].SM[nSM].SMlength,
                       (int)ec_slave[cnt].SM[nSM].SMflags, ec_slave[cnt].SMtype[nSM]);
        }
        for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
        {
            printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                   (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
                   ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
                   ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
        }
        printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
               ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
        printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
        ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
        /* SII general section */
        if (ssigen)
        {
            ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
            ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
            ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
            ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
            if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
            {
                ec_slave[cnt].blockLRW = 1;
                ec_slave[0].blockLRW++;
            }
            ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
            ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
            ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
        }
        printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
               ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
        printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
               ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
        if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
            si_sdo(cnt);
        if(printMAP)
        {
            if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                si_map_sdo(cnt);
            else
                si_map_sii(cnt);
        }
    }
}

int si_map_sdo(int slave)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;

    printf("PDO mapping according to CoE :\n");
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM); nSM = 0;
    /* read SyncManager Communication Type object count */
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > EC_MAXSM)
            nSM = EC_MAXSM;
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            rdl = sizeof(tSM); tSM = 0;
            /* read SyncManager Communication Type */
            wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if (wkc > 0)
            {
                if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                    printf("Activated SM type workaround, possible incorrect mapping.\n");
                }
                if(tSM)
                    tSM += SMt_bug_add; // only add if SMt > 0

                if (tSM == 3) // outputs
                {
                    /* read the assign RXPDO */
                    printf("  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap[0]), outputs_bo );
                    outputs_bo += Tsize;
                }
                if (tSM == 4) // inputs
                {
                    /* read the assign TXPDO */
                    printf("  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap[0]), inputs_bo );
                    inputs_bo += Tsize;
                }
            }
        }
    }

    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset)
{
    uint16 a , w, c, e, er, Size;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;
    Size = 0;
    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
        a = PDO->Startpos;
        w = ec_siigetbyte(slave, a++);
        w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
        /* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
            PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            /* number of entries in PDO */
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
            a++;
            obj_name = ec_siigetbyte(slave, a++);
            a += 2;
            c += 2;
            if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
            {
                str_name[0] = 0;
                if(obj_name)
                    ec_siistring(str_name, slave, obj_name);
                if (t)
                    printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                else
                    printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                printf("     addr b   index: sub bitl data_type    name\n");
                /* read all entries defined in PDO */
                for (er = 1; er <= e; er++)
                {
                    c += 4;
                    obj_idx = ec_siigetbyte(slave, a++);
                    obj_idx += (ec_siigetbyte(slave, a++) << 8);
                    obj_subidx = ec_siigetbyte(slave, a++);
                    obj_name = ec_siigetbyte(slave, a++);
                    obj_datatype = ec_siigetbyte(slave, a++);
                    bitlen = ec_siigetbyte(slave, a++);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;

                    PDO->BitSize[PDO->nPDO] += bitlen;
                    a += 2;

                    /* skip entry if filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                    {
                        str_name[0] = 0;
                        if(obj_name)
                            ec_siistring(str_name, slave, obj_name);

                        printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                        printf(" %-12s %s\n", dtype2string(obj_datatype), str_name);
                    }
                    bitoffset += bitlen;
                    totalsize += bitlen;
                }
                PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
                Size += PDO->BitSize[PDO->nPDO];
                c++;
            }
            else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
            {
                c += 4 * e;
                a += 8 * e;
                c++;
            }
            if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
        }
        while (c < PDO->Length);
    }
    if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}


int si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    printf("PDO mapping according to SII :\n");

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8*)&IOmap), outputs_bo );
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8*)&IOmap), inputs_bo );
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

void si_sdo(int cnt)
{
    int i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if( ec_readODlist(cnt, &ODlist))
    {
        printf(" CoE Object Description found, %d entries.\n",ODlist.Entries);
        for( i = 0 ; i < ODlist.Entries ; i++)
        {
            ec_readODdescription(i, &ODlist);
            while(EcatError) printf("%s", ec_elist2string());
            printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
                   ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i], ODlist.Name[i]);
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(i, &ODlist, &OElist);
            while(EcatError) printf("%s", ec_elist2string());
            for( j = 0 ; j < ODlist.MaxSub[i]+1 ; j++)
            {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
                {
                    printf("  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x Name: %s\n",
                           j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j], OElist.Name[j]);
                    if ((OElist.ObjAccess[j] & 0x0007))
                    {
                        printf("          Value :%s\n", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
                    }
                }
            }
        }
    }
    else
    {
        while(EcatError) printf("%s", ec_elist2string());
    }
}

int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;
    int abs_offset, abs_bit;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
        /* number of available sub indexes */
        nidx = rdat;
        bsize = 0;
        /* read all PDO's */
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            rdl = sizeof(rdat); rdat = 0;
            /* read PDO assign */
            wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            /* result is index of PDO */
            idx = etohl(rdat);
            if (idx > 0)
            {
                rdl = sizeof(subcnt); subcnt = 0;
                /* read number of subindexes of PDO */
                wkc = ec_SDOread(slave,idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                subidx = subcnt;
                /* for each subindex */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    rdl = sizeof(rdat2); rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    bitlen = LO_BYTE(rdat2);
                    bsize += bitlen;
                    obj_idx = (uint16)(rdat2 >> 16);
                    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;
                    ODlist.Slave = slave;
                    ODlist.Index[0] = obj_idx;
                    OElist.Entries = 0;
                    wkc = 0;
                    /* read object entry from dictionary if not a filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                        wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                    printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                    if((wkc > 0) && OElist.Entries)
                    {
                        printf(" %-12s %s\n", dtype2string(OElist.DataType[obj_subidx]), OElist.Name[obj_subidx]);
                    }
                    else
                        printf("\n");
                    bitoffset += bitlen;
                };
            };
        };
    };
    /* return total found bitlength (PDO) */
    return bsize;
}

char* dtype2string(uint16 dtype)
{
    switch(dtype)
    {
    case ECT_BOOLEAN:
        sprintf(hstr, "BOOLEAN");
        break;
    case ECT_INTEGER8:
        sprintf(hstr, "INTEGER8");
        break;
    case ECT_INTEGER16:
        sprintf(hstr, "INTEGER16");
        break;
    case ECT_INTEGER32:
        sprintf(hstr, "INTEGER32");
        break;
    case ECT_INTEGER24:
        sprintf(hstr, "INTEGER24");
        break;
    case ECT_INTEGER64:
        sprintf(hstr, "INTEGER64");
        break;
    case ECT_UNSIGNED8:
        sprintf(hstr, "UNSIGNED8");
        break;
    case ECT_UNSIGNED16:
        sprintf(hstr, "UNSIGNED16");
        break;
    case ECT_UNSIGNED32:
        sprintf(hstr, "UNSIGNED32");
        break;
    case ECT_UNSIGNED24:
        sprintf(hstr, "UNSIGNED24");
        break;
    case ECT_UNSIGNED64:
        sprintf(hstr, "UNSIGNED64");
        break;
    case ECT_REAL32:
        sprintf(hstr, "REAL32");
        break;
    case ECT_REAL64:
        sprintf(hstr, "REAL64");
        break;
    case ECT_BIT1:
        sprintf(hstr, "BIT1");
        break;
    case ECT_BIT2:
        sprintf(hstr, "BIT2");
        break;
    case ECT_BIT3:
        sprintf(hstr, "BIT3");
        break;
    case ECT_BIT4:
        sprintf(hstr, "BIT4");
        break;
    case ECT_BIT5:
        sprintf(hstr, "BIT5");
        break;
    case ECT_BIT6:
        sprintf(hstr, "BIT6");
        break;
    case ECT_BIT7:
        sprintf(hstr, "BIT7");
        break;
    case ECT_BIT8:
        sprintf(hstr, "BIT8");
        break;
    case ECT_VISIBLE_STRING:
        sprintf(hstr, "VISIBLE_STRING");
        break;
    case ECT_OCTET_STRING:
        sprintf(hstr, "OCTET_STRING");
        break;
    default:
        sprintf(hstr, "Type 0x%4.4X", dtype);
    }
    return hstr;
}

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
    int l = sizeof(usdo) - 1, i;
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
    if (EcatError)
    {
        return ec_elist2string();
    }
    else
    {
        switch(dtype)
        {
        case ECT_BOOLEAN:
            u8 = (uint8*) &usdo[0];
            if (*u8) sprintf(hstr, "TRUE");
            else sprintf(hstr, "FALSE");
            break;
        case ECT_INTEGER8:
            i8 = (int8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %d", *i8, *i8);
            break;
        case ECT_INTEGER16:
            i16 = (int16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %d", *i16, *i16);
            break;
        case ECT_INTEGER32:
        case ECT_INTEGER24:
            i32 = (int32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %d", *i32, *i32);
            break;
        case ECT_INTEGER64:
            i64 = (int64*) &usdo[0];
            sprintf(hstr, "0x%16.16" PRIx64 " %" PRId64, *i64, *i64);
            break;
        case ECT_UNSIGNED8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %u", *u8, *u8);
            break;
        case ECT_UNSIGNED16:
            u16 = (uint16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %u", *u16, *u16);
            break;
        case ECT_UNSIGNED32:
        case ECT_UNSIGNED24:
            u32 = (uint32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %u", *u32, *u32);
            break;
        case ECT_UNSIGNED64:
            u64 = (uint64*) &usdo[0];
            sprintf(hstr, "0x%16.16" PRIx64 " %" PRIu64, *u64, *u64);
            break;
        case ECT_REAL32:
            sr = (float*) &usdo[0];
            sprintf(hstr, "%f", *sr);
            break;
        case ECT_REAL64:
            dr = (double*) &usdo[0];
            sprintf(hstr, "%f", *dr);
            break;
        case ECT_BIT1:
        case ECT_BIT2:
        case ECT_BIT3:
        case ECT_BIT4:
        case ECT_BIT5:
        case ECT_BIT6:
        case ECT_BIT7:
        case ECT_BIT8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%x", *u8);
            break;
        case ECT_VISIBLE_STRING:
            strcpy(hstr, usdo);
            break;
        case ECT_OCTET_STRING:
            hstr[0] = 0x00;
            for (i = 0 ; i < l ; i++)
            {
                sprintf(es, "0x%2.2x ", usdo[i]);
                strcat( hstr, es);
            }
            break;
        default:
            sprintf(hstr, "Unknown type");
        }
        return hstr;
    }
}
