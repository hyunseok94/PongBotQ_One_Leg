/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */

// **********Basic libraries*********//
#include <sys/mman.h>
#include <signal.h>
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

#define NUM_OF_ELMO 3
#define _USE_DC

//#define R2D 180/PI
//#define D2R PI/180

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
//*************** 1. Variables ****************/
// Ethercat
char ecat_ifname[32] = "eth0";
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
unsigned int cycle_ns = 1000000;
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

// Elmo setting
UINT16 maxTorque = 3500;
bool Encoder_Reset_Flag = true;
int Gear[NUM_OF_ELMO] = {50, 50, 50};
int Ratio[NUM_OF_ELMO] = {1, 1, 256};
double ratedCur[NUM_OF_ELMO] = {2850, 2850, 8900};
int32_t Resolution[NUM_OF_ELMO] = {65536, 65536, 16384}; //16384(2^14)

// Gain
VectorXd kp_joint_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd kd_joint_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd CTC_Torque = VectorXd::Zero(NUM_OF_ELMO);
VectorXd kp_EP_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd kd_EP_HS = VectorXd::Zero(NUM_OF_ELMO);

//Save
#define SAVE_LENGTH 8    //The number of data
#define SAVE_COUNT 10000 //Save Time = 10000[ms]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];

// Trajectory
double dt = 0.001;
double cycle_time_HS = 0.0;
unsigned int cnt = 0;
int Controlmode_HS = 0;
unsigned int cnt_HS = 0;
VectorXd actual_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd ABS_actual_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd Incre_actual_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd Incre_actual_joint_pos_offset_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd actual_joint_vel_HS = VectorXd::Zero(NUM_OF_ELMO);

VectorXd init_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd target_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd target_joint_vel_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd goal_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd goal_joint_vel_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd joint_pos_err_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd joint_vel_err_HS = VectorXd::Zero(NUM_OF_ELMO);

VectorXd actual_EP_pos_local_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd pre_actual_EP_pos_local_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd actual_EP_vel_local_HS = VectorXd::Zero(NUM_OF_ELMO);

VectorXd actual_EP_pos_global_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd pre_actual_EP_pos_global_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd actual_EP_vel_global_HS = VectorXd::Zero(NUM_OF_ELMO);

VectorXd init_EP_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd target_EP_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd target_EP_vel_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd goal_EP_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd goal_EP_vel_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd EP_pos_err_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd EP_vel_err_HS = VectorXd::Zero(NUM_OF_ELMO);

VectorXd actual_Base_pos_global_HS = VectorXd::Zero(NUM_OF_ELMO);
MatrixXd J_HS = MatrixXd::Zero(3, 3);
VectorXd Cart_Controller_HS = VectorXd::Zero(NUM_OF_ELMO);

bool tmp_Control_mode_flag = false;
bool Control_mode_flag = false;
unsigned int cnt_Control_mode = 0;


// ROS Param
ros::Subscriber S_Mode;
ros::Publisher P_data;
int ros_exit = 0;
std_msgs::Float64MultiArray m_data;

double Thread_time = 0.0;

//Xenomai
RT_TASK RT_task1;
RT_TASK RT_task2;
RT_TASK RT_task3;
RT_MUTEX mutex_desc; //mutex
RTIME now, previous;

clock_t start_point, end_point;

double del_time = 0.0;
double del_time2 = 0.0;

//RBDL
int version_test;
Model* pongbot_q_model = new Model();

//Thread// 
//pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_t p_thread1;
//pthread_t p_thread2;


//**********************************************//
//*************** 2. Functions ****************//
void motion_task(void* arg); // : Thread 1
void print_task(void* arg); // : Thread 2
void catch_signal(int sig); // : Catch "Ctrl + C signal"
void ServoOn(void); // : Servo ON
void ServoOff(void); // : Servo Off
void EncoderRead(void); // : q, q_dot Read
double Count2Deg(int Gear_Ratio, INT32 Count); // : Transform function
double Count2DegDot(int Gear_Ratio, INT32 CountPerSec); // : Transform function
double Count2Rad(int Gear_Ratio, INT32 Count); // : Transform function
double Count2RadDot(int Gear_Ratio, INT32 CountPerSec); // : Transform function
double Count2Rad_ABS(int _Resolution, INT32 Count); // : Transform function
INT32 Count_tf(int _Ratio, INT32 _Count_in); // : Transform function
INT16 Cur2Tor(double targetCur, double _ratedCur); // : Transform function
void initial_param_setting(void); // : parameter initial setting
void Test_Pos_Traj_HS(void);
void ComputeTorqueControl_HS(void);
void jointController(void);
void Cal_Fc_HS(void);

VectorXd FK_HS(VectorXd joint_pos);
VectorXd IK_HS(VectorXd EP_pos);
MatrixXd Jacobian_HS(VectorXd joint_pos);
VectorXd Localization_Hip2Base_Pos_HS(VectorXd EP_pos_local_hip);
void DataSave(void);
void FileSave(void);

// ROS function
void Callback1(const std_msgs::Int32 &msg);
void ROS_MSG(void);

//RBDL
//void setRobotModel(Model* getModel);
//RigidBodyDynamics::Model* m_pModel; //* URDF Model
//RigidBodyDynamics::Math::VectorNd RobotState;
//RigidBodyDynamics::Math::VectorNd RobotStatedot;
//RigidBodyDynamics::Math::VectorNd RobotState2dot;
//RigidBodyDynamics::Math::VectorNd BasePosOri;
//RigidBodyDynamics::Math::VectorNd BaseVel;
//RigidBodyDynamics::Math::VectorNd JointAngle;
//RigidBodyDynamics::Math::VectorNd JointVel;

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
                    printf("Re mapping for ELMO...\n");
                    ob3 = 0;
                    wkc_count += ec_SDOwrite(k + 1, 0x1c12, 0x00, FALSE, sizeof (ob3), &ob3, EC_TIMEOUTRXM);
                    wkc_count += ec_SDOwrite(k + 1, 0x1c13, 0x00, FALSE, sizeof (ob3), &ob3, EC_TIMEOUTRXM);

                    ob2 = 0x1605;
                    wkc_count += ec_SDOwrite(k + 1, 0x1c12, 0x01, FALSE, sizeof (ob2), &ob2, EC_TIMEOUTRXM);
                    ob3 = 1;
                    wkc_count += ec_SDOwrite(k + 1, 0x1c12, 0x00, FALSE, sizeof (ob3), &ob3, EC_TIMEOUTRXM);

                    ob2 = 0x1A03;
                    wkc_count += ec_SDOwrite(k + 1, 0x1c13, 0x01, FALSE, sizeof (ob2), &ob2, EC_TIMEOUTRXM);
                    ob2 = 0x1A1E;
                    wkc_count += ec_SDOwrite(k + 1, 0x1c13, 0x02, FALSE, sizeof (ob2), &ob2, EC_TIMEOUTRXM);
                    ob3 = 2;
                    //ob3=1;
                    wkc_count += ec_SDOwrite(k + 1, 0x1c13, 0x00, FALSE, sizeof (ob3), &ob3, EC_TIMEOUTRXM);
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

int main(int argc, char *argv[]) {

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    rt_mutex_create(&mutex_desc, "MyMutex");

    printf("ROS Setting ...\n");
    ros::init(argc, argv, "elmo_pkgs");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1000);
    //ros::Rate loop_rate(100);
    S_Mode = nh.subscribe("Mode", 1, Callback1);
    P_data = nh.advertise<std_msgs::Float64MultiArray>("ROS_DATA", 1);
    m_data.data.resize(10);
    sleep(1);

    printf("Init main ...\n");
    sleep(1);

    // Display Adapter name
    printf("Use default adapter %s ...\n", ecat_ifname);
    sleep(1);

    // Initial Setting
    printf("Initialize Prams...\n");
    initial_param_setting();


    // Thread Setting Start
    printf("Create Thread ...\n");
    for (int i = 1; i < 4; ++i) {
        printf("%d...\n", i);
        sleep(1);
    }

    rt_task_create(&RT_task1, "Motion_task", 0, 99, 0);
    rt_task_create(&RT_task2, "Print_task", 0, 70, 0);

    rt_task_start(&RT_task1, &motion_task, NULL);
    rt_task_start(&RT_task2, &print_task, NULL);
    // Thread Setting End


    while (ros::ok()) {
        rt_mutex_acquire(&mutex_desc, TM_INFINITE);
        // printf("main_running\n");
        rt_mutex_release(&mutex_desc);
        ROS_MSG();
        ros::spinOnce();
    }
    FileSave();
    return 0;
}

void motion_task(void* arg) {

    printf("Motion Thread Start \n");

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
        
#ifdef _USE_DC     
        rt_ts += (RTIME) (cycle_ns + toff);
        rt_task_sleep_until(rt_ts);
#else  
        rt_task_wait_period(NULL);
#endif

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
        //        if (inOP == TRUE) {
        //            if (!sys_ready) {
        //                if (stick == 0)
        //                    printf("waiting for system ready...\n");
        //                if (stick % 10 == 0)
        //                    // printf("%l\n", stick / 10);
        //                    stick++;
        //            }
        //        }
        previous = now;
        start_point = clock();
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
            rt_mutex_acquire(&mutex_desc, TM_INFINITE);
            EncoderRead();
            Test_Pos_Traj_HS();
            ComputeTorqueControl_HS();
            jointController();
            //DataSave();
        }

        DataSave();
        rt_mutex_release(&mutex_desc);
        
        now = rt_timer_read();
        end_point = clock();
        del_time = (double)(now - previous)/1000000;
        del_time2 = (double)(rt_timer_read() - previous)/1000000;
        
        
    }
    
    rt_task_sleep(cycle_ns);
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

    rt_printf("End simple test, close socket\n");
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

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

    rt_printf("Print Thread Start \n");

    while (print_run) {
        rt_task_wait_period(NULL);
        //now = rt_timer_read();
        rt_mutex_acquire(&mutex_desc, TM_INFINITE);

        if (inOP == TRUE) {
            if (!sys_ready) {
                if (stick == 0)
                    rt_printf("waiting for system ready...\n");
                if (stick % 10 == 0)
                    // rt_printf("%i\n", stick / 10);
                    stick++;
            } else {
                rt_printf("tmp_flag=%d\n", tmp_Control_mode_flag);
                rt_printf("cnt=%d\n", cnt_Control_mode);
                rt_printf("flag=%d\n", Control_mode_flag);
                rt_printf("Controlmode=%d\n", Controlmode_HS);
                rt_printf("_______________________________\n");

                for (int i = 0; i < NUM_OF_ELMO; ++i) {
                    //ELMO Status
                    rt_printf("ELMO_Drive#%i\n", i + 1);
                    rt_printf("Status word = 0x%X\n", ELMO_drive_pt[i].ptInParam->StatusWord);
                    rt_printf("actual_q(degree) = %3f\n", actual_joint_pos_HS[i] * R2D);
                    rt_printf("ABS_actual_q(degree) = %3f\n", ABS_actual_joint_pos_HS[i] * R2D);
                    rt_printf("actual_q_dot=%3f\n", actual_joint_vel_HS[i]);
                    rt_printf("init_q(degree)=%3f\n", init_joint_pos_HS[i] * R2D);
                    rt_printf("goal_q(degree)=%3f\n", goal_joint_pos_HS[i] * R2D);
                    rt_printf("target_q(degree)=%3f\n", target_joint_pos_HS[i] * R2D);
                    rt_printf("CTC_Torque=%3f\n", CTC_Torque[i]);
                    rt_printf("target_vel=%3f\n", target_joint_vel_HS[i]);
                    rt_printf("___________________________________________________\n");
                }
                //                printf("actual_x=%3f[m]\n", actual_EP_pos_local_HS(0));
                //                printf("actual_y=%3f[m]\n", actual_EP_pos_local_HS(1));
                //                printf("actual_z=%3f[m]\n", actual_EP_pos_local_HS(2));
                //                printf("_______________________________________

//                rt_printf("Thread_time : %ld.%06ld ms\n", (long) (now - previous) / 1000000, (long) (now - previous) % 1000000);
                rt_printf("Thread_time1 : %f ms........\n", del_time);
                rt_printf("Thread_time2 : %f ms........\n", del_time2);
                rt_printf("Thread_time3 : %f ms........\n", (double)(end_point - start_point)/CLOCKS_PER_SEC*1000.0);  //CLOCKS_PER_SEC=1000000
                // printf("Thread_time=%6f [s]\n", Thread_time);____________\n");
                //                                printf("init_x=%3f[m]\n", init_EP_pos_HS(0));
                //                                printf("init_y=%3f[m]\n", init_EP_pos_HS(1));
                //                                printf("init_z=%3f[m]\n", init_EP_pos_HS(2));
                //                                printf("___________________________________________________\n");
                //                                printf("goal_x=%3f[m]\n", goal_EP_pos_HS(0));
                //                                printf("goal_y=%3f[m]\n", goal_EP_pos_HS(1));
                //                                printf("goal_z=%3f[m]\n", goal_EP_pos_HS(2));
                //printf("___________________________________________________\n");
                //                printf("target_x=%3f[m]\n", target_EP_pos_HS(0));
                //                printf("target_y=%3f[m]\n", target_EP_pos_HS(1));
                //                printf("target_z=%3f[m]\n", target_EP_pos_HS(2));
                rt_printf("-----------------------------------------------------------------------\n");
            }

        }
        rt_mutex_release(&mutex_desc);
    }
}

void catch_signal(int sig) {
    FileSave();
    printf("Program END...\n");
    rt_task_delete(&RT_task1);
    rt_task_delete(&RT_task2);
    rt_mutex_delete(&mutex_desc);
    ros::shutdown();

    sleep(2); //after 2[s] Program end
    exit(1);
}

void ServoOn(void) {
    //servo-on	
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        controlword = 0;
        started[i] = ServoOn_GetCtrlWrd(ELMO_drive_pt[i].ptInParam->StatusWord, &controlword);
        // printf("Started[%d] is %d\n", i, started[i]);
        ELMO_drive_pt[i].ptOutParam->ControlWord = controlword;
        if (started[i]) ServoState |= (1 << i);
    }
}

void ServoOff(void) {
    printf("function is Servo_off");
    //Servo OFF
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
    }
}

void EncoderRead(void) {
    if (Encoder_Reset_Flag == true) {
        for (int i = 0; i < NUM_OF_ELMO; i++) {
            ABS_actual_joint_pos_HS[i] = Count2Rad_ABS(Resolution[i], Count_tf(Ratio[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
            Incre_actual_joint_pos_offset_HS[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
            actual_joint_pos_HS[i] = ABS_actual_joint_pos_HS[i] + Incre_actual_joint_pos_HS[i];
            actual_EP_pos_local_HS = FK_HS(actual_joint_pos_HS);
            pre_actual_EP_pos_local_HS = actual_EP_pos_local_HS;
        }
        Encoder_Reset_Flag = false;
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        Incre_actual_joint_pos_HS[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue) - Incre_actual_joint_pos_offset_HS[i];
        actual_joint_pos_HS[i] = ABS_actual_joint_pos_HS[i] + Incre_actual_joint_pos_HS[i];
        actual_joint_vel_HS[i] = Count2RadDot(Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
    }

    actual_EP_pos_local_HS = FK_HS(actual_joint_pos_HS);
    actual_EP_vel_local_HS = (actual_EP_pos_local_HS - pre_actual_EP_pos_local_HS) / dt;
    pre_actual_EP_pos_local_HS = actual_EP_pos_local_HS;

    actual_EP_pos_global_HS << actual_EP_pos_local_HS(0), actual_EP_pos_local_HS(1), 0;
    actual_EP_vel_global_HS = (actual_EP_pos_global_HS - pre_actual_EP_pos_global_HS) / dt;
    pre_actual_EP_pos_global_HS = actual_EP_pos_global_HS;

    actual_Base_pos_global_HS << 0, 0, -actual_EP_pos_local_HS(2);

    J_HS = Jacobian_HS(actual_joint_pos_HS);
}

double Count2Deg(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 360 / (2048 * Gear_Ratio);
    return th; // [deg]
}

double Count2DegDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 360 / (2048 * Gear_Ratio);
    return th_dot; // [deg/s]
}

double Count2Rad(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 2 * PI / (2048 * Gear_Ratio);
    return th; // [rad]
}

double Count2RadDot(int Gear_Ratio, INT32 CountPerSec) {
    double th_dot = (double) CountPerSec * 2 * PI / (2048 * Gear_Ratio);
    return th_dot; // [rad]
}

double Count2Rad_ABS(int _Resolution, INT32 Count) {
    double th = (double) Count * 2 * PI / (_Resolution);
    return th;
}

INT32 Count_tf(int _Ratio, INT32 _Count_in) {
    INT32 _Count_out = (INT32) _Count_in / (_Ratio);
    return _Count_out;
}

INT16 Cur2Tor(double targetCur, double _ratedCur) {
    INT16 inputTor = targetCur * 1000 / _ratedCur;

    return inputTor;
}

void initial_param_setting(void) {
    rbdl_check_api_version(RBDL_API_VERSION);
    version_test=rbdl_get_api_version();
    printf("rbdl api version = %d\n", version_test);
    Addons::URDFReadFromFile("/home/rclab_catkin_ws/src/PongBotQ_One_Leg/model/PONGBOT_ONE_LEG/urdf/PONGBOT_ONE_LEG.urdf",pongbot_q_model,true,true);
    //setRobotModel(pongbot_q_model);
    
    kp_joint_HS << 10000.0, 10000.0, 10000.0;
    kd_joint_HS << 500.0, 500.0, 500.0;
    kp_EP_HS << 20000, 20000, 10000;
    kd_EP_HS << 1000, 1000, 500;
}

void Test_Pos_Traj_HS(void) {
    //    if (cnt < 3000) {
    //        for (int i = 0; i < NUM_OF_ELMO; ++i) {
    //            target_joint_pos_HS[i] = actual_joint_pos_HS[i];
    //            target_joint_vel_HS[i] = 0;
    //        }
    //    } else {
    //        Controlmode_HS = 1;
    //    }
    if (Controlmode_HS == 0) {
        target_joint_pos_HS = actual_joint_pos_HS;
        target_joint_vel_HS << 0, 0, 0;

        target_EP_pos_HS = actual_EP_pos_local_HS;
        target_EP_vel_HS << 0, 0, 0;
    }

    if (Controlmode_HS == 1 && Control_mode_flag == true) {
        if (cnt_HS == 0) {
            cycle_time_HS = 3.0;
            goal_joint_pos_HS << 0 * D2R, 45 * D2R, -90 * D2R;
            //goal_EP_pos_HS << 0.0, 0.105, -0.45;
            //target_EP_pos_HS = actual_EP_pos_local_HS;
            //init_EP_pos_HS = target_EP_pos_HS;
            //target_EP_vel_HS << 0, 0, 0;
            for (int i = 0; i < NUM_OF_ELMO; ++i) {
                target_joint_pos_HS[i] = actual_joint_pos_HS[i];
                target_joint_vel_HS[i] = 0;
                init_joint_pos_HS[i] = target_joint_pos_HS[i];
            }
            cnt_HS++;
        } else if (cnt_HS < (unsigned int) (cycle_time_HS / dt)) {
            for (int i = 0; i < NUM_OF_ELMO; ++i) {
                //    target_EP_pos_HS[i] = init_EP_pos_HS[i]+(goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
                //  target_EP_vel_HS[i] = (goal_EP_pos_HS[i] - init_EP_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
                target_joint_pos_HS[i] = init_joint_pos_HS[i] + (goal_joint_pos_HS[i] - init_joint_pos_HS[i]) / 2.0 * (1 - cos(PI / cycle_time_HS * cnt_HS * dt));
                target_joint_vel_HS[i] = (goal_joint_pos_HS[i] - init_joint_pos_HS[i]) / 2.0 * PI / cycle_time_HS * sin(PI / cycle_time_HS * cnt_HS * dt);
            }
            cnt_HS++;
        } else {
            //target_EP_pos_HS = goal_EP_pos_HS;
            // target_EP_vel_HS << 0, 0, 0;
            for (int i = 0; i < NUM_OF_ELMO; ++i) {
                target_joint_pos_HS[i] = goal_joint_pos_HS[i];
                target_joint_vel_HS[i] = 0;
            }
        }
        //target_joint_pos_HS = IK_HS(target_EP_pos_HS);
        //target_joint_vel_HS = J_HS.inverse() * target_EP_vel_HS;
    } else {
        cnt_HS = 0;
    }

    if (tmp_Control_mode_flag == true) {
        if (cnt_Control_mode < 3000) {
            Control_mode_flag = false;
            cnt_HS = 0;
        } else {
            Control_mode_flag = true;
            tmp_Control_mode_flag = false;
            cnt_Control_mode = 0;
        }
        cnt_Control_mode++;
    }
    cnt++;
}

void ComputeTorqueControl_HS(void) {

    Cal_Fc_HS();

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        CTC_Torque[i] = kp_joint_HS[i] * (target_joint_pos_HS[i] - actual_joint_pos_HS[i]) + kd_joint_HS[i] * (target_joint_vel_HS[i] - actual_joint_vel_HS[i]);
    }
    // CTC_Torque = -J_HS.transpose() * Cart_Controller_HS;
}

void jointController(void) {
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ELMO_drive_pt[i].ptOutParam->TargetTorque = Cur2Tor(CTC_Torque[i], ratedCur[i]);
    }
}

void Callback1(const std_msgs::Int32 &msg) {
    Controlmode_HS = msg.data;
    tmp_Control_mode_flag = true;
}

void ROS_MSG(void) {
    m_data.data[0] = actual_joint_pos_HS[0] * R2D;
    m_data.data[1] = actual_joint_pos_HS[1] * R2D;
    m_data.data[2] = actual_joint_pos_HS[2] * R2D;
    m_data.data[3] = target_joint_pos_HS[0] * R2D;
    m_data.data[4] = target_joint_pos_HS[1] * R2D;
    m_data.data[5] = target_joint_pos_HS[2] * R2D;
    P_data.publish(m_data);
}

VectorXd FK_HS(VectorXd joint_pos) {
    VectorXd EP_pos_HS(3);
    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    //FL_EP
    q1 = joint_pos[0];
    q2 = joint_pos[1];
    q3 = joint_pos[2];

    EP_pos_HS[0] = -L2 * sin(q2) - L3 * (cos(q2) * sin(q3) + cos(q3) * sin(q2));
    EP_pos_HS[1] = L1 * cos(q1) + L2 * cos(q2) * sin(q1) - L3 * (sin(q1) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * sin(q1));
    EP_pos_HS[2] = L1 * sin(q1) - L2 * cos(q1) * cos(q2) - L3 * (cos(q1) * cos(q2) * cos(q3) - cos(q1) * sin(q2) * sin(q3));

    return EP_pos_HS;
}

VectorXd IK_HS(VectorXd EP_pos) {
    VectorXd joint_pos(3);

    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    static double x = 0;
    static double y = 0;
    static double z = 0;
    static double q1_cal = 0;

    //    x = -EP_pos[0];
    //    y = EP_pos[1];
    //    z = EP_pos[2];
    //
    //    joint_pos[0] = atan2(y , abs(z)) - PI / 2 + acos(L1 / (sqrt(pow(y, 2) + pow(z, 2))));
    //    joint_pos[1] = -(-atan2(x, sqrt(abs(-pow(L1, 2) + pow(y, 2) + pow(z, 2)))) - acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2)))));
    //    joint_pos[2] = -(PI - acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2)) / (2 * L2 * L3)));

    //RL_joints
    x = EP_pos[0];
    y = EP_pos[1];
    z = EP_pos[2];
    if (actual_joint_pos_HS[0] >= 0) {
        joint_pos[0] = (acos(-z / sqrt(pow(y, 2) + pow(z, 2))) + acos(L1 / sqrt(pow(y, 2) + pow(z, 2))) - PI / 2);
    } else {
        joint_pos[0] = -(PI - (acos(-y / sqrt(pow(y, 2) + pow(z, 2))) + acos(L1 / sqrt(pow(y, 2) + pow(z, 2)))));
    }
    q1_cal = joint_pos[0];
    joint_pos[1] = (acos((pow(L2, 2) - pow(L3, 2) + pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow(z - L1 * sin(q1_cal), 2)) / (2 * L2 * sqrt(pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow((z - L1 * sin(q1_cal)), 2)))) - atan(x / sqrt(pow(y, 2) + pow(z, 2) - pow(L1, 2))));
    joint_pos[2] = -(acos((pow(x, 2) + pow((y - L1 * cos(q1_cal)), 2) + pow((z - L1 * sin(q1_cal)), 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3)));


    return joint_pos;
}

MatrixXd Jacobian_HS(VectorXd joint_pos) {
    MatrixXd J(3, 3);

    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    //FL_EP
    q1 = joint_pos[0];
    q2 = joint_pos[1];
    q3 = joint_pos[2];

    J(0, 0) = 0;
    J(0, 1) = -L2 * cos(q2) + L3 * (sin(q2) * sin(q3)) - L3 * (cos(q3) * cos(q2));
    J(0, 2) = -L3 * (cos(q2) * cos(q3)) + L3 * (sin(q3) * sin(q2));

    J(1, 0) = -L1 * sin(q1) + L2 * cos(q2) * cos(q1) - L3 * (cos(q1) * sin(q2) * sin(q3)) + L3 * (cos(q2) * cos(q3) * cos(q1));
    J(1, 1) = -L2 * sin(q2) * sin(q1) - L3 * (sin(q1) * cos(q2) * sin(q3)) - L3 * (sin(q2) * cos(q3) * sin(q1));
    J(1, 2) = -L3 * (sin(q1) * sin(q2) * cos(q3)) - L3 * (cos(q2) * sin(q3) * sin(q1));

    J(2, 0) = L1 * cos(q1) + L2 * sin(q1) * cos(q2) + L3 * (sin(q1) * cos(q2) * cos(q3)) - L3 * (sin(q1) * sin(q2) * sin(q3));
    J(2, 1) = L2 * cos(q1) * sin(q2) + L3 * (cos(q1) * sin(q2) * cos(q3)) + L3 * (cos(q1) * cos(q2) * sin(q3));
    J(2, 2) = L3 * (cos(q1) * cos(q2) * sin(q3)) + L3 * (cos(q1) * sin(q2) * cos(q3));

    return J;
}

void DataSave(void) {
    save_array[save_cnt][0] = cnt_HS / 1000;
    save_array[save_cnt][1] = Controlmode_HS;
    save_array[save_cnt][2] = actual_EP_vel_local_HS[0];
    save_array[save_cnt][3] = actual_EP_vel_local_HS[1];
    save_array[save_cnt][4] = actual_EP_vel_local_HS[2];
    save_array[save_cnt][5] = ELMO_drive_pt[0].ptInParam->StatusWord;
    save_array[save_cnt][6] = ELMO_drive_pt[1].ptInParam->StatusWord;
    save_array[save_cnt][7] = ELMO_drive_pt[2].ptInParam->StatusWord;

    if (save_cnt < SAVE_COUNT - 1)
        save_cnt++;
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

void Cal_Fc_HS(void) {
    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        Cart_Controller_HS[i] = kp_EP_HS[i] * (actual_EP_pos_local_HS[i] - target_EP_pos_HS[i]) + kd_EP_HS[i] * (actual_EP_vel_local_HS[i] - target_EP_vel_HS[i]);
    }
}

//void setRobotModel(Model* getModel){
//    m_pModel=getModel;
//    m_pModel->gravity=Vector3d(0.,0.,-9.81);
//}