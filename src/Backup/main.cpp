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



// **********Xenomai libraries*********//
#include <alchemy/task.h>
#include <alchemy/timer.h>

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

#define R2D 180/PI
#define D2R PI/180

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

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

// loop (main & thread)
int main_run = 1;
int motion_run = 1;
int print_run = 1;
int ros_run = 1;

//Thread// 
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t p_thread1;
pthread_t p_thread2;
int recv_fail_cnt = 0;

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
void test_task(void *arg);

//**********************************************//
//*************** 2. Functions ****************//
void* motion_task(void* arg); // : Thread 1
void* print_task(void* arg); // : Thread 2
void* ros_task(void* arg); // : Thread 3
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

    mlockall(MCL_CURRENT|MCL_FUTURE);
    
    printf("ROS Setting ...\n");
    ros::init(argc, argv, "elmo_pkgs");
    ros::NodeHandle nh;
    //ros::Rate loop_rate(1000);
    ros::Rate loop_rate(100);
    S_Mode = nh.subscribe("Mode", 1, Callback1);
    P_data = nh.advertise<std_msgs::Float64MultiArray>("ROS_DATA", 1);
    m_data.data.resize(10);
    sleep(1);

    printf("Init main ...\n");
    sleep(1);

    printf("Use default adapter %s ...\n", ecat_ifname);
    sleep(1);

    printf("Initialize Prams...\n");
    initial_param_setting();

    printf("Create Thread ...\n");
    for (int i = 1; i < 4; ++i) {
        printf("%d...\n", i);
        sleep(1);
    }
    
    rt_task_create(&RT_task1, "Test_task", 0, 99, 0);
    rt_task_start(&RT_task1, &test_task, NULL);
    
    pthread_create(&p_thread1, NULL, motion_task, NULL);
    pthread_create(&p_thread2, NULL, print_task, NULL);
   
    //while (main_run) {
    while (ros::ok()) {
        pthread_mutex_lock(&mutex);
        // printf("main_running\n");
        pthread_mutex_unlock(&mutex);
        //usleep(1000); //cycle = 1.0 [s]
        ROS_MSG();
        ros::spinOnce();
    }
    FileSave();
    return 0;
}

void* motion_task(void* arg) {
    //Thread Time Measure
    struct timespec begin, end;
    struct timespec tim;

    tim.tv_sec = 0;
    tim.tv_nsec = 1000000; //ns

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    printf("Motion Thread Start \n");

    if (ecat_init() == false) {
        motion_run = 0;
    }
    for (int i = 0; i < NUM_OF_ELMO; ++i) {
        ec_dcsync0(1 + i, TRUE, cycle_ns, 0); // SYNC0,1 on slave 1
    }

    ec_send_processdata();

    while (motion_run) {

        clock_gettime(CLOCK_MONOTONIC, &begin); //Time measure
        pthread_mutex_lock(&mutex);

        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < 3 * (NUM_OF_ELMO)) {
            recv_fail_cnt++;
        }

        if (inOP == TRUE) {
            if (!sys_ready) {
                if (stick == 0)
                    printf("waiting for system ready...\n");
                if (stick % 10 == 0)
                    // printf("%l\n", stick / 10);
                    stick++;
            }
        }
        // realtime action...
        if (sys_ready) {
            EncoderRead();
            Test_Pos_Traj_HS();
            ComputeTorqueControl_HS();
            jointController();
            //            DataSave();
        } else {
            ServoOn();
            //printf("0\n");
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
        }
        pthread_mutex_unlock(&mutex);
        pthread_testcancel(); //cancel point
        //usleep(1000); //1 ms cycle

        //nanosleep((const struct timespec[]){{0,10000000L}},NULL);
        nanosleep(&tim, NULL);
        clock_gettime(CLOCK_MONOTONIC, &end); //Time measure
        Thread_time = (end.tv_sec - begin.tv_sec)+(end.tv_nsec - begin.tv_nsec) / 1000000000.0;

        DataSave();
    }
    return 0;
}

void* print_task(void* arg) {

    struct timespec tim;

    tim.tv_sec = 0;
    tim.tv_nsec = 1000000; //ns

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    printf("Print Thread Start \n");

    while (print_run) {
        pthread_mutex_lock(&mutex);

        if (inOP == TRUE) {
            if (!sys_ready) {
                if (stick == 0)
                    printf("waiting for system ready...\n");
                if (stick % 10 == 0)
                    // rt_printf("%i\n", stick / 10);
                    stick++;
            } else {
//                printf("tmp_flag=%d\n", tmp_Control_mode_flag);
//                printf("cnt=%d\n", cnt_Control_mode);
//                printf("flag=%d\n", Control_mode_flag);
//                printf("Controlmode=%d\n", Controlmode_HS);
//                printf("_______________________________\n");
                for (int i = 0; i < NUM_OF_ELMO; ++i) {
                    //printf("Print Thread Running \n");
                    // ELMO Status
                   // printf("ELMO_Drive#%i\n", i + 1);
                    //printf("Status word = 0x%X\n", ELMO_drive_pt[i].ptInParam->StatusWord);
                    // printf("actual_q(degree) = %3f\n", actual_joint_pos_HS[i] * R2D);
                    //                    printf("ABS_actual_q(degree) = %3f\n", ABS_actual_joint_pos_HS[i] * R2D);
                    // printf("actual_q_dot=%3f\n", actual_joint_vel_HS[i]);
                    // printf("init_q(degree)=%3f\n", init_joint_pos_HS[i] * R2D);
                    //p/rintf("goal_q(degree)=%3f\n", goal_joint_pos_HS[i] * R2D);
                    //printf("target_q(degree)=%3f\n", target_joint_pos_HS[i] * R2D);
                    // printf("CTC_Torque=%3f\n", CTC_Torque[i]);
                    // printf("target_vel=%3f\n", target_joint_vel_HS[i]);
                    //printf("___________________________________________________\n");
                }
               // printf("Thread_time=%6f [s]\n", Thread_time);
                //                printf("actual_x=%3f[m]\n", actual_EP_pos_local_HS(0));
                //                printf("actual_y=%3f[m]\n", actual_EP_pos_local_HS(1));
                //                printf("actual_z=%3f[m]\n", actual_EP_pos_local_HS(2));
                //                printf("___________________________________________________\n");
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
                printf("-----------------------------------------------------------------------\n");
            }

        }
        pthread_mutex_unlock(&mutex);
        pthread_testcancel(); //cancel point
        //usleep(1000); //1ms cycle
        nanosleep(&tim, NULL);
    }
    return 0;
}

//void* ros_task(void* arg) {
//
//    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
//    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
//    printf("Print ROS Start \n");
//
//    ros::NodeHandle nh;
//
//    ros::Rate loop_rate(1000);
//
//    while (ros::ok()) {
//        pthread_mutex_lock(&mutex);
//        ROS_INFO("ROS!");
//        std::cout<<"ROS2!"<<std::endl;
//        pthread_mutex_unlock(&mutex);
//        ros::spin();
//        pthread_testcancel(); //cancel point
//        loop_rate.sleep();
//    }
//}

void catch_signal(int sig) {
    FileSave();
    printf("Program END...\n");
    pthread_cancel(p_thread1);
    pthread_cancel(p_thread2);
    ros::shutdown();
    rt_task_delete(&RT_task1);
    pthread_mutex_destroy(&mutex);

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
    //const double L1 = 0.105;
    const double L1 = 0.1045;
    const double L2 = 0.305;
    const double L3 = 0.309;
    //const double L3 = 0.294;

    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    //FL_EP
    q1 = joint_pos[0];
    q2 = joint_pos[1];
    q3 = joint_pos[2];
    //printf("q1=%3f,q2=%3f,q3=%3f\n", q1*R2D, q2*R2D, q3 * R2D);
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

void test_task(void *arg){
    RTIME now, previous;
        
    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    previous=rt_timer_read();
    
    while(main_run){
        rt_task_wait_period(NULL);
        now=rt_timer_read();
        
        rt_printf("Time since last turn : %ld.%06ld ms\n",
                (long)(now-previous)/1000000,
                (long)(now-previous)%1000000);
        previous=now;
    }
}