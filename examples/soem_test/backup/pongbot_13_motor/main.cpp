// **********Basic libraries*********//
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

// **********Xenomai libraries*********//
#include <alchemy/task.h>
#include <alchemy/timer.h>

// **********SOEM library*********//
#include "ethercat.h"
#include "pdo_def.h"
#include "ecat_dc.h"
#include "Eigen/Dense"

#define NUM_OF_ELMO 13

//Save
#define SAVE_LENGTH 8    //The number of data
#define SAVE_COUNT 10000 //Save Time = 10000[ms]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];

//Xenomai
long Thread_time_upper;
long Thread_time_lower;
double Thread_time1;
double Thread_time2;
RT_TASK demo_task;
void DataSave(void);
void FileSave(void);

//SOEM
bool ecat_init(void);
void ServoOn(void);
void ServoOff(void);

int ethercat_run = 1;

int ServoState = 0;
int servo_ready = 0;
int sys_ready = 0;
double started[NUM_OF_ELMO] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t Resolution[NUM_OF_ELMO] = {262144, 262144, 16384, 262144, 262144, 16384, 524288, 262144, 262144, 16384, 262144, 262144, 16384}; //16384(2^14)
int Gear[NUM_OF_ELMO] = {50, 50, 50, 50, 50, 50, 200, 50, 50, 50, 50, 50, 50};
int Ratio[NUM_OF_ELMO] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
double ratedCur[NUM_OF_ELMO] = {2.85, 2.85, 8.9, 2.85, 2.85, 8.9, 2.85, 2.85, 2.85, 8.9, 2.85, 2.85, 8.9};
double Kt[NUM_OF_ELMO] = {0.159, 0.159, 0.210, 0.159, 0.159, 0.210, 0.159, 0.159, 0.159, 0.210, 0.159, 0.159, 0.210};

uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;
unsigned int cycle_ns = 1000000; // Control Cycle 1[ms]

int recv_fail_cnt = 0;

//char ecat_ifname[32] = "rteth0";
char ecat_ifname[32] = "eth0";
//char ecat_ifname[32] = "eth1";
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
UINT16 maxTorque = 3500;

using Eigen::VectorXd;

bool Encoder_Reset_Flag = true;
VectorXd actual_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd ABS_actual_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd Incre_actual_joint_pos_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd Incre_actual_joint_pos_offset_HS = VectorXd::Zero(NUM_OF_ELMO);
VectorXd actual_joint_vel_HS = VectorXd::Zero(NUM_OF_ELMO);

double Count2Deg(int Gear_Ratio, INT32 Count);
double Count2DegDot(int Gear_Ratio, INT32 CountPerSec);
double Count2Rad(int Gear_Ratio, INT32 Count);
double Count2RadDot(int Gear_Ratio, INT32 CountPerSec);
double Count2Rad_ABS(int _Resolution, INT32 Count);
INT32 Count_tf(int _Ratio, INT32 _Count_in);
void EncoderRead(void);
void Max_Time_Save(double now_time);
double max_time = 0.0;

void demo(void *arg) {
    RTIME now1, previous1;
    RTIME now2, previous2;

    if (ecat_init() == false) {
        ethercat_run = 0;
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

    while (ethercat_run) {

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
            rt_printf("Time since last turn: %6f ms / %6f ms / %6f ms\n", Thread_time1, Thread_time2, max_time);
            rt_printf("_________________________________________\n");
            rt_printf("Status word = 0x%X / 0x%X / 0x%X / 0x%X /0x%X /0x%X /0x%X /0x%X /0x%X /0x%X /0x%X /0x%X /0x%X \n", ELMO_drive_pt[0].ptInParam->StatusWord, ELMO_drive_pt[1].ptInParam->StatusWord, ELMO_drive_pt[2].ptInParam->StatusWord, ELMO_drive_pt[3].ptInParam->StatusWord, ELMO_drive_pt[4].ptInParam->StatusWord, ELMO_drive_pt[5].ptInParam->StatusWord, ELMO_drive_pt[6].ptInParam->StatusWord, ELMO_drive_pt[7].ptInParam->StatusWord, ELMO_drive_pt[8].ptInParam->StatusWord, ELMO_drive_pt[9].ptInParam->StatusWord, ELMO_drive_pt[10].ptInParam->StatusWord, ELMO_drive_pt[11].ptInParam->StatusWord, ELMO_drive_pt[12].ptInParam->StatusWord);
            rt_printf("actual_q(degree) = %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f \n", actual_joint_pos_HS[0] * R2D, actual_joint_pos_HS[1] * R2D, actual_joint_pos_HS[2] * R2D, actual_joint_pos_HS[3] * R2D, actual_joint_pos_HS[4] * R2D, actual_joint_pos_HS[5] * R2D, actual_joint_pos_HS[6] * R2D, actual_joint_pos_HS[7] * R2D, actual_joint_pos_HS[8] * R2D, actual_joint_pos_HS[9] * R2D, actual_joint_pos_HS[10] * R2D, actual_joint_pos_HS[11] * R2D, actual_joint_pos_HS[12] * R2D);
            rt_printf("actual_q_dot= %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f / %3f/ %3f / %3f / %3f\n", actual_joint_vel_HS[0], actual_joint_vel_HS[1], actual_joint_vel_HS[2], actual_joint_pos_HS[3], actual_joint_pos_HS[4], actual_joint_pos_HS[5], actual_joint_pos_HS[6], actual_joint_pos_HS[7], actual_joint_pos_HS[8], actual_joint_pos_HS[9], actual_joint_pos_HS[10], actual_joint_pos_HS[11], actual_joint_pos_HS[12]);
            rt_printf("_________________________________________\n");
        }
        now1 = rt_timer_read();
        now2 = rt_timer_read();

        Thread_time1 = (double) (now1 - previous1) / 1000000;
        Thread_time2 = (double) (now2 - previous2) / 1000000;
        Max_Time_Save(Thread_time1);
 
        //DataSave();
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

void catch_signal(int sig) {
    FileSave();
}

int main(int argc, char* argv[]) {
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    rt_task_create(&demo_task, "trivial", 0, 99, 0);
    rt_task_start(&demo_task, &demo, NULL);
    pause();
    rt_task_delete(&demo_task);

    return 0;
}

void DataSave(void) {
    save_array[save_cnt][0] = Thread_time1;
    save_array[save_cnt][1] = Thread_time2;

    if (save_cnt < SAVE_COUNT - 1)
        save_cnt++;
}

void FileSave(void) {
    FILE *fp;

    fp = fopen("Example2_Data.txt", "w");
    for (int j = 0; j <= SAVE_COUNT - 1; ++j) {
        for (int i = 0; i <= SAVE_LENGTH - 1; ++i) {
            fprintf(fp, "%f\t", save_array[j][i]);
        }
        fprintf(fp, "%f\n", save_array[j][SAVE_LENGTH - 1]);
    }

    fclose(fp);
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
                    //printf("Re mapping for ELMO...\n");
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
    if (Encoder_Reset_Flag == true) {
        for (int i = 0; i < NUM_OF_ELMO; i++) {
            ABS_actual_joint_pos_HS[i] = Count2Rad_ABS(Resolution[i], Count_tf(Ratio[i], ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
            Incre_actual_joint_pos_offset_HS[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
            actual_joint_pos_HS[i] = ABS_actual_joint_pos_HS[i] + Incre_actual_joint_pos_HS[i];
        }
        Encoder_Reset_Flag = false;
    }

    for (int i = 0; i < NUM_OF_ELMO; i++) {
        Incre_actual_joint_pos_HS[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue) - Incre_actual_joint_pos_offset_HS[i];
        actual_joint_pos_HS[i] = ABS_actual_joint_pos_HS[i] + Incre_actual_joint_pos_HS[i];
        actual_joint_vel_HS[i] = Count2RadDot(Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
    }
}

double Count2Deg(int Gear_Ratio, INT32 Count) {
    double th = (double) Count * 360 / (2048 * Gear_Ratio);
    return th; // 
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

void Max_Time_Save(double now_time) {

    if (max_time < now_time) {
        max_time = now_time;
    }
}