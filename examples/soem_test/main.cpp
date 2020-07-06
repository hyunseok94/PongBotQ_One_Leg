// **********Basic libraries*********//
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

// **********Xenomai libraries*********//
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <ostream>
#include <iostream>

// **********SOEM library*********//
#include "ethercat.h"
#include "pdo_def.h"
#include "ecat_dc.h"

#define NUM_OF_ELMO 3
//#define _USE_DC
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
double started[NUM_OF_ELMO] = {0};
uint16_t controlword = 0;
long stick = 0;
unsigned long ready_cnt = 0;
unsigned int cycle_ns = 1000000; // Control Cycle 1[ms]

int recv_fail_cnt = 0;

//char ecat_ifname[32] = "rteth0";
char ecat_ifname[32] = "enp5s0";
//char ecat_ifname[32] = "eno1";
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
RTIME now1, previous1;
RTIME now2, previous2;

RTIME A, B, C, D, E, F, G, H;
double interval1;
double interval2;
double interval3;

static int RS3_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
static int RS3_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
static int RS3_write32(uint16 slave, uint16 index, uint8 subindex, uint32 value);

void demo(void *arg) {


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
        previous1 = rt_timer_read();
        previous2 = now2;

        ec_send_processdata();
        A = rt_timer_read();
        interval1 = (double) (A - previous1) / 1000000;
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        std::cout << "EC=" << EC_TIMEOUTRET << std::endl;
        std::cout << "wk=" << wkc << std::endl;
        if (wkc < 3 * (NUM_OF_ELMO)) {
            recv_fail_cnt++;
        }
        B = rt_timer_read();
        interval2 = (double) (B - A) / 1000000;
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

        }

        now1 = rt_timer_read();
        now2 = rt_timer_read();

        Thread_time1 = (double) (now1 - previous1) / 1000000;
        Thread_time2 = (double) (now2 - previous2) / 1000000;
        interval3 = (double) (now1 - B) / 1000000;
        //previous = now;
        printf("Time since last turn: %6f ms / %6f ms\n", Thread_time1, Thread_time2);
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

    save_array[save_cnt][2] = interval1;
    save_array[save_cnt][3] = interval2;
    save_array[save_cnt][4] = interval3;

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
            for (int k = 0; k < NUM_OF_ELMO; k++) {
                if (ec_slavecount >= 1) {
                    //printf("Re mapping for ELMO...\n");
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
