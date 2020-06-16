/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 *
 * Port for Raspberry pi by Ho Tam - thanhtam.h[at]gmail.com
 */

#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>

 // Xenomai
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

// Ethercat && SPI
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"
#include "pdo_def.h"
#include "ecat_dc.h"
#include "wiznet_drv.h"

#include <sys/socket.h>
#include <netinet/in.h>

//#include <Eigen/dense>
//#include <eigen2/Eigen/Dense>

#define R2D 180/PI
#define D2R PI/180
typedef enum {false,true} bool;
unsigned int cnt = 0;
unsigned int cnt_HS = 0;

// Main
#define NSEC_PER_SEC 			1000000000
//#define EC_TIMEOUTMON 500
#define NUM_OF_ELMO	3        //The number of Elmo driver(KHS)
#define _USE_DC
#define _WRITE_MODEOP_SDO

ELMO_Drive_pt ELMO_drive_pt[NUM_OF_ELMO];    // Regist the number of structors according to the number of elmo drive(KHS)

// 1-cos 
unsigned int offset_PITCH = 2047;
unsigned int offset_ROLL = 2047;

unsigned int cycle_ns = 1000000; /* 1 ms */

								 //EtherCat
char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

//Xenomai Thread
RT_TASK motion_task;
RT_TASK print_task;
RTIME now, previous;

//EtherCat
long ethercat_time_send, ethercat_time_read = 0;
long ethercat_time = 0, worst_time = 0;
char ecat_ifname[32] = "PiCAT";
int run = 1;
int sys_ready = 0;
int i, oloop, iloop, k, wkc_count;


int started[NUM_OF_ELMO] = { 0,0,0 }, ServoState = 0;
int zeroPose[NUM_OF_ELMO] = { 0,0,0}, zeroPosState = 0;
uint8 servo_ready = 0, servo_prestate = 0;
uint8 zeroPos_ready = 0, zeroPos_prestate = 0;

//double sine_amp=11500, f=0.2, period;
int recv_fail_cnt = 0; //motion_run

//variables for pdo re-mapping (sdo write) EtherCAT
int os;
uint32_t ob;
uint16_t ob2;
uint8_t  ob3;
uint32_t modeOp=0x20a01b20;

unsigned long ready_cnt = 0;
uint16_t controlword = 0;

int Gear[NUM_OF_ELMO] = { 50,50,50 };
int des_torque_[1] = { 0 };
//int32_t Resolution[NUM_OF_ELMO]={16384};//16384(2^14)
int32_t Resolution[NUM_OF_ELMO]={16384,65536,65536};//16384(2^14)
//int Ratio[NUM_OF_ELMO]={256};
int Ratio[NUM_OF_ELMO]={256,1,1};
double dt = 0.001;

//double ratedCur[NUM_OF_ELMO] = { 8900 };
double ratedCur[NUM_OF_ELMO] = {8900, 2850,2850 };

double actual_joint_pos_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0};
double ABS_actual_joint_pos_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0};
double Incre_actual_joint_pos_HS[NUM_OF_ELMO]={0.0, 0.0, 0.0};
double Incre_actual_joint_pos_offset_HS[NUM_OF_ELMO]={0.0, 0.0, 0.0};
double actual_joint_vel_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0};
double init_joint_pos_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0 };
double target_joint_pos_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0};
double target_joint_vel_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0 };
double goal_joint_pos_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0 };
double goal_joint_vel_HS[NUM_OF_ELMO] = { 0.0, 0.0 , 0.0};
double joint_pos_err_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0};
double joint_vel_err_HS[NUM_OF_ELMO] = { 0.0, 0.0, 0.0};
double CTC_Torque[NUM_OF_ELMO] = { 0.0, 0.0, 0.0 };
double kp_joint_HS[NUM_OF_ELMO] = { 10000.0, 10000.0, 10000.0 };
double kd_joint_HS[NUM_OF_ELMO] = { 500.0, 500.0, 500.0};
double actual_joint_vel_offset_HS[NUM_OF_ELMO]={0.0, 0.0, 0.0};

int controlmode = 0;

double Count2Deg(int Gear_Ratio, INT32 Count);
double Count2DegDot(int Gear_Ratio, INT32 CountPerSec);
double Count2Rad(int Gear_Ratio, INT32 Count);
double Count2RadDot(int Gear_Ratio, INT32 CountPerSec);
INT16 Cur2Tor(double targetCur,double _ratedCur);
INT32 Count_tf(int _Ratio, INT32 _Count_in);
double Count2Rad_ABS(int _Resolution, INT32 Count);

void ServoOn(void);
void ServoOff(void);

uint8 modeOp_Torque = OP_MODE_CYCLIC_SYNC_TORQUE;
uint8 modeOp_Home = OP_MODE_HOMING;
INT8  modeOp_HomePos = 35;

uint8 modeOpdisplay;
UINT16 maxTorque = 3500;
INT16 targetTor[NUM_OF_ELMO] = { 0 };


long stick = 0;

// HS codes
double cycle_time_HS = 0.0;
int Num_of_motion=0;

void Test_Pos_Traj_HS(void);
void ComputeTorqueControl_HS(void);
void EncoderRead(void);
void jointController(void);
bool Encoder_Reset_Flag=true;
void DataSave(void);

long Time_motion_run;
RTIME pre_Time, now_Time;
unsigned int save_cnt=0;
void FileSave(void);

# define SAVE_LENGTH 4
# define SAVE_COUNT 10000
double save_array[SAVE_COUNT][SAVE_LENGTH];
    
boolean ecat_init(void)
{
	printf("function is ecat_init");
	needlf = FALSE;
	inOP = FALSE;

	rt_printf("Starting simple test\n");

	wiznet_hw_config(8, 0, 0); //31.25 Mhz, don't reset link 

	if (ec_init(ecat_ifname))
	{
		rt_printf("ec_init on %s succeeded.\n", ecat_ifname); //ifname
		/* find and auto-config slaves */

		if (ec_config_init(FALSE) > 0)
		{
			rt_printf("%d slaves found and configured.\n", ec_slavecount);
            for(int i=0;i<NUM_OF_ELMO;++i){		
			 rt_printf("Has CA? %d %s\n", i+1 ,ec_slave[i+1].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
		    }

			/** CompleteAccess disabled for Elmo driver */
			for(int i=1;i<NUM_OF_ELMO; ++i){
				ec_slave[i+1].CoEdetails ^= ECT_COEDET_SDOCA;
			}
			//ec_slave[1].CoEdetails ^= ECT_COEDET_SDOCA;
			//ec_slave[2].CoEdetails ^= ECT_COEDET_SDOCA;

			ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

			// PDO re-mapping //
			for (k = 0; k < NUM_OF_ELMO; ++k)
			{
				if (ec_slavecount >= 1)
				{
					printf("Re mapping for ELMO...\n");
					ob3=0;
					 wkc_count+=ec_SDOwrite(k+1,0x1c12,0x00,FALSE,sizeof(ob3),&ob3,EC_TIMEOUTRXM);   
                     wkc_count+=ec_SDOwrite(k+1,0x1c13,0x00,FALSE,sizeof(ob3),&ob3,EC_TIMEOUTRXM);
                                       
                     ob2=0x1605;
                     wkc_count+=ec_SDOwrite(k+1,0x1c12,0x01,FALSE,sizeof(ob2),&ob2,EC_TIMEOUTRXM);
                     ob3=1;
                     wkc_count+=ec_SDOwrite(k+1,0x1c12,0x00,FALSE,sizeof(ob3),&ob3,EC_TIMEOUTRXM);
                     
                     ob2=0x1A03;
                     wkc_count+=ec_SDOwrite(k+1,0x1c13,0x01,FALSE,sizeof(ob2),&ob2,EC_TIMEOUTRXM);
                     ob2=0x1A1E;
                     wkc_count+=ec_SDOwrite(k+1,0x1c13,0x02,FALSE,sizeof(ob2),&ob2,EC_TIMEOUTRXM);
                     ob3=2;
                     //ob3=1;
                     wkc_count+=ec_SDOwrite(k+1,0x1c13,0x00,FALSE,sizeof(ob3),&ob3,EC_TIMEOUTRXM);
				}

			}

			 ec_config_map(&IOmap);

#ifdef _USE_DC
			ec_configdc();
#endif		
			rt_printf("Slaves mapped, state to SAFE_OP.\n");
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
#ifdef _USE_DC
			//FOR DC----
			/* configure DC options for every DC capable slave found in the list */
			rt_printf("DC capable : %d\n", ec_configdc());
			//---------------
#endif
			oloop = ec_slave[0].Obytes;
			if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
			//if (oloop > 8) oloop = 8;
			iloop = ec_slave[0].Ibytes;
			if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
			//if (iloop > 8) iloop = 8;

			rt_printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

			rt_printf("Request operational state for all slaves\n");
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			rt_printf("Calculated workcounter %d\n", expectedWKC);
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			/* request OP state for all slaves */

			ec_writestate(0);
			ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP

			if (ec_slave[0].state == EC_STATE_OPERATIONAL)
			{
				rt_printf("Operational state reached for all slaves.\n");
				wkc_count = 0;

				for (k = 0; k < NUM_OF_ELMO; ++k)
				{
					ELMO_drive_pt[k].ptOutParam = (ELMO_DRIVE_RxPDO_t*)ec_slave[k + 1].outputs;
					ELMO_drive_pt[k].ptInParam = (ELMO_DRIVE_TxPDO_t*)ec_slave[k + 1].inputs;
					ELMO_drive_pt[k].ptOutParam->ModeOfOperation = OP_MODE_CYCLIC_SYNC_TORQUE;
					//modeOpdisplay = ELMO_drive_pt[k].ptInParam->ModeOfOperationDisplay;
					ELMO_drive_pt[k].ptOutParam->MaxTorque = maxTorque;
				}
				inOP = TRUE;
			}
			else
			{
				rt_printf("Not all slaves reached operational state.\n");
				ec_readstate();
				for (i = 1; i <= ec_slavecount; ++i)
				{
					if (ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						rt_printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
							i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
				for (i = 0; i < NUM_OF_ELMO; ++i)
					ec_dcsync01(i + 1, FALSE, 0, 0, 0); // SYNC0,1 
			}

		}
		else
		{
			printf("No slaves found!\n");
			inOP = FALSE;
		}

	}
	else
	{
		rt_printf("No socket connection on %s\nExcecute as root\n", ecat_ifname);
		return FALSE;
	}

	return inOP;
}

// main task
void motion_run(void* arg)
{
	RTIME now, previous;

	if (ecat_init() == FALSE)
	{
		run = 0;
		//printf("EtherCat COM fail(motion)\n");
		return;	//all initialization stuffs here
	}
	rt_task_sleep(1e6);

#ifdef _USE_DC
	//for dc computation
	long  long toff;
	long long cur_DCtime = 0, max_DCtime = 0;
	unsigned long long  cur_dc32 = 0, pre_dc32 = 0;
	int32_t shift_time = 380000; //dc event shifted compared to master reference clock
	long long  diff_dc32;

	for (i = 0; i < NUM_OF_ELMO; ++i)
		ec_dcsync0(1 + i, TRUE, cycle_ns, 0); // SYNC0,1 on slave 1

	RTIME cycletime = cycle_ns, cur_time = 0;
	RTIME cur_cycle_cnt = 0, cycle_time;
	RTIME remain_time, dc_remain_time;
	toff = 0;

	RTIME rt_ts;
	//get DC time for first time
	ec_send_processdata();

	cur_time = rt_timer_read();			//get current master time
	cur_cycle_cnt = cur_time / cycle_ns;	//calcualte number of cycles has passed
	cycle_time = cur_cycle_cnt * cycle_ns;
	remain_time = cur_time % cycle_ns;	//remain time to next cycle, test only

	rt_printf("cycle_cnt=%lld\n", cur_cycle_cnt);
	rt_printf("remain_time=%lld\n", remain_time);

	wkc = ec_receive_processdata(EC_TIMEOUTRET); 	//get reference DC time
	cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);	//only consider first 32-bit
	dc_remain_time = cur_dc32 % cycletime;				//remain time to next cycle of REF clock, update to master
	rt_ts = cycle_time + dc_remain_time;				//update master time to REF clock

	rt_printf("dc remain_time=%lld\n", dc_remain_time);

	rt_task_sleep_until(rt_ts);

#else  //nonDC
	ec_send_processdata();
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
#endif



	while (run)
	{
		//wait for next cycle	
#ifdef _USE_DC		   
		rt_ts += (RTIME)(cycle_ns + toff);
		rt_task_sleep_until(rt_ts);
#else
		rt_task_wait_period(NULL);
#endif
		previous = rt_timer_read();

		ec_send_processdata();
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		if (wkc < 3 * (NUM_OF_ELMO))
			recv_fail_cnt++;

		now = rt_timer_read();
		ethercat_time = (long)(now - previous);

#ifdef _USE_DC	   
		cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff); 	//use 32-bit only
		if (cur_dc32 > pre_dc32)							//normal case
			diff_dc32 = cur_dc32 - pre_dc32;
		else												//32-bit data overflow
			diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
		pre_dc32 = cur_dc32;
		cur_DCtime += diff_dc32;

		toff = dc_pi_sync(cur_DCtime, cycletime, shift_time);

		if (cur_DCtime > max_DCtime) max_DCtime = cur_DCtime;
#endif		   

		if (inOP == TRUE)
		{
			if (!sys_ready)
			{
				if (stick == 0)
					rt_printf("waiting for system ready...\n");
				if (stick % 10 == 0)
					rt_printf("%i\n", stick / 10);
				stick++;
			}
		}
		// realtime action...
		if (sys_ready)
		{
			pre_Time=rt_timer_read();
					
			EncoderRead();
			Test_Pos_Traj_HS();
			ComputeTorqueControl_HS();
			jointController();
			//DataSave();
			
			now_Time=rt_timer_read();
			Time_motion_run=(long)(now_Time-pre_Time);
			
		}
		else
		{
			ServoOn();

			if (ServoState == (1 << NUM_OF_ELMO) - 1) //all servos are in ON state
			{
				if (servo_ready == 0)
					servo_ready = 1;
			}
			if (servo_ready) ready_cnt++;
			if (ready_cnt >= 5000) //wait for 5s after servo-on
			{
				ready_cnt = 10000;
				sys_ready = 1;
			}
			//printf("servo_state=%d",servo_ready);
			//printf("\n");

			//printf("sys_state=%d",sys_ready);
			//printf("\n");
		}

		if (sys_ready)
			if (worst_time < ethercat_time) worst_time = ethercat_time;

	}

	//rt_task_sleep(cycle_ns);
#ifdef _USE_DC
	for (i = 0; i < NUM_OF_ELMO; ++i)
		ec_dcsync0(i + 1, FALSE, 0, 0); // SYNC0,1 on slave 1
#endif

	ServoOff();

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

void Test_Pos_Traj_HS(void)
{
	if(cnt<3000){
	  for(int i=0; i<NUM_OF_ELMO; ++i){
		target_joint_pos_HS[i]=actual_joint_pos_HS[i];
		target_joint_vel_HS[i]=0;
	  }
	}
	else {
		controlmode = 1;
	}

	/*for (int i=0; i<NUM_OF_ELMO; ++i)
		{
			modeOpdisplay = ELMO_drive_pt[i].ptInParam->ModeOfOperationDisplay;
		}*/

	if (controlmode == 1) {

		if (cnt_HS == 0) {
			cycle_time_HS = 3.0;
			goal_joint_pos_HS[0] = -90*D2R;
			goal_joint_pos_HS[1] = 45*D2R;
			goal_joint_pos_HS[2]=0*D2R;
			for (int i = 0; i < NUM_OF_ELMO; ++i) {
				target_joint_pos_HS[i] = actual_joint_pos_HS[i];
				target_joint_vel_HS[i] = 0;
				init_joint_pos_HS[i] = target_joint_pos_HS[i];
			}
			cnt_HS++;
		}
		else if (cnt_HS < (unsigned int)(cycle_time_HS / dt)) {
			for (int i = 0; i < NUM_OF_ELMO; ++i) {
				target_joint_pos_HS[i] = init_joint_pos_HS[i] + (goal_joint_pos_HS[i]-init_joint_pos_HS[i])/2.0*(1-cos(PI/cycle_time_HS*cnt_HS*dt));
				target_joint_vel_HS[i] = (goal_joint_pos_HS[i]-init_joint_pos_HS[i])/2.0*PI/cycle_time_HS*sin(PI/cycle_time_HS*cnt_HS*dt);
			}
			cnt_HS++;
		}
		else {
				for(int i=0; i<NUM_OF_ELMO;++i){		
	        target_joint_pos_HS[i]=goal_joint_pos_HS[i];
	        target_joint_vel_HS[i]=0;
		}
			controlmode = 0;
		    
		}
	}
	else {
		cnt_HS = 0;
	}
	cnt++;
}

void print_run(void* arg)
{
	int i;
	unsigned long itime = 0;
	long stick = 0;
	rt_task_set_periodic(NULL, TM_NOW, 1e8);

	while (1)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle
		if (inOP == TRUE)
		{
			if (!sys_ready)
			{
				if (stick == 0)
					rt_printf("waiting for system ready...\n");
				if (stick % 10 == 0)
					rt_printf("%i\n", stick / 10);
				stick++;
			}
			else
			{
				itime++;
				//rt_printf("Time=%06d.%01d, \e[32;1m fail=%ld\e[0m, ecat_T=%ld, maxT=%ld\n", itime/10, itime%10, recv_fail_cnt,  ethercat_time/1000, worst_time/1000);
				for (i = 0; i < NUM_OF_ELMO; ++i)
				{
					// ELMO Status
					rt_printf("ELMO_Drive#%i\n", i + 1);
					rt_printf("Status word = 0x%X\n", ELMO_drive_pt[i].ptInParam->StatusWord);
					//rt_printf("Control Word = 0x%X\n", ELMO_drive_pt[i].ptOutParam->ControlWord);
					//rt_printf("modeOp = %i\n", modeOpdisplay);
					//rt_printf("mode=%d\n", controlmode);
					rt_printf("motion_thread=%ld[us]\n",Time_motion_run/1000);

					//rt_printf("cnt=%d\n", cnt_HS);
					//rt_printf("number of motion=%d\n",Num_of_motion);
					rt_printf("target_q(degree)=%3f\n", target_joint_pos_HS[i] * R2D);
					//rt_printf("target_q_dot(degree)=%3f\n", target_joint_vel_HS[i] * R2D);
					rt_printf("init_q(degree)=%3f\n", init_joint_pos_HS[i] * R2D);
					rt_printf("goal_q(degree)=%3f\n", goal_joint_pos_HS[i] * R2D);
					//rt_printf("kp=%3f\n",kp_joint_HS[i]);
					//rt_printf("torque=%3f\n",CTC_Torque[i]);
					//rt_printf("current(target)=%d\n",Cur2Tor(CTC_Torque[i],ratedCur[i]));
					//rt_printf("current(actual)=%d\n",ELMO_drive_pt[i].ptInParam->StatusWord);
					//rt_printf("Sensor Data:\n");
					rt_printf("actual_q(degree) = %3f\n", actual_joint_pos_HS[i] * R2D);
					rt_printf("ABS_actual_q(degree) = %3f\n", ABS_actual_joint_pos_HS[i] * R2D);
					//rt_printf("actual_q_dot=%3f\n",actual_joint_vel_HS[i]);		                                     
				}
				rt_printf("-----------------------------------------------------------------------\n");
			}

		}
		else {
			rt_printf("Ethercat Com fail(print)");
			rt_printf("\n");
		}
	}
}

void catch_signal(int sig)
{
	printf("function is catch_signal");
	
	//FileSave();
	
	run = 0;
	usleep(5e5);
	rt_task_delete(&motion_task);
	rt_task_delete(&print_task);
	exit(1);
}

int main(int argc, char* argv[])
{

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);
    
	printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
	mlockall(MCL_CURRENT | MCL_FUTURE);

	cycle_ns = 1000000; // nanosecond
	//period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	printf("use default adapter %s\n", ecat_ifname);


    //cpu_set_t cpu_set_motion;
	//CPU_ZERO(&cpu_set_motion);
	//CPU_SET(0, &cpu_set_motion); //assign CPU#0 for ethercat task
	
	//cpu_set_t cpu_set_print;
	//CPU_ZERO(&cpu_set_print);
	//sCPU_SET(1, &cpu_set_print); //assign CPU#1 (or any) for main task
	
	
	rt_task_create(&motion_task, "Motion_task", 0, 99, 0);
	//rt_task_set_affinity(&motion_task, &cpu_set_motion);
	rt_task_start(&motion_task, &motion_run, NULL);
	
	rt_task_create(&print_task, "Print_task", 0, 85, 0);
	//rt_task_set_affinity(&print_task, &cpu_set_print);
	rt_task_start(&print_task, &print_run, NULL);

	while (run)
	{

		usleep(5000);
	}
    

	printf("End program\n");
	return (0);
}


void ServoOn(void)
{
	//servo-on	
	for (i = 0; i < NUM_OF_ELMO; i++)
	{
		controlword = 0;
		started[i] = ServoOn_GetCtrlWrd(ELMO_drive_pt[i].ptInParam->StatusWord, &controlword);
		ELMO_drive_pt[i].ptOutParam->ControlWord = controlword;
		if (started[i]) ServoState |= (1 << i);
	}
}

void ServoOff(void)
{
	printf("function is Servo_off");
	//Servo OFF
	for (i = 0; i < NUM_OF_ELMO; i++)
	{
		ELMO_drive_pt[i].ptOutParam->ControlWord = 0; //Servo OFF (Disable voltage, transition#9)
	}
}

double Count2Deg(int Gear_Ratio, INT32 Count)
{
	double th = (double)Count * 360 / (2048 * Gear_Ratio);
	return th; // [deg]
}

double Count2DegDot(int Gear_Ratio, INT32 CountPerSec)
{
	double th_dot = (double)CountPerSec * 360 / (2048 * Gear_Ratio);
	return th_dot; // [deg/s]
}

double Count2Rad(int Gear_Ratio, INT32 Count)
{
	double th = (double)Count * 2 * PI / (2048 * Gear_Ratio);
	return th; // [rad]
}

double Count2RadDot(int Gear_Ratio, INT32 CountPerSec)
{
	double th_dot = (double)CountPerSec * 2 * PI / (2048 * Gear_Ratio);
	return th_dot; // [rad]
}

double Count2Rad_ABS(int _Resolution, INT32 Count)
{
  double th=(double)Count * 2 * PI / (_Resolution);
  return th;
}

INT32 Count_tf(int _Ratio, INT32 _Count_in)
{
 // INT32 Count14=(INT32)Count32/(256);
 INT32 _Count_out=(INT32)_Count_in/(_Ratio);
  return _Count_out;
}


INT16 Cur2Tor(double targetCur, double _ratedCur)
{
   INT16 inputTor = targetCur * 1000 / _ratedCur;

	return inputTor;
}

void ComputeTorqueControl_HS(void)
{
	for (int i = 0; i < NUM_OF_ELMO; i++) {
		CTC_Torque[i] = kp_joint_HS[i] * (target_joint_pos_HS[i] - actual_joint_pos_HS[i]) + kd_joint_HS[i] * (target_joint_vel_HS[i] - actual_joint_vel_HS[i]);
		//CTC_Torque[i] = kp_joint_HS[i] * (target_joint_pos_HS[i] - actual_joint_pos_HS[i]);
	}
}

void EncoderRead(void)
{
	if(Encoder_Reset_Flag==true){
    for (int i = 0; i < NUM_OF_ELMO; i++) {
        ABS_actual_joint_pos_HS[i] = Count2Rad_ABS(Resolution[i],Count_tf(Ratio[i],ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
		Incre_actual_joint_pos_offset_HS[i]=Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
	 }
	 Encoder_Reset_Flag=false;
	}
	
	
	    //Incre_actual_joint_pos_HS[0] = Count2Rad(Gear[0], ELMO_drive_pt[0].ptInParam->PositionActualValue)-Incre_actual_joint_pos_offset_HS[0];
		//actual_joint_pos_HS[0]= Incre_actual_joint_pos_HS[0];
		//actual_joint_vel_HS[0] = Count2RadDot(Gear[0], ELMO_drive_pt[0].ptInParam -> VelocityActualValue);
		
	for (int i = 0; i < NUM_OF_ELMO; i++) {
		//ABS_actual_joint_pos_HS[i] = Count2Rad_ABS(Resolution[i],Count_tf(Ratio[i],ELMO_drive_pt[i].ptInParam->AuxiliaryPositionActualValue));
		Incre_actual_joint_pos_HS[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue)-Incre_actual_joint_pos_offset_HS[i];
		//Incre_actual_joint_pos_HS[i] = Count2Rad(Gear[i], ELMO_drive_pt[i].ptInParam->PositionActualValue);
		actual_joint_pos_HS[i]=ABS_actual_joint_pos_HS[i]+Incre_actual_joint_pos_HS[i];
		//actual_joint_pos_HS[i]= Incre_actual_joint_pos_HS[i];
		actual_joint_vel_HS[i] = Count2RadDot(Gear[i], ELMO_drive_pt[i].ptInParam -> VelocityActualValue);
	
    }
}

void jointController(void)
{
	for (int i=0; i<NUM_OF_ELMO; i++){	 			
	 ELMO_drive_pt[i].ptOutParam->TargetTorque = Cur2Tor(CTC_Torque[i],ratedCur[i]);	
    }
}


void DataSave(void){
	save_array[save_cnt][0] = cnt_HS/1000;
	save_array[save_cnt][1] = target_joint_pos_HS[0];
	save_array[save_cnt][2] = target_joint_vel_HS[0];
	save_array[save_cnt][3] = Time_motion_run;
	if(save_cnt<SAVE_COUNT-1)
	save_cnt++;
}

void FileSave(void){
	FILE *fp; 
    
	fp = fopen("HSData.txt","w");
	for(int j = 0; j <= SAVE_COUNT -1; ++j)
	{
		for (int i=0; i<=SAVE_LENGTH-1; ++i){
		fprintf(fp,"%f\t",save_array[j][i]);
	}
	fprintf(fp,"%f\n",save_array[j][SAVE_LENGTH-1]);
	}
	
	fclose(fp);
}

