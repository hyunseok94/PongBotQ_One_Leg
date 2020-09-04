#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>

//PCAN Driver
#include <fcntl.h>      // for pcan     O_RDWR
#include <libpcan.h>    // for pcan library.

//Save
#define SAVE_LENGTH 8    //The number of data
#define SAVE_COUNT 3600000 //Save Time = 10000[ms]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];
long Thread_time_upper;
long Thread_time_lower;
double Thread_time1;
double Thread_time2;
RTIME now1, previous1;
RTIME now2, previous2;

RT_TASK demo_task;

//CAN
HANDLE can_handle = nullptr;

void DataSave(void);
void FileSave(void);

void demo(void *arg) {
    rt_task_set_periodic(NULL, TM_NOW, 1e6);
    now2 = rt_timer_read();

    while (1) {
        rt_task_wait_period(NULL);
        previous1 = rt_timer_read();
        previous2 = now2;


        //  Thread_time_upper=(long)(now-previous)/1000000;
        //    Thread_time_lower=(long)(now-previous)%1000000;
        //printf("Time since last turn: %ld.%06ld ms\n", (long)(now-previous)/1000000,(long)(now-previous)%1000000);

        //printf("Time since last turn: %ld.%06ld ms\n", Thread_time_upper, Thread_time_lower);

        now1 = rt_timer_read();
        now2 = rt_timer_read();

        Thread_time1 = (double) (now1 - previous1) / 1000000;
        Thread_time2 = (double) (now2 - previous2) / 1000000;

        printf("Time since last turn: %6f ms / %6f ms\n", Thread_time1, Thread_time2);
        DataSave();
    }
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
    
    //can_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
    
    CAN_Init(can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);

    printf("Status = %i\n", CAN_Status(can_handle));

    if(CAN_Status(can_handle)==32)
    {
        printf("can open\n");
    }
    else
    {
        printf("can Init fail\n");
        //exit(0);
    }


    //printf("Starting cyclic task...\n");

    //char str[20];
    //sprintf(str, "send task");
    //int rtcreateflag=rt_task_create(&Can_send_task, str, 0, 50, 0);
//    if(rtcreateflag!=0)
//    {
//        printf("rt cansend create fail.\n");
//        exit(0);
//        this->close();
//
//    }
//
//    int rtstartflag=rt_task_start(&Can_send_task, &Can_send_task_proc, 0);
//    if(rtstartflag!=0)
//    {
//        printf("rt start fail.\n");
//        exit(0);
//        this->close();
//    }
    
    
    pause();
    rt_task_delete(&demo_task);

    return 0;
}

void DataSave(void) {
    save_array[save_cnt][0] = Thread_time1;
    save_array[save_cnt][1] = Thread_time2;
    //    save_array[save_cnt][2] = Thread_time;


    if (save_cnt < SAVE_COUNT - 1)
        save_cnt++;
}

void FileSave(void) {
    FILE *fp;

    fp = fopen("Example1_Data.txt", "w");
    for (int j = 0; j <= SAVE_COUNT - 1; ++j) {
        for (int i = 0; i <= SAVE_LENGTH - 1; ++i) {
            fprintf(fp, "%f\t", save_array[j][i]);
        }
        fprintf(fp, "%f\n", save_array[j][SAVE_LENGTH - 1]);
    }

    fclose(fp);
}
