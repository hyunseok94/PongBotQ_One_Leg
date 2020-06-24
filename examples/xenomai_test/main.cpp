#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>

//Save
#define SAVE_LENGTH 8    //The number of data
#define SAVE_COUNT 10000 //Save Time = 10000[ms]
unsigned int save_cnt = 0;
double save_array[SAVE_COUNT][SAVE_LENGTH];
long Thread_time_upper;
long Thread_time_lower;
double Thread_time;

RT_TASK demo_task;
void DataSave(void);
void FileSave(void);

void demo(void *arg)
{
    RTIME now, previous;
    
    rt_task_set_periodic(NULL, TM_NOW, 1e6);
    previous =rt_timer_read();
    
    while(1){
     rt_task_wait_period(NULL);
     now=rt_timer_read();
    
    Thread_time_upper=(long)(now-previous)/1000000;
    Thread_time_lower=(long)(now-previous)%1000000;
    //printf("Time since last turn: %ld.%06ld ms\n", (long)(now-previous)/1000000,(long)(now-previous)%1000000);
    Thread_time=(double) (now-previous)/1000000;
    
    printf("Time since last turn: %6f ms\n", Thread_time);
    //printf("Time since last turn: %ld.%06ld ms\n", Thread_time_upper, Thread_time_lower);
    
    previous=now;
    DataSave();
    }
}

void catch_signal(int sig)
{
     FileSave();
}

int main(int argc, char* argv[])
{
    signal(SIGTERM,catch_signal);
    signal(SIGINT,catch_signal);
    
    mlockall(MCL_CURRENT|MCL_FUTURE);
    
    rt_task_create(&demo_task,"trivial", 0, 99,0);
    rt_task_start(&demo_task,&demo,NULL);
    pause();
    rt_task_delete(&demo_task);
    
    return 0;
}

void DataSave(void) {
    save_array[save_cnt][0] = Thread_time_upper;
    save_array[save_cnt][1] = Thread_time_lower;
    save_array[save_cnt][2] = Thread_time;
    

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