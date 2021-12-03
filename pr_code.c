/**
Draft for Raven II User Program
by Sean Fabrega
GOAL: Be able to run ROS, ambf_simulator, and ambf_raven_controller with
one file, this file. This program should:
    = Prompt user to hit enter to start roscore (in the background, child)
    = Take output from roscore and display it in GUI window
    = Prompt user to hit enter to start ./ambf_simulator -l 3 (in the background, child)
    = Take output from ./ambf_simulator and display it in GUI window (new tab?)
    = Prompt user to hit enter to start ./ambf_raven_controller (in the foreground, child)
    = if user quits ./ambf_raven_controller, exit it, roscore and ambf_simulator
    = once all of the child processes are exited, the parent process should ask if
    it should reboot roscore, ambf_simulator, etc.
SOURCES
    = https://tldp.org/LDP/lpg/node11.html
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>

pid_t * pid_array;
int num_processes = 4;
int f_desc[2];

int run_processes(void){
  pid_array[0] = getpid();
  pid_array[1] = fork();
  if (pid_array[1] == 0){
    //child process 1
    pid_array[1] = getpid();
    printf("roscore process %d ready!\n", pid_array[1]);
    f_desc[0] = open("roscore.txt", O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (f_desc[0] < 0){
      fprintf(stderr, "Open failed\n");
      exit(1);
    }
    if(dup2(f_desc[0], STDOUT_FILENO) == -1){
      fprintf(stderr, "Dup2 failed\n");
      close(f_desc[0]);
      exit(1);
    }
    if(close(f_desc[0]) < 0){
      fprintf(stderr, "Close failed\n");
      exit(1);
    }
    char * args1[] = {"roscore", NULL};
    printf("testing output here\n");
    if(execvp(args1[0], args1) == -1){
      fprintf(stderr, "Unable to execute \'roscore\'\n");
      exit(1);
    }
  }else{
    //parent process
    sleep(4);
    printf("parent %d ready!\n", pid_array[0]);

    pid_array[2] = fork();
    if (pid_array[2] == 0){
      pid_array[2] = getpid();
      printf("sim process %d ready!\n", pid_array[2]);
      f_desc[1] = open("sim.txt", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
      if(dup2(f_desc[1], STDOUT_FILENO) == -1){
        fprintf(stderr, "Dup2 failed\n");
        exit(1);
      }
      close(f_desc[1]);
      char * args2[] = {"./ambf_new/ambf/bin/lin-x86_64/ambf_simulator","-l","3", NULL};
      if(execvp(args2[0], args2) == -1){
        fprintf(stderr, "Unable to execute \'simulator\'\n");
        exit(1);
      }
    }
    sleep(4);
    printf("parent %d ready!\n", pid_array[0]);

    pid_array[3] = fork();
    if (pid_array[3] == 0){
      //child process
      pid_array[3] = getpid();
      printf("client process %d ready!\n", pid_array[3]);
      char * args3[] = {"./ambf_new/ambf/ambf_controller/raven2/scripts/ambf_raven_controller.py", NULL};
      if(execvp(args3[0], args3) == -1){
        fprintf(stderr, "Unable to execute \'client\'\n");
        exit(1);
      }
      exit(0);
    }else{
      wait(NULL);
      printf("Shutting down simulator...\n");
      kill(pid_array[1], SIGTERM);
      kill(pid_array[2], SIGTERM);
      free(pid_array);
      exit(0);
    }
  }
  return 0;
}

int main(int argc, char const *argv[]) {
  if (argc > 1){
    fprintf(stderr, "pr_code doesn't expect any arguments\n");
    exit(0);
  }
  printf("WELCOME TO THE RAVEN II SIMULATOR CLIENT\n");
  printf("==========================================\n");
  sleep(1);

  printf("Starting simulator...\n");
  pid_array = malloc(sizeof(*pid_array) * num_processes);
  if (pid_array == NULL){
    fprintf(stderr, "Memory allocation failed.\n");
    exit(1);
  }
  sleep(3);
  run_processes();
  sleep(3);
  return 0;
}
