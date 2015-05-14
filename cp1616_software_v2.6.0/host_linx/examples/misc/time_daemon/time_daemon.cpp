#include <unistd.h>
#include <cstdlib>
#include <sys/stat.h> 
#include "../../../servlib/inc/servusrx.h"
#include <sys/time.h>


int main()
{
    //process id and session id
    pid_t pid,sid;
    
    //for calculating the time
    struct timeval testtime;
    struct timezone testzone;
    
    //parameter of the time function
    PNIO_CP_SET_TIME_TYPE zeit;

    
    
    //create child process
    pid =fork();
    
    //did we get a bad ID? => exit with error
    if(pid<0)
    {
      exit(EXIT_FAILURE);
    }
    //did we get a god ID? => close parrent process
    if(pid>0)
    {
      exit(EXIT_SUCCESS);
    }
  
    //change the mode mask of the file

  
    umask(0); 
        
    
        
    //Creat new  sid for the childprocess
    sid = setsid();
    if (sid < 0) 
    {
	    //exit if there was an error
	    exit(EXIT_FAILURE);
    }
  
    //Change the working directory to root
    if ((chdir("/")) < 0) 
    {
	    //exit if there was an error
	    exit(EXIT_FAILURE);
    }
  
  
    //Close the standard file descriptors
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
	
    //infinite loop
    while(true)
    {
      gettimeofday(&testtime, &testzone); //gettime
      
      
      zeit.unix_utc_time = testtime.tv_sec - (testzone.tz_minuteswest*60); //timezone
    
      SERV_CP_set_time( 1,  &zeit ); //call time function
      sleep(600);  //wait 10 min
    }


}
