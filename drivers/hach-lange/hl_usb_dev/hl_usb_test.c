/***************************************************************************
                          usb_test.c  -  description
                             -------------------
    begin                : Die Jun 15 2004
    copyright            : (C) 2004 by Thomas Siegmund
    email                : tsiegmund@hach-lange.de
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <linux/wait.h>
#include <errno.h>
#include <signal.h>
#include <sys/poll.h>
#include <errno.h>
#include <assert.h>


#include "usbn9603register.h"
#include "usbn9603ioctl.h"
//defined in UsbDescriptor.h
#define NUM_OF_ENDPOINTS   7
#include "UsbState.h"

#define RECBUFSIZE   (1024 * 1024 *2)


#define MAXDATASIZE       2147483647
BYTE recbuf[RECBUFSIZE];
BYTE sendbuf[RECBUFSIZE];

char* getEPstate(int state)
{

   switch(state)
   {
    case EP_NOT_ASSIGNED:
       return "EP_NOT_ASSIGNED";
    case EP_ENABLED:
       return "EP_ENABLED";
    case EP_DISABLED:
       return "EP_DISABLED";
    case EP_IN_PROGRESS:
       return "EP_IN_PROGRESS";
    case EP_STALL:
       return "EP_STALL";
    default:
       return "??? unknown ???";
   }
}

int fd=0;
int rv;
struct pollfd pfd;
BYTE *precbuf = recbuf;
BYTE *psendbuf = sendbuf;
BYTE *psendold;
DWORD  io_data = 0;
DWORD datasize=0;
struct _USB_STATE  UsbState;
int toggle = 0;
char pcback[256];


void timer_handler(int signum)
{/*
 static int count = 0;
 printf ("timer expired %d times\n", ++count);
*/
static int pollreq =0;
int ret;

datasize = MAXDATASIZE;

printf("Timer function \n");

    //switch(signum)!!!????

    pfd.events =  POLLIN | POLLPRI;

//    assert(pfd.fd != NULL);
//    assert(fd != NULL);
    
    rv = poll(&pfd, 1, 0);

    if(pfd.revents & POLLERR)
    {
        ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
        printf("Get state of the USB, ret: %d\n", ret);
        printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
    }


    if(pfd.revents & POLLIN || pfd.revents & POLLPRI)
    {
         if((ret = read(fd, precbuf, datasize)) >= 0)
         {
              if(ret == 0)
              {
                   rv = poll(&pfd, 1, 0);
                   if(pfd.revents & POLLHUP)
                   {
                        ioctl(fd, USB_IO_GET_READ_STREAM_SIZE, &datasize);
                        printf("read geschaft, data size: %d\n", datasize);

                        memcpy(pcback, precbuf, datasize);

                        toggle = 1;

                        //exit(0);
                   }
               }
          }
     }

     printf("next poll request: %d \n", ++pollreq);

    if(pollreq>20)
       exit(0);
}

////////////////////////////////////
int main(int argc, char * argv[])
{
//extern int errno;
char buffer[12];
char *arg = argv[1];
int val;
int ret,i; 

int loopr, loopw;
char todo;

int flag;
int timeout=0;
int pollreq=0;

struct sigaction sa;
struct itimerval timer;

psendold = psendbuf;

 /* Install timer_handler as the signal handler for SIGVTALRM. */
memset (&sa, 0, sizeof (sa));
sa.sa_handler = &timer_handler;
sigaction(/*SIGVTALRM*/SIGALRM, &sa, NULL);

/* Configure the timer to expire after 250 msec... */
timer.it_value.tv_sec = 0;
timer.it_value.tv_usec = 300000;

/* ... and every 250 msec after that. */
timer.it_interval.tv_sec = 0;
timer.it_interval.tv_usec = 300000;

  memset(recbuf, 0, RECBUFSIZE);
  
  fd = 0;
#define  TESTSTR  "TEST GAMMA PC\n\r"
char send2PC[32];
//char *errstr;

     memcpy(send2PC, TESTSTR, sizeof(TESTSTR));

//  flag = O_SYNC;
    flag = O_NONBLOCK;      
/*
    fd = open("/dev/USBN9603_CTRL", O_RDWR);
      if(fd<=0)
        printf("uanbale to open the driver for the device USBN9603_CTRL\n");
    
   
    while(1)
    {
      fd = open("/dev/USBN9603_CTRL", O_RDWR);
      if(fd<=0)
        printf("uanbale to open the driver for the device USBN9603_CTRL\n");
      else
        break;
    }
*/      
     printf("%s read\n", (flag==O_NONBLOCK) ? "Non blocking" : "blocking");

     pfd.events =  POLLIN | POLLPRI;
      
    while(1)//todo != 'q')
    {
        todo =  getchar();

        switch(todo)    
        {
		
		case 'a':
			printf("open /dev/SsiAngle\n");
		
			fd = 0;
			errno = 0;
			 
            fd = open("/dev/SsiAngle", O_RDWR);
			//ioctl(fd, SSI_IO_START, 0);
			
			printf("open /dev/SsiAngle fd: %d\n", fd);
			if(fd <0) printf("Error open: error %d  error: %s\n", errno, strerror(errno));
		
		break;
/************************** vendor OUT transfer *************************************************/
          case 'v'://vendor OUT transfer -> receive data
              printf("open /dev/USBN9603_CTRL\n");

              while(1)  {
                fd = open("/dev/USBN9603_CTRL", O_RDWR);
                if(fd<=0)
                  printf("uanbale to open the driver for the device USBN9603_CTRL\n");
                else
                  break;
              }

              ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
              printf("Get state of the USB, ret: %d\n", ret);
              printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
              printf("----- Endpoint state --------\n");
              for(i=0; i<NUM_OF_ENDPOINTS; i++)
              {
                   printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
              }

              // deliver fd to poll
              pfd.fd = fd;
              printf("Start receive vendor out request data via EP0\n");

              fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | flag);

              datasize = MAXDATASIZE;    

              printf("1st poll request\n");
          
              while(1)
              {
                rv = poll(&pfd, 1, 0);
              
                printf("rv = %d ",rv);

                if(pfd.revents & POLLERR)
                {
                   ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
                   printf("Get state of the USB, ret: %d\n", ret);
                   printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
                   printf("----- Endpoint state --------\n");
                   for(i=0; i<NUM_OF_ENDPOINTS; i++)
                   {
                      printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
                   }
                }             
              
                if(pfd.revents & POLLIN || pfd.revents & POLLPRI)
                {
                    if((ret = read(fd, precbuf, datasize)) >= 0)
                    {
                       printf("Cntrl read ret %d\n", ret);         

                       if(ret == 0)
                       {
                           rv = poll(&pfd, 1, 0);
                           printf("rv = %d ",rv);

                           if(pfd.revents & POLLHUP)
                           {
                             ioctl(fd, USB_IO_GET_READ_STREAM_SIZE, &datasize);
                             printf("Read geschaft, data size: %d\n", datasize);

                             for(i=0; i<datasize; i++)
                             {
                               printf("%c",precbuf[i]);
                               if(i>0 && !(i%80))
                                 printf("\n");
                             }                             
                             break;
                           }
                       }                  
                    }
                }
               
               sleep(1);
               printf("next poll request\n");               
          }

          break;
/**********************************************************/
          case 'm':
          printf("open /dev/USBN9603_MEAS to receive data vi abulk transfer & send data via interrupt transfer\n");

          while(1)
          {
             fd = open("/dev/USBN9603_MEAS", O_RDWR);
             if(fd<=0)
                  printf("uanbale to open the driver for the device USBN9603_MEAS\n");
             else
                break;
          }
          fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | flag);
          datasize = MAXDATASIZE;

          //deliver fd to poll function
          pfd.fd = fd;
          
          ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
          printf("Get state of the USB, ret: %d\n", ret);
          printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
          printf("----- Endpoint state --------\n");
          for(i=0; i<NUM_OF_ENDPOINTS; i++)
          {
               printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
          }

          printf("Start receive bulk out request data via EP2\n");
          
          pollreq = 0;  
          
          for(loopr=0; loopr<20; loopr++)
          {               
              while(1)
              {
                  rv = poll(&pfd, 1, 0);
                            
                  if(pfd.revents & POLLERR)
                  {
                      ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
                      printf("Get state of the USB, ret: %d\n", ret);
                      printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");

                  }

                  if(pfd.revents & POLLIN || pfd.revents & POLLPRI)
                  {
                      if((ret = read(fd, precbuf, datasize)) >= 0)
                      {
                        printf("read ret %d\n", ret); 
                        if(ret == 0)
                        {
                            rv = poll(&pfd, 1, 0);
                            if(pfd.revents & POLLHUP)
                            {
                                ioctl(fd, USB_IO_GET_READ_STREAM_SIZE, &datasize);
                                printf("read geschaft, data size: %d\n", datasize);
                                goto out;
                            }
                        }
                      }
                  }
                  sleep(1);
                  printf("next poll request: %d \n", pollreq++);
            
              } //while

          }//for
out:
          for(loopr=0; loopr<datasize; loopr++)
             printf("%c", precbuf[loopr]);
          printf("\nready\n#");
          break;
/***************************************************************/
          case 'r':
          printf("open /dev/USBN9603_BULK to receive data vi abulk transfer & send data via bulk transfer\n");

          while(1)
          {
             fd = open("/dev/USBN9603_BULK", O_RDWR);
             if(fd<=0)
                  printf("uanbale to open the driver for the device USBN9603_BULK\n");
             else
                break;
          }
          fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | flag);
          datasize = RECBUFSIZE; //MAXDATASIZE;

          //deliver fd to poll function
          pfd.fd = fd;

          ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
          printf("Get state of the USB, ret: %d\n", ret);
          printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
          printf("----- Endpoint state --------\n");
          for(i=0; i<NUM_OF_ENDPOINTS; i++)
          {
               printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
          }

          printf("Start receive bulk out request data via EP2\n");

          pollreq = 0;
              while(1)
              {
                  rv = poll(&pfd, 1, 0);

                  if(pfd.revents & POLLERR)
                  {
                      ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
                      printf("Get state of the USB, ret: %d\n", ret);
                      printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
                  }

                  if(pfd.revents & POLLIN || pfd.revents & POLLPRI)
                  {
                      errno = 0;
                      if((ret = read(fd, precbuf, datasize)) >= 0)
                      {
                        printf("read ret %d\n", ret);
                        if(ret == 0)
                        {
                            rv = poll(&pfd, 1, 0);
                            if(pfd.revents & POLLHUP)
                            {
                                ioctl(fd, USB_IO_GET_READ_STREAM_SIZE, &datasize);
                                printf("read geschaft, data size: %d\n", datasize);
                                goto out1;
                            }
                        }
                      }
                      else //  if(ret <0)
                        printf("write returned with errno: %d  error: %s\n", errno, strerror(errno));
                  }
                  
                  sleep(1);
                  pollreq++;

                  if(!(pollreq%10))
                    printf("next poll request: %d \n", pollreq);

                  if(pollreq > 1000)
                    goto out1;

              } //while

out1:
          printf("\nready\n#");
          break;
//////////////////////////////////          
          case 't':
               while(1)
               {
                  timeout = 2000;
                  errno = 0;

//                printf("Set Time out of Write function, value %d ms\n and send data via bulk transfer", timeout);
                  ret = ioctl(fd, USB_IO_SET_TIMEOUT_WRITE_BULK, &timeout);

//                printf("write(fd: %d, precbuf: 0x%p, datasize: %d\n", fd, precbuf, datasize);   
               
                  if((ret=write(fd, precbuf, datasize)) >= 0)
                  {                    
                    if(ret == datasize)
                    {
                      printf("write geschaftt!!!\n");
                      break;
                    }   
                  }
                  if(ret <0)
                  {
                    printf("write returned with errno: %d  error: %s\n", errno, strerror(errno));
                    break;
                  }
                }                         
          break;
/*
          case 'i':
                timeout = 0;
                datasize = sizeof(unsigned long);

                fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_SYNC);
                ret = ioctl(fd, USB_IO_SET_TIMEOUT_WRITE, &timeout);
                printf("Send jiffies only\n");
                              
                ioctl(fd, USB_IO_BULK_DATA_IN, &datasize );

          break;
*/
          case 's':
               ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
               printf("Get state of the USB, ret: %d\n", ret);
               printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
               printf("----- Endpoint state --------\n");
               for(i=0; i<NUM_OF_ENDPOINTS; i++)
               {
                  printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
               }               
               break;
               
          case 'q': 
              //ioctl(fd, USB_IO_CONN_USB, DISCON_USB);
              //printf("Disconnect the USB bus\n");
              ret = close(fd);
              printf("Close command, file should be closed now, ret: %d\n", ret);
              fd = 0;
              return 0;
              break;
/*
          case 'b':                            
            printf("Send datasize: %d via Interrupt transfer\n", io_data);
            io_data = datasize;
            ioctl(fd, USB_IO_BULK_DATA_IN, &io_data );

            printf("start Bulk IN transfer with break\n");
           
             printf("user break write\n");
            while(1)
            {
              if((ret=write(fd, psendbuf, io_data)) >= 0)
              {
                  printf("ret %d \n", ret);      

                  for(i=0; i<20;i++)
                    printf("loops %d\n",i);
                  
                   printf("now break down\n");
                   ret = ioctl(fd, USB_IO_USER_BREAK);
                   printf("USB_IO_USER_BREAK ret: %d\n",ret);                                 
                  
                   if(ret == io_data)
                   {
                      printf("write geschaftt!!!\n");
                      break;
                   }
              }
              if(ret <0)
              {
                 printf("ret %d \n", ret);
                 break;
              }
            }
          break;
*/
          case 'l':
          printf("open /dev/USBN9603_MEAS to receive data vi abulk transfer & send data via interrupt transfer\n");

          while(1)
          {
             fd = open("/dev/USBN9603_MEAS", O_RDWR);
             if(fd<=0)
                  printf("uanbale to open the driver for the device USBN9603_MEAS\n");
             else
                break;
          }
          fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | flag);
          datasize = MAXDATASIZE;

          //deliver fd to poll function
          pfd.fd = fd;

          ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
          printf("Get state of the USB, ret: %d\n", ret);
          printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
          printf("----- Endpoint state --------\n");
          for(i=0; i<NUM_OF_ENDPOINTS; i++)
          {
               printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
          }

          printf("Start receive bulk out request data via EP2\n");

          pollreq = 0;  
                    
          loopr= loopw=0;          
          for(loopr=0; loopr<20; loopr++)
          {
                            
//              fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | flag);
              datasize = MAXDATASIZE;

              while(1)
              {
                  rv = poll(&pfd, 1, 0);

                  if(pfd.revents & POLLERR)
                  {
                      ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
                      printf("Get state of the USB, ret: %d\n", ret);
                      printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
                     
                  }

                  if(pfd.revents & POLLIN || pfd.revents & POLLPRI)
                  {
                      if((ret = read(fd, precbuf, datasize)) >= 0)
                      {
                        if(ret == 0)
                        {
                            rv = poll(&pfd, 1, 0);
                            if(pfd.revents & POLLHUP)
                            {
                                ioctl(fd, USB_IO_GET_READ_STREAM_SIZE, &datasize);
                                printf("read geschaft, data size: %d\n", datasize);
                                break;
                            }
                        }
                      }
                  }
                  sleep(1);                 
                  printf("next poll request: %d \n", pollreq++);
                  /*
                  todo = getchar();
                  printf("todo: %c\n", todo);
                  if(todo == 'q')
                     goto lout;
                    */
              } //while
          
//////////////////////////////////////////////    
              memcpy(psendbuf, precbuf, datasize);

              for(loopw=0; loopw<5; loopw++)
              {
                  printf("loopw: %d\n", loopw);
                  
                  timeout = 2000;
//                  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_SYNC);

                  printf("Set Time out of Write function, value %d ms\n", timeout);
                  ret = ioctl(fd, USB_IO_SET_TIMEOUT_WRITE_MEASUREMENT, &timeout);

                  printf("Send datasize: %d via Interrupt transfer\n", datasize);
//                  io_data = datasize;
                  ioctl(fd, USB_IO_SET_MEAS_DATA_SIZE, &datasize );

                  if((ret=write(fd, psendbuf, datasize)) >= 0)
                  {                       
                     if(ret == io_data)
                     {
                       printf("write geschaftt!!!\n");
                     }
                  }
                  if(ret <0)
                  {
                    printf("ret %d \n", ret);
                      //errno auswerten!!!
                      printf("write returned with errno: %d  error: %s\n", errno, strerror(errno));
                  }
              }//for(loopw
              
       }//for(loopr
//lout:
       printf("ready\n#");
       break;
/*********************************************************
       case 't':
          printf("open /dev/USBN9603_MEAS to receive data vi abulk transfer & send data via interrupt transfer\n");                  
                      
          while(1)
          {
             fd = open("/dev/USBN9603_MEAS", O_RDWR | flag);
             if(fd<=0)
                  printf("uanbale to open the driver for the device USBN9603_MEAS\n");
             else
                break;
          }
          datasize = MAXDATASIZE;
          //deliver fd to poll function
          pfd.fd = fd;

          ret = ioctl(fd, USB_IO_GET_USB_STATUS, &UsbState);
          printf("Get state of the USB, ret: %d  file descriptor: %d \n", ret, fd);
          printf("USB Bus state: %s\n",(UsbState.BusState==USB_CONNECTED) ? "Gamma is connected to PC" : "Gamma isn't connected");
          printf("----- Endpoint state --------\n");
          for(i=0; i<NUM_OF_ENDPOINTS; i++)
          {
               printf("endpoint_%d:  %s\n", i, getEPstate(UsbState.EpState[i]) );
          }

          printf("Start receive bulk out request data via EP2\n");

          sleep(2);

          setitimer(ITIMER_REAL, &timer, NULL);
          
          loopr= loopw=0;
         // for(loopw=0; loopw<5; loopw++)
          while(loopr<56)
          {
          loopr++;

              if(toggle == 1)
              {
                 psendold = psendbuf;
                 psendbuf = pcback;                      
                 toggle=0;
                 printf("sende zurck Daten empfangen vom PC, pcback: %p\n", pcback);
              }
              else
              {
                 psendbuf = psendold;
                 psendbuf = send2PC;                     
                 datasize = sizeof(TESTSTR);
              }
                            
              timeout = 20000;
              //printf("Set Time out of Write function, value %d ms\n", timeout);
              ret = ioctl(fd, USB_IO_SET_TIMEOUT_WRITE, &timeout);

              printf("Send datasize: %d via Interrupt transfer toggle: %d\n", datasize, toggle);
              
              ioctl(fd, USB_IO_SET_MEAS_DATA_SIZE, &datasize );

              errno = 0;
              if((ret=write(fd, psendbuf, datasize)) >= 0)
              {
                  if(ret == datasize)
                  {
                      printf("write geschaftt!!!\n");
                  }
              }           
                  
              if(ret <0)
              {
                    printf("ret %d \n", ret);
                      //errno auswerten!!!
                    //if(ret == -ETIME)
                    printf("write returned with errno: %d  error: %s\n", errno, strerror(errno));
              }
             
         }//for(
              
       printf("ready\n#");
       break;
*/
        }//switch

    }//while

//  return 0;
}


/*
              if(pfd.revents & POLLIN) {printf("pollin\n");}
              if(pfd.revents & POLLPRI){printf("pollpri\n");}
              if(pfd.revents & POLLOUT){printf("pollout\n");}
              if(pfd.revents & POLLERR){printf("pollerr\n");}
              if(pfd.revents & POLLHUP){printf("pollhup \n");}
*/


