/*
 * dataq.c:
 *	Program to process DataQ data fromt DI-145 on the USB serial port.
*/

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include "dataq.h"

int           dataq;          // global storage for dataq device handle

DataQData     myDataQ;

/*
*********************************************************************************************************
*       name: dataq_data
*       function: provide other threads a pointer to the data structure holding the collected samples
*       parameter: NULL
*       The return value:  pointer to the global data structure holding collected samples
*********************************************************************************************************
*/
DataQData * dataq_data(void)
{
    return(&myDataQ);
}

/*
*********************************************************************************************************
*       name: initDataQSerial
*       function: initialize serial port and configure DataQ module
*       parameter: NULL
*       The return value:  success = 0
*********************************************************************************************************
*/
int initDataQSerial(void)
{
    int i;
    int msglen;
    int dataq;
    char tmpmsg[128];

    // Open port at 9600 baud for config.
    dataq = serialOpen("/dev/ttyACM0", 4800);
    if (dataq == -1)
    {
        return(-1);
    }

    // Stop sampling
    write(dataq, "stop\r", 5);
    usleep(10000);

    serialFlush(dataq);

    // Get manufacture
//    write(dataq, "info 0\r", 7);

    // Get device name
//    write(dataq, "info 1\r", 7);

    // Get firmware revision
//    write(dataq, "info 2\r", 7);

    // Get serial number
//    write(dataq, "info 6\r", 7);

    // Set output scan list order
    write(dataq, "slist 0 0\r", 10);
    usleep(20000);
    write(dataq, "slist 1 1\r", 10);
    usleep(20000);
    write(dataq, "slist 2 2\r", 10);
    usleep(20000);
    write(dataq, "slist 3 3\r", 10);
    usleep(20000);
    write(dataq, "slist 4 65535\r", 14);
    usleep(28000);

    // Set output format to binary
    write(dataq, "bin\r", 4);
    usleep(8000);

    // Start scan output
    write(dataq, "start\r", 6);
    usleep(12000);

    // flush input queue
    serialFlush(dataq);

    return (dataq);
}

/*
*********************************************************************************************************
*	name: run_dataq
*	function: initialize, start, and continuously process dataq stream
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/

void * run_dataq(void * x)
{
    unsigned char i, j;
    unsigned char buf[8];
    unsigned char sampleHalf;
    unsigned char dataqLen=8;
    int           sampleLen;
    short         sample;

    myDataQ.running = FALSE;

    dataq = initDataQSerial();
    if (dataq == -1)
    {
        fprintf(stderr, "\r\nFailed to initilize DataQ USB port\r\n");
        return(0);
    }

    myDataQ.running = TRUE;

    while(myDataQ.running)
    {
        sampleLen = serialDataAvail(dataq);
	if (sampleLen < 0)
        {
            fprintf(stderr, "DataQ serialDataAvail() returned error\r\n");
        }
        else if (sampleLen > 0)
        {
            for (j = 0; j < sampleLen; j++)
            {
                sampleHalf = serialGetchar(dataq);

                if ((sampleHalf & 0x01) == 0x00)
                {
                    dataqLen = 0;
                }

                if (dataqLen < 8)
                {
                    buf[dataqLen++] = (sampleHalf >> 1) & 0x7F;
                }

                if (dataqLen == 7)
                {
                    for (i = 0; i < 4; i++)
                    {
                        sample = ( ( buf[i * 2] >> 4 ) & 0x0007 ) | ( ( buf[i * 2 + 1] << 3 ) & 0x03F8 );
                        if (sample & 0x0200)
                        {
                            sample ^= 0x0200;
                        }
                        else
                        {
                            sample |= 0xFE00;
                        }
                        pthread_mutex_lock(&myDataQ.lock);
                        myDataQ.analog[i] = sample;
                        pthread_mutex_unlock(&myDataQ.lock);
                    }
                    pthread_mutex_lock(&myDataQ.lock);
                    myDataQ.digital = buf[6] & 0x03;
                    pthread_mutex_unlock(&myDataQ.lock);
                }
            }
        }
        usleep(2000);
    }

    // Stop sampling
    write(dataq, "stop\r", 5);
    usleep(10000);

    serialFlush(dataq);
    serialClose(dataq);
    return(0);
}

/*
*********************************************************************************************************
*       name: stop_dataq
*       function: stop dataq stream processing and close serial port
*       parameter: NULL
*       The return value:  NULL
*********************************************************************************************************
*/
void stop_dataq(void)
{
    // Stop sampling
    write(dataq, "stop\r", 5);

    myDataQ.running = FALSE;
}

