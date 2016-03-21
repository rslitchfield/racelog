/*
 * racelog.c:
 *	Main program to log race parameters from
 *         DataQ DI-145 4 channel digital-to-analog (throttle, brake, clutch, steering) & 2 channel digital input
 *         Digital pulse input (CoilX) engine speed
 *         uBlox NEO-M8N (RY835AI) GPS NMEA messages
 *	   BMP180 (RY835AI) Temperature compensated pressure
 *         MPU-6050 (RY835AI) 3-axis Gyro/Accelerometer
 *         HMC5983 (RY835AI) 3-axix Magnemometer
 */

#define BCM2835_NO_DELAY_COMPATIBILITY
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <bcm2835.h>
#include <pthread.h>
#include <wiringPi.h>
#include "gps.h"
#include "dataq.h"
#include "mpu6050.h"

#define	TACH_PIN 27  // Pin 13 = BCM 27; WiringPi GPIO 2

int run_log;
struct timespec gettime_spark;
int64_t spark_time[8];
static volatile int filter_count = 0;
static volatile unsigned short tachCounter = 0;
pthread_mutex_t tachLock;

void tachInterrupt (void)
{
    clock_gettime(CLOCK_REALTIME, &gettime_spark);
    
    pthread_mutex_lock(&tachLock);
    ++tachCounter;
    spark_time[filter_count++ & 7] = gettime_spark.tv_nsec;
    pthread_mutex_unlock(&tachLock);
}

/*
*********************************************************************************************************
*	name: main
*	function:
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/

int  main(void)
{
    int i, runTime;
    FILE * logFile;
    char filename[64];
    char temp[64];
    struct timespec gettime_now;
    struct timespec start_time;
    int64_t last_heartbeat, heartbeat_difference;
    pthread_t thread[3];
    pthread_mutex_t gps_lock;
    pthread_mutex_t i2c_lock;
    int policy;
    struct sched_param schedule;
    unsigned short lastTach = 0;
    unsigned short tachDiff = 0;
    unsigned short tach = 0;
    int64_t tach2 = 0;
    char sampleCount = 0;
    const unsigned char sampleCounts = 6;
    const unsigned short rpmPerCount = 1800 / 3 / sampleCounts;  // samplesPerMinute(1800) / 

    pthread_mutex_init(&tachLock, NULL);

    SituationData * mySituation;
    mySituation = gps_data();
    pthread_mutex_init(&mySituation->lock, NULL);
    SituationData snapGps;

    DataQData * myDataQ;
    myDataQ = dataq_data();
    pthread_mutex_init(&myDataQ->lock, NULL);
    DataQData snapDataQ;

    MPUData * myMPUData;
    myMPUData = mpu_data();
    pthread_mutex_init(&myMPUData->lock, NULL);
    MPUData snapMPUData;

    BMP180Data * myBMPData;
    myBMPData = bmp_data();
    pthread_mutex_init(&myBMPData->lock, NULL);
    BMP180Data snapBMPData;

    if ( pthread_create(&thread[0], NULL, run_gps, NULL) )
    {
        fprintf(stderr, "Failed to create GPS thread.\r\n");
        return -1;
    }
    pthread_setname_np(thread[0], "GPS");

    if ( pthread_create(&thread[1], NULL, run_dataq, NULL) )
    {
        fprintf(stderr, "Failed to create DataQ thread.\r\n");
        return -2;
    }
    pthread_setname_np(thread[1], "DataQ");

    if ( pthread_create(&thread[2], NULL, run_i2c1, NULL) )
    {
        fprintf(stderr, "Failed to create MPU6050/BMP180 thread\r\n");
        return -3;
    }
    pthread_setname_np(thread[2], "MPU6050_BMP180");

    if (wiringPiSetupGpio() >= 0)
    {
        if (wiringPiISR(TACH_PIN, INT_EDGE_RISING, &tachInterrupt) < 0)
        {
            fprintf(stderr, "Unable to setup Tachometer ISR\r\n");
        }
    }
    else
    {
        fprintf(stderr, "Unable to setup wiringPi\r\n");
    }

    usleep(2000000);

    while(1)
    {
        // Wait for recording signal
        while((snapDataQ.digital & 0x02) || !(snapDataQ.running))
        {
            usleep(200000);
            pthread_mutex_lock(&myDataQ->lock);
            snapDataQ.digital = myDataQ->digital;
            snapDataQ.running = myDataQ->running;
            pthread_mutex_unlock(&myDataQ->lock);
        }

        // Set log file name
        clock_gettime(CLOCK_REALTIME, &gettime_now);
        sprintf(filename, "/mnt/sda1/logs/race%02d%02d%02d.csv", (gettime_now.tv_sec % (60 * 60 * 24)) % 24, (gettime_now.tv_sec % (60 * 60)) % 60, gettime_now.tv_sec % 60);

        // Open log file
        logFile = fopen(filename, "a");
        if(logFile == NULL)
        {
            fprintf(stderr, "Failed to open log file for writing\r\n");
            return(-1);
        }

        // Write csv header once

        fprintf(logFile, "runTime,");

        fprintf(logFile, "a2dRun,");
        fprintf(logFile, "steer,");
        fprintf(logFile, "throttle,");
        fprintf(logFile, "brake,");
        fprintf(logFile, "spare,");
        fprintf(logFile, "event,");
//        fprintf(logFile, "record,");

        fprintf(logFile, "tach,");
        fprintf(logFile, "tach2");

        fprintf(logFile, "gpsRun,");
//        fprintf(logFile, "lastFixTime,");
        fprintf(logFile, "gpsLat,");
        fprintf(logFile, "gpsLon,");
        fprintf(logFile, "gpsAccuracy,");
//        fprintf(logFile, "gpsQual,");
        fprintf(logFile, "gpsAlt,");
//        fprintf(logFile, "gpsVertVel,");
        fprintf(logFile, "gpsVertAcc,");
        fprintf(logFile, "gpsSpeed,");
        fprintf(logFile, "gpsHeading,");
        fprintf(logFile, "gpsUTCdate,");
        fprintf(logFile, "gpsUTCtime,");
        fprintf(logFile, "gpsSatTracked,");
        fprintf(logFile, "gpsSatSeen,");

        fprintf(logFile, "mpuRun,");
        fprintf(logFile, "Gx,");
        fprintf(logFile, "Gy,");
        fprintf(logFile, "Gz,");
        fprintf(logFile, "Gtotal,");
        fprintf(logFile, "GyroX,");
        fprintf(logFile, "GyroY,");
        fprintf(logFile, "GyroZ,");
        fprintf(logFile, "mpuDieTemp,");

        fprintf(logFile, "ambientTemp,");
        fprintf(logFile, "pressureAlt");

        fprintf(logFile, "\r\n");

        fflush(logFile);

        clock_gettime(CLOCK_REALTIME, &gettime_now);
        last_heartbeat = gettime_now.tv_nsec;
        start_time.tv_sec = gettime_now.tv_sec;

        // continue to loop while recording signal is on
        while(!(snapDataQ.digital & 0x02))
        {
            clock_gettime(CLOCK_REALTIME, &gettime_now);
            heartbeat_difference = gettime_now.tv_nsec - last_heartbeat;
            if (heartbeat_difference < 0)
            {
                heartbeat_difference += 1000000000;
            }

            // 30 Hz loop
            if (heartbeat_difference > 33333333)
            {
                // update last heartbeat based on 30 Hz, not period (with jitter) when loop actually executed
                last_heartbeat += 33333333;
                if (last_heartbeat > 1000000000)
                {
                    last_heartbeat -= 1000000000;
                }

                // log proces time stamp of sample
                fprintf(logFile, "% 4d.%03d", gettime_now.tv_sec - start_time.tv_sec, gettime_now.tv_nsec/1000000);

                // process tach counter once every 6 30 Hz loops (5 Hz)
                if (++sampleCount >= sampleCounts)
                {
                    sampleCount = 0;
                    pthread_mutex_lock(&tachLock);
                    tachDiff = tachCounter - lastTach;
                    pthread_mutex_unlock(&tachLock);

                    lastTach += tachDiff;
                    if (tachDiff < sampleCounts * 20) // skip overflow (> 20,000 RPM), about once every 20 minutes
                    {
                        tach = tachDiff * rpmPerCount;
                    }
                }
                // process tach timer every 30 Hz loop (average last 8 sparks)
                pthread_mutex_lock(&tachLock);
                seven_spark_time = spark_time[spark_count] - spark_time[spark_count ? spark_count - 1 : 7];
                pthread_mutex_unlock(&tachLock);
                if (seven_spark_time < 0 )
                {
                    seven_spark_time += 1000000000;
                }
                if (seven_spark_time)
                {
                    tach2 = 140000000 / ( (seven_spark_time + 500) / 1000);
                }
                else
                {
                    tack2 = 0;
                }
                
                // take snapshot of DataQ data generated in seperate thread
                pthread_mutex_lock(&myDataQ->lock);
                snapDataQ.running = myDataQ->running;
                snapDataQ.analog[0] = myDataQ->analog[0];
                snapDataQ.analog[1] = myDataQ->analog[1];
                snapDataQ.analog[2] = myDataQ->analog[2];
                snapDataQ.analog[3] = myDataQ->analog[3];
                snapDataQ.digital = myDataQ->digital;
                pthread_mutex_unlock(&myDataQ->lock);

                // log DataQ data samples
                fprintf(logFile, ",%d", snapDataQ.running);
                fprintf(logFile, ",% 4d", snapDataQ.analog[0]);
                fprintf(logFile, ",% 4d", snapDataQ.analog[1]);
                fprintf(logFile, ",% 4d", snapDataQ.analog[2]);
                fprintf(logFile, ",% 4d", snapDataQ.analog[3]);
                fprintf(logFile, ",%d", snapDataQ.digital & 1);
//                fprintf(logFile, ",%d", (snapDataQ.digital >> 1) & 1);

                // log Tachometer sample
                fprintf(logFile, ",% 4d", tach);
                fprintf(logFile, ",% 4d", tach2);

                // take snapshot of GPS data generated in seperate thread
                pthread_mutex_lock(&mySituation->lock);
                snapGps.running = mySituation->running;
                snapGps.lastFixSinceMidnightUTC = mySituation->lastFixSinceMidnightUTC;
                snapGps.Lat = mySituation->Lat;
                snapGps.Lng = mySituation->Lng;
                snapGps.GeoidSep = mySituation->GeoidSep;
                snapGps.Accuracy = mySituation->Accuracy;
                snapGps.quality = mySituation->quality;
                snapGps.Alt = mySituation->Alt;
                snapGps.GPSVertVel = mySituation->GPSVertVel;
                snapGps.AccuracyVert = mySituation->AccuracyVert;
                snapGps.GroundSpeed = mySituation->GroundSpeed;
                snapGps.TrueCourse = mySituation->TrueCourse;
                snapGps.day = mySituation->day;
                snapGps.month = mySituation->month;
                snapGps.year = mySituation->year;
                snapGps.hour = mySituation->hour;
                snapGps.minute = mySituation->minute;
                snapGps.second = mySituation->second;
                snapGps.SatellitesTracked = mySituation->SatellitesTracked;
                snapGps.SatellitesSeen = mySituation->SatellitesSeen;
                pthread_mutex_unlock(&mySituation->lock);

                // log GPS samples
                fprintf(logFile, ",%d", snapGps.running);
//                fprintf(logFile, ",%d", snapGps.lastFixSinceMidnightUTC);  // Time of last GPS fix, in seconds, since midnight UTC
                fprintf(logFile, ",%11.7f", snapGps.Lat);                      // GPS Latitude in degrees
                fprintf(logFile, ",%12.7f", snapGps.Lng);                      // GPS Longitude in degrees
                fprintf(logFile, ",%4.1f", snapGps.Accuracy);                 // GPS Horizontal accuracy in meters (2-sigma)
//                fprintf(logFile, ",%d", snapGps.quality);                  // GPS Fix quality {2 = WAAS/DGPS}}
                fprintf(logFile, ",%4.f", snapGps.Alt);                      // GPS Altitude in feet
//                fprintf(logFile, ",%.1f", snapGps.GPSVertVel);               // GPS Vertical Velocity in ft/sec
                fprintf(logFile, ",%5.1f", snapGps.AccuracyVert);             // GPS Vertical accuracy in meters
                fprintf(logFile, ",% 3d", snapGps.GroundSpeed);              // GPS Ground speed in Knots
                fprintf(logFile, ",% 3d", snapGps.TrueCourse);               // GPS True Course in degrees
                fprintf(logFile, ",%02d-%02d-%04d", snapGps.day, snapGps.month, snapGps.year); // GPS UTC date
                fprintf(logFile, ",%02d:%02d:%02d", snapGps.hour, snapGps.minute, snapGps.second); // GPS UTC time
                fprintf(logFile, ",% 2d", snapGps.SatellitesTracked);        // GPS Satellites tracked (above horizon)
                fprintf(logFile, ",% 2d", snapGps.SatellitesSeen);           // GPS Satellites seen (signal received)

                // take snapshot of MPU-6050 data generated in seperate thread
                pthread_mutex_lock(&myMPUData->lock);
                snapMPUData.running = myMPUData->running;
                snapMPUData.Gx = myMPUData->Gx;
                snapMPUData.Gy = myMPUData->Gy;
                snapMPUData.Gz = myMPUData->Gz;
                snapMPUData.Gtotal = myMPUData->Gtotal;
                snapMPUData.Gyrox = myMPUData->Gyrox;
                snapMPUData.Gyroy = myMPUData->Gyroy;
                snapMPUData.Gyroz = myMPUData->Gyroz;
                snapMPUData.Temperature = myMPUData->Temperature;
                pthread_mutex_unlock(&myMPUData->lock);

                // log MPU-6050 data samples
                fprintf(logFile, ",%d", snapMPUData.running);
                fprintf(logFile, ",%+.2f", snapMPUData.Gx);                        // MPU6050 Accelerometer X-axis
                fprintf(logFile, ",%+.2f", snapMPUData.Gy);                        // MPU6050 Accelerometer Y-axis
                fprintf(logFile, ",%+.2f", snapMPUData.Gz);                        // MPU6050 Accelerometer Z-axis
                fprintf(logFile, ",%+.2f", snapMPUData.Gtotal);                    // Calculated total Gs
                fprintf(logFile, ",%+.1f", snapMPUData.Gyrox);                     // Rotational rate about X-axis
                fprintf(logFile, ",%+.1f", snapMPUData.Gyroy);                     // Rotational rate about Y-axis
                fprintf(logFile, ",%+.1f", snapMPUData.Gyroz);                     // Rotational rate about Z-axis
                fprintf(logFile, ",%5.1f", snapMPUData.Temperature);               // MPU6050 die temperature

                // take snapshot of BMP180 data generated in seperate thread
                pthread_mutex_lock(&myBMPData->lock);
                snapBMPData.temperature = myBMPData->temperature;
                snapBMPData.altitude = myBMPData->altitude;
                pthread_mutex_unlock(&myBMPData->lock);

                fprintf(logFile, ",%5.1f", snapBMPData.temperature);
                fprintf(logFile, ",%6.1f", snapBMPData.altitude);

                fprintf(logFile, "\r\n");                                           // create new line for next scan

                fflush(logFile);
            }
            usleep(2000);        // wait before checking for next scan completion
        }
        // Recording signal is off, close log file
        fclose(logFile);
    }
    // nothing below this line ever executes, but is included for proper programming etiquette

    // send stop signal to sub-threads
    stop_gps();
    stop_dataq();
    stop_i2c1();

    // clean up the mutexs
    pthread_mutex_destroy(&mySituation->lock);
    pthread_mutex_destroy(&myDataQ->lock);
    pthread_mutex_destroy(&myBMPData->lock);

    // wait for the sub-threads to complete before closing main process
    pthread_join(thread[0], NULL);
    pthread_join(thread[1], NULL);
    pthread_join(thread[2], NULL);

    return 0;           // this line never executes
}


