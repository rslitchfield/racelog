/*
 * gps.c:
 *	Program to process uBlox NMEA data fromt the RS-232 serial port.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "gps.h"

//TXD     -----   TX
//RXD     -----   RX

SituationData	mySituation;

/*
*********************************************************************************************************
*       name: gps_data
*       function: provide other threads a pointer to the global data structure holding sample data
*       parameter: none
*       The return value: pointer to the global data structure holding the data samples
*********************************************************************************************************
*/
SituationData * gps_data(void)
{
    return(&mySituation);
}

/*
*********************************************************************************************************
*       name: chksumUBX
*       function: calculate and store message checksum
*       parameter: message pointer, checksum pointer, message length
*       The return value:  calculated checksum stored at checksum pointer location
*********************************************************************************************************
*/
void chksumUBX(unsigned char * msg, unsigned char * chksum, int len)
{
    int i;

    chksum[0] = 0;
    chksum[1] = 0;
    for (i = 0; i < len; i++)
    {
        chksum[0] += msg[i];
        chksum[1] += chksum[0];
    }
    return;
}

/*
*********************************************************************************************************
*       name: makeUBXCFG
*       function: construct GPS configuration message
*       parameter: class, id, length, data
*       The return value:  constructed message in msg*, message length +=8
*********************************************************************************************************
*/
int makeUBXCFG(char msgClass, char id, int msglen, char* msg)
{
    int i;
    for (i = msglen - 1; i >= 0; i--)
    {
        msg[i+6] = msg[i];
    }
    msg[0] = 0xB5; // SYNC 1
    msg[1] = 0x62; // SYNC 2
    msg[2] = msgClass;
    msg[3] = id;
    msg[4] = msglen & 0xFF;
    msg[5] = (msglen >> 8) & 0xFF;
    chksumUBX(&msg[2],&msg[msglen+6],msglen+4);
    return (msglen+8);
}

/*
*********************************************************************************************************
*       name: initGPSSerial
*       function: initialize serial port and configure GPS module
*       parameter: NULL
*       The return value:  success = 0
*********************************************************************************************************
*/
int initGPSSerial() {
    int i;
    int msglen;
    int rs232;
    char tmpmsg[128];
    // Open port at 9600 baud for config.
    rs232 = serialOpen("/dev/ttyAMA0",9600);
    if (rs232 == -1)
    {
        return(-1);
    }

    // Set 10Hz (MAX) update.
    msglen = 6;
    memcpy(tmpmsg, (char[]) { 0x64, 0x00, 0x01, 0x00, 0x01, 0x00 }, msglen);
    msglen = makeUBXCFG(0x06, 0x08, msglen, &tmpmsg[0]);
    write(rs232, tmpmsg, msglen);

    usleep(30000); //delay(30);

    // Set navigation settings.
    msglen = 36;
    memcpy(tmpmsg, (char[]) { 0x05, // Set dyn and fixMode only.
                              0x00, // dyn.
                              0x07, // Airborne with >2g Acceleration.
                              0x02, // 3D only.
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // pad
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);
    msglen = makeUBXCFG(0x06, 0x24, msglen, &tmpmsg[0]);
    write(rs232, tmpmsg, msglen);

    usleep(100000); //delay(100);

    // GNSS configuration CFG-GNSS for ublox 7 higher, p. 125 (v8)

    // NOTE: Max position rate = 5 Hz if GPS+GLONASS used.
    // Disable GLONASS to enable 10 Hz solution rate. GLONASS is not used
    // for SBAS (WAAS), so little real-world impact.
    msglen = 44;
    memcpy(tmpmsg, (char[]) { 0x00, 0x20, 0x20, 0x05,                            // cfgGnss
                              0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,    // gps
                              0x01, 0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,    // sbas
                              0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,    // beidou
                              0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01,    // qzss
                              0x06, 0x04, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01}, msglen);  // glonass
    msglen = makeUBXCFG(0x06, 0x3E, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);

	// SBAS configuration for ublox 6 and higher
//	p.Write(makeUBXCFG(0x06, 0x16, 8, []byte{0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}))
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);
    msglen = makeUBXCFG(0x06, 0x16, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);

	// Message output configuration -- disable standard NMEA messages except 1Hz GGA
	//                                             Msg   DDC   UART1 UART2 USB   I2C   Res
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }, msglen);    // GGA
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }, msglen);    // GLL
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }, msglen);    // GSA
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }, msglen);    // GSV
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }, msglen);    // RMC
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 }, msglen);    // VGT
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // GRS
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // GST
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // ZDA
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // GBS
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // DTM
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // GNS
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // ???
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, msglen);    // VLW
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);

    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF1, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00 }, msglen);    // Ublox,0
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF1, 0x03, 0x00, 0x32, 0x00, 0x32, 0x00, 0x00 }, msglen);    // Ublox,3
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);
    msglen = 8;
    memcpy(tmpmsg, (char[]) { 0xF1, 0x04, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x00 }, msglen);    // Ublox,4
    msglen = makeUBXCFG(0x06, 0x01, msglen, tmpmsg);
    write(rs232, tmpmsg, msglen);

    // Reconfigure serial port.
    tmpmsg[0] = 0x01; // portID.
    tmpmsg[1] = 0x00; // res0.
    tmpmsg[2] = 0x00; // res1.
    tmpmsg[3] = 0x00; // res1.

        // 0000 0000 0000 0010 0011 0000 0000 0000
        // UART mode. 0 stop bits, no parity, 8 data bits.
    tmpmsg[4] = 0xC0;
    tmpmsg[5] = 0x08;
    tmpmsg[6] = 0x00;
    tmpmsg[7] = 0x00;

    uint32_t bdrt = 115200;
    tmpmsg[8] = bdrt & 0xFF;
    tmpmsg[9] = (bdrt >> 8) & 0xFF;
    tmpmsg[10] = (bdrt >> 16) & 0xFF;
    tmpmsg[11] = (bdrt >> 24) & 0xFF;

        // inProtoMask. NMEA and UBX.
    tmpmsg[12] = 0x03;
    tmpmsg[13] = 0x00;

        // outProtoMask. NMEA.
    tmpmsg[14] = 0x02;
    tmpmsg[15] = 0x00;

    tmpmsg[16] = 0x00;
    tmpmsg[17] = 0x00;

    tmpmsg[18] = 0x00;
    tmpmsg[19] = 0x00;

    msglen = 20;
    msglen = makeUBXCFG(0x06, 0x00, msglen, &tmpmsg[0]);
    write(rs232, &tmpmsg[0], msglen);
    usleep(100000); //delay(100);

    serialClose(rs232);
    usleep(250000); //delay(250);

    rs232 = serialOpen("/dev/ttyAMA0", 115200);
    if (rs232 == -1)
    {
        return(-1);
    }

    serialFlush(rs232);

    return (rs232);
}

/*
   validateNMEAChecksum determines if a string is a properly formatted NMEA sentence with a valid checksum.

   If the input string is valid, output is the input stripped of the "$" token and checksum, along with a boolean 'true'
   If the input string is the incorrect format, the checksum is missing/invalid, or checksum calculation fails, an error string and
   boolean 'false' are returned

   Checksum is calculated as XOR of all bytes between "$" and "*"
*/
int16_t validateNMEAChecksum(char s[], uint16_t length)
{
    uint16_t	i;
    char	s_out[1024];
    char	s_cs[2];
    char *      c_ptr;

    //validate format. NMEA sentences start with "$" and end in "*xx" where xx is the XOR value of all bytes between
    if (s[0] != '$')
    {
        return(-1);  // missing message start marker
    }
    if (strchr(s, '*') == NULL)
    {
        return(-1);  // missing message end marker
    }

    // strip leading "$" and split at "*"
    strcpy(s_out, &s[1]);
    c_ptr = strchr(s_out, '*');
    *c_ptr = 0x00;
    c_ptr = strchr(s, '*');
    c_ptr++;
    s_cs[0] = *c_ptr;
    c_ptr++;
    s_cs[1] = *c_ptr;

    uint8_t cs;
    if ((s_cs[0] <= '9') && (s_cs[0] >= '0'))
    {
        cs = (s_cs[0] - '0') * 16;
    }
    else if ((s_cs[0] <= 'F') && (s_cs[0] >= 'A'))
    {
        cs = (s_cs[0] - 'A' + 10) * 16;
    }
    else
    {
        return(-1);  // invalid checksum character
    }
    if ((s_cs[1] <= '9') && (s_cs[1] >= '0'))
    {
        cs += s_cs[1] - '0';
    }
    else if ((s_cs[1] <= 'F') && (s_cs[1] >= 'A'))
    {
        cs += s_cs[1] - 'A' + 10;
    }
    else
    {
        return(-1);  // invalid checksum character
    }
    uint8_t cs_calc = 0;
    for (i = 0; i <= strlen(s_out); i++)
    {
        cs_calc ^= s_out[i];
    }

    if ( cs_calc != cs )
    {
        return(-1);  // checksum failure
    }

    strcpy(s, s_out);
    return(0);
}

void processNMEALine(char * l)
{
    int  i, j, k;
    char x[256][16];
    uint16_t hr, min, sec, csec;
    float num, groundspeed;

    j = 0;
    k = 0;
    for (i = 0; i < strlen(l); i++)
    {
        if (l[i] == ',')
        {
            x[j][k] = '\0';
            j++;
            k = 0;
        }
        else
        {
            x[j][k] = l[i];
            k++;
        }
    }
    x[j][k] = '\0';
    j++;


    if (!strcmp(x[0], "PUBX"))
    { // UBX proprietary message
        if (!strcmp(x[1], "00"))
        { // position message
	    if (j < 20)
            {
                fprintf(stderr, "PUBX type 00 message fields less than 20, %d fields parsed\r\n", j);
                return;
            }
            // field 2 = time
            if (strlen(x[2]) < 9)
            {
                fprintf(stderr, "PUBX type 00 message time field less than 9 characters, %d characters parsed\r\n", strlen(x[2]));
                return;
            }
            csec = atoi(&x[2][7]);
            x[2][6] = '\0';
            sec = atoi(&x[2][4]);
            x[2][4] = '\0';
            min = atoi(&x[2][2]);
            x[2][2] = '\0';
            hr = atoi(x[2]);
            if (hr > 23 || hr < 0 || min > 59 || min < 0 || sec > 59 || sec < 0 || csec < 0)
            {
                fprintf(stderr, "PUBX type 00 invalid time, %d hours %d min, %d sec, %d csec\r\n", hr, min, sec, csec);
                return;
            }

            pthread_mutex_lock(&mySituation.lock);
            mySituation.lastFixSinceMidnightUTC = (uint32_t) ((hr * 60 * 60) + (min * 60) + sec);
            pthread_mutex_unlock(&mySituation.lock);

            // field 3-4 = lat
            if (strlen(x[3]) < 10)
            {
                fprintf(stderr, "PUBX type 00 message lat less than 10 characters, %d characters parsed\r\n", strlen(x[3]));
                return;
            }
            num = atof(&x[3][2]);
            x[3][2] = '\0';
            hr = atoi(x[3]);
            if (hr > 89 || hr < 0 || num >= 60 || num < 0)
            {
                fprintf(stderr, "PUBX type 00 message invalid lat, %d degrees %f minutes\r\n", hr, num);
                return;
            }

            pthread_mutex_lock(&mySituation.lock);
            mySituation.Lat = (float) hr + (float) (num/60.0);
            pthread_mutex_unlock(&mySituation.lock);

            if (!strcmp(x[4], "S"))
            { // South = negative.
                mySituation.Lat = -mySituation.Lat;
            }
            // field 5-6 = lon
            if (strlen(x[5]) < 11)
            {
                fprintf(stderr, "PUBX type 00 message lon less than 11 characters, %d characters parsed\r\n", strlen(x[5]));
                return;
            }
            num = atof(&x[5][3]);
            x[5][3] = '\0';
            hr = atoi(x[5]);
            if (hr > 179 || hr < 0 || num >= 60 || num < 0)
            {
                fprintf(stderr, "PUBX type 00 message invalid lon, %d degrees %f minutes\r\n", hr, num);
                return;
            }

            pthread_mutex_lock(&mySituation.lock);
            mySituation.Lng = (float) hr + (float) (num/60.0);
            pthread_mutex_unlock(&mySituation.lock);

            if (x[6] == "W")
            { // West = negative.
                mySituation.Lng = -mySituation.Lng;
            }
            // field 7 = height above ellipsoid, m
            num = (atof(x[7]) * 3.28084) - mySituation.GeoidSep; // convert to feet and offset by geoid separation;

            pthread_mutex_lock(&mySituation.lock);
            mySituation.Alt = num;
            pthread_mutex_unlock(&mySituation.lock);

            // field 8 = nav status
            // DR = dead reckoning, G2= 2D GPS, G3 = 3D GPS, D2= 2D diff, D3 = 3D diff, RK = GPS+DR, TT = time only
            if ((!strcmp(x[8], "D2")) || (!strcmp(x[8], "D3")))
            {
                pthread_mutex_lock(&mySituation.lock);
                mySituation.quality = 2;
                pthread_mutex_unlock(&mySituation.lock);
            }
            else if ((!strcmp(x[8], "G2")) || (!strcmp(x[8], "G3")))
            {
                pthread_mutex_lock(&mySituation.lock);
                mySituation.quality = 1;
                pthread_mutex_unlock(&mySituation.lock);
            }
            else if ((!strcmp(x[8], "DR")) || (!strcmp(x[8], "RK")))
            {
                pthread_mutex_lock(&mySituation.lock);
                mySituation.quality = 6;
                pthread_mutex_unlock(&mySituation.lock);
            }
            else if ((!strcmp(x[8], "NF")))
            {
                pthread_mutex_lock(&mySituation.lock);
                mySituation.quality = 0;
                pthread_mutex_unlock(&mySituation.lock);
                return;  // return false if no valid fix.
            }
            else
            {
                pthread_mutex_lock(&mySituation.lock);
                mySituation.quality = 0;
                pthread_mutex_unlock(&mySituation.lock);
            }

            // field 9 = horizontal accuracy, m
            num = atof(x[9]);

            pthread_mutex_lock(&mySituation.lock);
            mySituation.Accuracy = num; // UBX reports 1-sigma variation
            pthread_mutex_unlock(&mySituation.lock);

            // field 10 = vertical accuracy, m
            num = atof(x[10]);

            pthread_mutex_lock(&mySituation.lock);
            mySituation.AccuracyVert = num; // UBX reports 1-sigma variation
            pthread_mutex_unlock(&mySituation.lock);

            // field 11 = groundspeed, km/h
            groundspeed = atof(x[11]); // convert to knots

            pthread_mutex_lock(&mySituation.lock);
            mySituation.GroundSpeed = (uint16_t) (groundspeed);
            pthread_mutex_unlock(&mySituation.lock);

            // field 12 = track, deg
            hr = (uint16_t) (atof(x[12])); // true course

            pthread_mutex_lock(&mySituation.lock);
            mySituation.TrueCourse = hr;
            pthread_mutex_unlock(&mySituation.lock);

            // field 13 = vertical velocity, m/s
            num = atof(x[13]);

            pthread_mutex_lock(&mySituation.lock);
            mySituation.GPSVertVel = num * -3.28084; // convert to ft/sec and positive = up
            pthread_mutex_unlock(&mySituation.lock);

            // field 14 = age of diff corrections

            // field 18 = number of satellites
            hr = atoi(x[18]);

            pthread_mutex_lock(&mySituation.lock);
            mySituation.Satellites = hr;
            pthread_mutex_unlock(&mySituation.lock);
        }
        else if (!strcmp(x[1], "03"))
        { // satellite status message
            // field 2 = number of satellites tracked
            sec = 0; // satellites seen (signal present)
            min = atoi(x[2]);

            pthread_mutex_lock(&mySituation.lock);
            mySituation.SatellitesTracked = min;
            pthread_mutex_unlock(&mySituation.lock);

            // fields 3-8 are repeated block
            for (i = 0; i < min; i++)
            {
                if (strcmp(x[7+6*i], ""))
                {
                    sec++;
                }
            }

            pthread_mutex_lock(&mySituation.lock);
            mySituation.SatellitesSeen = sec;
            pthread_mutex_unlock(&mySituation.lock);

         // Reference for future constellation tracking
         //   for i:= 0; i < satTracked; i++
         //   {
         //       x[3+6*i] // sv number
         //       x[4+6*i] // status [ U | e | - ] for used / ephemeris / not used
         //       x[5+6*i] // azimuth, deg, 0-359
         //       x[6+6*i] // elevation, deg, 0-90
         //       x[7+6*i] // signal strength dB-Hz
         //       x[8+6*i] // lock time, sec, 0-64
         //   }
        }
        else if (!strcmp(x[1], "04"))
        { // clock message

            // field 5 is UTC week (epoch = 1980-JAN-06). If this is invalid, do not parse date / time
            hr = atoi(x[5]);
            if (hr < 1877 || hr >= 32767)
            { // unless we're in a flying Delorean, UTC dates before 2016-JAN-01 are not valid. Check underflow condition as well.
                fprintf(stderr, "PUBX type 04 message invalid clock week %d\r\n", hr);
            }
            else
            {
                // field 2 is UTC time
                if (strlen(x[2]) < 9)
                {
                    fprintf(stderr, "PUBX type 04 message time less than 9 characters, %d characters parsed\r\n", strlen(x[2]));
                }
                else
                {
                    csec = atoi(&x[2][7]);
                    x[2][6] = '\0';
                    sec = atoi(&x[2][4]);
                    x[2][4] = '\0';
                    min = atoi(&x[2][2]);
                    x[2][2] = '\0';
                    hr = atoi(x[2]);
                    if (hr > 23 || hr < 0 || min > 59 || min < 0 || sec > 59 || sec < 0 || csec < 0)
                    {
                        fprintf(stderr, "PUBX type 04 message invalid time, %d hours %d min %d sec %d csec\r\n", hr, min, sec, csec);
                    }
                    else
                    {
                        pthread_mutex_lock(&mySituation.lock);
                        mySituation.lastFixSinceMidnightUTC = (uint32_t) ((hr * 60 * 60) + (min * 60) + sec);
                        mySituation.hour = hr;
                        mySituation.minute = min;
                        mySituation.second = sec;
                        pthread_mutex_unlock(&mySituation.lock);
                    }
                }

                // field 3 is date
                if (strlen(x[3]) != 6)
                {
                     fprintf(stderr, "PUBX type 04 message less than 6 characters, %d characters parsed\r\n", strlen(x[3]));
                }
                    // Date of Fix, i.e 191115 =  19 November 2015 UTC  field 9
                sec = atoi(&x[3][4]);
                x[3][4] = '\0';
                min = atoi(&x[3][2]);
                x[3][2] = '\0';
                hr = atoi(x[3]);
                if (sec < 15 || min < 1 || min > 12 || hr < 1 || hr > 31)
                {
                    fprintf(stderr, "PUBX type 04 message invalid date, %d day %d month %d year\r\n", hr, min, sec);
                }
                else
                {
                    pthread_mutex_lock(&mySituation.lock);
                    mySituation.day = hr;
                    mySituation.month = min;
                    mySituation.year = sec + 2000;
                    pthread_mutex_unlock(&mySituation.lock);
                }
            }
        }
    }
}

/*
*********************************************************************************************************
*	name: main
*	function:
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/

void * run_gps(void * x)
{
    uint8_t i;
    uint8_t buf[3];
    int     rs232;
    char    nmeaLine[512];
    int32_t nmeaLen=0;
    int32_t sampleLen;

    pthread_mutex_lock(&mySituation.lock);
    mySituation.running = FALSE;
    pthread_mutex_unlock(&mySituation.lock);

    rs232 = initGPSSerial();
    if (rs232 == -1)
    {
        fprintf(stderr, "Failed to initilize RS232 port\r\n");
        return(0);
    }

    pthread_mutex_lock(&mySituation.lock);
    mySituation.running = TRUE;
    pthread_mutex_unlock(&mySituation.lock);

    while(mySituation.running)
    {
        sampleLen = serialDataAvail(rs232);
	if (sampleLen < 0)
        {
            fprintf(stderr, "GPS serialDataAvail() returned error\r\n");
            pthread_mutex_lock(&mySituation.lock);
            mySituation.running = FALSE;
            pthread_mutex_unlock(&mySituation.lock);
        }
        else if (sampleLen > 0)
        {
            for (i = 0; i < sampleLen; i++)
            {
                nmeaLine[nmeaLen] = serialGetchar(rs232);

                if (nmeaLine[nmeaLen] == '\n')
                {
                    nmeaLine[++nmeaLen] = 0x00;
                    if (validateNMEAChecksum(nmeaLine, nmeaLen) == 0)
                    {
                        processNMEALine(nmeaLine);
                        usleep(5000);
                    }
                    nmeaLen = 0;
                }
                else
                {
                    nmeaLen++;
                }
            }
        }
        else
        {
            usleep(5000);
        }
    }

    serialClose(rs232);
    return(0);
}

void stop_gps(void)
{
    pthread_mutex_lock(&mySituation.lock);
    mySituation.running = FALSE;
    pthread_mutex_unlock(&mySituation.lock);
}
