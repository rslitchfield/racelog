#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "I2CWrapper.h"
#include "mpu6050.h"
#include "hmc5983.h"

float GyroxOffset, GyroyOffset, GyrozOffset;

MPUData myMPUData;

MPUData * mpu_data(void)
{
    return(&myMPUData);
}

BMP180Data myBMPData;

BMP180Data * bmp_data(void)
{
    return(&myBMPData);
}

void Get_Accel_Values(int handle, GForceStruct * GData)
{
    char cval[14];

    I2CWrapperReadBlock(handle, MPU6050_RA_ACCEL_XOUT_H, 14, cval);
    GData->Gx = ((cval[0] << 8) | cval[1]);
    GData->Gy = ((cval[2] << 8) | cval[3]);
    GData->Gz = ((cval[4] << 8) | cval[5]);
    GData->Temperature = ((cval[6] << 8) | cval[7]);
    GData->Gyrox = ((cval[8] << 8) | cval[9]);
    GData->Gyroy = ((cval[10] << 8) | cval[11]);
    GData->Gyroz = ((cval[12] << 8) | cval[13]);
}

int MPU6050_Test_I2C(int handle)
{
    unsigned char Data = 0x00;
    Data = I2CWrapperReadByte(handle, MPU6050_RA_WHO_AM_I);

    if (Data == 0x68)
    {
        return 1;
    }

    fprintf(stderr, "ERROR: MPU6050 Read Test Failed, returned=%X Stopping\r\n",Data);

    return 0;
}

int HMC5983_Test_I2C(int handle)
{
    int i;
    unsigned char Data;

    for (i=0; i<3; i++)
    {
        Data = I2CWrapperReadByte(handle, HMC5983_RA_ID_A + (char) i);
        if (Data != HMC5983_id[i])
        {
            fprintf(stderr, "ERROR: HMC5983 I2C Read Test Failed, returned %2X for ID A Register\r\n", Data);
            return -1;
        }
        usleep(10000);
    }
    return 0;
}


void Setup_MPU6050(int handle)
{
    //Reset MPU6050
    I2CWrapperWriteByte(handle, MPU6050_RA_PWR_MGMT_1, 0b10000000);
    usleep(1000000);

    //Sets clock source to y-axis gyro reference w/ PLL and exit SLEEP mode
    I2CWrapperWriteByte(handle, MPU6050_RA_PWR_MGMT_1, 0b00000000);
    usleep(1000000);

    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    I2CWrapperWriteByte(handle, MPU6050_RA_PWR_MGMT_2, 0b00000000);

    //Sets acceleration scale to 16G
    I2CWrapperWriteByte(handle, MPU6050_RA_ACCEL_CONFIG, 0b00011000);

    //Sets gyroscope scale to 500 deg/sec
    I2CWrapperWriteByte(handle, MPU6050_RA_GYRO_CONFIG, 0b00001000);

//    //Reset I2C master
//    I2CWrapperWriteByte(handle, MPU6050_RA_USER_CTRL, 0b00000010);
//    usleep(10000);

//    //Set I2C Bypass Mode to BYPASS ENABLED
//    I2CWrapperWriteByte(handle, MPU6050_RA_INT_PIN_CFG, MPU6050_BYPASS_MODE);

/*
    //Change slave to HMC5983
    I2CWrapperSlaveAddress(handle, HMC5983_ADDRESS);

    if (HMC5983_Test_I2C(handle) < 0)
    {
//        printf("HMC5983 connection invalid\r\n");
    }
    else
    {
//        printf("HMC5983 connection valid\r\n");

        //Set HMC5983 Mode to continuous
        I2CWrapperWriteByte(handle, HMC5983_RA_MODE, 0x00);
//        printf("Continuous mode set\r\n");
        usleep(5000);

        //Set HMC5983 Configuration Register A to temp componsated, 75Hz sample rate
        I2CWrapperWriteByte(handle, HMC5983_RA_CONFIG_A, 0x98);
        usleep(5000);

        //Set HMC5983 Configuration Register B to Gain = 1 (default = 1)
        I2CWrapperWriteByte(handle, HMC5983_RA_CONFIG_B, 0x20);
        usleep(5000);
    }
    //Set slave address back to MPU6050
    I2CWrapperSlaveAddress(handle, MPU6050_ADDRESS);
*/
}

unsigned char GotInt_MPU6050(int handle)
{
    unsigned char uc_temp;

    // Do we have a new data?
    uc_temp = I2CWrapperReadByte(handle, MPU6050_RA_INT_STATUS);

    return (uc_temp & 1) == 1 ? 1 : 0;
}

int init_i2c(void)
{
    int handle;
    const BUS = 1;
    int loop;
    int CountSum;
    long  Xsum, Ysum, Zsum;
    GForceStruct Data;

    // Aquire handle to I2C driver
    handle = I2CWrapperOpen(BUS, MPU6050_ADDRESS);
    if (handle <0)
    {
        return -1;
    }

    // Check for valid MPU6050
    if (!MPU6050_Test_I2C(handle))
    {
        return -2;
    }

    // Configure MPU6050
    Setup_MPU6050(handle);

    // Calibrate gyroscope
    Xsum=Ysum=Zsum=CountSum=0;
    for(loop=0;loop<200;loop++)
    {
        while(!GotInt_MPU6050(handle));
        Get_Accel_Values(handle, &Data);
        Xsum += Data.Gyrox;
        Ysum += Data.Gyroy;
        Zsum += Data.Gyroz;
        CountSum++;
    }
    GyroxOffset = Xsum / CountSum;
    GyroyOffset = Ysum / CountSum;
    GyrozOffset = Zsum / CountSum;

   calibrate_BMP180(handle);

    return handle;
}

short swapBytes(short w)
{
    return(((w >> 8) & 0x00FF) | (w << 8));
}

void calibrate_BMP180(int handle)
{
    if (myBMPData.calibrated)
    {
        return;
    }

    //Change slave to BMP180
    I2CWrapperSlaveAddress(handle, BMP180_ADDRESS);

    // Note: these members of myBMPData are only used by this thread, so don't need to lock the mutex
    myBMPData.ac1 = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_AC1)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calAc1);
    myBMPData.ac2 = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_AC2)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calAc2);
    myBMPData.ac3 = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_AC3)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calAc3);
    myBMPData.ac4 = (unsigned short) swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_AC4)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calAc4);
    myBMPData.ac5 = (unsigned short) swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_AC5)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calAc5);
    myBMPData.ac6 = (unsigned short) swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_AC6)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calAc6);
    myBMPData.b1 = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_B1)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calB1);
    myBMPData.b2 = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_B2)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calB2);
    myBMPData.mb = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_MB)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calMB);
    myBMPData.mc = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_MC)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calMC);
    myBMPData.md = swapBytes(I2CWrapperReadWord(handle, BMP180_RA_CAL_MD)); //wiringPiI2CReadReg16(g_tBMP180.Bus, calMD);

    myBMPData.oss = 0b11;

    //Set slave address back to MPU6050
    I2CWrapperSlaveAddress(handle, MPU6050_ADDRESS);

    myBMPData.calibrated = TRUE;

    return;
}

unsigned short readUncompensatedTemp(int handle)
{
    unsigned short temp;

    //Change slave to BMP180
    I2CWrapperSlaveAddress(handle, BMP180_ADDRESS);

    if (I2CWrapperWriteByte(handle, BMP180_RA_CONTROL, BMP180_READ_TEMP_CMD) != 1) //wiringPiI2CWriteReg8(control, readTempCmd))
    {
        //Set slave address back to MPU6050
        I2CWrapperSlaveAddress(handle, MPU6050_ADDRESS);

        return 0;
    }
    usleep(5000);

    temp = (unsigned short) swapBytes(I2CWrapperReadWord(handle, BMP180_RA_TEMP_DATA));

    //Set slave address back to MPU6050
    I2CWrapperSlaveAddress(handle, MPU6050_ADDRESS);

    return(temp); //wiringPiI2CReadReg16(g_tBMP180.Bus, tempData));
}

short calcTemp(unsigned short utemp)
{
    long x1, x2;

    x1 = (((long) utemp - (long) myBMPData.ac6) * (long)myBMPData.ac5) >> 15;
    x2 = ((long)(myBMPData.mc << 11) / (x1 + myBMPData.md));

    myBMPData.b5 = x1 + x2;

    return ((myBMPData.b5 + 8) >> 4);
}

void measureTemperature(int handle)
{
    int temperature;

    if (!myBMPData.calibrated)
    {
        calibrate_BMP180(handle);
        return;
    }
    temperature = (float) calcTemp(readUncompensatedTemp(handle)) / 10;

    pthread_mutex_lock(&myBMPData.lock);
    myBMPData.temperature = temperature;
    pthread_mutex_unlock(&myBMPData.lock);
    return;
}

unsigned long readUncompensatedPressure(int handle)
{
    unsigned char data[3];

    //Change slave to BMP180
    I2CWrapperSlaveAddress(handle, BMP180_ADDRESS);

    if (I2CWrapperWriteByte(handle, BMP180_RA_CONTROL, BMP180_READ_PRESSURE_CMD | (myBMPData.oss << 6)) != 1) // wiringPiI2CWriteReg8(g_tBMP180.Bus, control, readPressureCmd + (g_tBMP180.oss << 6)))
    {
        fprintf(stderr, "Unable to send BMP180_READ_PRESSURE_CMD\r\n");

        //Set slave address back to MPU6050
        I2CWrapperSlaveAddress(handle, MPU6050_ADDRESS);

        return 0;
    }

    usleep( ( 2 + ( 3 << myBMPData.oss) ) * 1000 );

    data[0] = I2CWrapperReadByte(handle, BMP180_RA_PRESSURE_DATA); //wiringPiI2CReadReg8(g_tBMP180.Bus, pressureData);
    data[1] = I2CWrapperReadByte(handle, BMP180_RA_PRESSURE_DATA); //wiringPiI2CReadReg8(g_tBMP180.Bus, pressureData);
    data[2] = I2CWrapperReadByte(handle, BMP180_RA_PRESSURE_DATA); //wiringPiI2CReadReg8(g_tBMP180.Bus, pressureData);

    //Set slave address back to MPU6050
    I2CWrapperSlaveAddress(handle, MPU6050_ADDRESS);

//    printf("\r\n%d\r\n",(((unsigned long)(data[0]) << 16) | ((unsigned long)(data[1]) << 8) | (unsigned long)(data[2])) >> (8 - myBMPData.oss));

    return( (((unsigned long)(data[0]) << 16) | ((unsigned long)(data[1]) << 8) | (unsigned long)(data[2])) >> (8 - myBMPData.oss) );
}

long calcPressure(unsigned long upressure)
{
    long  b3, b6, x1, x2, x3, p;
    unsigned long b4, b7;

    b6 = myBMPData.b5 - 4000;

    // Calculate b3
    x1 = ((long)(myBMPData.b2) * (long)(b6*b6) >> 12) >> 11;
    x2 = ((long)(myBMPData.ac2) * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((long)(myBMPData.ac1) * 4 + x3) << myBMPData.oss) + 2) >> 2;

    // Calculate b4
    x1 = ((long)(myBMPData.ac3) * b6) >> 13;
    x2 = ((long)(myBMPData.b1) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = myBMPData.ac4 * (unsigned long)(x3 + 32768) >> 15;

    b7 = ((unsigned long)(upressure - b3) * (50000 >> myBMPData.oss));
    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return(p);
}

float calcAltitude(long pressure)
{
    return( 44330 * (1 - pow((double)(pressure)/p0, 0.190295)) );
}

void measurePressureAndAltitude(int handle)
{
    long pressure;
    float altitude;

    if (!myBMPData.calibrated)
    {
        calibrate_BMP180(handle);
        pthread_mutex_lock(&myBMPData.lock);
        myBMPData.pressure = 0;
        myBMPData.altitude = 0;
        pthread_mutex_unlock(&myBMPData.lock);
        return;
    }

    pressure = calcPressure(readUncompensatedPressure(handle));
    altitude = calcAltitude(myBMPData.pressure);

    pthread_mutex_lock(&myBMPData.lock);
    myBMPData.pressure = pressure;
    myBMPData.altitude = altitude;
    pthread_mutex_unlock(&myBMPData.lock);
    return;
}

void * run_i2c1(void * x)
{
    int handle;
    float Gtotal;
    float GSumSquare;
    const float AccFactor = 16.0 /32768.0;
    const float GyroFactor = 500.0 / 32768.0;
    GForceStruct Data;

    myMPUData.running = FALSE;

    handle = init_i2c();
    if (handle < 0)
    {
        return;
    }

    myMPUData.running = TRUE;

    while(myMPUData.running)
    {
        if (GotInt_MPU6050(handle))
        {
            Get_Accel_Values(handle, &Data);
            GSumSquare = ((float) Data.Gx) * Data.Gx;
            GSumSquare += ((float) Data.Gy) * Data.Gy;
            GSumSquare += ((float) Data.Gz) * Data.Gz;
            Gtotal = sqrt(GSumSquare);

            pthread_mutex_lock(&myMPUData.lock);
            myMPUData.Gx = AccFactor * Data.Gx;
            myMPUData.Gy = AccFactor * Data.Gy;
            myMPUData.Gz = AccFactor * Data.Gz;
            myMPUData.Gtotal = AccFactor * Gtotal;
            myMPUData.Temperature = (float)  Data.Temperature / 340.0 + 36.53;
            myMPUData.Gyrox = GyroFactor * (Data.Gyrox - GyroxOffset);
            myMPUData.Gyroy = GyroFactor * (Data.Gyroy - GyroyOffset);
            myMPUData.Gyroz = GyroFactor * (Data.Gyroz - GyrozOffset);
            pthread_mutex_unlock(&myMPUData.lock);

            measureTemperature(handle);
            measurePressureAndAltitude(handle);
        }
        usleep(1000);
    }

    close(handle);
    return;
}

void stop_i2c1(void)
{
    myMPUData.running = FALSE;
}

