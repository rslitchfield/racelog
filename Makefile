racelog:racelog.o dataq.o gps.o mpu6050.o I2CWrapper.o 
	gcc racelog.c dataq.c gps.c mpu6050.c I2CWrapper.c -o racelog -lbcm2835 -lwiringPi -lm -lpthread -lrt
