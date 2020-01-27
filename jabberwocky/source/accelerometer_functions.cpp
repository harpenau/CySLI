#include <MPU6050.h> 
#include <Wire.h>

//Objects
MPU6050 mpu;

extern double correction;

//MPU 6050 Accelerations
int16_t accX, accY, accZ;   // unfiltered accelerations (ft/s^2)

void GetAcc(){
   mpu.getAcceleration(&accX, &accY, &accZ);
   accX = map(accX, 0, 4096, 0, 32);
   accY = map(accY, 0, 4096, 0, 32)-correction; //if y is pointing up (or usb port) subtract 31
   accZ = map(accZ, 0, 4096, 0, 32); //pelican?
}

void MpuSetup(){
    //joins I2C bus (I2Cdev library doesn't do this automatically) (Was found in available rocket code on internet) 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    mpu.setFullScaleAccelRange(3);  //0=2g, 1=4g, 2=8g, 3=16g //pelicannnn
    mpu.setFullScaleGyroRange(2); //0=250 deg/s, 1=500 deg/s, 2=1000 deg/s, 3=2000 deg/s 
//    calibrateMPU();                                    // <------------------------does this exist?
}
