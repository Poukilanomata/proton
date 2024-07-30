# On fait un petit effort pour le placement du gyro sinon on va douiller: 
# Axes : z vers haut (yaw); x vers devant (roll), y (pitch)

import mpu6050
import time
from mpu9250_jmdev.registers import *
import numpy as np
from mpu9250_jmdev.mpu_9250 import MPU9250

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,  # In case the MPU9250 is connected to another I2C device
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)
from tqdm import tqdm



class MPUreader:
    def __init__(self, mpu) -> None:
        # Configure the MPU9250
        self.mpu = mpu
        self.mpu.configure()

        self.offsets = [[0, 0, 0], [0, 0, 0]] # accel - ypr
        self.raw_data = [[], [], 0] # accel - ypr - temp
        self.off_data = [[], [], 0]

    def calibrate_accel(self, num_samples = 100):
        print("Calibrating accelerometer...")
        print("Please put the wing horizontaly (and don't move big brain).")
        

        accel_data = []
        for i in tqdm(range(num_samples)):
            accel_data.append(self.mpu.readAccelerometerMaster())
            time.sleep(0.01)

        
        accel_offset = np.mean(accel_data)
        print("Done !")
        self.offsets[0] = accel_offset
        
        return accel_offset

    def calibrate_gyro(self, num_samples = 100):
        print("Calibrating gyroscope...")
        print("Please keep the sensor stationary during calibration.")
        
        gyro_data = []
        for _ in tqdm(range(num_samples)):
            gyro_data.append(self.mpu.readGyroscopeMaster())
            time.sleep(0.01)

        gyro_data = np.array(gyro_data)
        gx_offset, gy_offset, gz_offset = np.mean(gyro_data, axis=0)
        
        print("Calibration complete.")
        print("Gyroscope offsets: gx_offset={}, gy_offset={}, gz_offset={}".format(gx_offset, gy_offset, gz_offset))
        
        self.offsets[1] = [gx_offset, gy_offset, gz_offset]
        return gx_offset, gy_offset, gz_offset
    
    def calibration(self, num_samples = 100):
    
        accel_off, gyro_off = self.calibrate_accel(num_samples), self.calibrate_gyro(num_samples)

        return accel_off, gyro_off

    def read_data(self):
        # Read the accelerometer values
        accelerometer_data = mpu6050.get_accel_data()

        # Read the gyroscope values
        gyroscope_data = mpu6050.get_gyro_data()

        # Read temp
        temperature = mpu6050.get_temp()

        self.raw_data = [accelerometer_data, gyroscope_data, temperature]
        self.off_data = [accelerometer_data - self.offsets[0], gyroscope_data - self.offsets[1], temperature]
        return self.raw_data, self.off_data

