"""

"""

import ft4222
from ft4222.SPI import Cpha, Cpol
from ft4222.SPIMaster import Mode, Clock, SlaveSelect
from ft4222.GPIO import Dir, Port, Output
from random import randint

class IMU:
    def __init__(self, sampleRate, taps = 4, debug = False, burstMode = False,  timestamp_clock = None,  n_batches = 1):
        """
        Initializes the SPI interface

        Parameters:
        bitSize -> number of bits used for each measurement. Current implementation uses 1 bitSize, but can be altered if needed
        
        Returns:
        bool True if initialized properly
        """
        self.debug = debug
        self.ready = False
        self.burstMode = burstMode
        self.sampleRate = sampleRate
        self.timestamp_clock = timestamp_clock
        self.startTime = self.timestamp_clock.get_current_time()
        self.timestamp = self.timestamp_clock.get_current_time()
        self.imu_init(sampleRate)
        self.filter_init(taps)
        self.set_accel_scale_factor()
        self.set_gyro_scale_factor()
        self.dr_init()
        self.n_batches = n_batches
        self.xGyro = 0
        self.yGyro = 0
        self.zGyro = 0
        self.xAccel = 0
        self.yAccel = 0
        self.zAccel = 0


    def imu_init(self, rate):
        '''
        Creates the SPI connection and the us object. The ft4222 is a USB to Serial protocol converter chip. 
        
        Parameters: None
        
        Returns: True if succesfully connected. False otherwise. In Debug Will print recognized connected FTDI devices
        '''
        
        try:
            self.spi = ft4222.openByDescription(b'FT4222 A')
            self.gpio = ft4222.openByDescription(b'FT4222 B')
        except:
            if (self.debug):
                print(f"Failed to initialize Ft4222 SPI to USB interface. Displaying {devList} recognized device(s):")
                # list devices
                devList = ft4222.createDeviceInfoList()
                for i in range(devList):
                    print(ft4222.getDeviceInfoDetail(i, False))
            return False
        try:  
            self.spi.spiMaster_Init(Mode.SINGLE, Clock.DIV_64, Cpha.CLK_TRAILING, Cpol.IDLE_HIGH, SlaveSelect.SS0 )
            self.set_dec_rate(rate)
            return True
        except Exception as e:
            if (self.debug):
                print("Failed initialization")
                print(e)
            return False

    def set_dec_rate(self, rate):
        """
        Called in the imu_init function. This function calculates the decimation factor based on the user's desired sample rate. The default sample rate on
        the ADIS16465 is 2048 samples per second. Sample rates greater than 2048 will default to 2048. 
        Decimation filters necessarily divide sample rate by integers only. As a result, this function rounds the desired sample rate to the nearest possible sample rate."""
        rate = int(2048/rate) - 1
        
        if rate < 0:
            rate = 0
        if rate > 255:
            rate = 255
        write = 0xE400 + rate
        w = write.to_bytes(2, byteorder = 'big', signed = False)
        print(f"Setting dec rate. Sending : {w.hex()}")
        self.spi.spiMaster_SingleWrite(w, True)
        write = 0xE500
        w = write.to_bytes(2, byteorder = 'big', signed = False)
        self.spi.spiMaster_SingleWrite(w, True)
    
    def filter_init(self, taps):
        '''
        Sets the number of taps for the Bartlett Window FIR filter. See data sheet for frequency response. Must be an integer from 0-7 inclusive Numbers outside that range return false
        
        Parameters:
        taps -> 3 bit integer
        
        returns: False if invalid number passed'''

        if taps < 0 or taps > 7:
            return False
        write = 0xCC00 + taps
        w = write.to_bytes(2, byteorder = 'big', signed = False)
        self.spi.spiMaster_SingleWrite(w, True)

    def dr_init(self):
        self.gpio.gpio_Init(gpio3 = Dir.INPUT)
        # self.gpio.setWakeUpInterrupt(True)
        

    def read(self):
        """
        Sets self.xGyro, self.yGyro, self.zGyro, self.xAccel, self.yAccel, self.zAccel. 
        Returns False if checksum does not verify
        Burst read occurs when DIN = 0x3E00 and CS is pulled low.
        The data comes in one continuous stream of bits each 16 bits long. The order is as follows:

        DIAG_STAT, X_GYRO_OUT, Y_GYRO_OUT, Z_GYRO_OUT, X_ACCL_OUT, Y_ACCL_OUT, Z_ACCL_OUT, TEMP_OUT, SMPL_CNTR, and checksum.
        
        DIAG_STAT is a diagnosis parameter. bits 2-7 indicate tests. A 1 in any bit indicates failure of the test. Table as follows:
        7 -> Input clock out of sync
        6 -> Flash Memory Test
        5 -> Self test diagnostic error flag
        4 -> Sensor overrange
        3 -> SPI Communication failure
        2 -> Flash update failure

        Parameters:
        None
        
        Returns:
        None

        PSEUDO CODE:
        set CS low
        read 16*10 bits
        Split into 16 parameters
        call checksum function
        call Diag check
        convert to decimal, convert to real values
        """
        if self.burstMode:
            readings = self.burst_read()
            if(self.checkSum(readings)):

                rawXGyro = readings[2:4]
                rawYGyro = readings[4:6]
                rawZGyro = readings[6:8]
                rawXAccel = readings[8:10]
                rawYAccel = readings[10:12]
                rawZAccel = readings[12:14]
                
        
            else:
                print(f"Bad checksum. Prepare to crash")
        else:

            rawXGyro = self.read_from_register(0x0600)
            rawYGyro = self.read_from_register(0x0A00)
            rawZGyro = self.read_from_register(0x0E00)
            rawXAccel = self.read_from_register(0x1200)
            rawYAccel = self.read_from_register(0x1600)
            rawZAccel = self.read_from_register(0x1A00)
        timestamp = self.timestamp_clock.get_current_time()

        xGyro = self.real_gyro(self.twos_to_dec(rawXGyro))
        yGyro = self.real_gyro(self.twos_to_dec(rawYGyro))
        zGyro = self.real_gyro(self.twos_to_dec(rawZGyro))
        xAccel = self.real_accel(self.twos_to_dec(rawXAccel))
        yAccel = self.real_accel(self.twos_to_dec(rawYAccel))
        zAccel = self.real_accel(self.twos_to_dec(rawZAccel))
        
        # self.lastSample = self.get_time()
        if self.debug:
            print(f"XGyr Reading: {self.xGyro:.4f} YGyr Reading: {self.yGyro:.4f} ZGyr Reading: {self.zGyro:.4f} XAcl Reading: {self.xAccel:.4f} YAcl Reading: {self.yAccel:.4f} ZAcl Reading: {self.zAccel:.4f}       " ,end = '\r' )
        return xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, timestamp
        
    def burst_read(self):
        word = 0x6800
        word_bytes = word.to_bytes(2, byteorder='big', signed=False)

        self.spi.spiMaster_SingleWrite(word_bytes, False)
        return self.spi.spiMaster_SingleRead(20, True)
    def read_from_register(self, reg):
        lowReg = reg - 512
        upw = reg.to_bytes(2, byteorder = 'big', signed = False)
        low = lowReg.to_bytes(2, byteorder = 'big', signed = False)

        self.spi.spiMaster_SingleWrite(upw, True)

        ups = self.spi.spiMaster_SingleReadWrite(low, True)
        los = self.spi.spiMaster_SingleRead(2, True)
        return (ups+los)

    def twos_to_dec(self, sample):
        """
        Samples come in the form of 16 or 32 bit twos complement data. Converts to equivalent decimal

        Parameters:
        byte: sample in 16 or 32 bit hex

        Returns:
        int decimal equivalent
        """
        return int.from_bytes(sample, byteorder = 'big', signed = True)

    def checkSum(self, readings):
        check = readings[-2:]
        check = int.from_bytes(check, byteorder='big')
        sum_check = 0
        for i in range(18):
            sum_check += readings[i]
        # print(check,sum_check)
        return check == sum_check
    def real_accel(self, val):
        """
        Scales the decimal value acquired in twos_to_dec to the real acceleration in units depending on the scale factor

        Parameters:
        int: decimal converted from twos_to_dec function

        Returns:
        float: real acceleration with units corresponding to the scale factor. (mm/s^2)
        """
        return val*self.accelScaleFactor
    
    def real_gyro(self, val):
        """
        Scales the decimal value acquired in twos_to_dec to the real gyro in units depending on the scale factor

        Parameters:
        int: decimal converted from twos_to_dec function

        Returns:
        float: real gyro reading with units corresponding to the scale factor. (°/s)
        """
        return val*self.gyroScaleFactor

    
    def set_accel_scale_factor(self):

        """
        Sets the acceleration scale factor according to the datasheet formula. Units are mm/s^2. Data from the unit comes in mg.
        
        Parameters: None
        
        Returns: None
        """
        g = 9.81
        self.accelScaleFactor = 0.25/(2**(16))*g/(1000)
        if self.burstMode:
            self.accelScaleFactor = 0.25/(2**(0))*g/(1000)

    def set_gyro_scale_factor(self):
        """
        Sets the gyro scale factor according to the datasheet formula. Units are °/s.
        
        Parameters: None
        
        Returns: None
        """
        self.gyroScaleFactor = 0.005/(2**(16))
        if self.burstMode:
            self.gyroScaleFactor = 0.005/(2**(0))


if __name__ == "__main__":
    import spatialnde2 as snde
    timestamp_clock=snde.measurement_clock_cpp_steady("")
    o = IMU(sampleRate=180, taps = 16, debug = True, burstMode= False, n_batches=1, timestamp_clock=timestamp_clock)
    ns = 0

    count = 0
    try:
        while(1):
            o.gpio.gpio_Wait(3, 1, 0)
            o.read()
            count += 1
            # time.sleep(0.5)
            # ns = last - o.timestamp
            # print(o.zAccel)
            # sleep(0.5)
            # print(ns/1000000)
            # print(1000000000/ns)
            # last = time_ns()
    except KeyboardInterrupt:
        pass
    # duration = -(last - time_ns())/1e9
    # sr = count/duration
    # print(round(sr))
    # o.spi.close()
    # print(o.count)