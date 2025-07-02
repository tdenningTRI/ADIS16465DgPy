from dataguzzler_python.dgpy import Module as dgpy_Module
from dataguzzler_python.context import InitCompatibleThread,FormatCurContext
import threading
import spatialnde2 as snde
import numpy as np
from ADIS16465DgPy import ADIS16465 as ADIS



class ADIS_IMU(object,metaclass=dgpy_Module):
    def __init__(self, module_name, recdb, accel_channel_path, gyro_channel_path, imu, imu_n_measurements_per_batch = 1):
        self.readings = np.zeros(7, dtype = 'd')
        self.gyro_readings = np.ascontiguousarray(self.readings[0:3]) #[x,y,z] gyro readings
        self.accel_readings = np.ascontiguousarray(self.readings[3:6]) #[x,y,z] Accel readings
        self.timestamp = 0
        self.IMU = imu
        self.module_name = module_name
        self.recdb = recdb
        self.imu_n_measurements_per_batch = imu_n_measurements_per_batch
        self.relevant_batch = np.empty((0,7))
        transact = recdb.start_transaction()
        self.accel_channel= transact.define_channel(accel_channel_path,module_name)
        self.gyro_channel = transact.define_channel(gyro_channel_path,module_name)
        transact.end_transaction()
        self.connect_thread=threading.Thread(target=self.connect, daemon=True)
        self.connect_thread.start()
        
        pass

    def connect(self):
        InitCompatibleThread(self,"_connect_thread_IMU")

        while True:
            try: 
                self.get_readings()
                pass
            except Exception as e:
                print(e)
                
                pass
           
            pass
        pass

    def get_readings(self):
        while True:
            batched_measurements = []
            for measurement in range(self.imu_n_measurements_per_batch): 
                self.IMU.gpio.gpio_Wait(3, 1, 0)
                batched_measurements.append(self.IMU.read())
            # measurements = self.IMU.update()
            self.write_new_readings(np.array(batched_measurements))


    def write_new_readings(self, measurements):
        gyro_measurements = measurements[:,:3]
        accel_measurements = measurements[:,3:-1]
        first_timestamp = measurements[0, -1]

        transact = self.recdb.start_transaction(first_timestamp)
        accel_recording = snde.create_ndarray_ref(transact,self.accel_channel, snde.SNDE_RTN_FLOAT64)
        gyro_recording = snde.create_ndarray_ref(transact,self.gyro_channel, snde.SNDE_RTN_FLOAT64)
        transact.end_transaction()

        metadata = snde.constructible_metadata()

        metadata.AddMetaDatum(snde.metadatum("timestamp",int(first_timestamp.seconds_since_epoch()*1e9)))
        accel_recording.rec.metadata = metadata
        gyro_recording.rec.metadata = metadata

        accel_recording.rec.mark_metadata_done()
        gyro_recording.rec.mark_metadata_done()

        accel_recording.allocate_storage(accel_measurements.shape)
        gyro_recording.allocate_storage(gyro_measurements.shape)
        

        accel_recording.data[...] = accel_measurements
        gyro_recording.data[...] = gyro_measurements

        try:
            accel_recording.rec.mark_data_and_metadata_ready()
            gyro_recording.rec.mark_data_and_metadata_ready()

        except Exception as e: #Was failing at line 116 because no data in the recording, but marked ready.
            print(measurements)
            raise ValueError(f"{e}")
        
