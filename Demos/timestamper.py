import csv
import pandas as pd
import numpy as np

filename = "saved_measurement_data_for_ukf.csv"

column_names = ["Device", "Filter_Mode", "Timestamp", "ax,x", "ay,y", "az,z", "wx,q1", "wy,q2", "wz,q3", "NaN,q4"]
dataset = pd.read_csv(filename, names = column_names, skiprows = 5)

# print(dataset)

def check_if_timestamps_increase(df):
    check = pd.DataFrame(df['Timestamp'].diff())/1e9
    non_increasing_indecies = df.index[check.iloc[:,0]<=0].tolist()
    print(non_increasing_indecies)
    
    devices_ammended = set()
    for idx in non_increasing_indecies:
        devices_ammended.add(idx)

        devices_ammended.add(idx -1)
    result = sorted(devices_ammended)
    devices = df["Device"].iloc[result]
    print(devices)
    devices.to_csv("Device_list.csv")

def check_sample_rate(df, device):
    device_readings = df[df["Device"]==device]
    check = pd.DataFrame(device_readings['Timestamp'].diff())/1e9
    print(np.mean(1/check))
def sensuron_travel_checker(df):
    sensuron_readings = df[df["Device"]=="OptiTrack"]
    running = sensuron_readings[sensuron_readings["Filter_Mode"] == "Running"]
    x_traveled = running["ax,x"].iloc[-1] - running["ax,x"].iloc[0]
    y_traveled = running["ay,y"].iloc[-1] - running["ay,y"].iloc[0]
    z_traveled = running["az,z"].iloc[-1] - running["az,z"].iloc[0]
    print(x_traveled)
    print(y_traveled)
    print(z_traveled)
    dist = np.linalg.norm(np.array([x_traveled,y_traveled,z_traveled]))
    print(f"Distance Traveled: {dist}")

def imu_travel_checker(df):
    imu_readings = df[df["Device"]=="IMU"]
    running = imu_readings[imu_readings["Filter_Mode"] == "Running"].reset_index(drop=True)
    collected = pd.DataFrame()
    collected["TimeInterval"] = running["Timestamp"].diff()/1000000000
    # print(collected["TimeInterval"])
    collected['X_Velocity'] = 0.0
    collected['X_Distance'] = 0.0
    collected['Y_Velocity'] = 0.0
    collected['Y_Distance'] = 0.0
    collected['Z_Velocity'] = 0.0
    collected['Z_Distance'] = 0.0
    # print(collected.at[1,"TimeInterval"])
    for i in range(1, len(running)):
        # print(collected.at[i,'TimeInterval'])
        collected.at[i, "X_Velocity"] = collected.at[i-1, 'X_Velocity']+ running.at[i, "ax,x"]*collected.at[i,'TimeInterval']
        collected.at[i, "Y_Velocity"] = collected.at[i-1, 'Y_Velocity']+ (running.at[i, "ay,y"] - 9.8)*collected.at[i,'TimeInterval']
        collected.at[i, "Z_Velocity"] = collected.at[i-1, 'Z_Velocity']+ running.at[i, "az,z"]*collected.at[i,'TimeInterval']

    for i in range(1, len(running)):
        # print(collected.at[i,'TimeInterval'])
        collected.at[i, "X_Distance"] = collected.at[i-1, 'X_Distance']+ collected.at[i, "X_Velocity"]*collected.at[i,'TimeInterval']
        collected.at[i, "Y_Distance"] = collected.at[i-1, 'Y_Distance']+ collected.at[i, "Y_Velocity"]*collected.at[i,'TimeInterval']
        collected.at[i, "Z_Distance"] = collected.at[i-1, 'Z_Distance']+ collected.at[i, "Z_Velocity"]*collected.at[i,'TimeInterval']
    
    print(collected["Y_Distance"])
# imu_travel_checker(dataset)
# sensuron_travel_checker(dataset)
# check_if_timestamps_increase(dataset)
check_sample_rate(dataset, 'OptiTrack')
check_sample_rate(dataset, 'IMU')