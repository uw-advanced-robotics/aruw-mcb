
# CSV_1 Columns: Index,Time,((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorPrimaryImu).imuData).accG).coordinates_).data)._M_elems[0],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorPrimaryImu).imuData).accG).coordinates_).data)._M_elems[2],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorPrimaryImu).imuData).gyroOffsetRaw).coordinates_).data)._M_elems[2],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorPrimaryImu).imuData).gyroRadPerSec).coordinates_).data)._M_elems[0],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorPrimaryImu).imuData).gyroRadPerSec).coordinates_).data)._M_elems[1],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorPrimaryImu).imuData).gyroRadPerSec).coordinates_).data)._M_elems[2]

CSV_1 = "DatasampleDoubleISMWithMPU.csv"

# CSV_2 columns: Index,Time,((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorImu).imuData).accG).coordinates_).data)._M_elems[0],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorImu).imuData).accG).coordinates_).data)._M_elems[1],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorImu).imuData).accG).coordinates_).data)._M_elems[2],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorImu).imuData).gyroRadPerSec).coordinates_).data)._M_elems[0],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorImu).imuData).gyroRadPerSec).coordinates_).data)._M_elems[1],((((((aruwsrc::sentry::DriversSingleton::drivers).turretMajorImu).imuData).gyroRadPerSec).coordinates_).data)._M_elems[2]
CSV_2 = "DatasampleDoubleISM.csv"

# Get standard deviation of each column for both CSVs, and print them out in a nice format
# Then also get the standard deviation of the magnitude of the accel and the gyro individually, and print those out as well
import pandas as pd
import numpy as np

def print_stddevs(csv_file, label):
    df = pd.read_csv(csv_file)
    accel_cols = [col for col in df.columns if 'accG' in col]
    gyro_cols = [col for col in df.columns if 'gyroRadPerSec' in col]

    print(f"Standard deviations for {label}:")
    for col in accel_cols + gyro_cols:
        stddev = np.std(df[col])
        print(f"{col}: {stddev:.6f}")

    accel_magnitude = np.sqrt(np.sum(df[accel_cols]**2, axis=1))
    gyro_magnitude = np.sqrt(np.sum(df[gyro_cols]**2, axis=1))

    print(f"Accel magnitude stddev: {np.std(accel_magnitude):.6f}")
    print(f"Gyro magnitude stddev: {np.std(gyro_magnitude):.6f}")
    print()
    
print_stddevs(CSV_1, "MPU6500 IMU")
print_stddevs(CSV_2, "Turbomag IMU")