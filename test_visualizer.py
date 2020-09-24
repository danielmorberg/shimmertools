#testing AHRS with the Shimmer3

from shimmerdevice import *
from ahrs_visualizer import *

sample_rate = 51.2 #What the device is configured for
test = Shimmer3device('COM5')   #Serial port
test.update_calibration_matrices()
test.start_streaming()
test_visualizer = AHRS_Visualizer(sample_rate)
test_visualizer.start()
for x in range(int(20*sample_rate)): #20 seconds
    test_pkt = test.get_data(22)    # 1 for packet flag + 3 for timestamp + (2*3)*3 bytes for the 9 16-bit IMU data values
    if len(test_pkt) < 22:
        print("something is wrong")
    imu_data_matrices = test.calibrate_data_IMU(test_pkt, test.USE_WIDERANGE_ACCEL)
    test_visualizer.update_orientation(imu_data_matrices)

test.stop_streaming()
test.S.close()
print("done")
