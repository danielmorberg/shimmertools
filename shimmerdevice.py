#Helper class for Shimmer3 AHRS
#Requires a calibrated device configured for Wide-range accelerometer, gyroscope and magnetometer.

import numpy as np
import struct
import time
import serial
import math

def s8bit_to_int(s8bit):	#Signed 8bit to int conversion
	return((((s8bit >> 7) * 128) ^ s8bit) - ((s8bit >> 7) * 128))

def twos_complement(value, bitWidth):
	newData = int(value)
	if (value > (1 << (bitWidth - 1 ))):
		newData = -((value ^(int)(pow(2,bitWidth) - 1)) + 1)
	return newData;

class Shimmer3device:
	S = ""
	name = ""   #Just a QoL thing.
	USE_WIDERANGE_ACCEL = 0
	USE_LOWNOISE_ACCEL = 1

	#Default calibration matrices(excl. Low-noise accelerometer)
	aRx = np.array([[ 815, 0, 0 ], [ 0, 815, 0 ], [ 0, 0, 815 ]] )
	gRx = np.array([[ 65.5, 0, 0 ], [ 0, 65.5, 0 ], [ 0, 0, 65.5 ]] )
	mRx = np.array([[ 1100, 0, 0 ], [ 0, 1100, 0 ], [ 0, 0, 980 ]] )

	aKx = np.array([[ -1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, -1 ]])
	gKx = np.array([[ 0, -1, 0 ], [ -1, 0, 0 ], [ 0, 0, -1 ]])
	mKx = np.array([[ -1, 0, 0 ], [ 0, 1, 0 ], [ 0, 0, -1 ]])

	abx = np.array([ 0,  0 ,  0 ])
	gbx = np.array([ 0,  0 ,  0 ])
	mbx = np.array([ 0,  0 ,  0 ])

	def __init__(self, s):
	        self.S = serial.Serial(s)

	def parse_rx(self, packet, offset):
	        rx = np.array([[s8bit_to_int(packet[12+offset])/100,s8bit_to_int(packet[13+offset])/100,s8bit_to_int(packet[14+offset])/100],
	                   [s8bit_to_int(packet[15+offset])/100,s8bit_to_int(packet[16+offset])/100,s8bit_to_int(packet[17+offset])/100],
	                   [s8bit_to_int(packet[18+offset])/100,s8bit_to_int(packet[19+offset])/100,s8bit_to_int(packet[20+offset])/100]])
	        return rx


	def parse_bx(self, packet, offset):
	        bx = np.array([[int.from_bytes([packet[0+offset], packet[1+offset]], byteorder='big', signed=True)],
	                        [int.from_bytes([packet[2+offset], packet[3+offset]], byteorder='big', signed=True)],
	                        [int.from_bytes([packet[4+offset], packet[5+offset]], byteorder='big', signed=True)]])
	        return bx


	def parse_kx(self, packet, offset):
	        kx = np.array([[int.from_bytes([packet[6+offset], packet[7+offset]], byteorder='big', signed=True)],
	                        [int.from_bytes([packet[8+offset],  packet[9+offset]], byteorder='big', signed=True)],
	                        [int.from_bytes([packet[10+offset], packet[11+offset]],  byteorder='big', signed=True)]])
	        Kx = np.zeros((3,3),float)
	        np.fill_diagonal(Kx, kx)
	        return np.asmatrix(Kx)


	def update_calibration_matrices(self):
			self.S.write(struct.pack('B', 0x2C))	#Send get_calibration ('0x2C')
			time.sleep(0.8)
			self.S.read(1)	#Read ACK response
			self.S.read(1)	#Read Calibration Response flag

	        #84 byte array with 21 bytes for each sensor.
		    #0-20 Low-noise accelerometer
		    #21-41 Gyroscope
		    #42-62 Magnetometer
		    #63-83 Wide-range accelerometer
			cal_rsp = self.S.read(84)
			tmp_list = list()
			tmp_list.extend(cal_rsp)
			if len(tmp_list) < 84:  #PySerial is non-blocking and a 0.8 second wait should be enough to return a full response, but just in case.
			    self.S.read(84 - len(tmp_list))
			    tmp__list.extend(tmp)

			cal_rsp = tmp_list
			#Wide-range accelerometer
			self.abx = self.parse_bx(cal_rsp, 63)
			self.aKx = self.parse_kx(cal_rsp, 63)
			self.aRx = self.parse_rx(cal_rsp, 63)

			#Gyroscope
			self.gbx = self.parse_bx(cal_rsp, 21)
			self.gKx = self.parse_kx(cal_rsp, 21)*0.01  #For gyro only.
			self.gRx = self.parse_rx(cal_rsp, 21)

			#Magnetometer
			self.mbx = self.parse_bx(cal_rsp, 42)
			self.mKx = self.parse_kx(cal_rsp, 42)
			self.mRx = self.parse_rx(cal_rsp, 42)


	def calibrate_data_IMU(self, packet, use_accel):
		if use_accel is self.USE_LOWNOISE_ACCEL:
			print(" Only supporting wide-range accelerometer (for now)")
		else:
			acc_x_raw = twos_complement(int(int(packet[10] & 0xFF) + (int(packet[11] & 0xFF) << 8 )), 16)
			acc_y_raw = twos_complement(int(int(packet[12] & 0xFF) + (int(packet[13] & 0xFF) << 8 )), 16)
			acc_z_raw = twos_complement(int(int(packet[14] & 0xFF) + (int(packet[15] & 0xFF) << 8 )), 16)

		gyro_x_raw = twos_complement(int(int(packet[5] & 0xFF) + (int(packet[4] & 0xFF) << 8 )), 16)
		gyro_y_raw = twos_complement(int(int(packet[7] & 0xFF) + (int(packet[6] & 0xFF) << 8 )), 16)
		gyro_z_raw = twos_complement(int(int(packet[9] & 0xFF) + (int(packet[8] & 0xFF) << 8 )), 16)

		mag_x_raw = twos_complement(int(int(packet[17] & 0xFF) + (int(packet[16] & 0xFF) << 8 )), 16)
		mag_z_raw = twos_complement(int(int(packet[19] & 0xFF) + (int(packet[18] & 0xFF) << 8 )), 16) #Z and Y switched places (for some reason). This is correct according to Consensys
		mag_y_raw = twos_complement(int(int(packet[21] & 0xFF) + (int(packet[20] & 0xFF) << 8 )), 16)

		#Using the calibration formula from Censensys:
		aux = np.array([[acc_x_raw], [acc_y_raw], [acc_z_raw]])
		gux = np.array([[gyro_x_raw], [gyro_y_raw], [gyro_z_raw]])
		mux = np.array([[mag_x_raw], [mag_y_raw], [mag_z_raw]])

		calibrated_accel = np.dot( np.dot(np.linalg.inv(self.aRx), np.linalg.inv(self.aKx)), aux - self.abx )
		#print(calibrated_accel)
		calibrated_gyro = np.dot( np.dot(np.linalg.inv(self.gRx), np.linalg.inv(self.gKx)), gux - self.gbx )
		calibrated_mag = np.dot( np.dot(np.linalg.inv(self.mRx), np.linalg.inv(self.mKx)), mux - self.mbx )
		return [calibrated_accel, calibrated_gyro, calibrated_mag]



	def get_data(self, num_bytes):
			packet = list()
			read = self.S.read(num_bytes)
			packet.extend(read)
			while len(packet) < num_bytes:
				packet.extend(self.S.read(num_bytes - len(packet)))
				print(len(packet))
			return packet


	def start_streaming(self):
	        self.S.write(struct.pack('B', 0x07))	#Send start command ('0x07')
	        time.sleep(0.5)
	        self.S.read(1)	#Read ACK response

	def stop_streaming(self):
	        self.S.write(struct.pack('B', 0x20))	#Send start command ('0x20')
	        time.sleep(0.5)
	        self.S.read(1)	#Read ACK response
