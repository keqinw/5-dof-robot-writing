import serial
import binascii,time
import sys
import numpy as np

ser = serial.Serial("/dev/ttyUSB0", 9600,timeout=2)

mode = sys.argv[1]

a ='55 55 11 03 64 00 06 BC 02 05 BC 02 04 BC 02 03 BC 02'
b ='55 55 11 03 64 00 03 06 A8 01 05 EB 01 04 0E 02 03 09 02'

id_fraction = [-4.22,-3.778,-4.22,-4.22] # input~real-angle
deviation = [-5,-7,2,18]  # unit: degree



def transfer(id,angle):
	angle = float(angle) * 180 /3.14159
	#print(angle)
	cmd = int((angle+deviation[3-id])*id_fraction[3-id]+500) 
	cmd = str(hex(cmd))[2:].zfill(4)
	return cmd

def read_txt(path):
	with open(path, "r") as f:
		lines = f.read().splitlines()
	data = []
	for line in lines:
		data.append(line.split('\t'))
	return data



if __name__ == '__main__':

	points_list = np.load('/home/mac/Desktop/draw_project/actions.npy')
	print(points_list)


	if mode == 'back':
		d=bytes.fromhex(b)
		result=ser.write(d)
		time.sleep(1)

	if mode == 'go':

		num_joint = 4
		cmd_head = '55 55'
		cmd_num_joint = ' ' + str(hex(num_joint))[2:].zfill(2)
		cmd_length = ' '+ str(hex(3*num_joint+5))[2:].zfill(2)
		cmd_time = ' 2c 01'

		
		for i,points in enumerate(points_list):
			if i == 0:
				cmd_time = 'e8 03'
				sleep_time = 1
			else:
				cmd_time = ' 2c 01'
				sleep_time = 0.4
			
			cmd_position = ''
			for i,position in enumerate(points):
				
				cmd_position += (' 0'+str(6-i))
				cmd_position += (' '+transfer(i,position)[-2:])
				cmd_position += (' '+transfer(i,position)[:2])

			cmd = cmd_head + cmd_length + ' 03' + cmd_num_joint +cmd_time + cmd_position
			#print(cmd)
			d=bytes.fromhex(cmd)
			result=ser.write(d)
			time.sleep(sleep_time)
			

	#d=bytes.fromhex(b)
	#result=ser.write(d)

	#time.sleep(1)
	#count=ser.inWaiting()
	#if count: 
	#	data= str(binascii.b2a_hex(s.read(n)))[2:-1]
	#	print(data)

	ser.close()
