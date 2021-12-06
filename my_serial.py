import serial
import binascii,time
import sys

ser = serial.Serial("/dev/ttyUSB0", 9600,timeout=2)

mode = sys.argv[1]

a ='55 55 11 03 04 E8 03 06 BC 02 05 BC 02 04 BC 02 03 BC 02'
b ='55 55 11 03 04 E8 03 06 A8 01 05 EB 01 04 0E 02 03 85 03'

id_fraction = [-4.22,-3.778,-4.22,-4.22] # input~real-angle
deviation = [-5,-7,2,18]  # unit: degree



def transfer(id,angle):
	cmd = int((int(angle)+deviation[int(id)-3])*id_fraction[int(id)-3]+500) 
	cmd = str(hex(cmd))[2:].zfill(4)
	return cmd


if __name__ == '__main__':


	if mode == 'back':
		d=bytes.fromhex(b)

	if mode == 'single':

		id_raw = int(sys.argv[2])

		id = str(hex(id_raw))[2:].zfill(4)
		position = transfer(sys.argv[2],sys.argv[3])
		
		cmd_head = '55 55 08 03 01 E8 03' 
		cmd = cmd_head + ' ' + id[-2:]  + ' ' + position[-2:] + ' ' + position[:2]
		print(cmd)
		d=bytes.fromhex(cmd)

	if mode == 'multi':
		id_list = list(sys.argv[2].split(','))
		position_list = list(sys.argv[3].split(','))

		num_joint = len(id_list)
		cmd_head = '55 55'
		cmd_num_joint = ' ' + str(hex(num_joint))[2:].zfill(2)
		cmd_length = ' '+ str(hex(3*num_joint+5))[2:].zfill(2)
		cmd_time = ' e8 03'

		cmd_position = ''
		for id,position in zip(id_list,position_list):
			cmd_position += (' 0'+id)
			cmd_position += (' '+transfer(id,position)[-2:])
			cmd_position += (' '+transfer(id,position)[:2])




		cmd = cmd_head + cmd_length + ' 03' + cmd_num_joint +cmd_time + cmd_position
		print(cmd)
		d=bytes.fromhex(cmd)

		

	result=ser.write(d)

	#d=bytes.fromhex(b)
	#result=ser.write(d)

	#time.sleep(1)
	count=ser.inWaiting()
	if count: 
		data= str(binascii.b2a_hex(s.read(n)))[2:-1]
		print(data)

	ser.close()
