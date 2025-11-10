import serial
import time
import csv
import numpy

# Configure the serial connection
ser = serial.Serial('COM10', 115200)
f = open("demofile2.txt", "a")

motor1 = {"angle":0,"velocity":0,"torque":0,"temp":0}
motor2 = {"angle":0,"velocity":0,"torque":0,"temp":0}

# Initialize array storing accel vectors
accel_array = []

# Function to send a command to the Arduino
def send(command):
    ser.write(command.encode()) #command.encode converts to bits (UTF-8), ser.write requires command to be in bits and sends to serial port

with open('profiles/MOTORINPUTS_64_10000.csv', newline='') as csvfile: #create the points row by row
    read = csv.reader(csvfile) #csv.reader reads row by row
    time.sleep(2)
    for row in read:
        inputA = row[0]
        inputB = row[1]
        #print(inputA + "," + inputB + ";")
        send("_"+ inputA + "A_" + inputB + "B")
        print("_"+ inputA + "A_" + inputB + "B")
        time.sleep(0.3)
        
        # print(inputB)
        # send_command(inputB) #+";" to add semicolon to end of row
        # wait_for_ready(ser)
    numpy.savetxt("accel_vec.csv",accel_array,delimiter='\t')