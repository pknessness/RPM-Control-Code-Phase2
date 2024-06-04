import serial
import time
import csv
import numpy

# Configure the serial connection
ser = serial.Serial('COM6', 115200)

# Initialize array storing accel vectors
accel_array = []

# Function to send a command to the Arduino
def send_command(command):
    ser.write(command.encode()) #command.encode converts to bits (UTF-8), ser.write requires command to be in bits and sends to serial port
    time.sleep(0.005) #Pause
    #while ser.in_waiting > 0: #how many bytes waiting to be read by arduino, prevents program from stopping execution if no data
        #response = ser.readline().decode().strip() #ser.readline takes one line of text from the serial port, .decode turns from bytes to string (UTF-8), .strip removes any white spaces from the string
        #print(f"Arduino: {response}") #Only include if variable in arduino sends data back
def wait_for_ready(ser):
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(response)
            if "READY" in response:
                break
            else:
                accel_array.append(response)
        else:
            time.sleep(0.01)  # Short delay to reduce CPU usage


with open('MOTORINPUTS_64_10000.csv', newline='') as csvfile: #create the points row by row
    read = csv.reader(csvfile) #csv.reader reads row by row
    time.sleep(5)
    for row in read:
        inputA = row[0]
        inputB = row[1]
        print(inputA + "," + inputB + ";")
        send_command(inputA + "," + inputB + ";")
        wait_for_ready(ser)
        # print(inputB)
        # send_command(inputB) #+";" to add semicolon to end of row
        # wait_for_ready(ser)
    numpy.savetxt("accel_vec.csv",accel_array,delimiter=',')