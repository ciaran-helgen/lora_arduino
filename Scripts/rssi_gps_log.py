# Note: Ensure the arduino's publish rate is equal to that of the emlid
# this is a design flaw as they each block while waiting for the next line.
import serial
from sys import exit
from datetime import datetime
import numpy as np
 
BAUD_EMLID = 9600 # Emlid baud
BAUD_ARDUINO = 9600 # Arduino baud

#depending on the order you plug in the devices, switch AMC1/AMC0 if an indexing error occurs
PORT1 = "/dev/ttyACM0" #Emlid Port
PORT0 = "/dev/ttyACM1" #Arduino Port

elapsed_time = 0 #time elapsed since first message

first_msg = True # flag if first message, false on subsequent messages

init_coord = np.array((0, 0, 0)) # coordinate of first GPS message
coord = np.array((0, 0, 0)) # current position

rssi = 0
flush_msgs = 0

def main():
    global first_msg
    global flush_msgs
    try:
        arduino_ser = serial.Serial(PORT0, BAUD_ARDUINO, timeout = 1)  # open arduino serial port
        #arduino_ser.flushInput()
    except Exception as e:
        print("Could not open port", PORT0 , "exiting...")
        exit(1)
    try:
        emlid_ser = serial.Serial(PORT1, BAUD_EMLID, timeout = 1)  # open emlid serial port
        #emlid_ser.flushInput()
    except Exception as e:
        print("Could not open port", PORT1 , "exiting...")
        exit(1)

    while 1:
        f = open("test.csv", "a")

        try:
            #read several lines to flush out messages received before the emlid clock updates (time jump)
            #this leads to a several seconds long delay, and it doesn't always flush enough data
            while(flush_msgs < 30):
                arduino_data = arduino_ser.readline()
                emlid_data = emlid_ser.readline()
                flush_msgs+=1
                
            arduino_data = arduino_ser.readline()
            emlid_data = emlid_ser.readline()
            if len(emlid_data) > 0:

                emlid_data = emlid_data.decode('utf8').strip('\r\n').split() # get rid of carriage return and newline,
                                                                              # and split on spaces
                timestamp = emlid_data[0] + " " + emlid_data[1]                                                 
                curr_time = datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
                coord = np.array((float(emlid_data[2]), float(emlid_data[3]), float(emlid_data[4])))
                print(timestamp)
                
            if len(arduino_data) > 0:
                arduino_data = arduino_data.decode('utf-8').strip('\r\n')
                rssi = arduino_data

            if first_msg:
                init_coord = np.array((3898866.91144599, -588497.96461823, 4996654.0257)) #set the basestation coordinate. This is manually added from ReachView once the basestation averaging completes
                init_time = curr_time
                first_msg = False

            else:
                dist = np.linalg.norm(coord-init_coord) #calculate displacement from initial position
                #print(curr_time, init_time)
                elapsed_time = (curr_time - init_time).total_seconds()
                #print(coord)
                #print(dist)
                print(elapsed_time)
                #print(rssi)
                #f.write(timestamp + "," + str(elapsed_time) + "," + str(init_coord[0]) +  "," + str(init_coord[1]) +  ","+ str(coord[0]) +  "," + str(coord[1]) +  "," + dist + rssi + '\r\n' )
                f.write(str(dist) + str(rssi) + '\r\n' )
     
            #f.close() #close after each line is logged. This probably isn't necessary but I forgot to test. Speed bottleneck
            
        # cat END to end of CSV to indicate end of test. Useful for running multiple tests!
        except KeyboardInterrupt as e:
            f.write("END,END,END,END,END,END,END"+'\r\n' )
            f.close()
            print("Pressed CTRL-C, exiting...")
            exit(1)
        # any other exception
##        except Exception as e:
##            f.write("END,END,END,END,END,END,END"+'\r\n' )
##            f.close()
##            print(e)
##            exit(1)


if __name__ == "__main__":
    print('Running. Press CTRL-C to exit.')
    main()


