# Note: Ensure the arduino's publish rate is equal to that of the emlid
# this is a design flaw as they each block while waiting for the next line.
import serial
from sys import exit
from datetime import datetime
import numpy as np
from statistics import median
 
BAUD_EMLID = 9600 # Emlid baud
BAUD_ARDUINO = 9600 # Arduino baud

#depending on the order you plug in the devices, switch AMC1/AMC0 if an indexing error occurs
PORT1 = "/dev/ttyACM0" #Emlid Port
PORT0 = "/dev/ttyACM1" #Arduino Port

arduino_data = []
emlid_data = []

arduino_ser = None
emlid_ser = None

elapsed_time = 0 #time elapsed since first message

first_msg = True # flag if first message, false on subsequent messages

init_coord = np.array((0, 0, 0)) # coordinate of first GPS message
coord = np.array((0, 0, 0)) # current position

rssi = 0
filt_rssi = []
lp_rssi = 0

dist = 0

flush_msgs = 0

f = None

filt_alpha = 0.5

curr_time = None

timestamp = None

def main():
    global flush_msgs
    global arduino_ser
    global emlid_ser
    global f
    global first_msg
    global init_coord
    global curr_time
    global filt_alpha
    global filt_rssi
    global dist
    global rssi
    
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
        
    try:
        f = open("test.csv", "a")
        f.write("distance," + "rssi," +  "median," + "low pass" + '\r\n' )
    except Exception as e:
        print(e)
        exit(1)
    while 1:
        
        try:
            #read several lines to flush out messages received before the emlid clock updates (time jump)
            #this leads to a several seconds long delay, and it doesn't always flush enough data
            while(flush_msgs < 3):
                arduino_data = arduino_ser.readline()
                emlid_data = emlid_ser.readline()
                flush_msgs+=1
                
            # 
            if first_msg:
                get_data()
                init_coord = np.array((3898866.91144599, -588497.96461823, 4996654.0257)) #set the basestation coordinate. This is manually added from ReachView once the basestation averaging completes
                init_time = curr_time
                first_msg = False
            else:
                dist = np.linalg.norm(coord-init_coord) #calculate displacement from initial position
                #print(curr_time, init_time)
                #elapsed_time = (curr_time - init_time).total_seconds()
                #print(coord)
                #print(dist)
                #print(elapsed_time)
                #print("len: ", len(filt_rssi))
            # Filter the data
            if len(filt_rssi) == 0:
                tmp = median_rssi(5)
                filt_rssi = [tmp, tmp]
                
            elif len(filt_rssi) == 2:
                #rssi =         a*x(t)         +   (1 - a)*s(t-1)  
                lp_rssi = filt_alpha*filt_rssi[0] + (1-filt_alpha)*filt_rssi[1]
                filt_rssi[0] = median_rssi(5)
                filt_rssi[1] = lp_rssi
                #print(filt_rssi)
                
                f.write(str(dist) + "," +  str(rssi) + "," + str(filt_rssi[0]) + "," + str(filt_rssi[1]) + '\r\n' )
                #print(filt_rssi[0], ",", filt_rssi[1])

            
        # cat END to end of CSV to indicate end of test. Useful for running multiple tests!
        except KeyboardInterrupt as e:
            f.write("END,END,END,END,END,END,END"+'\r\n' )
            f.close()
            print("Pressed CTRL-C, exiting...")
            exit(1)
        # any other exception
        #except Exception as e:
        #    print(e)
        #    exit(1)

# return median over n samples
def median_rssi(n_samples):
    global rssi
    buf = []
    for n in range(0, n_samples):
        get_data()
        buf.append(int(rssi))
        
    return median(buf)

def get_data():
    global f
    global first_msg
    global init_coord
    global coord
    global rssi
    global init_time
    global arduino_ser
    global emlid_ser
    global curr_time
    global timestamp
    arduino_data = arduino_ser.readline()
    emlid_data = emlid_ser.readline()
    
    if (len(emlid_data) > 0) and (len(arduino_data) > 0):

        emlid_data = emlid_data.decode('utf8').strip('\r\n').split() # get rid of carriage return and newline,
                                                                      # and split on spaces
        timestamp = emlid_data[0] + " " + emlid_data[1]                                                 
        curr_time = datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
        coord = np.array((float(emlid_data[2]), float(emlid_data[3]), float(emlid_data[4])))
        #print(timestamp)
        arduino_data = arduino_data.decode('utf-8').strip('\r\n')
        rssi = arduino_data
        #print(rssi)
        #print("RSSI: ", rssi)
        
        #f.write(timestamp + "," + str(elapsed_time) + "," + str(init_coord[0]) +  "," + str(init_coord[1]) +  ","+ str(coord[0]) +  "," + str(coord[1]) +  "," +  rssi + '\r\n' )
        
    #f.close() #close after each line is logged. This probably isn't necessary but I forgot to test. Speed bottleneck


if __name__ == "__main__":
    print('Running. Press CTRL-C to exit.')
    main()


