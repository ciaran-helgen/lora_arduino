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
PORT1 = "/dev/ttyACM1" #Emlid Port
PORT0 = "/dev/ttyACM3" #Arduino Port

arduino_ser = None
emlid_ser = None

#arduino_data = []
#emlid_data = []

elapsed_time = 0 #time elapsed since first message

first_msg = True # flag if first message, false on subsequent messages

init_coord = np.array((0, 0, 0)) # coordinate of first GPS message
coord = np.array((0, 0, 0)) # current position

rssi = 0

lowpass_buf = [] #buffer for the low pass filter

raw_buf = []
median_buf = []

dist = 0

flush_msgs = 0

f = None

filt_alpha = 0.1

curr_time = None
init_time = None


N_SAMP = 5 # number of samples to buffer for median filtering


model_A = -49 # parameter 'A' for the distance model
model_PLE = 3

arduino_buf = [[]]*N_SAMP #initialise list of lists for buffer
emlid_buf = [[]]*N_SAMP

def main():
    global flush_msgs
    global arduino_ser
    global emlid_ser
    global f
    global first_msg
    global init_coord
    global init_time
    global curr_time
    global filt_alpha
    global lowpass_buf
    global dist
    global rssi

    global arduino_buf
    global emlid_buf

    st = None
    last_st = None
    
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
        f.write("time (s)," + "dist (true)," +"RSSI," + "median," + "low pass," + "dist est," +
                "A: " + str(model_A) + ",n: " + str(model_PLE) + ",window len: " + str(N_SAMP) + ",alpha: " + str(filt_alpha) +'\r\n' )
    except Exception as e:
        print(e)
        exit(1)
        
    while 1:
        
        try:
            #read several lines to flush out messages received before the emlid clock updates (time jump)
            #this leads to a several seconds long delay, and it doesn't always flush enough data
            while(flush_msgs < 5):
                #arduino_data = arduino_ser.readline()
                #emlid_data = emlid_ser.readline()
                get_data()
                flush_msgs+=1
            
            # 
            if first_msg:
                #init_time = get_data()[1]
                tmp = get_data()
                #init_time = tmp[1][0] + " " + tmp[1][0]
                init_time = datetime_from_emlid(tmp[1])
                #print( get_data()[1][] )
                init_coord = np.array((3898864.895, -588498.3162, 4996652.89)) #set the basestation coordinate. This is manually added from ReachView once the basestation averaging completes
                #init_time = curr_time
                #arduino_data = [] # clear leftover data from flushing above
                #emlid_data = []
                first_msg = False
            else:
                #print(len(arduino_buf))
                #print(len(arduino_data))
                #print("data: ", arduino_data)
                #print("buf: ", arduino_buf)
                if arduino_buf[0] == []:
                    for i in range(0, N_SAMP):
                        tmp = get_data()
                        arduino_buf[i] = tmp[0]
                        emlid_buf[i] = tmp[1][0:5] #only buffer elements 0 to 5; date, time, x, y, z
                else:
                    #rotate in new sample into right side of buffer (append to end of list and remove element 0)
                    tmp = get_data()
                    arduino_buf.append(tmp[0])
                    arduino_buf.pop(0)
                    emlid_buf.append(tmp[1][0:5]) #only buffer elements 0 to 5; date, time, x, y, z
                    emlid_buf.pop(0)
                #print(arduino_buf)
                #print(emlid_buf)

                #med_dist = median(emlid_buf[::][5])
                #tmp = emlid_buf[][3]
                #tmp = [row[2] for row in emlid_buf]
                tmp = column(2, emlid_buf)
                #print(tmp)
                #print(median(column(2, emlid_buf)))
                coord = np.array([median(map(float, column(2, emlid_buf))), median(map(float, column(3, emlid_buf))), median(map(float, column(4, emlid_buf)))])
                dist = np.linalg.norm(coord-init_coord)
                #print(dist)
                print(len(emlid_buf))
                median_rssi = median(map(float, column(0, arduino_buf)))
                #print(arduino_buf)
                #print("median: ", median_rssi)
        
                if last_st != None:
                    st = filt_alpha*median_rssi + (1-filt_alpha)*last_st
                    #print("Last ST here")
                else:
                    st = filt_alpha*median_rssi + (1-filt_alpha)*median_rssi
                last_st = st


                dist_est = 10**((model_A-st)/(10*model_PLE))

                #print("Distance Est: ", dist_est)
##                if len(lowpass_buf) < 2:
##                    lowpass_buf.append(median_rssi)
##                    lowpass_buf.append(median_rssi)
##                else:
##                    lowpass_buf.append(median_rssi)
##                    lowpass_buf.pop(0)
                #print("Lowpass: ", lowpass_buf)
                #lp_rssi = filt_alpha*
                #print("Filtered: ", st, median_rssi)

                #timestamp = emlid_buf[-1][0] + " " + emlid_buf[-1][1]                                                
                #curr_time = datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
                curr_time =  datetime_from_emlid(emlid_buf)
                elapsed_time = (curr_time - init_time).total_seconds()
                print("Elapsed: ", elapsed_time)

                #print(arduino_buf[0])
                #print(arduino_buf[0][0])
                
                
                f.write(str(elapsed_time) + "," + str(dist) + "," + str(arduino_buf[0][-1]) + "," +  str(median_rssi) + "," + str(st) + "," + str(dist_est) + '\r\n')

                if elapsed_time > 40:
                    f.write("END,END,END,END,END,END"+'\r\n' )
                    f.close()
                    print("Pressed CTRL-C, exiting...")
                    exit(1)
                #print(init_coord)
                #dist = median(np.linalg.norm(emlid_buf-init_coord)) #calculate displacement from initial position
                #print(curr_time, init_time)
                #
                #print(coord)
                #print(dist)
                #print(elapsed_time)
                #print("len: ", len(filt_rssi))
            # Filter the data
##            if len(med_filt_vals) == 0:
##                tmp = median_rssi(5)
##                med_filt_vals = [tmp, tmp]
##                
##            elif len(filt_rssi) == 2:
##                #rssi =         a*x(t)         +   (1 - a)*s(t-1)  
##                lp_rssi = filt_alpha*med_filt_vals[0] + (1-filt_alpha)*med_filt_vals[1]
##                filt_rssi[0] = median_rssi(5)
##                filt_rssi[1] = lp_rssi
##                #print(filt_rssi)
##                
##                f.write(str(dist) + "," +  str(rssi) + "," + str(filt_rssi[0]) + "," + str(filt_rssi[1]) + '\r\n' )
##                #print(filt_rssi[0], ",", filt_rssi[1])

            
        # cat END to end of CSV to indicate end of test. Useful for running multiple tests!
        except KeyboardInterrupt as e:
            f.write("END,END,END,END,END,END"+'\r\n' )
            f.close()
            print("Pressed CTRL-C, exiting...")
            exit(1)
        # any other exception
        #except Exception as e:
        #    print(e)
        #    exit(1)

### return median over n samples
##def median_rssi(n_samples):
##    global rssi
##    global buf
##    for n in range(0, n_samples):
##        get_data()
##        buf.append(int(rssi))
##        
##    return median(buf)

#def low_pass_rssi(rssi):

# returns the emlid and arduino data as a tuple of lists of strings
def get_data():
    global arduino_ser, emlid_ser       # global serial objects
    
    arduino_data = arduino_ser.readline()
    emlid_data = emlid_ser.readline()
    
    if (len(emlid_data) > 0) and (len(arduino_data) > 0):

        emlid_data = emlid_data.decode('utf8').strip('\r\n').split() # get rid of carriage return and newline, and split on spaces
        
        arduino_data = [arduino_data.decode('utf-8').strip('\r\n')]

        return (arduino_data, emlid_data)
    else:
        return (None, None)

# parses the emlid data string list and returns timestamp (datetime obj), current GPS coordinates (np array)
def parse_emlid(data):
    if len(data) > 0:

        timestamp = data[0] + " " + data[1]                                                 
        curr_time = datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
        coord = np.array((float(data[2]), float(data[3]), float(data[4])))

        return (curr_time, coord)
    
    else:
        return (None, None)

def parse_arduino(data):
    if len(data) > 0:
        rssi = data[0]
        return rssi
    return None

def calc_displacement(base, rover):
    return np.linalg.norm(rover-base) #calculate displacement from initial position

def buffer_raw_data(n_samples):
    global raw_buf
    if len(buf) < n_samples:
        for s in range(0, n_samples - 1):
            buf.append(parse_arduino(get_data()[0]))

def median_filt():
    global median_buf
    global raw_buf
    if len(median_buf) == 0:
        median_buf.extend([median(raw_buf), median(raw_buf)])
    else:
        median_buf.append(median(raw_buf))
        median_buf.pop(0)

#returns the data in column i over a provided matrix (list of lists)
def column(i, matrix):
    return [row[i] for row in matrix]

def datetime_from_emlid(data):
# first check if it's a buffer or single sample
    if type(data[0]) == list:
        timestamp = data[-1][0] + " " + data[-1][1]                                                
        return datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
    else:
        timestamp = data[0] + " " + data[1]                                                
        return datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
    

# return median over data provided
##def median_rssi(data):
##    return median(data)
# returns the emlid and arduino data as a list of strings
##def get_data():
##    global f
##    global first_msg
##    global init_coord
##    global coord
##    global rssi
##    global init_time
##    global arduino_ser
##    global emlid_ser
##    global curr_time
##    global timestamp
##    arduino_data = arduino_ser.readline()
##    emlid_data = emlid_ser.readline()
##    
##    if (len(emlid_data) > 0) and (len(arduino_data) > 0):
##
##        emlid_data = emlid_data.decode('utf8').strip('\r\n').split() # get rid of carriage return and newline,
##                                                                      # and split on spaces
##        timestamp = emlid_data[0] + " " + emlid_data[1]                                                 
##        curr_time = datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")
##        coord = np.array((float(emlid_data[2]), float(emlid_data[3]), float(emlid_data[4])))
##        #print(timestamp)
##        arduino_data = arduino_data.decode('utf-8').strip('\r\n')
##        rssi = arduino_data
        #print(rssi)
        #print("RSSI: ", rssi)
        
        #f.write(timestamp + "," + str(elapsed_time) + "," + str(init_coord[0]) +  "," + str(init_coord[1]) +  ","+ str(coord[0]) +  "," + str(coord[1]) +  "," +  rssi + '\r\n' )
        
    #f.close() #close after each line is logged. This probably isn't necessary but I forgot to test. Speed bottleneck


if __name__ == "__main__":
    print('Running. Press CTRL-C to exit.')
    main()


