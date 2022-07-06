import serial
import time
import threading
import re
from sys import exit

from datetime import datetime
import numpy as np
from statistics import median

from trilateration import trilateration

# class for multiple serial ports
class MSerialPort:    
    def __init__(self,port,baud):
            self.buffer = []
            self.BUF_LEN = 200
            self.port=serial.Serial(port,baud)
            self.port_open()
    def port_open(self):
            if not self.port.isOpen():
                    self.port.open()
    def port_close(self):
            self.port.close()
    def send_data(self,data):
            number=self.port.write(data)
            return number
    def read_data(self):
            seq = 0 # sequence counter for data
            while True:
                #print("looping port ", self.port.port)

                # check if there is data waiting to be read
                if self.port.in_waiting > 0:
                    #print("Data on port ", self.port.port)
                    # read all the waiting bytes
                    data = self.port.read(self.port.in_waiting)
                    # remove trailing newline, convert to string and split on newlines (separate buffered messages if read rate too low)
                    decoded_buf = re.split('\n', data.decode('utf-8').rstrip('\n'))
                    # add sequence number to each sample to help with data processing
                    for i, data in enumerate(decoded_buf):
                        decoded_buf[i] = (data, seq)
                        seq+=1

                    # ensure incoming data can fit inside our buffer
                    if len(decoded_buf) > self.BUF_LEN:
                         raise RuntimeError("Cannot buffer incoming data, buffer max length (" + str(self.BUF_LEN) +") exceeded. Increase buffer length or lower message publishing rate")   
                      
                    # is there enough space to append the incoming data within the max buffer length?
                    if len(decoded_buf) <= self.BUF_LEN - len(self.buffer):
                        #print("extending")
                        self.buffer.extend(decoded_buf)
                    # there isn't, pop off some old data to make room
                    elif len(decoded_buf) > self.BUF_LEN - len(self.buffer):
                        #print("shuffling")
                        del self.buffer[0:len(decoded_buf)]
                        self.buffer.extend(decoded_buf)
                    #print(self.buffer)
                
    def clear_buffer(self):
        self.buffer.clear()

    # retrieves the first n samples from the buffer and clears those indices for new data
    def read_from_buffer(self, n):
        if len(self.buffer) >= n:
            self.samples = self.buffer[0:n]
            del self.buffer[0:n]
            return self.samples
        else:
            return

class DisplacementEst:

    def __init__(self):

        self.dist0 = 0.000
        self.dist1 = 0.000
        self.dist2 = 0.000

        # coords relative to centre of drone
        self.c0 = np.array([0.00, -0.52, 0.00])
        self.c1 = np.array([0.00, 0.58, 0.00])
        self.c2 = np.array([-0.55, 0.00, 0.00])

        # x, y, z offset estimate
        self.x = 0.00
        self.y = 0.00
        self.z = 0.00

    def est_offsets(self):

        W = np.eye(3)  # Weights matrix

        # Get current distance measurements, s1, s2, s3
        S = np.array([self.dist0, self.dist1, self.dist2]) # Distance vector
        # coords of antennae relative to centre of drone
        P1 = self.c0
        P2 = self.c1
        P3 = self.c2
        #P = np.array([P1, P2, P3] ) # Reference points matrix
        P = np.column_stack([P1, P2, P3])
        
        N1, N2 = trilateration(P,S,W)

        # flattening the array is probably wasteful, but makes it easier to access the solution
        N1 = N1[1:].flatten()
        N2 = N2[1:].flatten()
        print("N1: ", N1)
        print("N2: ", N2)
        # set x and y offset
        self.x = N1[0]
        self.y = N1[1]
        # set z value to valid solution
        # Mat z will always be below drone for demo, so take negative z result
        if N1[2] < 0:
            self.z = N1[2]
        else:
            self.z = N2[2]

        #return the offset estimates
        return (self.x, self.y, self.z)

### Helpful Functions ###
# Extracts the timestamp from a single emlid message and converts it to a datetime object
def datetime_from_emlid_string(data):
    data = data[0][0].split()
    timestamp = str(data[0]) + " " + str(data[1])                                                
    return datetime.strptime(timestamp, "%Y/%m/%d %H:%M:%S.%f")

# start and end are datetime objects. Returns the time in seconds between start and end
def time_elapsed(start, end):
    return (end - start).total_seconds()

def coord_from_emlid(data):
    data = data[0][0].split()
    x = float(data[2])
    y = float(data[3])
    z = float(data[4])
    #print("x: ",x , "y: ",y, "z: ",z)
    return np.array([x, y, z])

def calc_displacement(rover, base):
    return np.linalg.norm(rover-base) #calculate displacement from initial position

if __name__=='__main__':

    dispEst = DisplacementEst() # Instance of displacement estimation object
    
    new_data_emlid = False # Flag which indicates that new values were read from the buffer
    # Arduino 1
    new_data_arduino_1 = False # Flag which indicates that new values were read from the buffer
    filter_buffer_1 = [] # Sliding window which holds the data for median filtering
    last_lp_val_1 = None  # The previous result of low pass filtering the RSSI 
    curr_lp_val_1 = None  # The current result of low pass filtering the RSSI. The Final RSSI value used to estimate the distance
    last_lp_val_1b = None  # The previous result of low pass filtering the RSSI, with other filter alpha
    curr_lp_val_1b = None
    # Arduino 2
    new_data_arduino_2 = False # Flag which indicates that new values were read from the buffer
    filter_buffer_2 = [] # Sliding window which holds the data for median filtering
    last_lp_val_2 = None  # The previous result of low pass filtering the RSSI 
    curr_lp_val_2 = None  # The current result of low pass filtering the RSSI. The Final RSSI value used to estimate the distance
    last_lp_val_2b = None 
    curr_lp_val_2b = None
    # Arduino 3
    new_data_arduino_3 = False # Flag which indicates that new values were read from the buffer
    filter_buffer_3 = [] # Sliding window which holds the data for median filtering
    last_lp_val_3 = None  # The previous result of low pass filtering the RSSI 
    curr_lp_val_3 = None  # The current result of low pass filtering the RSSI. The Final RSSI value used to estimate the distance
    last_lp_val_3b = None
    curr_lp_val_3b = None
    # Initial timestamp and current timestamp
    start_time = None
    init_emlid = None # the initial emlid data string
    
    ## Model & Filter Parameters ##
    FILTER_WIDTH = 5
    FILTER_ALPHA = 0.3
    FILTER_ALPHA_b = 0.1
    MODEL_A = -49
    MODEL_PLE = 3
    
    ###############################
    median_rssi = None
    # update /dev/ttyACMX here with current port name (check ls /dev/ttyACM* in terminal, or use Arduino IDE)
    emlidSerial=MSerialPort('/dev/ttyACM0',115200)      #instance of class for emlid
    arduinoSerial1=MSerialPort('/dev/ttyACM1',115200)    #instance of class for arduino
    arduinoSerial2=MSerialPort('/dev/ttyACM2',115200)    #instance of class for arduino
    arduinoSerial3=MSerialPort('/dev/ttyACM3',115200)    #instance of class for arduino
    # Show which ports were assigned
    print("emlid port: ", emlidSerial.port.port)
    print("arduino ports: ", arduinoSerial1.port.port, arduinoSerial2.port.port, arduinoSerial3.port.port)
    # flush out the previous buffer. port.reset_input_buffer() doesn't work (because there is no buffered data for a moment, so nothing to clear?)
    while emlidSerial.port.in_waiting > 0:
        emlidSerial.port.read(emlidSerial.port.in_waiting)
        arduinoSerial1.port.read(arduinoSerial1.port.in_waiting)
 
    thread_emlid = threading.Thread(target = emlidSerial.read_data)     # call read method in new thread
    thread_arduino_1 = threading.Thread(target = arduinoSerial1.read_data) # call read method in new thread
    thread_arduino_2 = threading.Thread(target = arduinoSerial2.read_data) # call read method in new thread
    thread_arduino_3 = threading.Thread(target = arduinoSerial3.read_data) # call read method in new thread
    
    thread_emlid.start()
    thread_arduino_1.start()
    thread_arduino_2.start()
    thread_arduino_3.start()
    print("threads started")

    f = open("logs/test.csv", "a")
    # Write the column headings to the file
    f.write("time (s)," + "dist (true)," +
            "RSSI 1," + "median 1," + "low pass 1a," + "low pass 1b," + "dist est 1," +
            "RSSI 2," + "median 2," + "low pass 2a," + "low pass 2b," + "dist est 2," +
            "RSSI 3," + "median 3," + "low pass 3a," + "low pass 3b," + "dist est 3," +
            "x (GT)," + "y (GT)," + "z (GT)," + "x," + "y," + "z," +
            "A: " + str(MODEL_A) + ",n: " + str(MODEL_PLE) + ",window len: " + str(FILTER_WIDTH) + ",alpha: " + str(FILTER_ALPHA) +'\r\n' )

    # Sample Emlid string format '2022/06/24 11:24:23.199   3898856.8398   -588516.5392   4996663.6044   5  12   5.5667   3.0063   4.9857   0.0000   0.0000   0.0000   0.00    0.0'

    # The basestation position must be added manually from the emlid rover webapp (192.168.42.1 on its AP)
    gnd_truth_base_pose = np.array([3898863.74910, -588498.20740, 4996649.5812])
    try:
        while True:
            #print(new_data_arduino_1, new_data_arduino_2, new_data_arduino_3)
            if new_data_emlid == False:
                vals_eml = emlidSerial.read_from_buffer(1) #read one sample from buffer
                if vals_eml is not None:
                    print("emlid: ", vals_eml)
                    new_data_emlid = True

            if new_data_arduino_1 == False:
                vals_ard_1 = arduinoSerial1.read_from_buffer(1)
                if vals_ard_1 is not None:
                    print("Arduino 1: ", vals_ard_1)
                    new_data_arduino_1 = True

            if new_data_arduino_2 == False:
                vals_ard_2 = arduinoSerial2.read_from_buffer(1)
                if vals_ard_2 is not None:
                    print("Arduino 2: ", vals_ard_2)
                    new_data_arduino_2 = True

            if new_data_arduino_3 == False:
                vals_ard_3 = arduinoSerial3.read_from_buffer(1)
                if vals_ard_3 is not None:
                    print("Arduino 3: ", vals_ard_3)
                    new_data_arduino_3 = True
                
            if new_data_emlid == True and new_data_arduino_1 == True and new_data_arduino_2 == True and new_data_arduino_3 == True:
                #print("filter buffer: ", filter_buffer_1)
                #print("emlid: ", vals_eml)
                #print("arduino: ", vals_ard_1)
                #print("Time Elapsed: ", datetime_from_emlid_string(vals_eml))
                # Buffer the arduino data
                #print("Emlid Buffer", emlidSerial.buffer)
                filter_buffer_1.append(vals_ard_1)
                filter_buffer_2.append(vals_ard_2)
                filter_buffer_3.append(vals_ard_3)
                
                if len(filter_buffer_1) == FILTER_WIDTH:
                    #print("buffer: ", filter_buffer_1)
                    ### Emlid Data ###
                    elapsed_time = time_elapsed(start_time, datetime_from_emlid_string(vals_eml)) #get the time elapsed since the initial timestamp
                    gnd_truth_rel_pose = coord_from_emlid(vals_eml) - gnd_truth_base_pose
                    gnd_truth_dist = calc_displacement(coord_from_emlid(vals_eml), gnd_truth_base_pose)
                    #print("Displacement: ", gnd_truth_dist)
                    #print("Relative Pose: ", gnd_truth_rel_pose)
                    ########
                    
                    ### Arduino Data #####
                    median_rssi_1 = int(median([x[0][0] for x in filter_buffer_1]))
                    median_rssi_2 = int(median([x[0][0] for x in filter_buffer_2]))
                    median_rssi_3 = int(median([x[0][0] for x in filter_buffer_3]))
                    #############
                    
                    ### Filtering ###
                    if last_lp_val_1 is not None:
                        curr_lp_val_1 = FILTER_ALPHA*median_rssi_1 + (1-FILTER_ALPHA)*last_lp_val_1
                        curr_lp_val_1b = FILTER_ALPHA_b*median_rssi_1 + (1-FILTER_ALPHA_b)*last_lp_val_1b
                    else:
                        curr_lp_val_1 = FILTER_ALPHA*median_rssi_1 + (1-FILTER_ALPHA)*median_rssi_1
                        curr_lp_val_1b = FILTER_ALPHA_b*median_rssi_1 + (1-FILTER_ALPHA_b)*median_rssi_1
                    last_lp_val_1 = curr_lp_val_1
                    last_lp_val_1b = curr_lp_val_1b

                    if last_lp_val_2 is not None:
                        curr_lp_val_2 = FILTER_ALPHA*median_rssi_2 + (1-FILTER_ALPHA)*last_lp_val_2
                        curr_lp_val_2b = FILTER_ALPHA_b*median_rssi_2 + (1-FILTER_ALPHA_b)*last_lp_val_2b
                    else:
                        curr_lp_val_2 = FILTER_ALPHA*median_rssi_2 + (1-FILTER_ALPHA)*median_rssi_2
                        curr_lp_val_2b = FILTER_ALPHA_b*median_rssi_2 + (1-FILTER_ALPHA_b)*median_rssi_2
                    last_lp_val_2 = curr_lp_val_2
                    last_lp_val_2b = curr_lp_val_2b

                    if last_lp_val_3 is not None:
                        curr_lp_val_3 = FILTER_ALPHA*median_rssi_3 + (1-FILTER_ALPHA)*last_lp_val_3
                        curr_lp_val_3b = FILTER_ALPHA_b*median_rssi_3 + (1-FILTER_ALPHA_b)*last_lp_val_3b
                    else:
                        curr_lp_val_3 = FILTER_ALPHA*median_rssi_3 + (1-FILTER_ALPHA)*median_rssi_3
                        curr_lp_val_3b = FILTER_ALPHA_b*median_rssi_3 + (1-FILTER_ALPHA_b)*median_rssi_3
                    last_lp_val_3 = curr_lp_val_3
                    last_lp_val_3b = curr_lp_val_3b
                    
                    ##############
                    
                    ### Model ###
                    dist_est_1 = 10**((MODEL_A-curr_lp_val_1)/(10*MODEL_PLE))
                    dist_est_2 = 10**((MODEL_A-curr_lp_val_2)/(10*MODEL_PLE))
                    dist_est_3 = 10**((MODEL_A-curr_lp_val_3)/(10*MODEL_PLE))
                    ##########################
                    dispEst.dist0 = dist_est_1
                    dispEst.dist1 = dist_est_2
                    dispEst.dist2 = dist_est_3
                    dispEst.est_offsets()
                    
                    ### Logging/File I/O #####
                    #print("median: ", median_rssi)
                    f.write(str(elapsed_time) + "," + str(gnd_truth_dist) + "," +
                            str(filter_buffer_1[0][0][0]) + "," +str(median_rssi_1) + "," + str(curr_lp_val_1) + "," + str(curr_lp_val_1b) + "," + str(dist_est_1) + "," +
                            str(filter_buffer_2[0][0][0]) + "," +str(median_rssi_2) + "," + str(curr_lp_val_2) + "," + str(curr_lp_val_2b) + "," + str(dist_est_2) + "," +
                            str(filter_buffer_3[0][0][0]) + "," +str(median_rssi_3) + "," + str(curr_lp_val_3) + "," + str(curr_lp_val_2b) + "," + str(dist_est_3) + "," +
                            str(gnd_truth_rel_pose[0]) + "," + str(gnd_truth_rel_pose[1]) + "," + str(gnd_truth_rel_pose[2])+ "," + str(dispEst.x) + "," + str(dispEst.y) + "," + str(dispEst.z)
                            + '\r\n')

                    # Remove oldest data to make room for next data
                    filter_buffer_1.pop(0) 
                    filter_buffer_2.pop(0)
                    filter_buffer_3.pop(0)
                    
                elif len(filter_buffer_1) < FILTER_WIDTH:
                    # Take time of last initial median sample as the initial time
                    start_time = datetime_from_emlid_string(vals_eml)
                    init_emlid = vals_eml

                # New data has been processed, lower the flags
                new_data_emlid = False
                new_data_arduino_1 = False
                new_data_arduino_2 = False
                new_data_arduino_3 = False
                
            #time.sleep(0.01) #a delay is not required. the loop should run as fast as possible so as to not miss data, but the buffer tolerates some delay
    #close ports, threads and exit on ctrl+c
    except KeyboardInterrupt as e:
        emlidSerial.port_close()
        arduinoSerial1.port_close()
        arduinoSerial2.port_close()
        arduinoSerial3.port_close()
        f.close()
        thread_emlid.join()
        thread_arduino_1.join()
        thread_arduino_2.join()
        thread_arduino_3.join()
        print("Pressed CTRL-C, exiting...")
        exit(1)
    #Some error occurred (probably the data could not be buffered); close ports, threads, print the error and exit
    except RuntimeError as e:
        emlidSerial.port_close()
        arduinoSerial1.port_close()
        arduinoSerial2.port_close()
        arduinoSerial3.port_close()
        thread_emlid.join()
        thread_arduino_1.join()
        thread_arduino_2.join()
        thread_arduino_3.join()
        print(e)
        exit(1)
 
