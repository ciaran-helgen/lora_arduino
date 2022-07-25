import matplotlib.pyplot as plt

import serial
from serial.tools import list_ports

import time
import threading
import re
from sys import exit

from datetime import datetime
import numpy as np
from statistics import median

from trilateration import trilateration

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')

def live_plotter(x_vec,y1_data,line1,identifier='',pause_time=0.1):
    if line1==[]:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(13,6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        line1, = ax.plot(x_vec,y1_data,'-o',alpha=0.8)        
        #update plot label/title
        plt.ylabel('Y Label')
        plt.title('Title: {}'.format(identifier))
        plt.show()
    
    # after the figure, axis, and line are created, we only need to update the y-data
    line1.set_ydata(y1_data)
    line1.set_xdata(x_vec)
    # adjust limits if new data goes beyond bounds
    if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])

    if np.min(x_vec)<=line1.axes.get_xlim()[0] or np.max(x_vec)>=line1.axes.get_xlim()[1]:
        plt.xlim([np.min(x_vec)-np.std(x_vec),np.max(x_vec)+np.std(x_vec)])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)
    
    # return line so we can update it again in the next iteration
    return line1

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
        self.c0 = np.array([0.00, 0.52, 0.00])
        self.c1 = np.array([0.00, -0.58, 0.00])
        self.c2 = np.array([0.55, 0.00, 0.00])

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
        try:
            self.x = N1[0].real
        except AttributeError as ae:
            self.x = N1[0]
        try:
            self.y = N1[1].real
        except AttributeError as ae:
            self.y = N1[1]
            
        # set z value to valid solution
        # Mat z will always be below drone for demo, so take negative z result
        if N1[2] < 0:
            try:
                self.z = N1[2].imaginary
            except AttributeError as ae:
                self.z = N1[2]
        else:
            try:
                self.z = N2[2].imaginary
            except AttributeError as ae:
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

# Dictionary for serial device hardware serial IDs (iSerial).
# You can filter on device ID and vendor ID (which is identical for each emlid M+ and each arduino)
# by plugging in one device you want to ID and using sudo lsusb -d 'VID:PID' -v
# e.g. for the arduinos:
# sudo lsusb -d '2341:8053' -v
# and for the emlids:
# sudo lsusb -d '3032:0012' -v
# you can get the ID of the hardware if it is unknown by running lsusb
device_ids = {
    "emlid1":   "8243BBABC5EFA493", #8243BBABC5EFA493
    "emlid2":   "824369F2F224BFBF", #824369F2F224BFBF
    "emlid3":   "82432A32FAC93609", #
    "arduino1": "B17BFAD9504E4B36322E314AFF031D2E", 
    "arduino2": "5EA85888504E4B36322E314AFF041E28", #5EA85888504E4B36322E314AFF041E28
    "arduino3": "C70C9777504E4B36322E314AFF04051C"  #C70C9777504E4B36322E314AFF04051C
    }

# get assigned port for hardware
def port_name_from_serial_number(serial_number_string):
    device_list = list_ports.comports()
    for device in device_list:
        #print(device.name)
        if (device.serial_number != None):
            if (device.serial_number == serial_number_string):
                port = device.device
                break
            else:
                port = None
    return(port)

if __name__=='__main__':
    size = 100
    x_vec = np.zeros(size)
    y_vec = np.zeros(size)
    line1 = []
    dispEst = DisplacementEst() # RSSI: Instance of displacement estimation object
    dispEst_gt = DisplacementEst() # Ground Truth: Instance of displacement estimation object

    # Emlid
    new_data_emlid_1 = False # Flags which indicates that new value(s) were read from emlid the buffer
    new_data_emlid_2 = False
    new_data_emlid_3 = False

    # Arduino 1
    new_data_arduino_1 = False # Flag which indicates that new values were read from the buffer
    filter_buffer_1 = [] # Sliding window which holds the data for median filtering
    last_lp_val_1 = None  # The previous result of low pass filtering the RSSI 
    curr_lp_val_1 = None  # The current result of low pass filtering the RSSI. The Final RSSI value used to estimate the distance
    # Arduino 2
    new_data_arduino_2 = False # Flag which indicates that new values were read from the buffer
    filter_buffer_2 = [] # Sliding window which holds the data for median filtering
    last_lp_val_2 = None  # The previous result of low pass filtering the RSSI 
    curr_lp_val_2 = None  # The current result of low pass filtering the RSSI. The Final RSSI value used to estimate the distance
    # Arduino 3
    new_data_arduino_3 = False # Flag which indicates that new values were read from the buffer
    filter_buffer_3 = [] # Sliding window which holds the data for median filtering
    last_lp_val_3 = None  # The previous result of low pass filtering the RSSI 
    curr_lp_val_3 = None  # The current result of low pass filtering the RSSI. The Final RSSI value used to estimate the distance
    # Initial timestamp and current timestamp
    start_time = None
    init_emlid = None # the initial emlid data string
    
    ## Model & Filter Parameters ##
    FILTER_WIDTH = 5
    FILTER_ALPHA = 0.3
    MODEL_A = -49
    MODEL_PLE = 3

    ERROR_MARGIN = 0.01 # factor to set bounds for replacing distance estimate with ground truth (decimal factor, e.g. 0.9*ground_truth <= estimate <= 1.1*ground_truth)
    ###############################
    median_rssi = None

    # Performance Metrics #
    replacement_count = 0 # How many times the estimate was replaced by ground truth
    # update /dev/ttyACMX here with current port name (check ls /dev/ttyACM* in terminal, or use Arduino IDE)

    # Emlid serial objects
    emlidSerial1=MSerialPort(port_name_from_serial_number(device_ids['emlid1']),115200)
    emlidSerial2=MSerialPort(port_name_from_serial_number(device_ids['emlid2']),115200)
    emlidSerial3=MSerialPort(port_name_from_serial_number(device_ids['emlid3']),115200)

    # Arduino serial objects    
    arduinoSerial1=MSerialPort(port_name_from_serial_number(device_ids['arduino1']),115200)
    arduinoSerial2=MSerialPort(port_name_from_serial_number(device_ids['arduino2']),115200)
    arduinoSerial3=MSerialPort(port_name_from_serial_number(device_ids['arduino3']),115200)
    # Show which ports were assigned
    print("emlid ports: ", emlidSerial1.port.port, emlidSerial2.port.port, emlidSerial3.port.port)
    print("arduino ports: ", arduinoSerial1.port.port, arduinoSerial2.port.port, arduinoSerial3.port.port)
    
    # flush out the previous buffer. port.reset_input_buffer() doesn't work (because there is no buffered data for a moment, so nothing to clear?)
    while emlidSerial1.port.in_waiting > 0:
        emlidSerial1.port.read(emlidSerial1.port.in_waiting)
    while emlidSerial2.port.in_waiting > 0:
        emlidSerial2.port.read(emlidSerial2.port.in_waiting)
    while emlidSerial3.port.in_waiting > 0:
        emlidSerial3.port.read(emlidSerial3.port.in_waiting)

    while arduinoSerial1.port.in_waiting > 0:
        arduinoSerial1.port.read(arduinoSerial1.port.in_waiting)
    while arduinoSerial2.port.in_waiting > 0:
        arduinoSerial2.port.read(arduinoSerial2.port.in_waiting)
    while arduinoSerial3.port.in_waiting > 0:
        arduinoSerial3.port.read(arduinoSerial3.port.in_waiting)

    # Prepare threads for continuous reading of port data
    thread_emlid_1 = threading.Thread(target = emlidSerial1.read_data)     # call read method in new thread
    thread_emlid_2 = threading.Thread(target = emlidSerial2.read_data)
    thread_emlid_3 = threading.Thread(target = emlidSerial3.read_data)
    thread_arduino_1 = threading.Thread(target = arduinoSerial1.read_data)
    thread_arduino_2 = threading.Thread(target = arduinoSerial2.read_data)
    thread_arduino_3 = threading.Thread(target = arduinoSerial3.read_data)

    # Start reading from ports on separate threads
    thread_emlid_1.start()
    thread_emlid_2.start()
    thread_emlid_3.start()
    thread_arduino_1.start()
    thread_arduino_2.start()
    thread_arduino_3.start()
    print("threads started")

    f = open("logs/test.csv", "a")
    # Write the column headings to the file
    f.write("time (s)," + "RSSI 1," + "RSSI 2," + "RSSI 3," + "low pass 1," + "low pass 2," + "low pass 3," +
            "dist 1 (est)," + "dist 1 (GT)," + "dist 2 (est)," + "dist 2 (GT)," + "dist 3 (est)," + "dist 3 (GT)," +
            "x (est)," + "x (GT)," + "y (est)," + "y (GT)," + "z (est)," + "z (GT)," +
            "A: " + str(MODEL_A) + ",n: " + str(MODEL_PLE) + ",window len: " + str(FILTER_WIDTH) + ",alpha: " + str(FILTER_ALPHA) +'\r\n' )

    # Sample Emlid string format '2022/06/24 11:24:23.199   3898856.8398   -588516.5392   4996663.6044   5  12   5.5667   3.0063   4.9857   0.0000   0.0000   0.0000   0.00    0.0'

    # The basestation position must be added manually from the emlid rover webapp (192.168.42.1 on its AP)
    gnd_truth_base_pos = np.array([3903508.6880, -580878.79570, 4993945.2052])
    try:
        while True:
            #print(new_data_arduino_1, new_data_arduino_2, new_data_arduino_3)
            if new_data_emlid_1 == False:
                vals_eml_1 = emlidSerial1.read_from_buffer(1) #read one sample from buffer
                if vals_eml_1 is not None:
                    print("emlid 1: ", vals_eml_1)
                    new_data_emlid_1 = True
                    
            if new_data_emlid_2 == False:
                vals_eml_2 = emlidSerial2.read_from_buffer(1) #read one sample from buffer
                if vals_eml_2 is not None:
                    print("emlid 2: ", vals_eml_2)
                    new_data_emlid_2 = True
                    
            if new_data_emlid_3 == False:
                vals_eml_3 = emlidSerial3.read_from_buffer(1) #read one sample from buffer
                if vals_eml_3 is not None:
                    print("emlid 3: ", vals_eml_3)
                    new_data_emlid_3 = True

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
                
            if new_data_emlid_1 == True and new_data_emlid_2 == True and new_data_emlid_3 == True and new_data_arduino_1 == True and new_data_arduino_2 == True and new_data_arduino_3 == True:
                #print("filter buffer: ", filter_buffer_1)
                #print("emlid: ", vals_eml)
                #print("arduino: ", vals_ard_1)
                #print("Time Elapsed: ", datetime_from_emlid_string(vals_eml))
                # Buffer the arduino data
                #print("Emlid Buffer", gnd_truth_base_pos.buffer)
                filter_buffer_1.append(vals_ard_1)
                filter_buffer_2.append(vals_ard_2)
                filter_buffer_3.append(vals_ard_3)
                
                if len(filter_buffer_1) == FILTER_WIDTH:
                    #print("buffer: ", filter_buffer_1)
                    ### Emlid Data ###
                    elapsed_time = time_elapsed(start_time, datetime_from_emlid_string(vals_eml_1)) #get the time elapsed since the initial timestamp
##                    gnd_truth_rel_pose = coord_from_emlid(vals_eml) - gnd_truth_base_pos
##                    gnd_truth_dist = calc_displacement(coord_from_emlid(vals_eml), gnd_truth_base_pos)

                    
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
                    else:
                        curr_lp_val_1 = FILTER_ALPHA*median_rssi_1 + (1-FILTER_ALPHA)*median_rssi_1
                    last_lp_val_1 = curr_lp_val_1

                    if last_lp_val_2 is not None:
                        curr_lp_val_2 = FILTER_ALPHA*median_rssi_2 + (1-FILTER_ALPHA)*last_lp_val_2
                    else:
                        curr_lp_val_2 = FILTER_ALPHA*median_rssi_2 + (1-FILTER_ALPHA)*median_rssi_2
                    last_lp_val_2 = curr_lp_val_2

                    if last_lp_val_3 is not None:
                        curr_lp_val_3 = FILTER_ALPHA*median_rssi_3 + (1-FILTER_ALPHA)*last_lp_val_3
                    else:
                        curr_lp_val_3 = FILTER_ALPHA*median_rssi_3 + (1-FILTER_ALPHA)*median_rssi_3
                    last_lp_val_3 = curr_lp_val_3
                    
                    ### Ground Truth Distances ###
                    dist_gt_1 = calc_displacement(coord_from_emlid(vals_eml_1), gnd_truth_base_pos)
                    dist_gt_2 = calc_displacement(coord_from_emlid(vals_eml_2), gnd_truth_base_pos)
                    dist_gt_3 = calc_displacement(coord_from_emlid(vals_eml_3), gnd_truth_base_pos)
                    
                    ### Model ###
                    dist_est_1 = 10**((MODEL_A-curr_lp_val_1)/(10*MODEL_PLE))
                    dist_est_2 = 10**((MODEL_A-curr_lp_val_2)/(10*MODEL_PLE))
                    dist_est_3 = 10**((MODEL_A-curr_lp_val_3)/(10*MODEL_PLE))
                    ##########################
                    ### Trilaterate Offsets ###
                    # update distance estimates
                    if dist_gt_1 - ERROR_MARGIN*dist_gt_1 <= dist_est_1 <= dist_gt_1 + ERROR_MARGIN*dist_gt_1:    # if the estimate is within the defined margin of the ground truth
                        dispEst.dist0 = dist_est_1
                    else:                                       # Estimate differs too greatly from ground truth, use ground truth instead
                        replacement_count += 1
                        dispEst.dist0 = dist_gt_1 
                    if dist_gt_2 - ERROR_MARGIN*dist_gt_2 <= dist_est_2 <= dist_gt_2 + ERROR_MARGIN*dist_gt_2:    # if the estimate is within the defined margin of the ground truth
                        dispEst.dist1 = dist_est_2
                    else:
                        replacement_count += 1
                        dispEst.dist1 = dist_gt_2
                    if dist_gt_3 - ERROR_MARGIN*dist_gt_3 <= dist_est_3 <= dist_gt_3 + ERROR_MARGIN*dist_gt_3:    # if the estimate is within the defined margin of the ground truth
                        dispEst.dist2 = dist_est_3
                    else:
                        replacement_count += 1
                        dispEst.dist2 = dist_gt_3

                    # trilaterate offset estimate
                    dispEst.est_offsets()
                    
                    # calculate ground truth offsets only (for debug)
                    dispEst_gt.dist0 = dist_gt_1
                    dispEst_gt.dist1 = dist_gt_2
                    dispEst_gt.dist2 = dist_gt_3
                    # trilaterate offset ground truth
                    dispEst_gt.est_offsets()
                    
                    ### Logging/File I/O #####
                    #print("median: ", median_rssi)
                    f.write(str(elapsed_time) + "," +
                            str(filter_buffer_1[0][0][0]) + "," + str(filter_buffer_2[0][0][0]) + "," + str(filter_buffer_3[0][0][0]) + "," +                               # Raw RSSI values
                            str(curr_lp_val_1) + "," + str(curr_lp_val_2) + "," + str(curr_lp_val_3) + "," +                                                                # Filtered RSSI
                            str(dist_est_1) + "," + str(dist_gt_1) + "," + str(dist_est_2) + "," + str(dist_gt_2) + "," + str(dist_est_3) + "," + str(dist_gt_3) + "," +      # Distance Estimates and Ground Truth
                            str(dispEst.x) + "," + str(dispEst_gt.x) + "," + str(dispEst.y) + "," + str(dispEst_gt.y) + "," + str(dispEst.z) + "," + str(dispEst_gt.z)        # Offset Estimates and Ground Truth
                            + ",reps," + str(replacement_count) + '\r\n')

##                    f.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},reps,{}\r\n".format(
##                        elapsed_time,                                                                   # timestamp
##                                                filter_buffer_1[0][0][0], filter_buffer_2[0][0][0], filter_buffer_3[0][0][0],   # Raw RSSI values
##                        curr_lp_val_1, curr_lp_val_2,, curr_lp_val_3,                                   # Filtered RSSI values
##                        dist_est_1, dist_gt_1, dist_est_2, dist_gt_2, dist_est_3, dist_gt_3,            # Distance Estimates and Ground Truth
##                        dispEst.x, dispEst_gt.x, dispEst.y, dispEst_gt.y, dispEst.z, dispEst_gt.z,      # Offset Estimates and Ground Truth
##                        replacement_count)                                                    
                    ## Plot the data ##
                    y_vec[-1] = dispEst.y
                    
                    x_vec[-1] = dispEst.x
                    line1 = live_plotter(x_vec,y_vec,line1)
                    y_vec = np.append(y_vec[1:],0.0)
                    x_vec = np.append(x_vec[1:],0.0)
                                        
                    # Remove oldest data to make room for next data
                    filter_buffer_1.pop(0) 
                    filter_buffer_2.pop(0)
                    filter_buffer_3.pop(0)
                    
                elif len(filter_buffer_1) < FILTER_WIDTH:
                    # Take time of last initial median sample as the initial time
                    start_time = datetime_from_emlid_string(vals_eml_1) # get starting time from an emlid
                    init_emlid = vals_eml_1 # unused, initial time is only relevant data for now

                # New data has been processed, lower the flags
                new_data_emlid_1 = False
                new_data_emlid_2 = False
                new_data_emlid_3 = False
                new_data_arduino_1 = False
                new_data_arduino_2 = False
                new_data_arduino_3 = False
                
            #time.sleep(0.01) #a delay is not required. the loop should run as fast as possible so as to not miss data, but the buffer tolerates some delay
    #close ports, threads and exit on ctrl+c
    except KeyboardInterrupt as e:
        emlidSerial1.port_close()
        emlidSerial2.port_close()
        emlidSerial3.port_close()
        arduinoSerial1.port_close()
        arduinoSerial2.port_close()
        arduinoSerial3.port_close()
        thread_emlid_1.join()
        thread_emlid_2.join()
        thread_emlid_3.join()
        thread_arduino_1.join()
        thread_arduino_2.join()
        thread_arduino_3.join()
        print("Pressed CTRL-C, exiting...")
        exit(1)
    #Some error occurred (probably the data could not be buffered); close ports, threads, print the error and exit
    except RuntimeError as e:
        emlidSerial1.port_close()
        emlidSerial2.port_close()
        emlidSerial3.port_close()
        arduinoSerial1.port_close()
        arduinoSerial2.port_close()
        arduinoSerial3.port_close()
        f.close()
        thread_emlid_1.join()
        thread_emlid_2.join()
        thread_emlid_3.join()
        thread_arduino_1.join()
        thread_arduino_2.join()
        thread_arduino_3.join()
        print(e)
        exit(1)
 
