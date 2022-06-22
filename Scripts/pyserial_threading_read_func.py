# use read() instead of readline()
import serial
import time
import threading
import re
from sys import exit

# class for multiple serial ports
class MSerialPort:    
    def __init__(self,port,baud):
            self.buffer = []
            self.BUF_LEN = 0
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
                # check if there is data waiting to be read
                if self.port.in_waiting > 0:
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

if __name__=='__main__':
    count = 0
    # update /dev/ttyACMX here with current port name (check ls /dev/ttyACM* in terminal, or use Arduino IDE)
    emlidSerial=MSerialPort('/dev/ttyACM0',115200)      #instance of class for emlid
    arduinoSerial=MSerialPort('/dev/ttyACM1',115200)    #instance of class for arduino
    # Show which ports were assigned
    print("emlid port: ", emlidSerial.port.port)
    print("arduino port: ", arduinoSerial.port.port)
    # flush out the previous buffer. port.reset_input_buffer() doesn't work (because there is no buffered data for a moment, so nothing to clear?)
    while emlidSerial.port.in_waiting > 0:
        emlidSerial.port.read(emlidSerial.port.in_waiting)
        arduinoSerial.port.read(arduinoSerial.port.in_waiting)
 
    thread_emlid = threading.Thread(target = emlidSerial.read_data)     # call read method in new thread
    thread_arduino = threading.Thread(target = arduinoSerial.read_data) # call read method in new thread
    
    thread_emlid.start()
    thread_arduino.start()
    print("threads started")
    try:
        while True:
            vals_eml = emlidSerial.read_from_buffer(2)
            vals_ard = arduinoSerial.read_from_buffer(2)
            if vals_eml is not None:
                print("emlid: ", vals_eml)

            
            if vals_ard is not None:
                print("arduino: ", vals_ard)
                
            time.sleep(0.2) #a delay is not required. the loop should run as fast as possible so as to not miss data, but the buffer tolerates some delay
    #close ports, threads and exit on ctrl+c
    except KeyboardInterrupt as e:
        emlidSerial.port_close()
        arduinoSerial.port_close()
        thread_emlid.join()
        thread_arduino.join()
        print("Pressed CTRL-C, exiting...")
        exit(1)
    #Some error occurred (probably the data could not be buffered); close ports, threads, print the error and exit
    except RuntimeError as e:
        emlidSerial.port_close()
        arduinoSerial.port_close()
        thread_emlid.join()
        thread_arduino.join()
        print(e)
        exit(1)
 
