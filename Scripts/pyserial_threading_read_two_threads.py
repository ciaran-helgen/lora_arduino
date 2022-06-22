# use read() instead of readline()
import serial
import time
import threading
from sys import exit

# class for multiple serial ports
class MSerialPort:
    buffer = []
    BUF_LEN = 50
    NEW_DATA = False
    last_data_len = 0
   
    def __init__(self,port,baud):
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
                    # remove any leading newlines. This will mostly remove singular newlines as described below 
                    #data = data.strip(b'\r\n')
                    # At high message freqs sometimes the data transmits separately from the newline the newline. Fix with following logic:
                    #if not data.endswith(b'\r\n'):
                        #no newline, add it
                    #    data = data + b'\r\n'
                    # break up the buffered data on newlines into a list
                    print(data)
                    decoded_buf = data.decode('utf-8').strip('\r\n').split('\r\n')
                    print(decoded_buf)
                    for i, data in enumerate(decoded_buf):
                        decoded_buf[i] = (data, seq)
                        seq+=1
                    #decoded_buf = [(i, seq+=1) for i in decoded_buf]
                    #seq+=1
                    #print("decoded:", decoded_buf)
                    # ensure incoming data can fit inside our buffer
                    if len(decoded_buf) > self.BUF_LEN:
                         raise Exception("Cannot buffer incoming data, buffer max length (" + str(self.BUF_LEN) +") exceeded. Increase buffer length or lower message publishing rate")   

                    #print(self.buffer)
                    #print(decoded_buf)
                        
                    # is there enough space to append the incoming data within the max buffer length?
                    if len(decoded_buf) <= self.BUF_LEN - len(self.buffer):
                        #print("extending")
                        self.buffer.extend(decoded_buf)
                        self.NEW_DATA = True
                    # there isn't, pop off some old data to make room
                    elif len(decoded_buf) > self.BUF_LEN - len(self.buffer):
                        #print("shuffling")
                        del self.buffer[0:len(decoded_buf)]
                        self.buffer.extend(decoded_buf)
                        self.NEW_DATA = True
                
    def clear_buffer(self):
        self.buffer.clear()

if __name__=='__main__':
    count = 0
    mSerial=MSerialPort('/dev/ttyACM0',115200)
    x = threading.Thread(target = mSerial.read_data) # call method in new thread
    x.start()
    print("thread started")
    try:
        while True:
##            if mSerial.NEW_DATA == True:
##                print(mSerial.buffer)
##                mSerial.NEW_DATA = False
            if mSerial.NEW_DATA == True:
                #print("new data")
                mSerial.NEW_DATA = False
                count+=1
            elif count >= mSerial.BUF_LEN:
                print(mSerial.buffer)
                count=0
            #print(mSerial.buffer)
            #time.sleep(0.0001) #loop runs 10x faster than message rate but does not print duplicates

    except KeyboardInterrupt as e:
        mSerial.port_close()
        x.join()
        print("Pressed CTRL-C, exiting...")
        exit(1)
##    except:
##        mSerial.port_close()
##        x.join()
 

