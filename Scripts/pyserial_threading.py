import serial
import time
import threading
from sys import exit

# class for multiple serial ports
class MSerialPort:
    buffer = [[]]
    BUF_LEN = 5
    NEW_DATA = False
    def __init__(self,port,baud):
            self.port=serial.Serial(port,baud)
##            if not self.port.isOpen():
##                    self.port.open()
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
            while True:
                data = self.port.readline()
                self.buffer.append(data.decode('utf-8').strip('\r\n').split())
                if len(self.buffer) > self.BUF_LEN:
                    self.buffer.pop(0)
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
                mSerial.NEW_DATA = False
                count+=1
            elif count >= mSerial.BUF_LEN:
                print(mSerial.buffer)
                count=0
            time.sleep(0.001) #loop runs 10x faster than message rate but does not print duplicates

    except KeyboardInterrupt as e:
        mSerial.port_close()
        x.join()
        print("Pressed CTRL-C, exiting...")
        exit(1)
##    except:
##        mSerial.port_close()
##        x.join()
 
