# This is a simple example of buffering and logging (printing) serial data in a non-blocking way.
# The data is read continuously by the port_read_callback read_cb into a buffer called buf.
# The loop_callback main_loop runs as fast as possible while no other callbacks are executing.
# main_loop checks if the buffer is full, and whether the data has already been logged. It then
# logs (prints) the data and raises the flag to indicate that it has been logged. On subsequent loops
# of main_loop, the data is not logged again even though the buffer is full, as no new data has been received.
# When the next port_read_callback is called, the buffer is cleared, to which main_loop lowers the flag.

import pyMultiSerial as p
import time
from sys import exit

f = None

count=0
buf = []
buf_len = 5
prev_len = 0

logged = False # Tracks whether or not the buffered message has been logged (printed)

ms = p.MultiSerial() # The multiserial object

ms.baudrate = 115200  
ms.timeout = 0 # Stop waiting for a message if longer than this time passes (seconds)

# The callback (type port_read_callback) for when new data is received on any port
def read_cb(port_num, serial_obj, data):
    global buf, buf_len, prev_len
    # Check which port it is
    print("cb")
##    if port_num == "/dev/ttyACM0":
##        prev_len = len(buf)
##        buf.append(data.strip('\r\n'))
##        if len(buf) > buf_len:
##            buf.pop(0)
##    #print("callback")
    return

# the main loop callback (type loop_callback). This executes continuously and yields to other callbacks. It runs
# at a faster rate than the port_read_callback so care must be taken to ensure the data is only logged once
##def main_loop():
##    global buf, buf_len, logged, prev_len, f
    #print(prev_len)
##    if prev_len - len(buf) < 0 and len(buf) == buf_len: # prev - curr < 0 for increasing buffer (except overflow)
##        #print(buf)
##        f.write(buf[0] + "," + buf[1] + "," + buf[2] + "," + buf[3] + "," + buf[4] + "," +'\r\n' )
##        buf.clear()

   # return

def kb_interrupt():
    print("exiting...")
    ms.Stop()
    f.close()
    exit(1)

ms.port_read_callback = read_cb # register the read_cb port_read_callback 
#ms.loop_callback = main_loop    # register the main_loop loop_callback
ms.interrupt_callback = kb_interrupt

f = open("test.csv", "a")

#ms.Start()                  # Blocking, continuous look with callbacks

