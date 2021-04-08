# there are some useful directions here:
# https://lyceum-allotments.github.io/2017/03/python-and-pipes-part-5-subprocesses-and-pipes/

import struct
import sys
import subprocess
#import prac1

def main():
    # I'm reading 2 bytes
    print("running the second program")
    # should this be stdin or stdout?
    value = sys.stdin.buffer.raw.read(2) # how many bytes to read
    # send something back to my main program
    #value = bytearray(value)
    print(type(value))
    sys.stdout.write(value)
    #sys.stdout.close()
    #sys.stdin.close()


if __name__ == "__main__":
    main()
