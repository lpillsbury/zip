# practice with subprocess
import struct
import sys
import subprocess

def main():
    print(sys.stdin.encoding)
    print(sys.stdout.encoding)
    pilot = subprocess.Popen(["python3", "prac2.py"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    val = bytearray(2)
    #val = bin(2)
    print(type(val))
    print("size of my packet: ", len(val))
    pilot.stdin.write(val)

    print("wrote to the subprocess")
    myval = pilot.stdout.read()
    print("the value that returned is: ")
    pilot.stdin.close()
    pilot.stdout.close()
    '''
    for i in range (0,5):
        #pilot.stdin.flush()
        # is it correct that this goes to stdin?
        pilot.stdin.write(bytearray(i))
        pilot.stdin.flush()
        #pilot.wait()
    pilot.stdin.close()
    pilot.stdout.close()
    '''

if __name__ == "__main__":
    main()
