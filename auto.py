import os
import time
import sys
import subprocess

def open_program(path_name):
    return subprocess.Popen(path_name)

def kill():
    os.system("TASKKILL /F /IM CarlaUE4.exe")
    time.sleep(10)
    os.system("TASKKILL /F /IM pygame window.exe")

def close_program(p):
    p.terminate()

def runningLoop():
    while(True):
        p = open_program("CarlaUE4")
        subprocess.check_call(['python','slave.py'], stdout=sys.stdout, stderr=subprocess.STDOUT)
        time.sleep(20)
        close_program(p)
        #kill() 

def main():
    runningLoop()