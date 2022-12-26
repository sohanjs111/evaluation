#resets rtabmap now new data gets published from the algorithm
import os
from time import sleep
import subprocess
import sys
full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
def removefile(name):
       try:
		os.remove(name)
		print(name+" removed!")
       except:
		print (name+" was not there")
removefile(path+"/rtabmap.txt")
while not os.path.exists(path+"/rtabmap.txt"):
    sleep(1)

while(1):
	time = os.path.getmtime(path+"/rtabmap.txt")
	sleep(2)
	time2=os.path.getmtime(path+"/rtabmap.txt")
	if time == time2:
		print("Unchanged")
		subprocess.Popen('''rosservice call /rtabmap/reset_odom''',
		    shell=True,  executable='/bin/bash')
	else:
		print("Changed")

