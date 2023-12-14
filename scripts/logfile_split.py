#!/usr/bin/python3

import os, sys
import readline


text = []
step = -1
filename = sys.argv[1]
print(filename)
with open(filename,'r') as f:
    while True:
        line = f.readline()
        if not line:
            break
        # print(line)
        if '0 Step ' in line:
            a = line.split('0 Step ')
            text.append(a[0])
            new_file_name = filename+"_"+str(step) + ".txt"
            f_w = open(new_file_name, 'w')
            f_w.writelines(text)
            #print text to some filename_step
            text = []
            text.append(a[1])
            step = step+1
        else:
            text.append(line)
 
# readline.set_completer_delims(' \t\n=')
# readline.parse_and_bind('tab: complete')

# # load file
# if(len(sys.argv) < 2):
#     file_directory = input('Please enter the full path directory to log file including extension: ')
# else:
#     file_directory = sys.argv[1]

# file_directory = os.path.expanduser(file_directory)

# # open file, exit if failed
# try:
#     file_data = open(str(file_directory), "r")
# except:
#     print("Unable to open file, please check if the file exists or it is permitted to read.")
#     exit()

# # Read until end and convert each value into numbers