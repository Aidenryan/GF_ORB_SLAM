# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

Result_root = '/home/achang/ivalab/tmp/'

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

print bcolors.ALERT + "====================================================================" + bcolors.ENDC

File_Setting = '../ORB_Data/Snakey_yaml/snakey_params.yaml'

# File_Vocab = '../ORB_Data/ORBvoc.txt'
File_Vocab = '../ORB_Data/ORBvoc.bin'
            
File_traj = Result_root + '/dryrun'

# cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(1000) + ' /snakey_slam_init_img ' + File_traj)
cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(1000) + ' /snakey_gait_exec_img ' + File_traj)
# cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(1000) + ' /happauge_video ' + File_traj)
            
print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            
print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
# proc_slam = subprocess.Popen(cmd_slam, shell=True)
proc_slam = subprocess.call(cmd_slam, shell=True)

print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC

