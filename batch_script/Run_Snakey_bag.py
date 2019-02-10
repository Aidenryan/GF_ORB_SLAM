# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


# SeqNameList = ['2018-08-30-22-39-58'];
# 
# SeqNameList = ['2018-08-31-16-38-54']; # working
# SeqNameList = ['2018-09-06-22-27-17']; # working
# SeqNameList = ['2018-09-07-14-34-03'];
SeqNameList = ['2018-09-07-16-28-53'];
# SeqNameList = ['2018-09-07-16-49-48'];
# SeqNameList = ['2018-09-07-17-06-09'];
#
# SeqNameList = ['2018-09-08-21-34-00'];
# SeqNameList = ['2018-09-08-21-48-05'];
# SeqNameList = ['2018-09-08-22-00-36'];



Result_root = '/home/achang/ivalab/tmp/'

# Number_GF_List = [60, 80, 100, 150, 200]; # [80, 100, 120]; # 
Number_GF_List = [1000]; # [400, 600, 800, 1000, 1500, 2000]; # 


Num_Repeating = 1 # 10 # 20 #  5 # 
SleepTime = 2 # 5 # 25

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

for ri, num_gf in enumerate(Number_GF_List):
    
    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = '../ORB_Data/Snakey_yaml/snakey_params.yaml'

            # File_Vocab = '../ORB_Data/ORBvoc.txt'
            File_Vocab = '../ORB_Data/ORBvoc.bin'
            File_rosbag  = '/home/achang/ivalab/bag_files/' + SeqName + '.bag'

            File_traj = Experiment_dir + '/' + SeqName

            # cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /snakey_slam_init_img ' + File_traj)
            cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /snakey_gait_exec_img ' + File_traj)
            # cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /happauge_video ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -r 3.0' # + ' -s 6' 
            
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill GF_ORB_SLAM', shell=True)
            # subprocess.call('pkill GF_ORB_SLAM', shell=True)
