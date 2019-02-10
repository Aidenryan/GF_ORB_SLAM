# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


# SeqNameList = ['left_cam', 'right_cam', 'not_exist'];
SeqNameList = ['left_cam'];

Result_root = '/mnt/DATA/tmp/NewCollege/ORBv1_Baseline/'

# Number_GF_List = [60, 80, 100, 130, 160, 200, 240]; # [80, 100, 120]; # 
Number_GF_List = [600, 800, 1000, 1500, 2000]; #  [1000] # 
Num_Repeating = 10 # 50 # 20 #  5 # 

SleepTime = 2 # 10 # 25

PrintLog = False

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
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            # File_Setting = '../ORB_Data/NewCollege_yaml/Bumblebee_lmk400.yaml'
            # File_Setting = '../ORB_Data/NewCollege_yaml/Bumblebee_lmk600.yaml'
            # File_Setting = '../ORB_Data/NewCollege_yaml/Bumblebee_lmk800.yaml'
            # File_Setting = '../ORB_Data/NewCollege_yaml/Bumblebee_lmk1000.yaml'
            # File_Setting = '../ORB_Data/NewCollege_yaml/Bumblebee_lmk1500.yaml'
            File_Setting = '../ORB_Data/NewCollege_yaml/Bumblebee_lmk2000.yaml'

            File_Log_details = Experiment_dir + '/' + SeqName + '_details.log'
            # File_Vocab = '../ORB_Data/ORBvoc.txt'
            File_Vocab = '../ORB_Data/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/New_College/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName
            
            # cmd_rmTraj = 'rm -f KeyFrameTrajectory.txt'
            # cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Ratio_GF_List[ri]) + ' /cam0/image_raw ' + File_traj)
            # cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /cam0/image_raw ' + File_traj)
            if(PrintLog):
                cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /cam0/image_raw ' + File_traj+ ' >> ' + File_Log_details)
            else:
                cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /cam0/image_raw ' + File_traj)

            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s 30' # + ' -q' # + ' -r 0.5'
            
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
# 