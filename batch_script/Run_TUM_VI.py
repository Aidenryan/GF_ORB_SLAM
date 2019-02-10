# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


SeqNameList = ['room1_512_16', 'not_exist'];
# SeqNameList = ['room1_512_16', 'room2_512_16', 'room3_512_16', 'room4_512_16', 'room5_512_16', 'room6_512_16', 'not_exist'];
# Result_root = '/mnt/DATA/tmp/TUM_VI/MaxVol/'
Result_root = '/mnt/DATA/tmp/TUM_VI/ORB_Baseline/'

# Number_GF_List = [100]; # [30, 60, 80, 100, 120, 160]; 
Number_GF_List = [1000]; # [300, 400, 600, 800, 1000, 1500, 2000];
Num_Repeating = 10 # 3 # 5 # 
SleepTime = 25

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

# for ri, ratio_gf in enumerate(Ratio_GF_List):
for ri, num_gf in enumerate(Number_GF_List):

    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))
    # Experiment_prefix = 'ObsRatio_' + str(int(ratio_gf*100)) + 'percent'
    # Experiment_prefix = 'Refer'

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        FirstLoop = True
        # for SequenceIdx in range(0, 12):
        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            # SeqName = 'dataset-' + SeqNameList[sn]
            SeqName = SeqNameList[sn] + '_cam0'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            # File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk300.yaml'
            # File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk400.yaml'
            # File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk600.yaml'
            # File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk800.yaml'
            File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk1000.yaml'
            # File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk1500.yaml'
            # File_Setting = '../ORB_Data/TUM_VI_yaml/TUM_VI_lmk2000.yaml'

            File_Vocab = '../ORB_Data/ORBvoc.txt'
            File_rosbag  = '/mnt/DATA/Datasets/TUM_VI/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName

            cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /camera/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag # + ' -r 0.5'
            
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
