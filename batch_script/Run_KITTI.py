# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


SeqIdxList =  [7, 8, 99]; # [2,99]; #
# Result_root = '/home/yipuzhao/Desktop/Full_Test/Ref/'
Result_root = '/mnt/DATA/tmp/KITTI/IROS18_Jac/Lmk_1500/MaxVol_lazier_pool1.25_auto/' 
# '/mnt/DATA/GoogleDrive/ORB_SLAM/KITTI_POSE_GF_SLAM/'
# Result_root = '/mnt/DATA/tmp/KITTI/ORB_Baseline/'
# Number_GF_List = [80, 120, 160, 200]; # [40, 60, 80, 120, 160, 200]; # [40, 60, 80, 120, 160]; # 
# Number_GF_List = [600, 800, 1000, 1200, 1500, 2000]; # [40, 60, 80];
Ratio_GF_List = [0.4, 0.6, 0.8]; 
Num_Repeating = 10 # 1 # 
FrameRate = 10 # 5 # 
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

if FrameRate < 10:
    fR = '0'+str(FrameRate)
else:
    fR = str(FrameRate)

for ri, ratio_gf in enumerate(Ratio_GF_List):
# for ri, num_gf in enumerate(Number_GF_List):

    # Experiment_prefix = 'ObsNumber_' + str(int(num_gf))
    Experiment_prefix = 'ObsRatio_' + str(int(ratio_gf*100)) + 'percent'
    # Experiment_prefix = 'Refer'

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        FirstLoop = True
        # for SequenceIdx in range(0, 12):
        for sn, SequenceIdx in enumerate(SeqIdxList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqIdx = str(SeqIdxList[sn]).zfill(2)
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqIdx + "; fR: " + fR

            # File_Setting = '../ORB_Data/KITTI_yaml/Settings_KITTI_Seq' + SeqIdx + '_lmk1000.yaml'
            # File_Setting = '../ORB_Data/KITTI_yaml/Settings_KITTI_Seq' + SeqIdx + '_lmk1200.yaml'
            File_Setting = '../ORB_Data/KITTI_yaml/Settings_KITTI_Seq' + SeqIdx + '_lmk1500.yaml'
            # File_Setting = '../ORB_Data/KITTI_yaml/Settings_KITTI_Seq' + SeqIdx + '_lmk2000.yaml'
            
            File_Vocab = '../ORB_Data/ORBvoc.txt'
            File_rosbag  = '/mnt/DATA/Datasets/Kitti_Dataset/BagFiles/Seq' + SeqIdx + '_' + '10' + '_NonLoopClosure.bag'
            File_traj = Experiment_dir + '/' + SeqIdx

            cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Ratio_GF_List[ri]) + ' /camera/image_raw ' + File_traj)
            # cmd_slam   = str('rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Number_GF_List[ri]) + ' /camera/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag # + ' -r 0.1'
            
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC
            
            # Run commands
            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill GF_ORB_SLAM', shell=True)
