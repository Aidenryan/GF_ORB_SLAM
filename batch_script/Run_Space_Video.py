# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

Result_root = '/mnt/DATA/GoogleDrive/ORB_SLAM/RNS1_flight_GF_SLAM/'
Ratio_GF_List = [0.2, 0.4, 0.6, 0.8, 1.0, -1]; # [0.2, 0.4, 0.6, 0.8];
Num_Repeating = 5 # 10 # 
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

for iteration in range(0, Num_Repeating):

    FirstLoop = True
    
    for ri, ratio_gf in enumerate(Ratio_GF_List):

        if ri > 0:
            prevRatioGF = Ratio_GF_List[ri-1]
        else:
            prevRatioGF = -1

        Experiment_prefix = 'Test2_' + str(int(prevRatioGF*100)) + 'percent'

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        # for SequenceIdx in range(0, 12):
        print bcolors.ALERT + "====================================================================" + bcolors.ENDC
        print bcolors.ALERT + "Round: " + str(iteration + 1)

# Settings_KITTI_VisualOdom_Seq00_05.yaml
        File_Setting = '../ORB_Data/RNS1_flight.yaml'
        File_Vocab = '../ORB_Data/ORBvoc.txt'
        File_rosbag  = '/mnt/DATA/Datasets/RNS1_flight/Bag_files/RNS1_flight_10fps.bag'

        File_cerr = Experiment_dir + '/RNS1_flight_10fps_Evaluation.m'
        File_KfTraj = Experiment_dir + '/RNS1_flight_10fps_KeyFrameTrajectory.txt'
        File_AfTraj = Experiment_dir + '/RNS1_flight_10fps_AllFrameTrajectory.txt'

        # cmd_rmTraj = 'rm -f KeyFrameTrajectory.txt'
        # cmd_slam   = str('nice -20 rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + File_Pose + ' ' + str(Ratio_GF_List[ri]) + ' 2>> ' + File_cerr  + '  &')
        cmd_slam   = str('nice -20 rosrun GF_ORB_SLAM GF_ORB_SLAM ' + File_Vocab + ' ' + File_Setting + ' ' + str(Ratio_GF_List[ri]) + ' 2>> ' + File_cerr  + '  &')
        cmd_rosbag = 'rosbag play ' + File_rosbag + ' -r 0.5'

        cmd_cpKfTraj = 'mv -v /home/yipuzhao/ros_workspace/package_dir/GF_ORB_SLAM/KeyFrameTrajectory.txt ' + File_KfTraj
        cmd_cpAfTraj = 'mv -v /home/yipuzhao/ros_workspace/package_dir/GF_ORB_SLAM/AllFrameTrajectory.txt ' + File_AfTraj

        print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
        print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC
        print bcolors.WARNING + "cmd_cpKfTraj: \n" + cmd_cpKfTraj + bcolors.ENDC
        print bcolors.WARNING + "cmd_cpAfTraj: \n" + cmd_cpAfTraj + bcolors.ENDC

        # Run commands
        # subprocess.call(cmd_rmTraj, shell=True)

        print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
        proc_slam = subprocess.Popen(cmd_slam, shell=True)

        print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
        time.sleep(SleepTime)

        if FirstLoop:
            FirstLoop = False
        else:
            print bcolors.OKGREEN + "Moving trajectory file FROM THE PREVIOUS SEQUENCE (stupid!!)" + bcolors.ENDC
            subprocess.call(cmd_cpKfTraj, shell=True)
            # subprocess.call(cmd_cpKfR, shell=True)
            # subprocess.call(cmd_cpFObs, shell=True)
            subprocess.call(cmd_cpAfTraj, shell=True)

        print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
        proc_bag = subprocess.call(cmd_rosbag, shell=True)
