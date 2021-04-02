import matplotlib.pyplot as plt
import pandas as pd 
import os 
import numpy as np

if __name__ == "__main__":

    # Read data placed in ../vo_poses.csv
    main_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "results")
    save_dir = os.path.join(main_dir, "plots")
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    
    vo_traj = os.path.join(main_dir, "stamped_traj_estimate.txt")
    gt_traj = os.path.join(main_dir, "stamped_groundtruth.txt")

    vo_data = np.loadtxt(vo_traj)
    gt_data = np.loadtxt(gt_traj)

    t, x_vo, y_vo, z_vo, roll_vo, pitch_vo, yaw_vo = vo_data[:,0], vo_data[:,1], vo_data[:,2], vo_data[:,3], vo_data[:,4], vo_data[:,5], vo_data[:,6]
    _, x_gt, y_gt, z_gt, roll_gt, pitch_gt, yaw_gt = gt_data[:,0], gt_data[:,1], gt_data[:,2], gt_data[:,3], gt_data[:,4], gt_data[:,5], gt_data[:,6]


    # Trajectory
    fig1,ax1 = plt.subplots(num=1)
    ax1.plot(x_vo, y_vo, '-', color='red', label='Visual odometry')
    ax1.plot(x_gt, y_gt, '-', color='darkblue', label='Ground truth')
    ax1.legend(loc='upper left', frameon=True)    
    ax1.set_ylabel('Latitude [m]')
    ax1.set_xlabel('Longitude [m]')
    ax1.set_title('Ground truth compared to visual odometry')
    fig1.savefig(os.path.join(save_dir, "trajectory.png"))


    plt.show()


