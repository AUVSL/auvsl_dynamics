import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

def plot_ld3():
    df = pd.read_csv("/home/justin/xout.csv")
    plt.plot(df['x'], df['y'], color='b')
    
    df = pd.read_csv("/home/justin/Downloads/LD3/localization_ground_truth/0001_LD_grass_GT.txt", header=None)
    #end_time = df[0][0] + 6
    plt.plot(df[1], df[2], color='r')
    
    plt.xlabel('m')
    plt.ylabel('m')
    plt.legend(['3D Model', 'Odometry'])
    #plt.title('Simulation vs Dataset Odometry')
    
    plt.show()


def plot_trajectory_34_55():
    model_pos = pd.read_csv("example_cv3_34.csv")
    bekker_pos = pd.read_csv("bekker_cv3_34.csv")
    gt_pos = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0034_CV_grass_GT.txt", header=None)
    
    end_time = gt_pos[0][0] + 12
    
    plt.subplot(1,2,1)
    plt.title("Trajectory 34 from CV3 Dataset")
    plt.plot(gt_pos[1][gt_pos[0] < end_time], gt_pos[2][gt_pos[0] < end_time])

    plt.plot(bekker_pos['x'][10000:15900], bekker_pos['y'][10000:15900], color='tab:green')
    plt.plot(bekker_pos['x'][15900:], bekker_pos['y'][15900:], color='tab:green')
    
    plt.plot(model_pos['x'][10000:15900], model_pos['y'][10000:15900], color='tab:orange')
    plt.plot(model_pos['x'][15900:], model_pos['y'][15900:], color='tab:orange')
    
    plt.xlabel("m")
    plt.ylabel("m")
    
    model_pos = pd.read_csv("example_cv3_55.csv")
    bekker_pos = pd.read_csv("bekker_cv3_55.csv")
    gt_pos = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0055_CV_grass_GT.txt", header=None)
    
    end_time = gt_pos[0][0] + 12
    
    plt.subplot(1,2,2)
    plt.title("Trajectory 55 from CV3 Dataset")
    line1, = plt.plot(gt_pos[1][gt_pos[0] < end_time], gt_pos[2][gt_pos[0] < end_time])

    line2, = plt.plot(bekker_pos['x'][10000:15950], bekker_pos['y'][10000:15950], color='tab:green')
    plt.plot(bekker_pos['x'][15950:], bekker_pos['y'][15950:], color='tab:green')
    
    line3, = plt.plot(model_pos['x'][10000:15950], model_pos['y'][10000:15950], color='tab:orange')
    plt.plot(model_pos['x'][15950:], model_pos['y'][15950:], color='tab:orange')
    
    plt.xlabel("m")
    plt.ylabel("m")
    plt.legend([line1, line2, line3], ["Ground Truth","Bekker Model", "Neural Network Approximation"])
    plt.tight_layout()
    plt.show()    


def plot_bumpy_circles():
    df = pd.read_csv("/home/justin/code/AUVSL_ROS/bumpy_circles.csv")
    plt.subplot(1,2,1)
    plt.title("Position")
    plt.plot(df['x'], df['y'], color='b')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    
    x_vals = np.linspace(-3, 3, 100)
    y_vals = np.maximum(-.5*np.cos(x_vals), np.zeros(100))
    
    plt.subplot(1,2,2)
    plt.title("Altitude")
    plt.plot(x_vals, y_vals, color='tab:orange')
    plt.plot(df['x'], df['z'], color='b')
    plt.legend(["Real Altitude","Vehicle Altitude"])
    plt.xlabel('x (m)')
    plt.ylabel('z (m)')    
    plt.show()


#plot_bumpy_circles()
#plot_trajectory_34_55()
