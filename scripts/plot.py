import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/xout.csv")
plt.plot(df['x'], df['y'], color='b')

df = pd.read_csv("/home/justin/xout2.csv")
plt.plot(df['x'], df['y'], color='r')

#df = pd.read_csv("/home/justin/xout_file.csv")
#plt.plot(df['x'], df['y'], color='r')

df = pd.read_csv("/home/justin/Downloads/CV3/localization_ground_truth/0055_CV_grass_GT.txt", header=None)
end_time = df[0][0] + 6
#plt.plot(df[1][df[0] < end_time], df[2][df[0] < end_time], color='g')
#plt.plot(df[1], df[2])

plt.xlabel('m')
plt.ylabel('m')
plt.legend(['3D Model', '2D Model', 'Odometry'])
plt.title('Simulation vs Dataset Odometry')

plt.show()

# plt.plot(df['qd1'])
# plt.plot(df['qd2'])
# plt.plot(df['qd3'])
# plt.plot(df['qd4'])
# plt.show()

# df = pd.read_csv("/home/justin/xout.csv")
# plt.plot(df['qd1'], color='r')
# plt.show()


# df = pd.read_csv("/home/justin/log.csv")
# plt.plot(df["zr1"], color='r', alpha=.5)
# plt.plot(df["zr2"], color='g', alpha=.5)
# plt.plot(df["zr3"], color='b', alpha=.5)
# plt.plot(df["zr4"], color='w', alpha=.5)
# plt.show()
