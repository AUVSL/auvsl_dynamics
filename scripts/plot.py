import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/xout.csv")
#df = df.loc[:400]

plt.plot(df['x'], df['y'], color='b')
plt.show()

plt.plot(df['z'], color='r')
plt.show()

#plt.plot(df['vx'])
# plt.plot(df['vy'])
# plt.plot(df['vz'])
# plt.legend(['vx','vy','vz'])
#plt.show()

# plt.plot(df['wx'])
# plt.plot(df['wy'])
# plt.plot(df['wz'])
# plt.legend(['wx','wy','wz'])
#plt.show()

# plt.plot(df['qd1'])
# plt.plot(df['qd2'])
# plt.plot(df['qd3'])
# plt.plot(df['qd4'])
# plt.show()

# df = pd.read_csv("/home/justin/xout.csv")
# plt.plot(df['qd1'], color='r')
# plt.show()


df = pd.read_csv("/home/justin/log.csv")
plt.plot(df["zr1"], color='r', alpha=.5)
plt.plot(df["zr2"], color='g', alpha=.5)
plt.plot(df["zr3"], color='b', alpha=.5)
plt.plot(df["zr4"], color='w', alpha=.5)
plt.show()
