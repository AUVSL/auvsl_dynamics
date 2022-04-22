import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/log.csv")

plt.plot(df['n0'])
plt.show()

plt.plot(df['n1'])
plt.show()

plt.plot(df['phi'])
plt.show()
