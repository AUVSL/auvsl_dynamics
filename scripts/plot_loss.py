import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/loss/loss_1648581329.csv")
#df = df.loc[:400]

plt.plot(df['loss'])
#plt.plot(df['lr'])
plt.show()
