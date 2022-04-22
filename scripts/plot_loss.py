import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D

df = pd.read_csv("/home/justin/loss/big.txt")

fig, ax1 = plt.subplots()

color = 'tab:orange'
ax1.set_xlabel("Training Epoch")
ax1.set_ylabel("Model Loss (m)")
line1, = ax1.plot(df['loss'], color=color)
#ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('Gradient Magnitude')  # we already handled the x-label with ax1
line2, = ax2.plot(df['norm'], color=color)
#ax2.tick_params(axis='y', labelcolor=color)

ax2.legend([line1, line2], ["Loss", "Gradient Magnitude"])

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()
