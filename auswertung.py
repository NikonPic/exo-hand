# %%
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# duration of experiment
time = 20

# datapath to .txt file
datapath = './v120.txt'

# read data from .txt file
data = pd.read_csv(datapath, header=None)

# convert force values from mN to N
data.iloc[:, 16:20] = data.iloc[:, 16:20] / 1000

# upper and lower force limits
F_upper = np.full(data.shape[0], 2.4)
F_lower = np.full(data.shape[0], 0.6)

# Variables for plots
datalength = data.shape[0]
x = np.arange(datalength)
frequency = datalength / time
seconds_4 = int(frequency * 4)

# Colors for the plots
color_1 = '#196774'
color_2 = '#A6BC09'
color_3 = '#F0941F'
color_4 = '#EF6024'
color_5 = '#202022'
color_6 = '#BD2A2E'

linewidth = 1.2

fig, axs = plt.subplots(2, 4, figsize=(14, 6), sharex=True)

axs[0, 1].sharey(axs[0, 0])
axs[0, 2].sharey(axs[0, 0])
axs[0, 3].sharey(axs[0, 0])

axs[1, 1].sharey(axs[1, 0])
axs[1, 2].sharey(axs[1, 0])
axs[1, 3].sharey(axs[1, 0])

# Plotting
a = plt.subplot(2, 4, 1)

plt.plot(x, data.iloc[:, 0], linewidth=linewidth, color=color_1)
plt.plot(x, data.iloc[:, 1], linewidth=linewidth, color=color_2)
plt.plot(x, data.iloc[:, 2], linewidth=linewidth, color=color_3)
plt.plot(x, data.iloc[:, 3], linewidth=linewidth, color=color_4)
plt.ylim([-100, 150])
plt.xlim([0, datalength])
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.title('index finger')
plt.ylabel('angle [°]')
plt.grid(0.1)




e = plt.subplot(2, 4, 5)
plt.plot(x, data.iloc[:, 16], linewidth=linewidth, color=color_5)
plt.xlabel('time [s]')
plt.ylabel('force [N]')
plt.xlim([0, datalength])
plt.yticks(np.arange(-1, 9, 1))
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.grid(0.1)
plt.ylim([-1, 3])

b = plt.subplot(2, 4, 2)
plt.plot(x, data.iloc[:, 4], linewidth=linewidth, color=color_1)
plt.plot(x, data.iloc[:, 5], linewidth=linewidth, color=color_2)
plt.plot(x, data.iloc[:, 6], linewidth=linewidth, color=color_3)
plt.plot(x, data.iloc[:, 7], linewidth=linewidth, color=color_4)
plt.ylim([-100, 150])
plt.xlim([0, datalength])
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.title('middle finger')
plt.grid(0.1)

f = plt.subplot(2, 4, 6)
plt.plot(x, data.iloc[:, 17], linewidth=linewidth, color=color_5)
plt.xlabel('time [s]')
plt.xlim([0, datalength])
plt.yticks(np.arange(-6, 9, 1))
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.grid(0.1)
plt.ylim([-1, 3])

c = plt.subplot(2, 4, 3)
plt.plot(x, data.iloc[:, 8], linewidth=linewidth, color=color_1)
plt.plot(x, data.iloc[:, 9], linewidth=linewidth, color=color_2)
plt.plot(x, data.iloc[:, 10], linewidth=linewidth, color=color_3)
plt.plot(x, data.iloc[:, 11], linewidth=linewidth, color=color_4)
plt.ylim([-100, 150])
plt.xlim([0, datalength])
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.title('ring finger')
plt.grid(0.1)

g = plt.subplot(2, 4, 7)
plt.plot(x, data.iloc[:, 18], linewidth=linewidth, color=color_5)
plt.xlabel('time [s]')
plt.xlim([0, datalength])
plt.yticks(np.arange(-6, 9, 1))
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.grid(0.1)
plt.ylim([-1, 3])

d = plt.subplot(2, 4, 4)
plt.plot(x, data.iloc[:, 12], linewidth=linewidth, color=color_1)
plt.plot(x, data.iloc[:, 13], linewidth=linewidth, color=color_2)
plt.plot(x, data.iloc[:, 14], linewidth=linewidth, color=color_3)
plt.plot(x, data.iloc[:, 15], linewidth=linewidth, color=color_4)
plt.ylim([-100, 150])
plt.xlim([0, datalength])
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.title('little finger')
plt.grid(0.1)



h = plt.subplot(2, 4, 8)
plt.plot(x, data.iloc[:, 19], linewidth=linewidth, color=color_5)
plt.xlabel('time [s]')
plt.xlim([0, datalength])
plt.yticks(np.arange(-6, 9, 1))
plt.xticks(np.arange(0, datalength, seconds_4), ['0', '4', '8', '12', '16', '20'])
plt.grid(0.1)
plt.ylim([-1, 3])


for ax in axs[1, :]:
    ax.axhline(y=2.5, color='red', linestyle='--', linewidth=1.2, alpha=0.8)
    ax.axhline(y=0.6, color='red', linestyle='--', linewidth=1.2, alpha=0.8)


plt.tight_layout(rect=[0, 0, 1, 0.96])

# Adding a legend at the top center of the overall figure
fig.legend(['rotatory sensor 1', 'rotatory sensor 2', 'rotatory sensor 3', 'vertical rotation axis'],
           loc='upper center', fontsize=9, ncol=4, )

plt.show()


# %%
