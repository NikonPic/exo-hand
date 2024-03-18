# %%
import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# %%
# Open serial port: Replace 'COM6' with your actual serial port
tmax = 15
ser = serial.Serial('COM6', 57600, timeout=1)
time.sleep(2)  # wait for the serial connection to initialize

start_time = time.time()  # Get the current time
firstline = True
with open("sensor_data.txt", "w") as file:
    while True:
        current_time = time.time()  # Update the current time each iteration
        if current_time - start_time > tmax:  # Check if 30 seconds have elapsed
            break  # Exit the loop

        if ser.in_waiting > 0:

            try:
                line = ser.readline().decode('utf-8').rstrip()
            except UnicodeDecodeError:
                continue
            
            # disgard the first line as it might be buggy
            if firstline:
                firstline = False
                continue

            print(line)  # Print to console (optional, for verification)
            file.write(line + "\n")
            file.flush()  # Ensure data is written to file system

ser.close()
# %%
data = []

# Open the file and read the data
with open("sensor_data.txt", "r") as file:
    for line in file:
        # Split the line into components, convert each to float, and append to data list
        values = line.strip().split(',')
        data.append([float(val) for val in values if val])

# Convert the list of lists into a 2D NumPy array for easier handling
data_array = np.array(data)

# Plotting
# Assuming each column is a separate variable you want to plot
# data_array.shape[1] is the number of columns
for i in range(data_array.shape[1]):
    plt.plot(data_array[:, i], label=f'Variable {i+1}')

plt.xlabel('Time Step')
plt.ylabel('Value')
plt.title('Sensor Data Over Time')
plt.legend()
plt.show()
# %%
plt.figure(figsize=(20, 20))

for i in range(4):
    plt.subplot(2, 4, i+1)
    plt.plot(data_array[:, i*4], label=f'Rot. Sensor 1')
    plt.plot(data_array[:, i*4+1], label=f'Rot. Sensor 2')
    plt.plot(data_array[:, i*4+2], label=f'Rot. Sensor 3')
    plt.plot(data_array[:, i*4+3], label=f'Rot. Sensor 4')


plt.subplot(2, 4, 5)
plt.plot(data_array[:, 16])
plt.subplot(2, 4, 6)
plt.plot(data_array[:, 17])
plt.subplot(2, 4, 7)
plt.plot(data_array[:, 18])
plt.subplot(2, 4, 8)
plt.plot(data_array[:, 19])
# %%
# Create figure and axes with shared x and y axes
fig, axs = plt.subplots(2, 4, figsize=(20, 10), sharey='row', sharex='col')

# Define labels for the legend
labels = ['Rot. Sensor 1', 'Rot. Sensor 2', 'Rot. Sensor 3', 'Rot. Sensor 4']

# Plot data for rotation sensors in the top row
for i in range(4):
    axs[0, i].grid(0.25)
    for j in range(4):
        axs[0, i].plot(data_array[:, i*4 + j], label=labels[j])
        
    axs[0, i].set_title(f'Finger {i+1} Rotation Sensors')

axs[0, 1].set_ylabel('Sensor Value')
    


# Plot data for force sensors in the bottom row
for i in range(4):
    axs[1, i].grid(0.25)
    axs[1, i].plot(data_array[:, 16+i], label=f'Force Sensor {i+1}')
    axs[1, i].set_title(f'Finger {i+1} Force')
    axs[1, i].set_xlabel('Time Step')

axs[1, 0].set_ylabel('Force Value')
    

# Create a shared legend for the top row
handles, labels = axs[0, 0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=4)

plt.tight_layout()  # Adjust layout
plt.subplots_adjust(top=0.9)  # Adjust top spacing to fit legend
plt.show()
# %%
