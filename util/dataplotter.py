import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np

data_set = 'auto_flight'

data_file = 'util\\decoded_'+data_set+'.BIN.txt'
data = np.genfromtxt(data_file, delimiter=',', names=True)
time_elapsed = data['timestamp']-data['timestamp'][0]
plt.plot(time_elapsed, data['roll_pos'])
plt.plot(time_elapsed, data['pitch_pos'])
plt.show()