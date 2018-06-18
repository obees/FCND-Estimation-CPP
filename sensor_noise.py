import numpy as np

gps_data = np.loadtxt('config/log/Graph1.txt', delimiter=',', dtype='Float64', skiprows=1)
print("gps:{}".format(np.std(gps_data,axis=0)))


acc_data = np.loadtxt('config/log/Graph2.txt', delimiter=',', dtype='Float64', skiprows=1)
print("acc:{}".format(np.std(acc_data,axis=0)))
