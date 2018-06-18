#import csv
#with open('../config/log/Graph1.txt', 'r') as csvfile:
#    spamreader = csv.reader(csvfile, delimiter=',')
#
#    for row in spamreader:
#        print(', '.join(row))
import numpy as np
#text_file = open("../config/log/Graph1.txt", "r")
#lines = text_file.read().split(',')
#print(lines)
#print(len(lines))
#text_file.close()
data = np.loadtxt('../config/log/Graph1.txt', delimiter=',', dtype='Float64', skiprows=1)
#print(data)
ecart_type = np.std(data, axis=0)
print(ecart_type)

data2 = np.loadtxt('../config/log/Graph2.txt', delimiter=',', dtype='Float64', skiprows=1)
#print(data)
ecart_type2 = np.std(data2, axis=0)
print(ecart_type2)

#data3 = data[:,1]
#print(data3)

#print(len(data))
#print(len(data3))

#ecart_type3 = np.std(data3)
#print(ecart_type3)
