import matplotlib.pyplot as plt
import numpy as np
import random
import csv
from sklearn.ensemble import RandomForestClassifier

degrees = []
with open('cases_R.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter = ',')
    
    start = int(input('Start num: ')) # starts at exact num
    stop = int(input('Stop num: '))
    count = 0
    for i in range(start-1):
        degrees.append(0)
        
    for row in plots:
        
        if count >= (start-1) and count <= (stop-1):
            xs = []
            ys = []
            xs.append(float(row[0]))
            xs.append(float(row[3]))
            xs.append(float(row[6]))
            
            ys.append(float(row[1]))
            ys.append(float(row[4]))
            ys.append(float(row[7]))
            
            plt.scatter(xs,ys)
            plt.xlim([-70,70])
            plt.ylim([-70,70])
            plt.title(str(count+1))
            #plt.savefig(str(plot) + '.png')
            plt.figure()
            plt.show()
            degrees.append(float(input("Enter degree " + str(count+1) + ": ")))
            
        count += 1
    rest = [0]*(350 - stop)
    degrees += rest
    
with open('labels.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    
    for i in range(len(degrees)):
        writer.writerow([degrees[i]])
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
