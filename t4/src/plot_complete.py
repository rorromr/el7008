#!/usr/bin/python
from __future__ import print_function, division

import sys, os
import matplotlib.pyplot as plt
import numpy as np

dir_name = str(sys.argv[1])
print('CVS ROC files: '+ dir_name)

fpr = np.zeros((100,))
tpr = np.zeros((100,))
theta = np.zeros((100,))
num_file = 0
for file_name in os.listdir(dir_name):
    if file_name.endswith('.cvs'):
        complete_file = dir_name+os.sep+file_name
        print('Loading: ' + complete_file)
        data = np.genfromtxt(complete_file, delimiter=',', names=['theta', 'tpr', 'fpr'])
        fpr += data['fpr']
        tpr += data['tpr']
        theta = data['theta']
        num_file += 1

# Normalizar
fpr = 1/num_file*fpr
tpr = 1/num_file*tpr
print('Saving complete ROC: complete_roc.csv')
np.savetxt("complete_roc.csv", np.transpose([theta, fpr, tpr]), delimiter=",")

plt.plot( fpr, tpr, color='r', label='roc')

plt.title('ROC')    
plt.xlabel('FPR')
plt.ylabel('TPR')
plt.show()
