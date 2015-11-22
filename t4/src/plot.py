#!/usr/bin/python
from __future__ import print_function

import sys
import matplotlib.pyplot as plt
import numpy as np

file_name = str(sys.argv[1])
print('Using ROC file: '+file_name)

data = np.genfromtxt(file_name, delimiter=',', names=['theta', 'tpr', 'fpr'])
plt.plot(data['fpr'], data['tpr'], color='r', label='roc')
plt.title('ROC')    
plt.xlabel('FPR')
plt.ylabel('TPR')
plt.show()