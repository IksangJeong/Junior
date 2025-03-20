import numpy as np

a  = np.array([1,2,3,4,5,6])
print (a)
print (a.shape)

a1 = a[np.newaxis, :]
print (a1)
print (a1.shape)

miles = np.array([1,2,3])
result = miles * 1.6
print (result)
