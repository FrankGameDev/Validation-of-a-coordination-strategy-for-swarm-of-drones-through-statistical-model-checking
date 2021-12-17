import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

x = np.linspace(-4*np.pi, 4*np.pi,50)
y = np.linspace(-4*np.pi, 4*np.pi,50)

z= x**2 + y**2;

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x,y,z)

plt.show()

