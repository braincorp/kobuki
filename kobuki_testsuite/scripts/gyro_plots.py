import numpy as np
import matplotlib.pyplot as plt
import sys

folder = sys.argv[1]
data    = np.genfromtxt(folder + "/old_imu_data.csv", delimiter=",")
data_bc = np.genfromtxt(folder + "/bc_imu_data.csv", delimiter=",")

plt.plot(data[:, 1], ".r", data_bc[:, 1], '.b')
plt.title("angle")
plt.figure()
plt.plot(data[:, 2], ".r", data_bc[:, 2], '.b')
plt.title("angular velocity")
plt.figure()
plt.plot(data[:, 1], data_bc[:, 1], ".")
plt.xlabel("kobuki")
plt.ylabel("bc")
plt.title("angle")

plt.figure()
plt.plot(data[:, 2], data_bc[:, 2], ".")
plt.xlabel("kobuki")
plt.ylabel("bc")
plt.title("angle velocity")

plt.figure()
plt.plot(data[:, 2] - data_bc[:, 2], ".")
plt.title("angle velocity diff")

plt.figure()
plt.plot(data[:, 1] - data_bc[:, 1], ".")
plt.title("angle diff")

x = data_bc[:, 2]
A = np.vstack([x, np.ones(len(x))]).T
y = data[:, 2]
m, c = np.linalg.lstsq(A, y)[0]
print m, c


plt.show()
