import numpy as np

data_rotate1 = np.genfromtxt('BOEING_VERTOL_VR12_Re1e6_Ncrit5_Viterna_for_NASA_QR.dat', skip_header=3)[:, 0:3]
data_rotate2 = np.genfromtxt('BOEING_VERTOL_VR12_Re1e6_Ncrit5_Viterna_for_NASA_LPC.dat', skip_header=3)[:, 0:3]
data_rotate1[:, 0] = data_rotate1[:, 0] / np.pi * 180
data_rotate2[:, 0] = data_rotate2[:, 0] / np.pi * 180
np.savetxt('BOEING_VERTOL_VR12_Re1e6_Ncrit5_Viterna_for_NASA_QR_.dat', data_rotate1)
np.savetxt('BOEING_VERTOL_VR12_Re1e6_Ncrit5_Viterna_for_NASA_LPC_.dat', data_rotate2)

# plt.plot(data_rotate1[:,0], data_rotate1[:,1])
# plt.plot(data_rotate2[:,0], data_rotate2[:,1])
# # plt.plot(data_rotate2[:,0]*np.pi/180, data_rotate2[:,1])
# plt.show()
