import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class Airfoil(object):
	"""
	Airfoil object that stores performance of each section airfoil
	Notes:
		> should be called using "load_airfoil" function
		> alpha is in deg
		> Cl_func and Cd_func use "cubic" spline interpolation
	"""
	def __init__(self):
		super(Airfoil, self).__init__()
		self.alpha_data = None
		self.Cl_data = None
		self.Cd_data = None
		self.Cl_func = None
		self.Cd_func = None

	def eval_Cl(self, alpha_deg):
		"""
		Evaluate Cl given alpha in deg
		"""
		return self.Cl_func(alpha_deg)

	def eval_Cd(self, alpha_deg):
		"""
		Evaluate Cd given alpha in deg
		"""
		return self.Cd_func(alpha_deg)

	def plot(self):
		"""
		Plot the real and interpolation data
		"""
		# Prepare interpolation data
		alpha_interp = np.linspace(-180,180,100)
		Cl_interp = self.Cl_func(alpha_interp)
		Cd_interp = self.Cd_func(alpha_interp)

		fig, axs = plt.subplots(nrows=2, ncols=1)
		axs[0].plot(self.alpha_data, self.Cl_data, 'o')
		axs[0].plot(alpha_interp, Cl_interp, '-')
		axs[0].set_xlabel('Alpha')
		axs[0].set_ylabel('Cl')
		axs[0].legend(['Real','Interpolated'])
		axs[1].plot(self.alpha_data, self.Cd_data, 'o')
		axs[1].plot(alpha_interp, Cd_interp, '-')
		axs[1].set_xlabel('Alpha')
		axs[1].set_ylabel('Cd')
		axs[1].legend(['Real','Interpolated'])
		fig.suptitle('Real vs Interpolated Airfoil Data')
		plt.show()

def load_airfoil(name):
	# Load airfoil data
	file_dir = os.path.dirname(os.path.abspath(__file__))
	airfoil_path = os.path.join(file_dir, 'Airfoils', name + '.dat')
	data = np.genfromtxt(airfoil_path, skip_header=14)

	# Instantiate an airfoil object
	a = Airfoil()
	a.alpha_data = data[:,0]
	a.Cl_data = data[:,1]
	a.Cd_data = data[:,2]
	a.Cl_func = interp1d(a.alpha_data, a.Cl_data, kind='quadratic')
	a.Cd_func = interp1d(a.alpha_data, a.Cd_data, kind='quadratic')

	return a

if __name__ == '__main__':
	airfoil = load_airfoil('CLARKY')
	airfoil.plot()



