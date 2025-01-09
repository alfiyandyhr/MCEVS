import os
import openmdao.api as om
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

class AirfoilCoeffs(om.ExplicitComponent):
	"""
	Parameter: airfoil
	Input: AoA
	Output: Cl, Cd
	"""
	def initialize(self):
		self.options.declare('airfoil', types=str)

	def setup(self):
		self.add_input('AoA', units='deg')
		self.add_output('Cl', units=None)
		self.add_output('Cd', units=None)
		self.declare_partials('*','*')

	def compute(self, inputs, outputs):
		airfoil = self.options['airfoil']
		AoA = inputs['AoA']

		if airfoil == 'CLARKY':
			if AoA < -9.25:
				outputs['Cl'] = 0.00016055*AoA**2 + 0.03038325*AoA - 0.12669162
				outputs['Cd'] = 0.00010179*AoA**2 + 0.01926335*AoA + 0.25451674
			elif AoA > 17.0:
				outputs['Cl'] =  0.00013341*AoA**2 - 0.02628272*AoA + 1.75924935
				outputs['Cd'] = -0.00010646*AoA**2 + 0.02097336*AoA - 0.23195915
			else:
				outputs['Cl'] = -0.00022620*AoA**3 + 0.00035166*AoA**2 + 0.11501989*AoA + 0.376
				outputs['Cd'] =  0.00000563*AoA**3 + 0.00026543*AoA**2 - 0.00170558*AoA + 0.00652

		elif airfoil == 'BOEING_VERTOL_VR12':
			if AoA < -9.5:
				outputs['Cl'] = -0.00004972*AoA**2 - 0.00942282*AoA - 0.58952914
				outputs['Cd'] = -0.00014875*AoA**2 - 0.02818766*AoA - 0.15965827
			elif AoA > 18.5:
				outputs['Cl'] =  0.00069132*AoA**2 - 0.13722607*AoA + 3.64317973
				outputs['Cd'] = -0.00020911*AoA**2 + 0.04150877*AoA - 0.59881363
			else:
				outputs['Cl'] = -0.00023201*AoA**3 + 0.00151456*AoA**2 + 0.11609899*AoA + 0.17357118
				outputs['Cd'] = -0.00000441*AoA**3 + 0.00038061*AoA**2 - 0.00225189*AoA + 0.00628659
				
	def compute_partials(self, inputs, partials):
		airfoil = self.options['airfoil']
		AoA = inputs['AoA']

		if airfoil == 'CLARKY':
			if AoA < -9.25:
				partials['Cl','AoA'] = 2*0.00016055*AoA + 0.03038325
				partials['Cd','AoA'] = 2*0.00010179*AoA + 0.01926335
			elif AoA > 17.0:
				partials['Cl','AoA'] =  2*0.00013341*AoA - 0.02628272
				partials['Cd','AoA'] = -2*0.00010646*AoA + 0.02097336
			else:
				partials['Cl','AoA'] = -3*0.00022620*AoA**2 + 2*0.00035166*AoA + 0.11501989
				partials['Cd','AoA'] =  3*0.00000563*AoA**2 + 2*0.00026543*AoA - 0.00170558

		elif airfoil == 'BOEING_VERTOL_VR12':
			if AoA < -9.5:
				partials['Cl','AoA'] = -2*0.00004972*AoA - 0.00942282
				partials['Cd','AoA'] = -2*0.00014875*AoA - 0.02818766
			elif AoA > 18.5:
				partials['Cl','AoA'] =  2*0.00069132*AoA - 0.13722607
				partials['Cd','AoA'] = -2*0.00020911*AoA + 0.04150877
			else:
				partials['Cl','AoA'] = -3*0.00023201*AoA**2 + 2*0.00151456*AoA + 0.11609899
				partials['Cd','AoA'] = -3*0.00000441*AoA**2 + 2*0.00038061*AoA - 0.00225189

if __name__ == '__main__':
	airfoil = load_airfoil('CLARKY')
	airfoil.plot()



