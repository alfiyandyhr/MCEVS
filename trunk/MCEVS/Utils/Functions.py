import numpy as np
import openmdao.api as om

class SoftMax(om.ExplicitComponent):
	"""
	Compute the soft (smooth) max function by KS aggregation
	Inputs:
		f1
		f2
	Outputs:
		fmax
	"""

	def initialize(self):
		self.options.declare('rho', types=int, default=10, desc='KS Factor')

	def setup(self):
		self.add_input('f1')
		self.add_input('f2')
		self.add_output('fmax')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		f1 = inputs['f1']
		f2 = inputs['f2']
		rho = self.options['rho']
		tmp = np.exp(rho * f1) + np.exp(rho * f2)

		outputs['fmax'] = (1/rho) * np.log(tmp)

	def compute_partials(self, inputs, partials):
		f1 = inputs['f1']
		f2 = inputs['f2']
		rho = self.options['rho']
		tmp = np.exp(rho * f1) + np.exp(rho * f2)

		partials['fmax', 'f1'] = np.exp(rho * f1) / tmp
		partials['fmax', 'f2'] = np.exp(rho * f2) / tmp