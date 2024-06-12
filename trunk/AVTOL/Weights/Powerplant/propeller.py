import numpy as np
import openmdao.api as om

class PropellerWeight(om.ExplicitComponent):
	"""
	Computes propellers weight
	Parameters:
		N_prop	: number of propellers
		N_bl	: number of blades per propeller
	Inputs:
		Prop|Diameter 	: propeller diameter [m]
		max_power		: maximum power, i.e., power during climb [W]
	Outputs:
		eVTOL|W_propellers	: weight of all propellers [kg]
	Notes:
		> Torenbeek method for Commercial Transport Airplanes
		> For GA airplanes, it is recommended to use propeller manufacturer data
		> If such data is not available, this method might be used
		> Valid only for motor shaft powers < 1100 kW
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('N_prop', types=int, desc='Number of propellers')
		self.options.declare('N_bl', types=int, desc='Number of blades per propeller')

	def setup(self):
		self.add_input('Prop|Diameter', units='m', desc='Propeller diameter')
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('eVTOL|W_propellers', units='kg', desc='Weight of all propellers')
		self.declare_partials('eVTOL|W_propellers', 'Prop|Diameter')
		self.declare_partials('eVTOL|W_propellers', 'max_power')

	def compute(self, inputs, outputs):
		N_prop = self.options['N_prop']
		N_bl = self.options['N_bl']
		D = inputs['Prop|Diameter']	# in [m]
		p_max = inputs['max_power']	# in [W]

		k_prop = 0.144 # for piston engines; 0.108 for turboprops

		# Calculating W_propellers
		m_to_ft = 3.28084**0.782
		W_to_hp = 0.00134102**0.782
		lb_to_kg = 0.453592

		W_prop = k_prop * (D * (p_max/N_prop) * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg
		W_propellers = N_prop * W_prop

		outputs['eVTOL|W_propellers'] = W_propellers # in [kg]

	def compute_partials(self, inputs, partials):
		N_prop = self.options['N_prop']
		N_bl = self.options['N_bl']
		D = inputs['Prop|Diameter']	# in [m]
		p_max = inputs['max_power']	# in [W]

		k_prop = 0.144 # for piston engines; 0.108 for turboprops

		# Calculating dWprops_dD and dWprops_dp
		m_to_ft = 3.28084**0.782
		W_to_hp = 0.00134102**0.782
		lb_to_kg = 0.453592

		dWprops_dD = N_prop * k_prop * 0.782 * D**(-0.218) * ((p_max/N_prop) * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg
		dWprops_dp = N_prop * k_prop * 0.782 * p_max**(-0.218) * (D / N_prop * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg

		partials['eVTOL|W_propellers', 'Prop|Diameter'] = dWprops_dD
		partials['eVTOL|W_propellers', 'max_power'] = dWprops_dp





