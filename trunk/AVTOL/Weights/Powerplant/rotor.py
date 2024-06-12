import numpy as np
import openmdao.api as om

class RotorWeight(om.ExplicitComponent):
	"""
	Computes rotors weight
	Parameters:
		N_rotor	: number of rotors
		N_bl	: number of blades per rotor
	Inputs:
		Rotor|Diameter 	: rotor diameter [m]
		max_power		: maximum power, i.e., power during climb [W]
	Outputs:
		eVTOL|W_rotors	: weight of all rotors [kg]
	Notes:
		> Torenbeek method for Commercial Transport Airplanes (propeller section)
		> For GA airplanes, it is recommended to use propeller manufacturer data
		> If such data is not available, this method might be used
		> Valid only for motor shaft powers < 1100 kW
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('N_bl', types=int, desc='Number of blades per rotor')

	def setup(self):
		self.add_input('Rotor|Diameter', units='m', desc='Rotor diameter')
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('eVTOL|W_rotors', units='kg', desc='Weight of all rotors')
		self.declare_partials('eVTOL|W_rotors', 'Rotor|Diameter')
		self.declare_partials('eVTOL|W_rotors', 'max_power')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		D = inputs['Prop|Diameter']	# in [m]
		p_max = inputs['max_power']	# in [W]

		k_prop = 0.144 # for piston engines; 0.108 for turboprops

		# Calculating W_rotors
		m_to_ft = 3.28084**0.782
		W_to_hp = 0.00134102**0.782
		lb_to_kg = 0.453592

		W_prop = k_prop * (D * (p_max/N_rotor) * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg
		W_rotors = N_rotor * W_prop

		outputs['eVTOL|W_rotors'] = W_rotors # in [kg]

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		D = inputs['Prop|Diameter']	# in [m]
		p_max = inputs['max_power']	# in [W]

		k_prop = 0.144 # for piston engines; 0.108 for turboprops

		# Calculating dWrotors_dD and dWrotors_dp
		m_to_ft = 3.28084**0.782
		W_to_hp = 0.00134102**0.782
		lb_to_kg = 0.453592

		dWrotors_dD = N_rotor * k_prop * 0.782 * D**(-0.218) * ((p_max/N_rotor) * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg
		dWrotors_dp = N_rotor * k_prop * 0.782 * p_max**(-0.218) * (D / N_rotor * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg

		partials['eVTOL|W_rotors', 'Prop|Diameter'] = dWrotors_dD
		partials['eVTOL|W_rotors', 'max_power'] = dWrotors_dp





