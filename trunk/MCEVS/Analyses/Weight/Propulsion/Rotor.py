import numpy as np
import openmdao.api as om
import warnings


class RotorAndHubWeight(om.ExplicitComponent):
	"""
	Computes rotors and hubs weight
	Parameters:
		N_rotor								: number of rotors
		tf 									: technology factor
	Inputs:
		max_thrust							: maximum thrust, i.e., thrust during climb [W]
	Outputs:
		Weight|propulsion|rotors_and_hubs	: weight of all rotors and hubs [kg]
	Notes:
		> technology_factor SHOULD BE 1, since the data is quite new
		> Valid for rotor with thrust > 200 N
	Source:  
    	Duffy, M., Sevier, A. E., Hupp, R., Perdomo, E., and Wakayama, S., “Propulsion Scaling Methods in the Era of Electric Flight,”
    	presented at the 2018 AIAA/IEEE Electric Aircraft Technologies Symposium, Cincinnati, Ohio, 2018. https://doi.org/10.2514/6.2018-4978
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('tf', types=float, desc='Technology factor')

	def setup(self):
		self.add_input('max_thrust', units='N', desc='Maximum thrust')
		self.add_output('Weight|propulsion|rotors_and_hubs', units='kg', desc='Weight of all rotors and hubs')
		self.declare_partials('Weight|propulsion|rotors_and_hubs', 'max_thrust')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		tf = self.options['tf']
		T_max = inputs['max_thrust']  # in [N]
		
		W_rotor_and_hub = 0.0095 * T_max - 1.9275  # in [kg]
		W_rotors_and_hubs = N_rotor * W_rotor_and_hub

		outputs['Weight|propulsion|rotors_and_hubs'] = tf * W_rotors_and_hubs

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		tf = self.options['tf']

		partials['Weight|propulsion|rotors_and_hubs', 'max_thrust'] = tf * N_rotor * 0.0095


class ExtraHubWeight(om.ExplicitComponent):
	"""
	Computes extra hub weight for multirotor and helicopter
	Parameters:
		N_rotor							: number of rotors
		N_bl							: number of blades per rotor
		g 								: gravitational acceleration [m/s**2]
		tf 								: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|propulsion|rotors 		: rotor weight [kg]
		Rotor|radius 					: rotor radius [m]
		Rotor|chord 					: rotor chord [m]
		Rotor|rpm 	 					: rotor rpm [RPM]
	Outputs:
		Weight|propulsion|extra_hubs	: weight of extra hubs [kg]
	Notes:
		> Extra hubs to penalize the heavier hub weights of multirotor/helicopter due to high speed loading
	Source:  
    		
    """
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('N_bl', types=int, desc='Number of blades per rotor')
		self.options.declare('g', types=float, default=9.80665, desc='Gravitational acceleration')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|propulsion|rotors', units='kg', desc='Rotor weight')
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|chord', units='m', desc='Rotor chord')
		self.add_input('Rotor|rpm', units='rpm', desc='Rotor rpm')
		self.add_output('Weight|propulsion|extra_hubs', units='kg', desc='Weight of extra hubs')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		g = self.options['g']
		tf = self.options['tf']
		W_rotors = inputs['Weight|propulsion|rotors']
		r = inputs['Rotor|radius']
		c = inputs['Rotor|chord']
		rpm = inputs['Rotor|rpm']

		m_to_ft1 = 3.28084**1.5 * 3.28084**0.43
		m_to_ft2 = 3.28084 * 3.28084**1.3 * 3.28084**0.67
		m_to_ft3 = 3.28084
		kg_to_lb = 2.20462
		lb_to_kg = 0.453592
		rpm_to_radps = 2 * np.pi / 60

		v_tip = rpm * r * rpm_to_radps

		# Polar moment of inertia of rotor's disk
		W_rotor = W_rotors / N_rotor
		J = 0.5 * W_rotor * r**2

		# Divided into three terms
		W1 = (0.0037 * N_bl**0.28 * r**1.5 * v_tip**0.43) * m_to_ft1
		W2 = (0.01742 * N_bl**0.66 * c * r**1.3 * v_tip**0.67) * m_to_ft2
		W3 = (g * J / r**2) * m_to_ft3 * kg_to_lb

		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			
			W_extra_hub = (W1 * (W2 + W3)**0.55) * lb_to_kg

		outputs['Weight|propulsion|extra_hubs'] = tf * N_rotor * W_extra_hub

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		g = self.options['g']
		tf = self.options['tf']
		W_rotors = inputs['Weight|propulsion|rotors']
		r = inputs['Rotor|radius']
		c = inputs['Rotor|chord']
		rpm = inputs['Rotor|rpm']

		m_to_ft1 = 3.28084**1.5 * 3.28084**0.43
		m_to_ft2 = 3.28084 * 3.28084**1.3 * 3.28084**0.67
		m_to_ft3 = 3.28084
		kg_to_lb = 2.20462
		lb_to_kg = 0.453592
		rpm_to_radps = 2 * np.pi / 60

		# Divided into three terms
		W1 = 0.0037 * N_bl**0.28 * r**1.93 * rpm**0.43 * rpm_to_radps**0.43 * m_to_ft1
		W2 = 0.01742 * N_bl**0.66 * c * r**1.97 * rpm**0.67 * rpm_to_radps**0.67 * m_to_ft2
		W3 = 0.5 * W_rotors / N_rotor * g * m_to_ft3 * kg_to_lb

		dW1_dr = 0.0037 * N_bl**0.28 * 1.93 * r**0.93 * rpm**0.43 * rpm_to_radps**0.43 * m_to_ft1
		dW1_drpm = 0.0037 * N_bl**0.28 * r**1.93 * 0.43 * rpm**(-0.57) * rpm_to_radps**0.43 * m_to_ft1
		dW2_dc = 0.01742 * N_bl**0.66 * r**1.97 * rpm**0.67 * rpm_to_radps**0.67 * m_to_ft2
		dW2_dr = 0.01742 * N_bl**0.66 * c * 1.97 * r**0.97 * rpm**0.67 * rpm_to_radps**0.67 * m_to_ft2
		dW2_drpm = 0.01742 * N_bl**0.66 * c * r**1.97 * 0.67 * rpm**(-0.33) * rpm_to_radps**0.67 * m_to_ft2
		dW3_dWr = 0.5 / N_rotor * g * m_to_ft3 * kg_to_lb

		dW_dWr = W1 * 0.55 * (W2 + W3)**(-0.45) * dW3_dWr * lb_to_kg
		dW_dr = (dW1_dr * (W2 + W3)**0.55 + W1 * 0.55 * (W2 + W3)**(-0.45) * dW2_dr) * lb_to_kg
		dW_dc = W1 * 0.55 * (W2 + W3)**(-0.45) * dW2_dc * lb_to_kg
		dW_drpm = (dW1_drpm * (W2 + W3)**0.55 + W1 * 0.55 * (W2 + W3)**(-0.45) * dW2_drpm) * lb_to_kg

		partials['Weight|propulsion|extra_hubs', 'Weight|propulsion|rotors'] = tf * N_rotor * dW_dWr
		partials['Weight|propulsion|extra_hubs', 'Rotor|radius'] = tf * N_rotor * dW_dr
		partials['Weight|propulsion|extra_hubs', 'Rotor|chord'] = tf * N_rotor * dW_dc
		partials['Weight|propulsion|extra_hubs', 'Rotor|rpm'] = tf * N_rotor * dW_drpm


class ExtraHubWeightWithFixedVtip(om.ExplicitComponent):
	"""
	Computes extra hub weight for multirotor and helicopter
	Parameters:
		N_rotor						: number of rotors
		N_bl						: number of blades per rotor
		g 							: gravitational acceleration [m/s**2]
		tf 							: technology factor (a reduction due to the use of composites, e.g., 0.8)
		v_tip 						: rotor tip speed [m/s]
	Inputs:
		Weight|propulsion|rotors 	: rotor weight [kg]
		Rotor|radius 				: rotor radius [m]
		Rotor|chord 				: rotor chord [m]
		Rotor|rpm 	 				: rotor rpm [RPM]
	Outputs:
		Weight|propulsion|extra_hubs	: weight of extra hubs [kg]
	Notes:
		> Extra hubs to penalize the heavier hub weights of multirotor/helicopter due to high speed loading
	Source:  
    		
    """
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('N_bl', types=int, desc='Number of blades per rotor')
		self.options.declare('v_tip', types=float, desc='Rotor tip speed')
		self.options.declare('g', types=float, default=9.80665, desc='Gravitational acceleration')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|propulsion|rotors', units='kg', desc='Rotor weight')
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Rotor|chord', units='m', desc='Rotor chord')
		self.add_output('Weight|propulsion|extra_hubs', units='kg', desc='Weight of extra hubs')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		v_tip = self.options['v_tip']
		g = self.options['g']
		tf = self.options['tf']
		W_rotors = inputs['Weight|propulsion|rotors']
		r = inputs['Rotor|radius']
		c = inputs['Rotor|chord']

		m_to_ft1 = 3.28084**1.5 * 3.28084**0.43
		m_to_ft2 = 3.28084 * 3.28084**1.3 * 3.28084**0.67
		m_to_ft3 = 3.28084
		kg_to_lb = 2.20462
		lb_to_kg = 0.453592

		# Polar moment of inertia of rotor's disk
		W_rotor = W_rotors / N_rotor
		J = 0.5 * W_rotor * r**2

		# Divided into three terms
		W1 = (0.0037 * N_bl**0.28 * r**1.5 * v_tip**0.43) * m_to_ft1
		W2 = (0.01742 * N_bl**0.66 * c * r**1.3 * v_tip**0.67) * m_to_ft2
		W3 = (g * J / r**2) * m_to_ft3 * kg_to_lb

		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			
			W_extra_hub = (W1 * (W2 + W3)**0.55) * lb_to_kg

		outputs['Weight|propulsion|extra_hubs'] = tf * N_rotor * W_extra_hub

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		v_tip = self.options['v_tip']
		g = self.options['g']
		tf = self.options['tf']
		W_rotors = inputs['Weight|propulsion|rotors']
		r = inputs['Rotor|radius']
		c = inputs['Rotor|chord']

		m_to_ft1 = 3.28084**1.5 * 3.28084**0.43
		m_to_ft2 = 3.28084 * 3.28084**1.3 * 3.28084**0.67
		m_to_ft3 = 3.28084
		kg_to_lb = 2.20462
		lb_to_kg = 0.453592

		# Divided into three terms
		W1 = 0.0037 * N_bl**0.28 * r**1.5 * v_tip**0.43 * m_to_ft1
		W2 = 0.01742 * N_bl**0.66 * c * r**1.3 * v_tip**0.67 * m_to_ft2
		W3 = 0.5 * W_rotors / N_rotor * g * m_to_ft3 * kg_to_lb

		dW1_dr = 0.0037 * N_bl**0.28 * 1.5 * r**0.5 * v_tip**0.43 * m_to_ft1
		dW2_dc = 0.01742 * N_bl**0.66 * r**1.3 * v_tip**0.67 * m_to_ft2
		dW2_dr = 0.01742 * N_bl**0.66 * c * 1.3 * r**0.3 * v_tip**0.67 * m_to_ft2
		dW3_dWr = 0.5 / N_rotor * g * m_to_ft3 * kg_to_lb

		dW_dWr = W1 * 0.55 * (W2 + W3)**(-0.45) * dW3_dWr * lb_to_kg
		dW_dr = (dW1_dr * (W2 + W3)**0.55 + W1 * 0.55 * (W2 + W3)**(-0.45) * dW2_dr) * lb_to_kg
		dW_dc = W1 * 0.55 * (W2 + W3)**(-0.45) * dW2_dc * lb_to_kg

		partials['Weight|propulsion|extra_hubs', 'Weight|propulsion|rotors'] = tf * N_rotor * dW_dWr
		partials['Weight|propulsion|extra_hubs', 'Rotor|radius'] = tf * N_rotor * dW_dr
		partials['Weight|propulsion|extra_hubs', 'Rotor|chord'] = tf * N_rotor * dW_dc


class RotorWeightRoskam(om.ExplicitComponent):
	"""
	Computes rotors weight
	Parameters:
		N_rotor	: number of rotors
		N_bl	: number of blades per rotor
		tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Rotor|radius 	: rotor radius [m]
		max_power		: maximum power, i.e., power during climb [W]
	Outputs:
		Weight|propulsion|rotors	: weight of all rotors [kg]
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
		self.options.declare('N_bl', default=4, types=int, desc='Number of blades per rotor')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('max_power', units='W', desc='Maximum power')
		self.add_output('Weight|propulsion|rotors', units='kg', desc='Weight of all rotors')
		self.declare_partials('Weight|propulsion|rotors', 'Rotor|radius')
		self.declare_partials('Weight|propulsion|rotors', 'max_power')

	def compute(self, inputs, outputs):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		r = inputs['Rotor|radius']   # in [m]
		p_max = inputs['max_power']  # in [W]
		tf = self.options['tf']

		k_prop = 0.144  # for piston engines; 0.108 for turboprops

		# Calculating W_rotors
		m_to_ft = 3.28084**0.782
		W_to_hp = 0.00134102**0.782
		lb_to_kg = 0.453592
		
		W_rotor = k_prop * (2 * r * (p_max / N_rotor) * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg
		W_rotors = N_rotor * W_rotor

		outputs['Weight|propulsion|rotors'] = tf * W_rotors  # in [kg]

	def compute_partials(self, inputs, partials):
		N_rotor = self.options['N_rotor']
		N_bl = self.options['N_bl']
		r = inputs['Rotor|radius'] 	 # in [m]
		p_max = inputs['max_power']  # in [W]
		tf = self.options['tf']

		k_prop = 0.144  # for piston engines; 0.108 for turboprops

		# Calculating dWrotors_dr and dWrotors_dp
		m_to_ft = 3.28084**0.782
		W_to_hp = 0.00134102**0.782
		lb_to_kg = 0.453592

		dWrotors_dr = N_rotor * k_prop * 0.782 * r**(-0.218) * (2 * (p_max / N_rotor) * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg
		dWrotors_dp = N_rotor * k_prop * 0.782 * p_max**(-0.218) * (2 * r / N_rotor * N_bl**0.5)**0.782 * m_to_ft * W_to_hp * lb_to_kg

		partials['Weight|propulsion|rotors', 'Rotor|radius'] = tf * dWrotors_dr
		partials['Weight|propulsion|rotors', 'max_power'] = tf * dWrotors_dp
