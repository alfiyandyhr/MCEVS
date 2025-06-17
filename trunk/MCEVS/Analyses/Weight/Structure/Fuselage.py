import numpy as np
import openmdao.api as om

class FuselageWeightRoskam(om.ExplicitComponent):
	"""
	Computes fuselage weight
	Parameters:
		n_pax	: number of passengers including the pilots
		l_fuse	: fuselage length, not including nose mounted nacelle length [m]
		p_max	: maximum fuselage perimeter [m]
		tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|takeoff : total take-off weight [kg]
	Outputs:
		Weight|fuselage : fuselage weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> This method does not account for pressurized fuselages
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('n_pax', types=int, desc='Number of passengers, including the pilots')
		self.options.declare('l_fuse', types=float, desc='Fuselage length')
		self.options.declare('p_max', types=float, desc='Maximum fuselage perimeter')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total weight')
		self.add_output('Weight|fuselage', units='kg', desc='Fuselage weight')
		self.declare_partials('Weight|fuselage', 'Weight|takeoff')

	def compute(self, inputs, outputs):
		n_pax = self.options['n_pax']
		l_fuse = self.options['l_fuse']		# in [m]
		p_max = self.options['p_max']		# in [m]
		tf = self.options['tf']
		W_takeoff = inputs['Weight|takeoff']	# in [kg]

		# Calculating W_fuselage
		kg_to_lb = 2.20462**0.144
		m_to_ft = 3.28084**0.383
		lb_to_kg = 0.453592
		
		W_fuselage = 14.86 * W_takeoff**0.144 * (l_fuse/p_max)**0.778 * l_fuse**0.383 * n_pax**0.455 * kg_to_lb * m_to_ft * lb_to_kg

		outputs['Weight|fuselage'] = tf * W_fuselage # in [kg]

	def compute_partials(self, inputs, partials):
		n_pax = self.options['n_pax']
		l_fuse = self.options['l_fuse'] 	# in [m]
		p_max = self.options['p_max'] 		# in [m]
		tf = self.options['tf']
		W_takeoff = inputs['Weight|takeoff']	# in [kg]

		kg_to_lb = 2.20462**0.144
		m_to_ft = 3.28084**0.383
		lb_to_kg = 0.453592
		dWfuse_dWtakeoff = 14.86 * 0.144 * W_takeoff**(-0.856) * (l_fuse/p_max)**0.778 * l_fuse**0.383 * n_pax**0.455 * kg_to_lb * m_to_ft * lb_to_kg

		partials['Weight|fuselage', 'Weight|takeoff'] = tf * dWfuse_dWtakeoff

class FuselageWeightM4ModelsForNASALPC(om.ExplicitComponent):
	"""
	Computes fuselage weight using M4 structural weight regression models for NACA LPC
	Parameters:
		tf 					: technology factor (default=1.0)
	Inputs:
		Wing|area 			: wing area [m**2]
		Wing|aspect_ratio 	: wing aspect ratio
		Fuselage|length 	: fuselage length [m]
		Weight|battery 		: battery weight [kg]
		v_cruise	 		: cruise speed [m/s]
	Outputs:
		Weight|wing 		: wing weight [kg]
	Notes:
		> Physics-based structural sizing models for a wide variety of design parameters
		> Then, the optimized weight results serve as empirical weight data from which new regression models
		  can be trained to predict structural component mass properties for subclasses of UAM vehicles.
		> The physics-based models are generated using M4 Structures Studio (M4SS)
		> M4SS is a parameterized structural modeling tool that creates finite element models for NASTRAN
		> It has been validated against historical aircraft weight data and used to predict structural weights for various UAM configurations
	Sources:
		1. Winter, T. F., Robinson, J. H., Sutton, M., Chua, J., Gamez, A., and Nascenzi, T.,
		   “Structural Weight Prediction for an Urban Air Mobility Concept,” presented at the AIAA AVIATION 2020 FORUM, VIRTUAL EVENT, 2020.
		   https://doi.org/10.2514/6.2020-2653
		2. Ruh, M. L., et al, “Large-Scale Multidisciplinary Design Optimization of a NASA Air Taxi Concept Using a Comprehensive Physics-Based System Model,”
		   presented at the AIAA SCITECH 2024 Forum, Orlando, FL, 2024. https://doi.org/10.2514/6.2024-0771
	"""

	def initialize(self):
		self.options.declare('tf', types=float, default=1.0, desc='Technology factor')

	def setup(self):
		self.add_input('Wing|area', units='m**2', desc='Wing area')
		self.add_input('Wing|aspect_ratio', units=None, desc='Wing aspect ratio')
		self.add_input('Fuselage|length', units='m', desc='Fuselage length')
		self.add_input('Weight|battery', units='kg', desc='Battery weight')
		self.add_input('v_cruise', units='m/s', desc='Cruise speed')
		self.add_output('Weight|fuselage', units='kg', desc='Wing weight')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		l_fuse = inputs['Fuselage|length']
		W_batt = inputs['Weight|battery']
		v_cruise = inputs['v_cruise']
		coeffs = [ 1.01472161e+00, -4.06758251e-01, 4.25974124e+01, 3.10575276e-02, 6.87345416e-02, -1.16727769e+02 ]

		W_fuse = coeffs[0]*S_wing + coeffs[1]*AR_wing + coeffs[2]*l_fuse + coeffs[3]*W_batt + coeffs[4]*v_cruise + coeffs[5]

		outputs['Weight|fuselage'] = tf * W_fuse

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		l_fuse = inputs['Fuselage|length']
		W_batt = inputs['Weight|battery']
		v_cruise = inputs['v_cruise']
		coeffs = [ 1.01472161e+00, -4.06758251e-01, 4.25974124e+01, 3.10575276e-02, 6.87345416e-02, -1.16727769e+02 ]

		partials['Weight|fuselage', 'S_wing'] = tf * coeffs[0]
		partials['Weight|fuselage', 'AR_wing'] = tf * coeffs[1]
		partials['Weight|fuselage', 'l_fuse'] = tf * coeffs[2]
		partials['Weight|fuselage', 'W_batt'] = tf * coeffs[3]
		partials['Weight|fuselage', 'v_cruise'] = tf * coeffs[4]
		
if __name__ == '__main__':
	
	model = om.Group()
	model.add_subsystem('m4_fuse_weight', FuselageWeightM4ModelsForNASALPC())

	prob = om.Problem(model, reports=False)
	prob.setup()

	prob.set_val('m4_fuse_weight.Wing|area', 19.53547845)
	prob.set_val('m4_fuse_weight.Wing|aspect_ratio', 12.12761)
	prob.set_val('m4_fuse_weight.Fuselage|length', 9.144)
	prob.set_val('m4_fuse_weight.Weight|battery', 819.95083265)
	prob.set_val('m4_fuse_weight.v_cruise', 67.056)

	prob.run_model()
	print(prob['m4_fuse_weight.Weight|fuselage'])		
