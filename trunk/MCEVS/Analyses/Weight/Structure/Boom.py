import numpy as np
import openmdao.api as om

class BoomWeightRoskam(om.ExplicitComponent):
	"""
	Computes booms weight using empirical equation for nacelles
	Parameters:
		tf 				: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		total_req_takeoff_power : total power required for liftrotors during hover climb (adapted for eVTOL) [W]
	Outputs:
		Weight|boom 	: boom(s) weight [kg]
	Notes:
		> Torenbeek Method
		> Used for multi-engine airplane with piston (turboprop) engines
		> Since it uses total_req_takeoff_power, it estimates the weights of all nacelles (booms)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('total_req_takeoff_power', units='W', desc='Total required takeoff power')
		self.add_output('Weight|booms', units='kg', desc='Total booms weight')
		self.declare_partials('Weight|booms', 'total_req_takeoff_power')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		total_req_takeoff_power = inputs['total_req_takeoff_power']	# in [W]

		# Calculating W_boom
		W_to_hp = 1/745.7
		lb_to_kg = 0.453592
		
		W_booms = 0.14 * total_req_takeoff_power * W_to_hp * lb_to_kg

		outputs['Weight|booms'] = tf * W_booms # in [kg]

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		total_req_takeoff_power = inputs['total_req_takeoff_power']	# in [W]

		W_to_hp = 1/745.7
		lb_to_kg = 0.453592
		dWbooms_dPtakeoff = 0.14 * W_to_hp * lb_to_kg

		partials['Weight|booms', 'total_req_takeoff_power'] = tf * dWbooms_dPtakeoff

class BoomWeightM4ModelsForNASALPC(om.ExplicitComponent):
	"""
	Computes boom weight using M4 structural weight regression models for NACA LPC
	Parameters:
		tf 					: technology factor (default=1.0)
	Inputs:
		Wing|area 			: wing area [m**2]
		Wing|aspect_ratio 	: wing aspect ratio
		Fuselage|length 	: fuselage length [m]
		Weight|battery 		: battery weight [kg]
		v_cruise	 		: cruise speed [m/s]
	Outputs:
		Weight|booms 		: total booms weight [kg]
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
		self.add_output('Weight|booms', units='kg', desc='Total boom weight')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		l_fuse = inputs['Fuselage|length']
		W_batt = inputs['Weight|battery']
		v_cruise = inputs['v_cruise']
		coeffs = [ 3.33883108e+00, 5.40016118e+00, -9.17180256e-02, 1.28646937e-04, -6.25697773e-03, -1.17008833e+01 ]

		W_booms = coeffs[0]*S_wing + coeffs[1]*AR_wing + coeffs[2]*l_fuse + coeffs[3]*W_batt + coeffs[4]*v_cruise + coeffs[5]

		outputs['Weight|booms'] = tf * W_booms

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		l_fuse = inputs['Fuselage|length']
		W_batt = inputs['Weight|battery']
		v_cruise = inputs['v_cruise']
		coeffs = [ 3.33883108e+00, 5.40016118e+00, -9.17180256e-02, 1.28646937e-04, -6.25697773e-03, -1.17008833e+01 ]

		partials['Weight|booms', 'S_wing'] = tf * coeffs[0]
		partials['Weight|booms', 'AR_wing'] = tf * coeffs[1]
		partials['Weight|booms', 'l_fuse'] = tf * coeffs[2]
		partials['Weight|booms', 'W_batt'] = tf * coeffs[3]
		partials['Weight|booms', 'v_cruise'] = tf * coeffs[4]
		
if __name__ == '__main__':
	
	model = om.Group()
	model.add_subsystem('m4_boom_weight', BoomWeightM4ModelsForNASALPC())

	prob = om.Problem(model, reports=False)
	prob.setup()

	prob.set_val('m4_boom_weight.Wing|area', 19.53547845)
	prob.set_val('m4_boom_weight.Wing|aspect_ratio', 12.12761)
	prob.set_val('m4_boom_weight.Fuselage|length', 9.144)
	prob.set_val('m4_boom_weight.Weight|battery', 819.95084106)
	prob.set_val('m4_boom_weight.v_cruise', 67.056)

	prob.run_model()
	print(prob['m4_boom_weight.Weight|booms'])
