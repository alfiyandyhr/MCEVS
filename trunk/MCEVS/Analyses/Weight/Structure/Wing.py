import openmdao.api as om


class WingWeightRoskam(om.ExplicitComponent):
	"""
	Computes cantilever wing weight
	Parameters:
		n_ult					: design ultimate load factor (default=3.0)
		tf 						: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|takeoff 			: total take-off weight [kg]
		Wing|area				: wing area [m**2]
		Wing|aspect_ratio		: wing aspect ratio
	Outputs:
		Weight|structure|wing	: wing weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
		> n_ult = 3.0, valid for light utility aircraft
		> This method includes:
			* weight of wing tip fairing
			* wing control surfaces
		> This method excludes:
			* fuel tanks
			* wing/fuselage spar carry-through structure
			* effect of sweep angle
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('n_ult', types=float, default=3.0, desc='Design ultimate load factor')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Wing|area', units='m**2', desc='Wing area')
		self.add_input('Wing|aspect_ratio', desc='Wing aspect ratio')
		self.add_output('Weight|structure|wing', units='kg', desc='Wing weight')
		self.declare_partials('Weight|structure|wing', 'Weight|takeoff')
		self.declare_partials('Weight|structure|wing', 'Wing|area')
		self.declare_partials('Weight|structure|wing', 'Wing|aspect_ratio')

	def compute(self, inputs, outputs):
		n_ult = self.options['n_ult']
		W_takeoff = inputs['Weight|takeoff']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		tf = self.options['tf']

		# Calculating W_wing
		kg_to_lb = 2.20462**0.397
		m2_to_ft2 = (3.28084 * 3.28084)**0.360
		lb_to_kg = 0.453592

		W_wing = 0.04674 * W_takeoff**0.397 * S_wing**0.360 * n_ult**0.397 * AR_wing**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		outputs['Weight|structure|wing'] = tf * W_wing  # in [kg]

	def compute_partials(self, inputs, partials):
		n_ult = self.options['n_ult']
		W_takeoff = inputs['Weight|takeoff']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		tf = self.options['tf']

		# Calculating W_wing
		kg_to_lb = 2.20462**0.397
		m2_to_ft2 = (3.28084 * 3.28084)**0.360
		lb_to_kg = 0.453592

		dWwing_dWtakeoff = 0.04674 * 0.397 * W_takeoff**(-0.603) * S_wing**0.360 * n_ult**0.397 * AR_wing**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		dWwing_dSwing = 0.04674 * W_takeoff**0.397 * 0.360 * S_wing**(-0.640) * n_ult**0.397 * AR_wing**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		dWwing_dARwing = 0.04674 * W_takeoff**0.397 * S_wing**0.360 * n_ult**0.397 * 1.712 * AR_wing**0.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		partials['Weight|structure|wing', 'Weight|takeoff'] = tf * dWwing_dWtakeoff
		partials['Weight|structure|wing', 'Wing|area'] = tf * dWwing_dSwing
		partials['Weight|structure|wing', 'Wing|aspect_ratio'] = tf * dWwing_dARwing


class WingWeightM4ModelsForNASALPC(om.ExplicitComponent):
	"""
	Computes wing weight using M4 structural weight regression models for NACA LPC
	Parameters:
		tf 						: technology factor (default=1.0)
	Inputs:
		Wing|area 				: wing area [m**2]
		Wing|aspect_ratio 		: wing aspect ratio
		Fuselage|length 		: fuselage length [m]
		Weight|battery 			: battery weight [kg]
		v_cruise	 			: cruise speed [m/s]
	Outputs:
		Weight|structure|wing 	: wing weight [kg]
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
		self.add_output('Weight|structure|wing', units='kg', desc='Wing weight')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		S_wing = inputs['Wing|area']
		AR_wing = inputs['Wing|aspect_ratio']
		l_fuse = inputs['Fuselage|length']
		W_batt = inputs['Weight|battery']
		v_cruise = inputs['v_cruise']
		coeffs = [1.11379136e+01, 3.14761829e+01, 7.89132288e-01, -2.14257921e-02, 2.40041303e-01, -3.20236992e+02]

		W_wing = coeffs[0] * S_wing + coeffs[1] * AR_wing + coeffs[2] * l_fuse + coeffs[3] * W_batt + coeffs[4] * v_cruise + coeffs[5]

		outputs['Weight|structure|wing'] = tf * W_wing

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		coeffs = [1.11379136e+01, 3.14761829e+01, 7.89132288e-01, -2.14257921e-02, 2.40041303e-01, -3.20236992e+02]

		partials['Weight|structure|wing', 'Wing|area'] = tf * coeffs[0]
		partials['Weight|structure|wing', 'Wing|aspect_ratio'] = tf * coeffs[1]
		partials['Weight|structure|wing', 'Fuselage|length'] = tf * coeffs[2]
		partials['Weight|structure|wing', 'Weight|battery'] = tf * coeffs[3]
		partials['Weight|structure|wing', 'v_cruise'] = tf * coeffs[4]
		

if __name__ == '__main__':
	
	model = om.Group()
	model.add_subsystem('m4_wing_weight', WingWeightM4ModelsForNASALPC())

	prob = om.Problem(model, reports=False)
	prob.setup()

	prob.set_val('m4_wing_weight.Wing|area', 19.53547845)
	prob.set_val('m4_wing_weight.Wing|aspect_ratio', 12.12761)
	prob.set_val('m4_wing_weight.Fuselage|length', 9.144)
	prob.set_val('m4_wing_weight.Weight|battery', 748.46990544)
	prob.set_val('m4_wing_weight.v_cruise', 67.056)

	prob.run_model()
	tf_structure = 0.8
	print(tf_structure * prob['m4_wing_weight.Weight|structure|wing'])
