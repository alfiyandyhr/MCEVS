import numpy as np
import openmdao.api as om

class WingWeight(om.ExplicitComponent):
	"""
	Computes cantilever wing weight
	Parameters:
		n_ult	: design ultimate load factor (default=3.0)
		tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
		eVTOL|S_wing	: wing area [m**2]
		eVTOL|AR_wing	: wing aspect ratio
	Outputs:
		Weights|Wing	: wing weight [kg]
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
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_input('eVTOL|S_wing', units='m**2', desc='Wing area')
		self.add_input('eVTOL|AR_wing', desc='Wing aspect ratio')
		self.add_output('Weights|Wing', units='kg', desc='Wing weight')
		self.declare_partials('Weights|Wing', 'eVTOL|W_takeoff')
		self.declare_partials('Weights|Wing', 'eVTOL|S_wing')
		self.declare_partials('Weights|Wing', 'eVTOL|AR_wing')

	def compute(self, inputs, outputs):
		n_ult = self.options['n_ult']
		W_takeoff = inputs['eVTOL|W_takeoff']
		S_wing = inputs['eVTOL|S_wing']
		AR_wing = inputs['eVTOL|AR_wing']
		tf = self.options['tf']

		# Calculating W_wing
		kg_to_lb = 2.20462**0.397
		m2_to_ft2 = (3.28084*3.28084)**0.360
		lb_to_kg = 0.453592

		W_wing = 0.04674 * W_takeoff**0.397 * S_wing**0.360 * n_ult**0.397 * AR_wing**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		outputs['Weights|Wing'] = tf * W_wing # in [kg]

	def compute_partials(self, inputs, partials):
		n_ult = self.options['n_ult']
		W_takeoff = inputs['eVTOL|W_takeoff']
		S_wing = inputs['eVTOL|S_wing']
		AR_wing = inputs['eVTOL|AR_wing']
		tf = self.options['tf']

		# Calculating W_wing
		kg_to_lb = 2.20462**0.397
		m2_to_ft2 = (3.28084*3.28084)**0.360
		lb_to_kg = 0.453592

		dWwing_dWtakeoff = 0.04674 * 0.397 * W_takeoff**(-0.603) * S_wing**0.360 * n_ult**0.397 * AR_wing**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		dWwing_dSwing = 0.04674 * W_takeoff**0.397 * 0.360 * S_wing**(-0.640) * n_ult**0.397 * AR_wing**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		dWwing_dARwing = 0.04674 * W_takeoff**0.397 * S_wing**0.360 * n_ult**0.397 * 1.712 * AR_wing**0.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		partials['Weights|Wing', 'eVTOL|W_takeoff'] = tf * dWwing_dWtakeoff
		partials['Weights|Wing', 'eVTOL|S_wing'] = tf * dWwing_dSwing
		partials['Weights|Wing', 'eVTOL|AR_wing'] = tf * dWwing_dARwing






