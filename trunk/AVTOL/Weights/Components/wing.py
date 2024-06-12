import numpy as np
import openmdao.api as om

class WingWeight(om.ExplicitComponent):
	"""
	Computes cantilever wing weight
	Parameters:
		wing_AR : wing aspect ratio
		n_ult	: design ultimate load factor
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
		eVTOL|S_wing	: wing area [m**2]
	Outputs:
		eVTOL|W_wing	: wing weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
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
		self.options.declare('wing_AR', types=float, desc='Wing aspect ratio')
		self.options.declare('n_ult', types=float, desc='Design ultimate load factor')

	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total weight')
		self.add_input('eVTOL|S_wing', units='m**2', desc='Wing area')
		self.add_output('eVTOL|W_wing', units='kg', desc='Wing weight')
		self.declare_partials('eVTOL|W_wing', 'eVTOL|W_takeoff')
		self.declare_partials('eVTOL|W_wing', 'eVTOL|S_wing')

	def compute(self, inputs, outputs):
		wing_AR = self.options['wing_AR']
		n_ult = self.options['n_ult']
		W_takeoff = inputs['eVTOL|W_takeoff']
		S_wing = inputs['eVTOL|S_wing']

		# Calculating W_wing
		kg_to_lb = 2.20462**0.397
		m2_to_ft2 = (3.28084*3.28084)**0.360
		lb_to_kg = 0.453592

		W_wing = 0.04674 * W_takeoff**0.397 * S_wing**0.360 * n_ult**0.397 * wing_AR**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		outputs['eVTOL|W_wing'] = W_wing # in [kg]

	def compute_partials(self, inputs, partials):
		wing_AR = self.options['wing_AR']
		n_ult = self.options['n_ult']
		W_takeoff = inputs['eVTOL|W_takeoff']
		S_wing = inputs['eVTOL|S_wing']

		# Calculating W_wing
		kg_to_lb = 2.20462**0.397
		m2_to_ft2 = (3.28084*3.28084)**0.360
		lb_to_kg = 0.453592

		dWwing_dWtakeoff = 0.04674 * 0.397 * W_takeoff**(-0.603) * S_wing**0.360 * n_ult**0.397 * wing_AR**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		dWwing_dSwing = 0.04674 * W_takeoff**0.397 * 0.360 * S_wing**(-0.640) * n_ult**0.397 * wing_AR**1.712 * kg_to_lb * m2_to_ft2 * lb_to_kg

		partials['eVTOL|W_wing', 'eVTOL|W_takeoff'] = dWwing_dWtakeoff
		partials['eVTOL|W_wing', 'eVTOL|S_wing'] = dWwing_dSwing






