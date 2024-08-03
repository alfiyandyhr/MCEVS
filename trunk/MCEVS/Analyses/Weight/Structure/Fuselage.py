import numpy as np
import openmdao.api as om

class FuselageWeight(om.ExplicitComponent):
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
