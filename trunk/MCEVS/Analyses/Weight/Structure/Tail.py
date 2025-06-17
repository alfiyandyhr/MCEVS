import numpy as np
import openmdao.api as om

class HorizontalTailWeightRoskam(om.ExplicitComponent):
	"""
	Computes horizontal tail weight
	Parameters:
		htail_AR				: horizontal tail aspect ratio (default=2.0)
		t_rh					: maximum root thickness [m]
		tf 						: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|takeoff 			: total take-off weight [kg]
		Htail|area				: horizontal tail area [m**2]
	Outputs:
		Weight|horizontal_tail 	: horizontal tail weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> This method does not account for horizontal tail sweep
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('htail_AR', types=float, desc='Horizontal tail aspect ratio')
		self.options.declare('t_rh', types=float, desc='Maximum root thickness')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Htail|area', units='m**2', desc='Horizontal tail area')
		self.add_output('Weight|horizontal_tail', units='kg', desc='Horizontal tail weight')
		self.declare_partials('Weight|horizontal_tail', 'Weight|takeoff')
		self.declare_partials('Weight|horizontal_tail', 'Htail|area')

	def compute(self, inputs, outputs):
		htail_AR = self.options['htail_AR']
		t_rh = self.options['t_rh']				# in [m]
		W_takeoff = inputs['Weight|takeoff']	# in [kg]
		S_htail = inputs['Htail|area']		# in [m**2]
		tf = self.options['tf']

		# Calculating W_htail
		kg_to_lb = 2.20462**0.887
		m_to_ft = 3.28084**0.223
		m2_to_ft2 = (3.28084*3.28084)**0.101
		lb_to_kg = 0.453592
		
		W_htail = (3.184 * W_takeoff**0.887 * S_htail**0.101 * htail_AR**0.138)/(174.04 * t_rh**0.223) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		outputs['Weight|horizontal_tail'] = tf * W_htail # in [kg]

	def compute_partials(self, inputs, partials):
		htail_AR = self.options['htail_AR']
		t_rh = self.options['t_rh']				# in [m]
		W_takeoff = inputs['Weight|takeoff']	# in [kg]
		S_htail = inputs['Htail|area']		# in [m**2]
		tf = self.options['tf']

		# Calculating dWhtail_dWtakeoff and dWhtail_dShtail
		kg_to_lb = 2.20462**0.887
		m_to_ft = 3.28084**0.223
		m2_to_ft2 = (3.28084*3.28084)**0.101
		lb_to_kg = 0.453592
		
		dWhtail_dWtakeoff = (3.184 * 0.887 * W_takeoff**(-0.113) * S_htail**0.101 * htail_AR**0.138)/(174.04 * t_rh**0.223) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg
		dWhtail_dShtail = (3.184 * W_takeoff**0.887 * 0.101 * S_htail**(-0.899) * htail_AR**0.138)/(174.04 * t_rh**0.223) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		partials['Weight|horizontal_tail', 'Weight|takeoff'] = tf * dWhtail_dWtakeoff
		partials['Weight|horizontal_tail', 'Htail|area'] = tf * dWhtail_dShtail


class VerticalTailWeightRoskam(om.ExplicitComponent):
	"""
	Computes vertical tail weight
	Parameters:
		vtail_AR				: horizontal tail aspect ratio (default=1.3)
		vtail_sweep				: vertical tail quarter chord sweep angle [deg]
		t_rv					: maximum root thickness [m]
		tf 						: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|takeoff 			: total take-off weight [kg]
		Vtail|area				: horizontal tail area [m**2]
	Outputs:
		Weight|vertical_tail 	: horizontal tail weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('vtail_AR', types=float, desc='Horizontal tail aspect ratio')
		self.options.declare('vtail_sweep', types=float, desc='Vertical tail quarter chord sweep angle')
		self.options.declare('t_rv', types=float, desc='Maximum root thickness')
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_input('Vtail|area', units='m**2', desc='Horizontal tail area')
		self.add_output('Weight|vertical_tail', units='kg', desc='Horizontal tail weight')
		self.declare_partials('Weight|vertical_tail', 'Weight|takeoff')
		self.declare_partials('Weight|vertical_tail', 'Vtail|area')

	def compute(self, inputs, outputs):
		vtail_AR = self.options['vtail_AR']
		vtail_sweep = self.options['vtail_sweep'] 	# in [deg]
		vtail_sweep *= np.pi/180 					# in [rad] 
		t_rv = self.options['t_rv']					# in [m]
		W_takeoff = inputs['Weight|takeoff']		# in [kg]
		S_vtail = inputs['Vtail|area']			# in [m**2]
		tf = self.options['tf']

		# Calculating W_vtail
		kg_to_lb = 2.20462**0.567
		m_to_ft = 3.28084**0.747
		m2_to_ft2 = (3.28084*3.28084)**1.249
		lb_to_kg = 0.453592
		
		W_vtail = (1.68 * W_takeoff**0.567 * S_vtail**1.249 * vtail_AR**0.482)/(639.95 * t_rv**0.747 * (np.cos(vtail_sweep))**0.882 ) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		outputs['Weight|vertical_tail'] = tf * W_vtail # in [kg]

	def compute_partials(self, inputs, partials):
		vtail_AR = self.options['vtail_AR']
		vtail_sweep = self.options['vtail_sweep'] 	# in [deg]
		vtail_sweep *= np.pi/180 					# in [rad] 
		t_rv = self.options['t_rv']					# in [m]
		W_takeoff = inputs['Weight|takeoff']		# in [kg]
		S_vtail = inputs['Vtail|area']			# in [m**2]
		tf = self.options['tf']

		# Calculating W_vtail
		kg_to_lb = 2.20462**0.567
		m_to_ft = 3.28084**0.747
		m2_to_ft2 = (3.28084*3.28084)**1.249
		lb_to_kg = 0.453592
		
		dWvtail_dWtakeoff = (1.68 * 0.567 * W_takeoff**(-0.433) * S_vtail**1.249 * vtail_AR**0.482)/(639.95 * t_rv**0.747 * (np.cos(vtail_sweep))**0.882 ) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg
		dWvtail_dSvtail = (1.68 * W_takeoff**0.567 * 1.249 * S_vtail**0.249 * vtail_AR**0.482)/(639.95 * t_rv**0.747 * (np.cos(vtail_sweep))**0.882 ) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		partials['Weight|vertical_tail', 'Weight|takeoff'] = tf * dWvtail_dWtakeoff
		partials['Weight|vertical_tail', 'Vtail|area'] = tf * dWvtail_dSvtail

class EmpennageWeightM4ModelsForNASALPC(om.ExplicitComponent):
	"""
	Computes HTail and VTail weights using M4 structural weight regression models for NACA LPC
	Parameters:
		tf 					: technology factor (default=1.0)
	Inputs:
		HTail|area 			: horizontal tail area [m**2]
		Vtail|area 			: vertical tail area [m**2]
	Outputs:
		Weight|empennage 	: empennage (HTail and VTail) weight [kg]
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
		self.add_input('HTail|area', units='m**2', desc='Horizontal tail area')
		self.add_input('VTail|area', units='m**2', desc='Vertical tail area')
		self.add_output('Weight|empennage', units='kg', desc='Empennage weight')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		S_htail = inputs['HTail|area']
		S_vtail = inputs['VTail|area']
		coeffs = [ 8.43266623, 10.05410839, -0.19944806479469435 ]
		W_wing = coeffs[0]*S_htail + coeffs[1]*S_vtail + coeffs[2]

		outputs['Weight|empennage'] = tf * W_wing

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		S_htail = inputs['HTail|area']
		S_vtail = inputs['VTail|area']
		coeffs = [ 8.43266623, 10.05410839, -0.19944806479469435 ]

		partials['Weight|empennage', 'S_htail'] = tf * coeffs[0]
		partials['Weight|empennage', 'S_vtail'] = tf * coeffs[1]
		
if __name__ == '__main__':
	
	model = om.Group()
	model.add_subsystem('m4_empennage_weight', EmpennageWeightM4ModelsForNASALPC())

	prob = om.Problem(model, reports=False)
	prob.setup()

	prob.set_val('m4_empennage_weight.HTail|area', 3.6707105940480003)
	prob.set_val('m4_empennage_weight.VTail|area', 2.54027104848)

	prob.run_model()
	print(prob['m4_empennage_weight.Weight|empennage'])






















