import numpy as np
import openmdao.api as om

class HorizontalTailWeight(om.ExplicitComponent):
	"""
	Computes horizontal tail weight
	Parameters:
		htail_AR	: horizontal tail aspect ratio (default=2.0)
		t_rh		: maximum root thickness [m]
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
		eVTOL|S_htail	: horizontal tail area [m**2]
	Outputs:
		Weights|Htail : horizontal tail weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> This method does not account for horizontal tail sweep
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('htail_AR', types=float, default=2.0, desc='Horizontal tail aspect ratio')
		self.options.declare('t_rh', types=float, units='m', desc='Maximum root thickness')

	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_input('eVTOL|S_htail', units='m**2', desc='Horizontal tail area')
		self.add_output('Weights|Htail', units='kg', desc='Horizontal tail weight')
		self.declare_partials('Weights|Htail', 'eVTOL|W_takeoff_')
		self.declare_partials('Weights|Htail', 'eVTOL|S_htail_')

	def compute(self, inputs, outputs):
		htail_AR = self.options['htail_AR']
		t_rh = self.options['t_rh']				# in [m]
		W_takeoff = inputs['eVTOL|W_takeoff']	# in [kg]
		S_htail = inputs['eVTOL|S_htail']		# in [m**2]

		# Calculating W_htail
		kg_to_lb = 2.20462**0.887
		m_to_ft = 3.28084**0.223
		m2_to_ft2 = (3.28084*3.28084)**0.101
		lb_to_kg = 0.453592
		
		W_htail = (3.184 * W_takeoff**0.887 * S_htail**0.101 * htail_AR**0.138)/(174.04 * t_rh**0.223) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		outputs['Weights|Htail'] = W_htail # in [kg]

	def compute_partials(self, inputs, partials):
		htail_AR = self.options['htail_AR']
		t_rh = self.options['t_rh']				# in [m]
		W_takeoff = inputs['eVTOL|W_takeoff']	# in [kg]
		S_htail = inputs['eVTOL|S_htail']		# in [m**2]

		# Calculating dWhtail_dWtakeoff and dWhtail_dShtail
		kg_to_lb = 2.20462**0.887
		m_to_ft = 3.28084**0.223
		m2_to_ft2 = (3.28084*3.28084)**0.101
		lb_to_kg = 0.453592
		
		dWhtail_dWtakeoff = (3.184 * 0.887 * W_takeoff**(-0.113) * S_htail**0.101 * htail_AR**0.138)/(174.04 * t_rh**0.223) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg
		dWhtail_dShtail = (3.184 * W_takeoff**0.887 * 0.101 * S_htail**(-0.899) * htail_AR**0.138)/(174.04 * t_rh**0.223) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		partials['Weights|Htail', 'eVTOL|W_takeoff'] = dWhtail_dWtakeoff
		partials['Weights|Htail', 'eVTOL|S_htail'] = dWhtail_dShtail


class VerticalTailWeight(om.ExplicitComponent):
	"""
	Computes vertical tail weight
	Parameters:
		vtail_AR	: horizontal tail aspect ratio (default=1.3)
		vtail_sweep	: vertical tail quarter chord sweep angle [deg]
		t_rv		: maximum root thickness [m]
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
		eVTOL|S_vtail	: horizontal tail area [m**2]
	Outputs:
		Weights|Vtail : horizontal tail weight [kg]
	Notes:
		> Class II Cessna method for General Aviation airplanes
		> Used for small, relatively low performance type airplanes
		> Maximum cruise speed <= 200 kts (320 kph, Mach 0.3)
	Source:
		Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
	"""
	def initialize(self):
		self.options.declare('vtail_AR', types=float, default=1.3, desc='Horizontal tail aspect ratio')
		self.options.declare('vtail_sweep', types=float, desc='Vertical tail quarter chord sweep angle')
		self.options.declare('t_rv', types=float, units='m', desc='Maximum root thickness')

	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_input('eVTOL|S_vtail', units='m**2', desc='Horizontal tail area')
		self.add_output('Weights|Vtail', units='kg', desc='Horizontal tail weight')
		self.declare_partials('Weights|Vtail', 'eVTOL|W_takeoff_')
		self.declare_partials('Weights|Vtail', 'eVTOL|S_vtail_')

	def compute(self, inputs, outputs):
		vtail_AR = self.options['vtail_AR']
		vtail_sweep = self.options['vtail_sweep'] 	# in [deg]
		vtail_sweep *= np.pi/180 					# in [rad] 
		t_rv = self.options['t_rv']					# in [m]
		W_takeoff = inputs['eVTOL|W_takeoff']		# in [kg]
		S_vtail = inputs['eVTOL|S_vtail']			# in [m**2]

		# Calculating W_vtail
		kg_to_lb = 2.20462**0.567
		m_to_ft = 3.28084**0.747
		m2_to_ft2 = (3.28084*3.28084)**1.249
		lb_to_kg = 0.453592
		
		W_vtail = (1.68 * W_takeoff**0.567 * S_vtail**1.249 * vtail_AR**0.482)/(639.95 * t_rv**0.747) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		outputs['Weights|Vtail'] = W_vtail # in [kg]

	def compute_partials(self, inputs, partials):
		vtail_AR = self.options['vtail_AR']
		vtail_sweep = self.options['vtail_sweep'] 	# in [deg]
		vtail_sweep *= np.pi/180 					# in [rad] 
		t_rv = self.options['t_rv']					# in [m]
		W_takeoff = inputs['eVTOL|W_takeoff']		# in [kg]
		S_vtail = inputs['eVTOL|S_vtail']			# in [m**2]

		# Calculating W_vtail
		kg_to_lb = 2.20462**0.567
		m_to_ft = 3.28084**0.747
		m2_to_ft2 = (3.28084*3.28084)**1.249
		lb_to_kg = 0.453592
		
		dWvtail_dWtakeoff = (1.68 * 0.567 * W_takeoff**(-0.433) * S_vtail**1.249 * vtail_AR**0.482)/(639.95 * t_rv**0.747) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg
		dWvtail_dSvtail = (1.68 * W_takeoff**0.567 * 1.249 * S_vtail**0.249 * vtail_AR**0.482)/(639.95 * t_rv**0.747) * kg_to_lb * m2_to_ft2 / m_to_ft * lb_to_kg

		partials['Weights|Vtail', 'eVTOL|W_takeoff'] = dWvtail_dWtakeoff
		partials['Weights|Vtail', 'eVTOL|S_vtail'] = dWvtail_dSvtail
























