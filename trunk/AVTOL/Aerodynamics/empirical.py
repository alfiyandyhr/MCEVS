import numpy as np
import openmdao.api as om

class CylindricalBodyDrag(om.ExplicitComponent):
	"""
	Computes the drag of a cylindrical body
	Parameters:
		rho_air				: air density [kg/m**3]
	Inputs:
		Rotor|radius		: rotor radius [m]
		eVTOL|Cruise_speed 	: cruising speed of the eVTOL [m/s]
		Body|sin_beta 		: body/fuselage tilt angle
	Outputs:
		Aero|Drag 			: aerodynamic drag [N]
	Notes:
		> Assumes that the body/fuselage is a cylinder whose radius is 58% of rotor radius
		> The length-to-diameter ratio is assumed to be 2.5
	Source:
		B. Govindarajan and A. Sridharan, “Conceptual Sizing of Vertical Lift Package Delivery Platforms,”
		Journal of Aircraft, vol. 57, no. 6, pp. 1170–1188, Nov. 2020, doi: 10.2514/1.C035805.
	"""
	def initialize(self):
		self.options.declare('rho_air', types=float, default=1.225, desc='Air density')
		
	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('eVTOL|Cruise_speed', units='m/s', desc='Cruise speed')
		self.add_input('Body|sin_beta', desc='sin(beta), beta: incidence angle of body')
		self.add_output('Aero|Drag', units='N', desc='Drag of the body')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		rho_air = self.options['rho_air']
		r = inputs['Rotor|radius']
		v = inputs['eVTOL|Cruise_speed']
		sin_beta = inputs['Body|sin_beta']

		S_body = 1.682 * r**2 				# body area
		CD_body = 0.1 + 0.2 * sin_beta**3 	# body drag coefficient

		outputs['Aero|Drag'] = 0.5 * rho_air * v * v * S_body * CD_body

	def compute_partials(self, inputs, partials):
		rho_air = self.options['rho_air']
		r = inputs['Rotor|radius']
		v = inputs['eVTOL|Cruise_speed']
		sin_beta = inputs['Body|sin_beta']

		S_body = 1.682 * r**2 				# body area
		dS_dr = 1.682 * 2 * r
		CD_body = 0.1 + 0.2 * sin_beta**3 	# body drag coefficient
		dCD_dsinB = 0.2 * 3 * sin_beta**2

		partials['Aero|Drag', 'Rotor|radius'] = 0.5 * rho_air * v * v * CD_body * dS_dr
		partials['Aero|Drag', 'eVTOL|Cruise_speed'] = rho_air * v * S_body * CD_body
		partials['Aero|Drag', 'Body|sin_beta'] = 0.5 * rho_air * v * v * S_body * dCD_dsinB

class MultirotorParasiteDragCoeff(om.ExplicitComponent):
	"""
	Computes the drag of a multirotor body
	Parameters:
		N_rotor				: number or rotors
		rho_air				: air density [kg/m**3]
	Inputs:
		eVTOL|W_takeoff 	: total take-off weight [kg]
		eVTOL|Cruise_speed 	: cruising speed of the eVTOL [m/s]
		Rotor|radius		: rotor radius [m]
	Outputs:
		Aero|Drag 			: aerodynamic drag [N]
	Notes:
		> Based on an empirical equivalent flat plat drag area
		> As a function of gross weight
	Source:
		1. Russell, C., et al., “Wind Tunnel and Hover Performance Test Results for Multicopter UAS Vehicles,”
		   Report TM-2018-219758, NASA, February 2018.
		2. A. R. Kadhiresan and M. J. Duffy, “Conceptual Design and Mission Analysis for eVTOL Urban Air Mobility Flight Vehicle Configurations,”
		   in AIAA Aviation 2019 Forum, Dallas, Texas: American Institute of Aeronautics and Astronautics, Jun. 2019. doi: 10.2514/6.2019-2873.
	"""
	def initialize(self):
		self.options.declare('N_rotor', types=int, desc='Number of rotors')
		self.options.declare('rho_air', types=float, default=1.225, desc='Air density')
		
	def setup(self):
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_input('eVTOL|Cruise_speed', units='m/s', desc='Cruise speed')
		self.add_output('Aero|Drag', units='N', desc='Drag of a multirotor body')
		self.declare_partials('*', '*')

	def compute(self, inputs, outputs):
		rho_air = self.options['rho_air']
		N_rotor = self.options['N_rotor']
		r = inputs['Rotor|radius']				# in [m]
		W_takeoff = inputs['eVTOL|W_takeoff']	# in [kg]
		v = inputs['eVTOL|Cruise_speed']

		# Equivalent flat plate area "f"
		kg_to_lb = 2.20462**0.8903
		ft2_to_m2 = 0.3048*0.3048

		f = 0.0327 * W_takeoff**0.8903 * kg_to_lb * ft2_to_m2

		# Parasite drag coefficient
		S_ref = N_rotor * np.pi * r**2 # total rotor area
		CD0 = f/S_ref

		outputs['Aero|Drag'] = 0.5 * rho_air * v * v * S_ref * CD0

	def compute_partials(self, inputs, partials):
		rho_air = self.options['rho_air']
		N_rotor = self.options['N_rotor']
		r = inputs['Rotor|radius']				# in [m]
		W_takeoff = inputs['eVTOL|W_takeoff']	# in [kg]
		v = inputs['eVTOL|Cruise_speed']		# in [m/s**2]

		# Equivalent flat plate area "f"
		kg_to_lb = 2.20462**0.8903
		ft2_to_m2 = 0.3048*0.3048

		f = 0.0327 * W_takeoff**0.8903 * kg_to_lb * ft2_to_m2
		df_dW = 0.0327 * 0.8903 * W_takeoff**(-0.1097) * kg_to_lb * ft2_to_m2

		partials['Aero|Drag', 'Rotor|radius'] = 0.0
		partials['Aero|Drag', 'eVTOL|W_takeoff'] = 0.5 * rho_air * v**2 * df_dW
		partials['Aero|Drag', 'eVTOL|Cruise_speed'] = rho_air * v * f

















