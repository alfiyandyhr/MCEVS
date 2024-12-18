import numpy as np
import openmdao.api as om
import openvsp as vsp
from MCEVS.Applications.OpenVSP.Utils import calc_wetted_area

class ParasiteDragFidelityOne(om.ExplicitComponent):
	"""
	Computes the parasite drag coefficient via a component build-up approach (fidelity one)
	Parameters:
		vehicle				: MCEVS vehicle object
		rho_air				: air density [kg/m**3]
		mu_air 				: air dynamic viscosity [Ns/m**2]
	Inputs:
		Aero|speed 			: air speed of the eVTOL [m/s]
		Rotor|radius		: rotor radius [m] 
		Wing|area 			: wing area [m**2]
	Outputs:
		Aero|Cd0			: parasite drag coefficient
		Aero|parasite_drag	: parasite drag [N]
	Notes:
		> 
	Source:
		1. https://openvsp.org/wiki/doku.php?id=parasitedrag
		2. Raymer, D. P. Aircraft Design: A Conceptual Approach. Reston, Virginia: American Institute of Aeronautics and Astronautics, Inc., 2006.
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
		self.options.declare('segment_name', types=str, desc='Segment name')

	def setup(self):
		self.add_input('Aero|speed', units='m/s', desc='Air speed')
		self.add_input('Rotor|radius', units='m', desc='Rotor radius')
		self.add_input('Wing|area', units='m**2', desc='Wing reference area')
		self.add_output('Aero|Cd0', desc='Parasite drag coefficient')
		self.add_output('Aero|parasite_drag', units='N', desc='Parasite drag of a winged config')
		self.declare_partials('*', '*', method='fd')

	def compute(self, inputs, outputs):
		vehicle = self.options['vehicle']
		rho_air = self.options['rho_air']
		mu_air = self.options['mu_air']
		segment_name = self.options['segment_name']
		v = inputs['Aero|speed']		# in [m/s**2]
		if vehicle.configuration == 'Multirotor':
			r_rotor = inputs['Rotor|radius']
			S_ref = np.pi*r_rotor**2
		elif vehicle.configuration == 'LiftPlusCruise':
			S_wing = inputs['Wing|area']	# in [m**2]
			S_ref = S_wing 	# ref area is the wing area

		if vehicle.Cd0[segment_name] is None:
			f = calc_flat_plate_drag(vehicle, rho_air, mu_air, v) # in [m**2]
			CD0 = f/S_ref
			vehicle.Cd0[segment_name] = CD0
		else:
			CD0 = vehicle.Cd0[segment_name]

		outputs['Aero|Cd0'] = CD0
		outputs['Aero|parasite_drag'] = 0.5 * rho_air * v * v * S_ref * CD0

	# def compute_partials(self, inputs, partials):
	# 	vehicle = self.options['vehicle']
	# 	rho_air = self.options['rho_air']
	# 	mu_air = self.options['mu_air']
	# 	v = inputs['Aero|speed']		# in [m/s**2]
	# 	S_wing = inputs['Wing|area']	# in [m**2]

	# 	f = calc_flat_plate_drag(vehicle, rho_air, mu_air, v) # in [m**2]
	# 	S_ref = S_wing
	# 	CD0 = f/S_ref

	# 	partials['Aero|Cd0', 'Aero|speed'] = 0
	# 	partials['Aero|Cd0', 'Wing|area'] = -f/(S_ref**2)
	# 	partials['Aero|parasite_drag', 'Aero|speed'] = rho_air * v * f
	# 	partials['Aero|parasite_drag', 'Wing|area'] = 0
		

def calc_flat_plate_drag(vehicle:object, rho_air:float, mu_air:float, v_inf:float):
	"""
	Calculating parasite drag via a component build-up approach
	Component list (e.g. for a LiftPlusCruise)
		1. Fuselage
		2. Wing
		3. H Tail
		4. V Tail
		5. Boom 1
		6. Boom 2
		7. Boom 3
		8. Boom 4
		9. Rotor Hub 1
		10. Rotor Hub 2
		11. Rotor Hub 3
		12. Rotor Hub 4
		13. Rotor Blade 1_1
		14. Rotor Blade 1_2
		15. Rotor Blade 2_1
		16. Rotor Blade 2_2
		17. Rotor Blade 3_1
		18. Rotor Blade 3_2
		19. Rotor Blade 4_1
		20. Rotor Blade 4_2
		21. Propeller Hub 1
		22. Propeller Blade 1_1
		23. Propeller Blade 1_2
		24. Propeller Blade 1_3
		25. Propeller Blade 1_4
		26. Nose Strut LG
		27. Main Strut LG 1
		28. Main Strut LG 2
		29. Nose Wheel LG
		30. Main Wheel LG 1
		31. Main Wheel LG 2
	"""
	# Flow condition
	Re_L = rho_air * v_inf / mu_air

	# Wetted area !!!expensive, evaluate once!!!
	if vehicle.S_wetted is None:
		S_wetted = calc_wetted_area(vehicle)
		S_wetted = np.array(list(S_wetted.values()))[:-6] # excluding lg struts and wheels
		vehicle.S_wetted = S_wetted
	else:
		S_wetted = vehicle.S_wetted

	if vehicle.configuration == 'Multirotor':
		n_components = 1 + int(vehicle.lift_rotor.n_rotor)

		# Fineness ratio (thickness to chord or length to diameter)
		FR = np.ones(n_components)
		FR[0] = vehicle.fuselage.fineness_ratio
		for i in range(vehicle.boom.number_of_booms):
			if len(vehicle.boom.thickness_to_chord_ratio) == 1:
				FR[1+i] = vehicle.boom.thickness_to_chord_ratio
			else:
				FR[1+i] = vehicle.boom.thickness_to_chord_ratio[i]

		# Form factors (Hoerner equations)
		FF = np.ones(n_components)
		FF[0] = 1.0 + 1.5/(FR[0]**1.5) + 7.0/(FR[0]**3)
		for i in range(vehicle.boom.number_of_booms):
			FF[1+i] = 1.0 + 2.0*FR[1+i] + 60*FR[1+i]**4

		# Reference lengths
		L_ref = np.ones(n_components)
		L_ref[0] = vehicle.fuselage.length
		for i in range(vehicle.boom.number_of_booms):
			L_ref[1+i] = np.sqrt(vehicle.boom.area[i]/vehicle.boom.aspect_ratio[i])

		# Reynold numbers
		Re = Re_L * L_ref

		# Coefficient of frictions Cf (Schlichting compressible)
		# Cf = Cf_100%turb - %lam * Cf_partialturb + %lam * Cf_partiallam
		# Cf_100%turb = f_turb(Re)
		# Cf_partialturb = f_turb(%lam*Re)
		# Cf_partiallam = f_lam(%lam*Re)
		# assumption: fully turbulent (i.e., %lam = 0)
		Cf = 0.455 / ((np.log10(Re))**2.58)

		# Interference factor Q (fuselage=1.0, wing=1.0, htail=1.08, vtail=1.03, boom=1.3)
		Q = 1.3 * np.ones(n_components)
		Q[0] = 1.0

		# print(S_wetted.shape, Q.shape, Cf.shape, FF.shape)
		# print(S_wetted, Q, Cf, FF)

		# Flat plate drag
		f = S_wetted * Q * Cf * FF
		
		# Parasite drag of landing gear (strut and wheel)
		CD_pi = np.array([0.13, 0.13, 0.13, 0.13, 0.13, 0.13])
		S_front = np.array([0.0328496, 0.0328496, 0.0328496, 0.101213, 0.101213, 0.101213])
		f_LG = CD_pi * S_front

		# f total
		f_total = np.sum(f) + np.sum(f_LG)

	elif vehicle.configuration == 'LiftPlusCruise':
		n_components = 4 + int(vehicle.lift_rotor.n_rotor/2)

		# Fineness ratio (thickness to chord or length to diameter)
		FR = np.ones(n_components)
		FR[0] = vehicle.fuselage.fineness_ratio
		FR[1] = vehicle.wing.thickness_to_chord_ratio
		FR[2] = vehicle.horizontal_tail.thickness_to_chord_ratio
		FR[3] = vehicle.vertical_tail.thickness_to_chord_ratio
		for i in range(vehicle.boom.number_of_booms):
			FR[4+i] = vehicle.boom.fineness_ratio

		# Form factors (Hoerner equations)
		FF = np.ones(n_components)
		FF[0] = 1.0 + 1.5/(FR[0]**1.5) + 7.0/(FR[0]**3)
		FF[1] = 1.0 + 2.0*FR[1] + 60*FR[1]**4
		FF[2] = 1.0 + 2.0*FR[2] + 60*FR[2]**4
		FF[3] = 1.0 + 2.0*FR[3] + 60*FR[3]**4
		for i in range(vehicle.boom.number_of_booms):
			FF[4+i] = 1.0 + 1.5/(FR[4+i]**1.5) + 7.0/(FR[4+i]**3)

		# Reference lengths
		L_ref = np.ones(n_components)
		L_ref[0] = vehicle.fuselage.length
		L_ref[1] = np.sqrt(vehicle.wing.area/vehicle.wing.aspect_ratio)
		L_ref[2] = np.sqrt(vehicle.horizontal_tail.area/vehicle.horizontal_tail.aspect_ratio)
		L_ref[3] = np.sqrt(vehicle.vertical_tail.area/vehicle.vertical_tail.aspect_ratio)
		for i in range(vehicle.boom.number_of_booms):
			L_ref[4+i] = vehicle.boom.length

		# Reynold numbers
		Re = Re_L * L_ref

		# Coefficient of frictions Cf (Schlichting compressible)
		# Cf = Cf_100%turb - %lam * Cf_partialturb + %lam * Cf_partiallam
		# Cf_100%turb = f_turb(Re)
		# Cf_partialturb = f_turb(%lam*Re)
		# Cf_partiallam = f_lam(%lam*Re)
		# assumption: fully turbulent (i.e., %lam = 0)
		Cf = 0.455 / ((np.log10(Re))**2.58)

		# Interference factor Q (fuselage=1.0, wing=1.0, htail=1.08, vtail=1.03, boom=1.3)
		Q = 1.3 * np.ones(n_components)
		Q[0] = 1.0
		Q[1] = 1.0
		Q[2] = 1.08
		Q[3] = 1.03

		# print(S_wetted.shape, Q.shape, Cf.shape, FF.shape)
		# print(S_wetted, Q, Cf, FF)
		# Flat plate drag
		f = S_wetted * Q * Cf * FF
		
		# Parasite drag of landing gear (strut and wheel)
		CD_pi = np.array([0.13, 0.13, 0.13, 0.13, 0.13, 0.13])
		S_front = np.array([0.0328496, 0.0328496, 0.0328496, 0.101213, 0.101213, 0.101213])
		f_LG = CD_pi * S_front

		# f total
		f_total = np.sum(f) + np.sum(f_LG)

	return f_total
