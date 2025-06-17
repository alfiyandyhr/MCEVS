import numpy as np
import openmdao.api as om
import openvsp as vsp
from MCEVS.Analyses.Aerodynamics.Empirical import RotorHubParasiteDragFidelityZero
from MCEVS.Applications.OpenVSP.Utils import calc_wetted_area

class ParasiteDragViaDragBuildUpApproach(om.Group):
	"""
	Computes the parasite drag coefficient via a component build-up approach (fidelity one) and empirical rotor hub drag
	"""
	def initialize(self):
		self.options.declare('vehicle', types=object, desc='Vehicle object')
		self.options.declare('rho_air', types=float, desc='Air density')
		self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
		self.options.declare('segment_name', types=str, desc='Segment name')

	def setup(self):

		vehicle = self.options['vehicle']
		rho_air = self.options['rho_air']
		mu_air = self.options['mu_air']
		segment_name = self.options['segment_name']

		self.add_subsystem('parasite_drag_without_rotor_hub',
							ParasiteDragNonHubFidelityOne(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name=segment_name),
							promotes_inputs=['Aero|speed'] if vehicle.configuration == 'Multirotor' else ['Aero|speed', 'Wing|area'],
							promotes_outputs=['Aero|f_non_hub', 'Aero|parasite_drag_non_hub'] if vehicle.configuration == 'Multirotor' else ['Aero|f_non_hub', 'Aero|parasite_drag_non_hub', 'Aero|Cd0'])

		self.add_subsystem('parasite_drag_rotor_hub',
							RotorHubParasiteDragFidelityZero(vehicle=vehicle, rho_air=rho_air),
							promotes_inputs=['Weight|takeoff', 'Aero|speed'],
							promotes_outputs=['Aero|f_rotor_hub', 'Aero|parasite_drag_rotor_hub'])

		# Sum the vehicle parasite drag via build-up approach and empirical rotor hub drag
		adder = om.AddSubtractComp()

		adder.add_equation('Aero|f_total',
							units='m**2',
							input_names=['Aero|f_non_hub', 'Aero|f_rotor_hub'],
							scaling_factors=[1., 1.])
		adder.add_equation('Aero|parasite_drag',
							input_names=['Aero|parasite_drag_non_hub', 'Aero|parasite_drag_rotor_hub'],
							units='N',
							scaling_factors=[1., 1.])
		self.add_subsystem('total_parasite_drag',
							adder,
							promotes_inputs=['*'],
							promotes_outputs=['Aero|f_total', 'Aero|parasite_drag'])

class ParasiteDragNonHubFidelityOne(om.ExplicitComponent):
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
		if self.options['vehicle'].configuration == 'LiftPlusCruise':
			self.add_input('Wing|area', units='m**2', desc='Wing reference area')
			self.add_output('Aero|Cd0', units=None, desc='Parasite drag coefficient')
		self.add_output('Aero|f_non_hub', units='m**2', desc='Total quivalent flat plate area of vehicle without rotor hubs')
		self.add_output('Aero|parasite_drag_non_hub', units='N', desc='Parasite drag')
		self.declare_partials('*', '*', method='cs')

	def compute(self, inputs, outputs):
		vehicle = self.options['vehicle']
		rho_air = self.options['rho_air']
		mu_air = self.options['mu_air']
		segment_name = self.options['segment_name']
		v = inputs['Aero|speed']		# in [m/s**2]

		if vehicle.configuration == 'LiftPlusCruise':
			S_ref = inputs['Wing|area']	# in [m**2]

		if vehicle.configuration == 'Multirotor':
			if vehicle.f_total_non_hub[segment_name] is None:
				flat_plate_areas = calc_flat_plate_area(vehicle, rho_air, mu_air, v)
				f_total = flat_plate_areas['f_total']
				vehicle.fuselage.flat_plate_area[segment_name] = flat_plate_areas['f_fuselage']
				vehicle.boom.flat_plate_area[segment_name] = flat_plate_areas['f_booms']
				vehicle.landing_gear.flat_plate_area[segment_name] = flat_plate_areas['f_landing_gears']
				vehicle.f_total_non_hub[segment_name] = f_total
			else:
				f_fuselage = vehicle.fuselage.flat_plate_area[segment_name]
				f_booms = vehicle.boom.flat_plate_area[segment_name]
				f_landing_gears = vehicle.landing_gear.flat_plate_area[segment_name]
				f_total = vehicle.f_total_non_hub[segment_name]
				
		elif vehicle.configuration == 'LiftPlusCruise':
			if vehicle.f_total_non_hub_non_wing[segment_name] is None:
				flat_plate_areas = calc_flat_plate_area(vehicle, rho_air, mu_air, v)
				f_fuse = flat_plate_areas['f_fuselage']
				f_wing = flat_plate_areas['f_wing']
				f_htail = flat_plate_areas['f_horizontal_tail']
				f_vtail = flat_plate_areas['f_vertical_tail']
				f_booms = flat_plate_areas['f_booms']
				f_LGs = flat_plate_areas['f_landing_gears']
				f_total_non_hub_non_wing = f_fuse + f_htail + f_vtail + f_booms + f_LGs
				Cd0_wing = f_wing/S_ref
				vehicle.fuselage.flat_plate_area[segment_name] = f_fuse
				vehicle.wing.Cd0[segment_name] = Cd0_wing[0]
				vehicle.horizontal_tail.flat_plate_area[segment_name] = f_htail
				vehicle.vertical_tail.flat_plate_area[segment_name] = f_vtail
				vehicle.boom.flat_plate_area[segment_name] = f_booms
				vehicle.landing_gear.flat_plate_area[segment_name] = f_LGs
				vehicle.f_total_non_hub_non_wing[segment_name] = f_total_non_hub_non_wing
				f_total = f_total_non_hub_non_wing + Cd0_wing * S_ref
			else: 
				f_fuselage = vehicle.fuselage.flat_plate_area[segment_name]
				Cd0_wing = vehicle.wing.Cd0[segment_name]
				f_htail = vehicle.horizontal_tail.flat_plate_area[segment_name]
				f_vtail = vehicle.vertical_tail.flat_plate_area[segment_name]
				f_booms = vehicle.boom.flat_plate_area[segment_name]
				f_landing_gears = vehicle.landing_gear.flat_plate_area[segment_name]
				f_total = vehicle.f_total_non_hub_non_wing[segment_name] + Cd0_wing * S_ref

		outputs['Aero|f_non_hub'] = f_total
		outputs['Aero|parasite_drag_non_hub'] = 0.5 * rho_air * v * v * f_total

		if vehicle.configuration == 'LiftPlusCruise':
			outputs['Aero|Cd0'] = f_total / S_ref

def calc_flat_plate_area(vehicle:object, rho_air:float, mu_air:float, v_inf:float):
	"""
	Calculating parasite drag via a component build-up approach
	Component list for a Multirotor
		1. Fuselage
		2. Boom 1
		3. Boom 2
		4. Boom 3
		5. Boom 4
		6. Skid front 1
		7. Skid front 2
		8. Skid side 1
		9. Skid side 2
		10. Skid rear 1
		11. Skid rear 2
	Component list for a LiftPlusCruise
		1. Fuselage
		2. Wing
		3. H Tail
		4. V Tail
		5. Boom 1
		6. Boom 2
		7. Boom 3
		8. Boom 4
		9. Nose Strut LG
		10. Main Strut LG 1
		11. Main Strut LG 2
		12. Nose Wheel LG
		13. Main Wheel LG 1
		14. Main Wheel LG 2
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

		if vehicle.landing_gear.gear_type == 'wheeled':
			# Parasite drag of landing gear (strut and wheel)
			CD_pi = np.array([0.13, 0.13, 0.13, 0.13, 0.13, 0.13])
			S_front = np.array([0.0328496, 0.0328496, 0.0328496, 0.101213, 0.101213, 0.101213])
		elif vehicle.landing_gear.gear_type == 'skid':
			# Parasite drag of landing skid (front, side, rear)
			CD_pi = np.array([1.01, 1.01, 1.01, 1.01, 1.01, 1.01])
			S_front = np.array([0.0156419, 0.0156419, 0.00823901, 0.00823901, 0.0177222, 0.0177222])
		f_LG = CD_pi * S_front

		# Bookkeeping
		results = {}
		results['f_total'] = np.sum(f) + np.sum(f_LG)
		results['f_fuselage'] = f[0]
		results['f_booms'] = f[1] + f[2] + f[3] + f[4]
		results['f_landing_gears'] = np.sum(f_LG)

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
		S_front = np.array([0.0333752, 0.0333752, 0.0333752, 0.101213, 0.101213, 0.101213])
		f_LG = CD_pi * S_front

		# Bookkeeping
		results = {}
		results['f_total'] = np.sum(f) + np.sum(f_LG)
		results['f_fuselage'] = f[0]
		results['f_wing'] = f[1]
		results['f_horizontal_tail'] = f[2]
		results['f_vertical_tail'] = f[3]
		results['f_booms'] = f[4] + f[5] + f[6] + f[7]
		results['f_landing_gears'] = np.sum(f_LG)

	return results
