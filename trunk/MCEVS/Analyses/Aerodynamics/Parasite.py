import numpy as np
import openvsp as vsp
from MCEVS.Applications.OpenVSP.Utils import calc_wetted_area

def calc_parasite_drag_coeff(vehicle:object, constant:object, v_inf:float):
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
	Re_L = constant['rho'] * v_inf / constant['mu']

	# Wetted area
	S_wetted = calc_wetted_area(vehicle)
	S_wetted = np.array(list(S_wetted.values()))[:-6]

	if vehicle.configuration == 'LiftPlusCruise':
		S_ref = vehicle.wing.area
		n_components = 4 + (2+vehicle.lift_rotor.n_blade) * vehicle.lift_rotor.n_rotor
		n_components += (1+vehicle.propeller.n_blade)*vehicle.propeller.n_propeller

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
		for j in range(vehicle.lift_rotor.n_rotor):
			L_ref[5+i+j] = vehicle.lift_rotor.hub_max_diameter
		for k in range(vehicle.lift_rotor.n_rotor*vehicle.lift_rotor.n_blade):
			L_ref[6+i+j+k] = vehicle.lift_rotor.radius
		for l in range(vehicle.propeller.n_propeller):
			L_ref[7+i+j+k+l] = vehicle.propeller.hub_length
		for m in range(vehicle.propeller.n_propeller*vehicle.propeller.n_blade):
			L_ref[8+i+j+k+l+m] = vehicle.propeller.radius

		# Reynold numbers
		Re = Re_L * L_ref

		# Coefficient of frictions Cf (Schlichting compressible)
		# Cf = Cf_100%turb - %lam * Cf_partialturb + %lam * Cf_partiallam
		# Cf_100%turb = f_turb(Re)
		# Cf_partialturb = f_turb(%lam*Re)
		# Cf_partiallam = f_lam(%lam*Re)
		# assumption: fully turbulent (i.e., %lam = 0)
		Cf = 0.455 / ((np.log10(Re))**2.58)

		# Interference factor Q (assumed = 1.0)
		Q = np.ones(n_components)

		# Flat plate drag
		f = S_wetted * Q * Cf * FF

		# Cd0 components
		Cd0 = f/S_ref
		
		# Parasite drag of landing gear (strut and wheel)
		CD_pi = np.array([0.05, 0.05, 0.05, 0.25, 0.25, 0.25])
		S_front = np.array([0.0328496, 0.0328496, 0.0328496, 0.101213, 0.101213, 0.101213])
		D_per_q = CD_pi * S_front
		Cd0_LG = D_per_q/S_ref

		# Cd0 total
		Cd0_total = np.sum(Cd0) + np.sum(Cd0_LG)

	return Cd0_total
