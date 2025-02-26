from MCEVS.Vehicles.Container import MultirotorEVTOL, LiftPlusCruiseEVTOL
from MCEVS.Analyses.Aerodynamics.Parasite import calc_flat_plate_drag
from MCEVS.Constants.Container import EarthGravityAndAtmosphere
import numpy as np

estimate_aero1 = False
estimate_aero2 = True

# Constants
# constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=6025*0.3048)
constants = EarthGravityAndAtmosphere('US_Standard_1976').compute_constants(altitude=10000*0.3048)

if estimate_aero1:
	vehicle1 = MultirotorEVTOL()
	vehicle1.add_component(kind='battery', density=400.0, efficiency=1.0, max_discharge=1.0)
	vehicle1.add_component(kind='fuselage', length=21.0*0.3048, max_diameter=6.7455*0.3048, number_of_passenger=6, technology_factor=0.8)
	vehicle1.add_component(kind='landing_gear', gear_type='skid', skid_length=9.36364*0.3048, skid_height=6.0*0.3048)
	t_per_c_1		= (0.2+0.2+0.36)/3
	t_per_c_2		= (0.2+0.2+0.36)/3
	t_per_c_3		= (0.27273+0.27273+0.2+0.32)/4
	t_per_c_4		= (0.27273+0.27273+0.2+0.32)/4
	t_per_c_list 	= [t_per_c_1, t_per_c_2, t_per_c_3, t_per_c_4]
	span1 			= (6.46552+5.97448)*0.3048
	span2 			= (2.49015+2.02370+8.10000)*0.3048
	span_list 		= [span1, span1, span2, span2]
	sweep_list 		= [-45.0, -45.0, (46.68595+46.68595+40.0)/3, (46.68595+46.68595+40.0)/3]
	vehicle1.add_component(kind='boom', represented_as='wing', thickness_to_chord_ratio=t_per_c_list,
						  number_of_booms=4, span_list=span_list, sweep_list=sweep_list)
	r_lift_rotor 	= 13.1 * 0.3048
	# r_lift_rotor 	= 18.318/2 * 0.3048
	r_to_R_list 	= [0.175, 0.285, 0.395, 0.505, 0.615, 0.725, 0.835, 0.945]
	c_to_R_list 	= [0.071475, 0.069425, 0.067375, 0.065325, 0.063275, 0.061225, 0.059175, 0.057125]
	pitch_list 		= 20.0 + np.array([6.90, 5.58, 4.26, 2.94, 1.62, 0.30, -1.02, -2.34])
	c_lift_rotor 	= np.sum(c_to_R_list)/8 * r_lift_rotor
	vehicle1.add_component(kind='lift_rotor', n_rotor=4, n_blade=3, n_section=8, airfoil='BOEING_VERTOL_VR12',
						  radius=r_lift_rotor, hub_radius=0.12*r_lift_rotor, chord=c_lift_rotor,
						  r_to_R_list=r_to_R_list, c_to_R_list=c_to_R_list, pitch_list=pitch_list,
						  figure_of_merit=0.75, technology_factor=0.8)	
	vehicle1.add_component(kind='landing_gear', gear_type='skid', skid_length=9.36364*0.3048, skid_heights=[2.46136*0.3048, 2.78871*0.3048])
	# vehicle1.print_info()

	Cd0_total1 = calc_flat_plate_drag(vehicle1, constants['rho'], constants['mu'], 50.4156)[0]/(np.pi*r_lift_rotor**2)
	print(f'Cd0_total vehicle1 = {Cd0_total1}')
	print(f"Total parasite drag vehicle1 = {0.5*constants['rho']*50.4156**2*Cd0_total1*np.pi*r_lift_rotor**2}")

if estimate_aero2:
	vehicle2 = LiftPlusCruiseEVTOL()
	vehicle2.add_component(kind='battery', density=400.0, efficiency=1.0, max_discharge=1.0)
	vehicle2.add_component(kind='fuselage', length=30.0*0.3048, max_diameter=6.12707*0.3048, number_of_passenger=6, technology_factor=0.8)
	vehicle2.add_component(kind='wing', area=210.27814*0.3048**2, aspect_ratio=12.12761, thickness_to_chord_ratio=0.16998, ultimate_load_factor=3.0, technology_factor=0.8)
	vehicle2.add_component(kind='horizontal_tail', area=39.51120*0.3048**2, aspect_ratio=4.30363, taper_ratio=0.6, max_root_thickness=0.15*3.78750*0.3048, thickness_to_chord_ratio=0.15, technology_factor=0.8)
	vehicle2.add_component(kind='vertical_tail', area=27.34325*0.3048**2, aspect_ratio=1.17990, max_root_thickness=0.11500*9.73719*0.3048, sweep_angle=35, thickness_to_chord_ratio=(0.115+0.14+0.15+0.15+0.115)/5, technology_factor=0.8)
	vehicle2.add_component(kind='landing_gear', gear_type='wheeled', strut_length=0.3048, ultimate_load_factor=5.7, technology_factor=0.8)
	
	r_lift_rotor 	= 5.0*0.3048
	r_to_R_list1 	= [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
	c_to_R_list1 	= [0.548288, 0.530863, 0.513438, 0.496013, 0.478588, 0.461163, 0.443738, 0.426313]
	pitch_list1		= 20.0 + np.array([7.359375, 5.878125, 4.396875, 2.915625, 1.434375, -0.046875, -1.528125, -3.009375])
	c_lift_rotor 	= np.sum(c_to_R_list1)/8 * r_lift_rotor
	
	r_propeller 	= 4.5*0.3048
	r_to_R_list2 	= [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
	c_to_R_list2 	= [0.500656, 0.484769, 0.468881, 0.452994, 0.437106, 0.421219, 0.405331, 0.389444]
	pitch_list2		= 20.0 + np.array([17.591875, 14.135625, 10.679375, 7.223125, 3.766875, 0.310625, -3.145625, -6.601875])
	c_propeller 	= np.sum(c_to_R_list2)/8 * r_propeller

	vehicle2.add_component(kind='lift_rotor', n_rotor=8, n_blade=2, n_section=8, airfoil='BOEING_VERTOL_VR12', radius=r_lift_rotor, hub_radius=0.21*r_lift_rotor, chord=c_lift_rotor, r_to_R_list=r_to_R_list1, c_to_R_list=c_to_R_list1, pitch_list=pitch_list1, hub_length=0.3048, hub_max_diameter=2.25*0.3048, figure_of_merit=0.75, technology_factor=0.8)
	vehicle2.add_component(kind='propeller', n_propeller=1, n_blade=6, n_section=8, airfoil='BOEING_VERTOL_VR12', radius=r_propeller, hub_radius=0.21*r_propeller, chord=c_propeller, r_to_R_list=r_to_R_list2, c_to_R_list=c_to_R_list2, pitch_list=pitch_list2, hub_length=0.3048, hub_max_diameter=1.5*0.3048, figure_of_merit=0.75, technology_factor=0.8)

	vehicle2.add_component(kind='boom', represented_as='fuselage', length=16.0*0.3048, max_diameter=0.3048, number_of_booms=4)
	# vehicle2.print_info()

	Cd0_total2 = calc_flat_plate_drag(vehicle2, constants['rho'], constants['mu'], 57.6178)[0]/(210.27814*0.3048**2)
	print(f'Cd0_total vehicle2 = {Cd0_total2}')
	print(f"Total parasite drag vehicle2 = {0.5*constants['rho']*57.6178**2*Cd0_total2*(210.27814*0.3048**2)}")
