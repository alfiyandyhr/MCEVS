from MCEVS.Vehicles.Container import MultirotorEVTOL
from MCEVS.Vehicles.Container import LiftPlusCruiseEVTOL
import numpy as np

def StandardMultirotorEVTOL(design_var:dict,
							operation_var:{'RPM_lift_rotor': None},
							technology_factors={'tf_structure':1.0,
												'tf_propulsion':1.0,
												'tf_equipment':1.0},
							n_pax=6, payload_per_pax=100.0, mtow=None):

	r_lift_rotor 		= design_var['r_lift_rotor']
	r_to_R_list 		= [0.175, 0.285, 0.395, 0.505, 0.615, 0.725, 0.835, 0.945]
	c_to_R_list 		= [0.071475, 0.069425, 0.067375, 0.065325, 0.063275, 0.061225, 0.059175, 0.057125]
	w_to_R_list 		= [2*(0.175-0.12), 0.285-0.175, 0.395-0.285, 0.505-0.395, 0.615-0.505, 0.725-0.615, 0.835-0.725, 0.945-0.835]
	pitch_list 			= np.array([6.90, 5.58, 4.26, 2.94, 1.62, 0.30, -1.02, -2.34])
	pitch_linear_grad 	= -12.0
	mean_c_to_R 		= np.sum(c_to_R_list)/8
	t_per_c_1			= (0.2+0.2+0.36)/3
	t_per_c_2			= (0.2+0.2+0.36)/3
	t_per_c_3			= (0.27273+0.27273+0.2+0.32)/4
	t_per_c_4			= (0.27273+0.27273+0.2+0.32)/4
	t_per_c_list 		= [t_per_c_1, t_per_c_2, t_per_c_3, t_per_c_4]
	span1 				= (6.46552+5.97448)*0.3048
	span2 				= (2.49015+2.02370+8.10000)*0.3048
	span_list 			= [span1, span1, span2, span2]
	sweep_list 			= [-45.0, -45.0, (46.68595+46.68595+40.0)/3, (46.68595+46.68595+40.0)/3]

	RPM_lift_rotor 		= operation_var['RPM_lift_rotor'] if 'RPM_lift_rotor' in list(operation_var.keys()) else None

	vehicle = MultirotorEVTOL(tf_structure=technology_factors['tf_structure'],
							  tf_propulsion=technology_factors['tf_propulsion'],
							  tf_equipment=technology_factors['tf_equipment'],
							  mtow=mtow)

	vehicle.add_component(kind='battery', density=250, efficiency=0.85, max_discharge=0.8)

	if n_pax == 4:
		vehicle.add_component(kind='fuselage', length=5.2, max_diameter=1.8, number_of_passenger=n_pax, payload_per_pax=payload_per_pax)
		vehicle.add_component(kind='landing_gear', gear_type='wheeled', strut_length=0.3, ultimate_load_factor=5.7, technology_factor=0.8)
		vehicle.add_component(kind='boom', represented_as='wing', thickness_to_chord_ratio=t_per_c_list, number_of_booms=4, span_list=span_list, sweep_list=sweep_list)
		vehicle.add_component(kind='lift_rotor', n_rotor=4, n_blade=5, n_section=8, airfoil='BOEING_VERTOL_VR12',
							  radius=r_lift_rotor, hub_radius=0.12*r_lift_rotor, mean_c_to_R=mean_c_to_R,
							  r_to_R_list=r_to_R_list, c_to_R_list=c_to_R_list, w_to_R_list=w_to_R_list, pitch_list=pitch_list,
							  global_twist=4.1, Cd0=0.0089, figure_of_merit=0.75, RPM=RPM_lift_rotor, motor_power_margin=50.0)
	
	elif n_pax == 6:
		vehicle.add_component(kind='fuselage', length=21.0*0.3048, max_diameter=6.7455*0.3048, number_of_passenger=n_pax, payload_per_pax=payload_per_pax)
		vehicle.add_component(kind='landing_gear', gear_type='skid', skid_length=9.36364*0.3048, skid_heights=[2.46136*0.3048, 2.78871*0.3048])
		vehicle.add_component(kind='boom', represented_as='wing', thickness_to_chord_ratio=t_per_c_list, number_of_booms=4, span_list=span_list, sweep_list=sweep_list)
		vehicle.add_component(kind='lift_rotor', n_rotor=4, n_blade=3, n_section=8, airfoil='BOEING_VERTOL_VR12_Viterna_for_NASA_QR',
							  radius=r_lift_rotor, hub_radius=0.12*r_lift_rotor, mean_c_to_R=mean_c_to_R,
							  r_to_R_list=r_to_R_list, c_to_R_list=c_to_R_list, w_to_R_list=w_to_R_list, pitch_list=pitch_list,
							  global_twist=4.1, Cd0=0.0089, figure_of_merit=0.75, RPM=RPM_lift_rotor, motor_power_margin=50.0)

	return vehicle

def StandardLiftPlusCruiseEVTOL(design_var:dict,
								operation_var:{'RPM_lift_rotor':None, 'RPM_propeller':None},
								technology_factors={'tf_structure':1.0,
													'tf_propulsion':1.0,
													'tf_equipment':1.0},
								n_pax=6, payload_per_pax=100.0, mtow=None):

	r_lift_rotor 		= design_var['r_lift_rotor']
	r_to_R_list1 		= [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
	c_to_R_list1 		= [0.548288, 0.530863, 0.513438, 0.496013, 0.478588, 0.461163, 0.443738, 0.426313]
	w_to_R_list1 		= [2*(0.259375-0.21), 0.358125-0.259375, 0.456875-0.358125, 0.555625-0.456875, 0.654375-0.555625, 0.753125-0.654375, 0.851875-0.753125, 0.950625-0.851875]
	pitch_list1			= np.array([7.359375, 5.878125, 4.396875, 2.915625, 1.434375, -0.046875, -1.528125, -3.009375])
	pitch_linear_grad1 	= -15.0
	global_twist1 		= 15.0
	mean_c_to_R1 		= np.sum(c_to_R_list1)/8
	
	r_propeller 		= design_var['r_propeller']
	r_to_R_list2 		= [0.259375, 0.358125, 0.456875, 0.555625, 0.654375, 0.753125, 0.851875, 0.950625]
	c_to_R_list2 		= [0.500656, 0.484769, 0.468881, 0.452994, 0.437106, 0.421219, 0.405331, 0.389444]
	w_to_R_list2 		= [2*(0.259375-0.21), 0.358125-0.259375, 0.456875-0.358125, 0.555625-0.456875, 0.654375-0.555625, 0.753125-0.654375, 0.851875-0.753125, 0.950625-0.851875]
	pitch_list2			= np.array([17.591875, 14.135625, 10.679375, 7.223125, 3.766875, 0.310625, -3.145625, -6.601875])
	pitch_linear_grad2 	= -15.0
	global_twist2 		= 15.0
	mean_c_to_R2 		= np.sum(c_to_R_list2)/8

	wing_area 			= design_var['wing_area']
	wing_AR 			= design_var['wing_aspect_ratio']

	RPM_lift_rotor 		= operation_var['RPM_lift_rotor'] if 'RPM_lift_rotor' in list(operation_var.keys()) else None
	RPM_propeller 		= operation_var['RPM_propeller'] if 'RPM_propeller' in list(operation_var.keys()) else None

	vehicle = LiftPlusCruiseEVTOL(tf_structure=technology_factors['tf_structure'],
								  tf_propulsion=technology_factors['tf_propulsion'],
								  tf_equipment=technology_factors['tf_equipment'],
								  mtow=mtow)

	vehicle.add_component(kind='battery', density=250.0, efficiency=0.85, max_discharge=0.8)

	if n_pax == 4:
		l_fuse 		= 9.0
		l_boom  	= 1.6*8.0/30.0*l_fuse/1.6*r_lift_rotor if r_lift_rotor>=1.6 else 1.6*8.0/30.0*l_fuse
		d_boom 		= 1.0/30.0*l_fuse
		l_rotor_hub = 1.0/30.0*l_fuse
		d_rotor_hub = 2.25/10.0*r_lift_rotor
		l_prop_hub 	= 1.0/30.0*l_fuse
		d_prop_hub 	= 1.5/9.0*r_propeller
		
		vehicle.add_component(kind='fuselage', length=l_fuse, max_diameter=1.8, number_of_passenger=n_pax, payload_per_pax=payload_per_pax)
		vehicle.add_component(kind='wing', airfoil='LS417', area=wing_area, aspect_ratio=wing_AR, thickness_to_chord_ratio=0.16998, ultimate_load_factor=3.0)
		vehicle.add_component(kind='horizontal_tail', area=2.0, aspect_ratio=2.0, taper_ratio=0.6, max_root_thickness=0.15*1.25, thickness_to_chord_ratio=0.15)
		vehicle.add_component(kind='vertical_tail', area=2.5, aspect_ratio=1.2, max_root_thickness=0.12*2.92, sweep_angle=35, thickness_to_chord_ratio=0.136)
		vehicle.add_component(kind='landing_gear', gear_type='wheeled', strut_length=0.3, ultimate_load_factor=5.7)
		vehicle.add_component(kind='boom', represented_as='fuselage', length=l_boom, max_diameter=d_boom, number_of_booms=4)
		
		vehicle.add_component(kind='lift_rotor', n_rotor=8, n_blade=2, n_section=8, airfoil='BOEING_VERTOL_VR12',
							  radius=r_lift_rotor, hub_radius=0.21*r_lift_rotor, mean_c_to_R=mean_c_to_R1, r_to_R_list=r_to_R_list1, c_to_R_list=c_to_R_list1,
							  pitch_list=pitch_list1, global_twist=global_twist1, hub_length=l_rotor_hub, hub_max_diameter=d_rotor_hub,
							  figure_of_merit=0.75, RPM=RPM_lift_rotor, motor_power_margin=50.0)
		vehicle.add_component(kind='propeller', n_propeller=1, n_blade=4, n_section=8, airfoil='BOEING_VERTOL_VR12',
							  radius=r_propeller, hub_radius=0.21*r_propeller, mean_c_to_R=mean_c_to_R2, r_to_R_list=r_to_R_list2, c_to_R_list=c_to_R_list2,
							  pitch_list=pitch_list2, global_twist=global_twist2, hub_length=l_prop_hub, hub_max_diameter=d_prop_hub,
							  figure_of_merit=0.75, Cd0=0.0089, RPM=RPM_propeller, motor_power_margin=50.0)

	if n_pax == 6:
		vehicle.add_component(kind='fuselage', length=30.0*0.3048, max_diameter=6.12707*0.3048, number_of_passenger=n_pax, payload_per_pax=payload_per_pax)
		vehicle.add_component(kind='wing', airfoil='LS417', area=wing_area, aspect_ratio=wing_AR, thickness_to_chord_ratio=0.16998, ultimate_load_factor=3.0)
		vehicle.add_component(kind='horizontal_tail', area=39.51120*0.3048**2, aspect_ratio=4.30363, taper_ratio=0.6, max_root_thickness=0.15*3.78750*0.3048, thickness_to_chord_ratio=0.15)
		vehicle.add_component(kind='vertical_tail', area=27.34325*0.3048**2, aspect_ratio=1.17990, max_root_thickness=0.11500*9.73719*0.3048, sweep_angle=35, thickness_to_chord_ratio=(0.115+0.14+0.15+0.15+0.115)/5)
		vehicle.add_component(kind='landing_gear', gear_type='wheeled', strut_length=0.3048, ultimate_load_factor=5.7)
		vehicle.add_component(kind='boom', represented_as='fuselage', length=16.0*0.3048, max_diameter=0.3048, number_of_booms=4)

		vehicle.add_component(kind='lift_rotor', n_rotor=8, n_blade=2, n_section=8, airfoil='BOEING_VERTOL_VR12_Viterna_for_NASA_LPC',
							  radius=r_lift_rotor, hub_radius=0.21*r_lift_rotor, mean_c_to_R=mean_c_to_R1, r_to_R_list=r_to_R_list1, c_to_R_list=c_to_R_list1, w_to_R_list=w_to_R_list1,
							  pitch_list=pitch_list1, global_twist=global_twist1, hub_length=0.3048, hub_max_diameter=2.25*0.3048,
							  figure_of_merit=0.75, Cd0=0.0089, RPM=RPM_lift_rotor, motor_power_margin=50.0)
		vehicle.add_component(kind='propeller', n_propeller=1, n_blade=6, n_section=8, airfoil='BOEING_VERTOL_VR12_Viterna_for_NASA_LPC',
							  radius=r_propeller, hub_radius=0.21*r_propeller, mean_c_to_R=mean_c_to_R2, r_to_R_list=r_to_R_list2, c_to_R_list=c_to_R_list2, w_to_R_list=w_to_R_list2,
							  pitch_list=pitch_list2, global_twist=global_twist2, hub_length=0.3048, hub_max_diameter=1.5*0.3048,
							  figure_of_merit=0.75, Cd0=0.0089, RPM=RPM_propeller, motor_power_margin=50.0)

	return vehicle