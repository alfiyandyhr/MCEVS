import numpy as np


def promote_indeps_var_comp(ivc, vehicle, mission, fidelity):

    # --- Design parameters --- #

    if vehicle.configuration == 'Multirotor':
        r_lift_rotor = vehicle.lift_rotor.radius 					            # m
        r_hub_lift_rotor = vehicle.lift_rotor.hub_radius 			            # m
        mean_c_to_R_lift_rotor = vehicle.lift_rotor.mean_c_to_R
        global_twist_lift_rotor = vehicle.lift_rotor.global_twist               # deg

    elif vehicle.configuration == 'LiftPlusCruise':
        r_lift_rotor = vehicle.lift_rotor.radius 					            # m
        r_hub_lift_rotor = vehicle.lift_rotor.hub_radius 			            # m
        mean_c_to_R_lift_rotor = vehicle.lift_rotor.mean_c_to_R
        global_twist_lift_rotor = vehicle.lift_rotor.global_twist               # deg
        if vehicle.lift_rotor.clearance['type'] == 3:
            s_inner_lift_rotor = vehicle.lift_rotor.clearance['s_inner']
            s_outer_lift_rotor = vehicle.lift_rotor.clearance['s_outer']
        r_propeller = vehicle.propeller.radius 					                # m
        mean_c_to_R_propeller = vehicle.propeller.mean_c_to_R
        wing_area = vehicle.wing.area 							                # m**2
        wing_aspect_ratio = vehicle.wing.aspect_ratio
        wing_airfoil_CL_alpha = vehicle.wing.airfoil.CL_alpha 		            # 1/rad
        wing_airfoil_CL_0 = vehicle.wing.airfoil.CL_0
        htail_area = vehicle.horizontal_tail.area 				                # m**2
        htail_aspect_ratio = vehicle.horizontal_tail.aspect_ratio
        htail_max_root_thickness = vehicle.horizontal_tail.max_root_thickness 	# m
        vtail_area = vehicle.vertical_tail.area 					            # m**2
        vtail_aspect_ratio = vehicle.vertical_tail.aspect_ratio
        vtail_max_root_thickness = vehicle.vertical_tail.max_root_thickness 	# m
        vtail_sweep_angle = vehicle.vertical_tail.sweep_angle 			        # deg
        l_fuse = vehicle.fuselage.length 						                # m

    if vehicle.configuration == 'Multirotor':
        ivc.add_output('LiftRotor|radius', r_lift_rotor, units='m')
        ivc.add_output('LiftRotor|mean_c_to_R', mean_c_to_R_lift_rotor, units=None)
        ivc.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
        ivc.add_output('LiftRotor|global_twist', global_twist_lift_rotor, units='deg')

    elif vehicle.configuration == 'LiftPlusCruise':
        ivc.add_output('LiftRotor|radius', r_lift_rotor, units='m')
        ivc.add_output('LiftRotor|mean_c_to_R', mean_c_to_R_lift_rotor, units=None)
        ivc.add_output('LiftRotor|hub_radius', r_hub_lift_rotor, units='m')
        ivc.add_output('LiftRotor|global_twist', global_twist_lift_rotor, units='deg')
        if vehicle.lift_rotor.clearance['type'] == 3:
            ivc.add_output('LiftRotor|s_inner', s_inner_lift_rotor, units=None)
            ivc.add_output('LiftRotor|s_outer', s_outer_lift_rotor, units=None)
        ivc.add_output('Propeller|radius', r_propeller, units='m')
        ivc.add_output('Propeller|mean_c_to_R', mean_c_to_R_propeller, units=None)
        ivc.add_output('Wing|area', wing_area, units='m**2')
        ivc.add_output('Wing|aspect_ratio', wing_aspect_ratio)
        ivc.add_output('Wing|airfoil|CL_alpha', wing_airfoil_CL_alpha, units='1/rad')
        ivc.add_output('Wing|airfoil|CL_0', wing_airfoil_CL_0)
        ivc.add_output('HorizontalTail|area', htail_area, units='m**2')
        ivc.add_output('HorizontalTail|aspect_ratio', htail_aspect_ratio)
        ivc.add_output('HorizontalTail|max_root_thickness', htail_max_root_thickness, units='m')
        ivc.add_output('VerticalTail|area', vtail_area, units='m**2')
        ivc.add_output('VerticalTail|aspect_ratio', vtail_aspect_ratio)
        ivc.add_output('VerticalTail|max_root_thickness', vtail_max_root_thickness, units='m')
        ivc.add_output('VerticalTail|sweep_angle', vtail_sweep_angle, units='deg')
        ivc.add_output('Fuselage|length', l_fuse, units='m')

    # --- Variables needed for BEMT --- #

    if fidelity['power_model']['hover_climb'] == 'BladeElementMomentumTheory':
        n_sections = vehicle.lift_rotor.n_section
        r_to_R_list = vehicle.lift_rotor.r_to_R_list
        c_to_R_list = vehicle.lift_rotor.c_to_R_list
        w_to_R_list = vehicle.lift_rotor.w_to_R_list
        if vehicle.lift_rotor.pitch_linear_grad is not None:
            ivc.add_output('LiftRotor|pitch_linear_grad', vehicle.lift_rotor.pitch_linear_grad, units='deg')
        else:
            pitch_list = np.array(vehicle.lift_rotor.pitch_list)
            for i in range(n_sections):
                ivc.add_output(f'LiftRotor|Section{i+1}|pitch', pitch_list[i], units='deg')
        for i in range(n_sections):
            ivc.add_output(f'LiftRotor|Section{i+1}|r_to_R', r_to_R_list[i], units=None)
            ivc.add_output(f'LiftRotor|Section{i+1}|c_to_R', c_to_R_list[i], units=None)
            ivc.add_output(f'LiftRotor|Section{i+1}|w_to_R', w_to_R_list[i], units=None)

    # --- Mission parameters --- #

    for segment in mission.segments:
        if segment.kind not in ['ConstantPower', 'NoCreditClimb', 'NoCreditDescent', 'ReserveCruise', 'HoverStay']:
            ivc.add_output(f'Mission|segment_{segment.id}|speed', segment.speed, units='m/s')
            ivc.add_output(f'Mission|segment_{segment.id}|distance', segment.distance, units='m')
        if segment.kind == 'HoverClimbConstantSpeed':
            ivc.add_output('LiftRotor|HoverClimb|RPM', vehicle.lift_rotor.RPM['hover_climb'], units='rpm')
        if segment.kind == 'CruiseConstantSpeed':
            if vehicle.configuration == 'Multirotor':
                ivc.add_output('LiftRotor|Cruise|RPM', vehicle.lift_rotor.RPM['cruise'], units='rpm')
            elif vehicle.configuration == 'LiftPlusCruise':
                ivc.add_output('Propeller|Cruise|RPM', vehicle.propeller.RPM['cruise'], units='rpm')
        if segment.kind == 'ClimbConstantVyConstantVx':
            if vehicle.configuration == 'Multirotor':
                ivc.add_output('LiftRotor|Climb|RPM', vehicle.lift_rotor.RPM['climb'], units='rpm')
            elif vehicle.configuration == 'LiftPlusCruise':
                ivc.add_output('Propeller|Climb|RPM', vehicle.propeller.RPM['climb'], units='rpm')
        if segment.kind == 'DescentConstantVyConstantVx':
            if vehicle.configuration == 'Multirotor':
                ivc.add_output('LiftRotor|Descent|RPM', vehicle.lift_rotor.RPM['descent'], units='rpm')
            elif vehicle.configuration == 'LiftPlusCruise':
                ivc.add_output('Propeller|Descent|RPM', vehicle.propeller.RPM['descent'], units='rpm')

    return ivc
