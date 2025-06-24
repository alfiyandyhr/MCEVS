from MCEVS.Vehicles.Standard import StandardMultirotorEVTOL
from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Utils.Plots import plot_geometries
import pandas as pd

config_list = ['multirotor', 'liftcruise']
objective_list = ['takeoff_weight', 'energy', 'mission_time']

battery = 400 			# Wh/kg [250, 400, 550]
mission_range = 100 	# km [10,...,220]

for config in config_list:

    vehicle_list = []
    label_list = []
    cruise_speed_list = []

    for obj in objective_list:

        data = pd.read_csv(f'minimizing_{obj}/{config}/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv')

        design_var = {}
        operation_var = {'RPM_lift_rotor': {'hover_climb': None}}

        if config == 'multirotor':
            design_var['r_lift_rotor'] = data[data['mission_range'] == mission_range]['LiftRotor|radius'].to_numpy()[0]
            operation_var['RPM_lift_rotor']['cruise'] = data[data['mission_range'] == mission_range]['LiftRotor|Cruise|RPM'].to_numpy()[0]
            vehicle = StandardMultirotorEVTOL(design_var, operation_var, n_pax=6)

        elif config == 'liftcruise':
            design_var['wing_area'] = data[data['mission_range'] == mission_range]['Wing|area'].to_numpy()[0]
            design_var['wing_aspect_ratio'] = data[data['mission_range'] == mission_range]['Wing|aspect_ratio'].to_numpy()[0]
            design_var['r_lift_rotor'] = data[data['mission_range'] == mission_range]['LiftRotor|radius'].to_numpy()[0]
            design_var['r_propeller'] = data[data['mission_range'] == mission_range]['Propeller|radius'].to_numpy()[0]
            operation_var['RPM_propeller'] = {'cruise': data[data['mission_range'] == mission_range]['Propeller|Cruise|RPM'].to_numpy()[0]}
            vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, n_pax=6)

        # Building lists
        if obj == 'takeoff_weight':
            obj_label = 'weight-minimal'
        elif obj == 'energy':
            obj_label = 'energy-minimal'
        elif obj == 'mission_time':
            obj_label = 'time-minimal'
        label_list.append(obj_label)
        vehicle_list.append(vehicle)
        cruise_speed_list.append(data[data['mission_range'] == mission_range]['cruise_speed'].to_numpy()[0])

    plot_geometries(vehicle_list, label_list, cruise_speed_list, figname=f'{config}_objs_opt_geom', savefig=False)
