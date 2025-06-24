from MCEVS.Vehicles.Standard import StandardLiftPlusCruiseEVTOL
from MCEVS.Utils.Plots import plot_geometries
import pandas as pd

utopian_data = pd.read_csv('optimal_results_by_scenario_by_year.csv')

scenario_list = ['conservative', 'nominal', 'aggresive']

for scenario in scenario_list:

    vehicle_list = []
    label_list = []
    cruise_speed_list = []

    if scenario == 'conservative':
        year_list = [2030, 2044, 2070]
    if scenario == 'nominal':
        year_list = [2030, 2043, 2070]
    if scenario == 'aggresive':
        year_list = [2030, 2041, 2070]

    for year in year_list:

        data = utopian_data[utopian_data['scenario'] == scenario]

        design_var = {}
        operation_var = {'RPM_lift_rotor': {'hover_climb': None}}

        design_var['wing_area'] = data[data['year'] == year]['Wing|area'].to_numpy()[0]
        design_var['wing_aspect_ratio'] = data[data['year'] == year]['Wing|aspect_ratio'].to_numpy()[0]
        design_var['r_lift_rotor'] = data[data['year'] == year]['LiftRotor|radius'].to_numpy()[0]
        design_var['r_propeller'] = data[data['year'] == year]['Propeller|radius'].to_numpy()[0]
        operation_var['RPM_propeller'] = {'cruise': data[data['year'] == year]['Propeller|Cruise|RPM'].to_numpy()[0]}
        vehicle = StandardLiftPlusCruiseEVTOL(design_var, operation_var, n_pax=6)

        # Building lists
        if year in [2030, 2070]:
            obj_label = f'Opt in {year}'
        else:
            obj_label = fr'Min $\psi$ ({year})'
        label_list.append(obj_label)
        vehicle_list.append(vehicle)
        cruise_speed_list.append(data[data['year'] == year]['cruise_speed'].to_numpy()[0])

    plot_geometries(vehicle_list, label_list, cruise_speed_list, figname=f'opt_geoms_{scenario}', savefig=False)
