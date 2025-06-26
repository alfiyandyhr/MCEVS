from MCEVS.Utils.Plots import plot_weight_breakdown, plot_power_energy_breakdown
import pandas as pd

do_plot_weight_breakdown = True
do_plot_power_energy_breakdown = True

config_list = ['multirotor', 'liftcruise']
objective_list = ['takeoff_weight', 'energy', 'mission_time']

battery = 400 			# Wh/kg [250, 400, 550]
mission_range = 100 	# km [10,...,220]

if do_plot_weight_breakdown:

    for config in config_list:

        data_list = []

        for obj in objective_list:

            data = pd.read_csv(f'minimizing_{obj}/{config}/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv')
            data_list.append(data[data['mission_range'] == mission_range])

        if config == 'multirotor':
            vehicle_config = 'Multirotor'
        elif config == 'liftcruise':
            vehicle_config = 'Lift + Cruise'

        plot_weight_breakdown(data_list, ['Weight-minimal', 'Energy-minimal', 'Time-minimal'], figname=f'figures/weight_breakdown_{config}',
                              figtitle=f'Component Weight Breakdown for {vehicle_config}', savefig=False)

if do_plot_power_energy_breakdown:

    for config in config_list:

        data_list = []

        for obj in objective_list:

            data = pd.read_csv(f'minimizing_{obj}/{config}/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv')
            data_list.append(data[data['mission_range'] == mission_range])
        
        if config == 'multirotor':
            vehicle_config = 'Multirotor'
        elif config == 'liftcruise':
            vehicle_config = 'Lift + Cruise'

        plot_power_energy_breakdown(data_list, ['Weight-minimal', 'Energy-minimal', 'Time-minimal'], figname=f'figures/power_energy_breakdown_{config}',
                                    indexes=['Power|segment_1', 'Power|segment_3', 'Power|segment_5', 'Energy|entire_mission'],
                                    figtitle=f'Power Energy Breakdown for {vehicle_config}', savefig=False)
