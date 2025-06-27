from MCEVS.Utils.Plots import plot_weight_breakdown_pie_chart
import numpy as np
import pandas as pd

selection_by = 'weight'  # ['weight', 'energy']

config_list = ['multirotor', 'liftcruise']
battery_list = [250, 400, 550]

mission_range = 70 	# km [10,...,220]
cruise_speed = 220  # km/h [80,...,320]

for config in config_list:

    data_list = []
    mtow_list = []

    for i, battery in enumerate(battery_list):

        data = pd.read_csv(f'selection_by_{selection_by}/{config}/battery_{battery}_Whpkg/results_without_speed_as_design_var.csv')
        data_list.append(data[(data['mission_range'] == mission_range) & (data['cruise_speed'] == cruise_speed)])
        mtow_list.append(data_list[i]['Weight|takeoff'].to_numpy()[0])

    if config == 'multirotor':
        vehicle_config = 'Multirotor'
    elif config == 'liftcruise':
        vehicle_config = 'Lift + Cruise'

    plot_weight_breakdown_pie_chart(data_list,
                                    label_list=[f'${{\\mathbf{{Battery= {battery_list[0]} Wh/kg}}}}$\nMTOW= {np.round(mtow_list[0])} kg',
                                                f'${{\\mathbf{{Battery= {battery_list[1]} Wh/kg}}}}$\nMTOW= {np.round(mtow_list[1])} kg',
                                                f'${{\\mathbf{{Battery= {battery_list[2]} Wh/kg}}}}$\nMTOW= {np.round(mtow_list[2])} kg'],
                                    figtitle=f'Component Weight Breakdown for {vehicle_config}\nRange= {mission_range} km; Speed= {cruise_speed} km/h', savefig=False,
                                    figname=f'figures/component_weight_breakdown_{config}')
