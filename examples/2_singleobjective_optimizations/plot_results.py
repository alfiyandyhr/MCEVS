import pandas as pd
import matplotlib.pyplot as plt

plot_minimizing_takeoff_weight = False
plot_minimizing_energy = False
plot_minimizing_mission_time = False

plot_minimizing_all_objs = True

plot_multi_range_at_optimal_speeds = False

savefig = False

configurations = ['multirotor', 'liftcruise']

if plot_minimizing_takeoff_weight or plot_minimizing_energy or plot_minimizing_mission_time:

    for configuration in configurations:

        if plot_minimizing_takeoff_weight:
            folder = 'minimizing_takeoff_weight'
            label = 'Weight-minimal'
            df_label = 'Weight|takeoff'
            # df_label = 'Energy|entire_mission'
            # df_label = 'cruise_speed'
            y_label = 'Takeoff weight (kg)'
        if plot_minimizing_energy:
            folder = 'minimizing_energy'
            label = 'Energy-minimal'
            df_label = 'Energy|entire_mission'
            # df_label = 'Weight|takeoff'
            # df_label = 'cruise_speed'
            y_label = 'Energy (kWh)'
        if plot_minimizing_mission_time:
            folder = 'minimizing_mission_time'
            label = 'Time-minimal'
            df_label = 'mission_time'
            # df_label = 'Weight|takeoff'
            # df_label = 'Energy|entire_mission'
            y_label = 'Mission time (mins)'

        data_df1 = pd.read_csv(f'{folder}/{configuration}/battery_250_Whpkg/results_with_speed_as_design_var.csv')
        data_df2 = pd.read_csv(f'{folder}/{configuration}/battery_400_Whpkg/results_with_speed_as_design_var.csv')
        data_df3 = pd.read_csv(f'{folder}/{configuration}/battery_550_Whpkg/results_with_speed_as_design_var.csv')
        data_list = [data_df1, data_df2, data_df3]

        for i in range(3):
            data_list[i] = data_list[i][data_list[i]['success']]

        plt.plot(data_list[0]['mission_range'], data_list[0][df_label], '-o', label=f'{label} {configuration}; 250 Wh/kg')
        plt.plot(data_list[1]['mission_range'], data_list[1][df_label], '-o', label=f'{label} {configuration}; 400 Wh/kg')
        plt.plot(data_list[2]['mission_range'], data_list[2][df_label], '-o', label=f'{label} {configuration}; 550 Wh/kg')

        plt.xlabel('Mission range (km)')
        plt.ylabel(y_label)
        plt.legend()
        plt.grid()
        plt.show()

if plot_minimizing_all_objs:

    config_list = ['multirotor', 'liftcruise']
    obj_list = ['takeoff_weight', 'energy', 'mission_time']
    battery_list = ['250', '400', '550']

    data_list = []
    for config in config_list:
        for battery in battery_list:
            for obj in obj_list:
                data_list.append(pd.read_csv(f'minimizing_{obj}/{config}/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv'))

    for i in range(9):
        data_list[i] = data_list[i][data_list[i]['success']]
        data_list[i]['mission_time'] *= 60
        data_list[i + 9] = data_list[i + 9][data_list[i + 9]['success']]
        data_list[i + 9]['mission_time'] *= 60

    fig, axs = plt.subplots(3, 3, figsize=(9, 7), sharex=True)

    for i, battery in enumerate(battery_list):
        for j, obj in enumerate(obj_list):

            if obj == 'takeoff_weight':
                obj_ylabel = 'Takeoff weight (kg)'
                obj_dflabel = 'Weight|takeoff'
            elif obj == 'energy':
                obj_ylabel = 'Mission energy (kWh)'
                obj_dflabel = 'Energy|entire_mission'
            elif obj == 'mission_time':
                obj_ylabel = 'Mission time (mins)'
                obj_dflabel = 'mission_time'

            axs[j, i].plot(data_list[i * 3 + 0]['mission_range'], data_list[i * 3 + 0][obj_dflabel], 'r.', label='Weight-minimal multirotor' if i == 0 and j == 0 else None)
            axs[j, i].plot(data_list[i * 3 + 1]['mission_range'], data_list[i * 3 + 1][obj_dflabel], 'r--', label='Energy-minimal multirotor' if i == 0 and j == 0 else None)
            axs[j, i].plot(data_list[i * 3 + 2]['mission_range'], data_list[i * 3 + 2][obj_dflabel], 'r-', label='Time-minimal multirotor' if i == 0 and j == 0 else None)
            axs[j, i].plot(data_list[i * 3 + 9 + 0]['mission_range'], data_list[i * 3 + 9 + 0][obj_dflabel], 'b.', label='Weight-minimal lift+cruise' if i == 0 and j == 0 else None)
            axs[j, i].plot(data_list[i * 3 + 9 + 1]['mission_range'], data_list[i * 3 + 9 + 1][obj_dflabel], 'b--', label='Energy-minimal lift+cruise' if i == 0 and j == 0 else None)
            axs[j, i].plot(data_list[i * 3 + 9 + 2]['mission_range'], data_list[i * 3 + 9 + 2][obj_dflabel], 'b-', label='Time-minimal lift+cruise' if i == 0 and j == 0 else None)
            if i * 3 + j in [0, 1, 2]:
                axs[j, i].set_ylabel(obj_ylabel)
            if i * 3 + j in [2, 5, 8]:
                axs[j, i].set_xlabel('Mission range (km)')
            if i * 3 + j in [0, 3, 6]:
                axs[j, i].set_title(f'Battery {battery} Wh/kg', size=10.0)

    fig.suptitle('Single-objective optimization results')
    fig.legend(ncols=2, bbox_to_anchor=(0.78, 0.96))
    plt.subplots_adjust(left=0.09, bottom=0.1, right=0.97, top=0.81, hspace=0.1)
    plt.savefig('figures/opt_exploring_objectives.pdf', format='pdf', dpi=300) if savefig else plt.show()

if plot_multi_range_at_optimal_speeds:

    objective_list = ['takeoff_weight', 'energy', 'mission_time']

    for objective in objective_list:

        battery_list = [250, 400, 550]

        data_dict = {}
        for battery in battery_list:

            data_df = pd.read_csv(f'minimizing_{objective}/multirotor/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv')
            data_df2 = pd.read_csv(f'minimizing_{objective}/liftcruise/battery_{battery}_Whpkg/results_with_speed_as_design_var.csv')

            data_df = data_df[data_df['success']]
            data_df2 = data_df2[data_df2['success']]

            data_dict[f'{battery}'] = [data_df, data_df2]

        if objective == 'takeoff_weight':
            metric_tag = 'Weight|takeoff'
            y_label = 'Takeoff Weight (kg)'
            title_tag = 'Takeoff Weight'
            fig_tag = 'weight'

        elif objective == 'energy':
            metric_tag = 'Energy|entire_mission'
            y_label = 'Energy Consumption (kWh)'
            title_tag = 'Energy'
            fig_tag = 'energy'

        elif objective == 'mission_time':
            metric_tag = None
            y_label = 'Mission Time (mins)'
            title_tag = 'Mission Time'
            fig_tag = 'time'

        # --- Objective Function vs Mission Range --- #

        fig, axes = plt.subplots(1, 3, figsize=(12, 5), sharey=False)

        for i, ax in enumerate(axes):

            if objective in ['takeoff_weight', 'energy']:
                ax.plot(data_dict[f'{battery_list[i]}'][0]['mission_range'], data_dict[f'{battery_list[i]}'][0][f'{metric_tag}'], '-o', ms=4, label='Multirotor' if i == 0 else None)
                ax.plot(data_dict[f'{battery_list[i]}'][1]['mission_range'], data_dict[f'{battery_list[i]}'][1][f'{metric_tag}'], '-o', ms=4, label='Lift+Cruise' if i == 0 else None)

            elif objective in ['mission_time']:
                R_0 = data_dict[f'{battery_list[i]}'][0]['mission_range']
                v_0 = data_dict[f'{battery_list[i]}'][0]['cruise_speed']
                R_1 = data_dict[f'{battery_list[i]}'][1]['mission_range']
                v_1 = data_dict[f'{battery_list[i]}'][1]['cruise_speed']
                ax.plot(data_dict[f'{battery_list[i]}'][0]['mission_range'], R_0 / v_0 * 60 + 5.34, '-o', ms=4, label='Multirotor' if i == 0 else None)
                ax.plot(data_dict[f'{battery_list[i]}'][1]['mission_range'], R_1 / v_1 * 60 + 5.34, '-o', ms=4, label='Lift+Cruise' if i == 0 else None)

            ax.set_title(f'Battery GED= {battery_list[i]} Wh/kg')
            ax.set_xlabel('Mission range (km)')
            if i == 0:
                ax.set_ylabel(y_label)

        fig.legend(loc='upper center', bbox_to_anchor=(0.5, 0.94), ncol=2, prop={'size': 12})
        fig.suptitle(f'Optimal {title_tag} vs Mission Range', size=16)
        plt.subplots_adjust(bottom=0.25, top=0.80, wspace=0.25)
        plt.savefig(f'figures/{fig_tag}_vs_range.pdf', format='pdf', dpi=300) if savefig else plt.show()

        # --- Equivalent L/D vs Mission Range --- #

        fig, axes = plt.subplots(1, 3, figsize=(12, 5), sharey=True)

        for i, ax in enumerate(axes):

            g = 9.81  # m/s**2

            W_0 = data_dict[f'{battery_list[i]}'][0]['Weight|takeoff']
            v_0 = data_dict[f'{battery_list[i]}'][0]['cruise_speed'] * 1000 / 3600  # m/s
            P_0 = data_dict[f'{battery_list[i]}'][0]['Power|segment_3'] * 1000  # W

            W_1 = data_dict[f'{battery_list[i]}'][1]['Weight|takeoff']
            v_1 = data_dict[f'{battery_list[i]}'][1]['cruise_speed'] * 1000 / 3600  # m/s
            P_1 = data_dict[f'{battery_list[i]}'][1]['Power|segment_3'] * 1000  # W

            ax.plot(data_dict[f'{battery_list[i]}'][0]['mission_range'], W_0 * g * v_0 / P_0, '-o', ms=4, label='Multirotor' if i == 0 else None)
            ax.plot(data_dict[f'{battery_list[i]}'][1]['mission_range'], W_1 * g * v_1 / P_1, '-o', ms=4, label='Lift+Cruise' if i == 0 else None)

            ax.set_title(f'Battery GED: {battery_list[i]} Wh/kg')
            ax.set_xlabel('Mission range (km)')
            if i == 0:
                ax.set_ylabel(r'$L/D_{e} = W*v/P$')

        if objective == 'takeoff_weight':
            title_tag2 = 'Weight'
        elif objective == 'energy':
            title_tag2 = 'Energy'
        elif objective == 'mission_time':
            title_tag2 = 'Time'

        fig.legend(loc='upper center', bbox_to_anchor=(0.5, 0.94), ncol=2, prop={'size': 12})
        fig.suptitle(f'Equivalent L/D vs Mission Range for {title_tag2}-Minimal Designs', size=16)
        plt.subplots_adjust(bottom=0.25, top=0.80, wspace=0.1)
        plt.savefig(f'figures/{fig_tag}_LbyD_vs_range.pdf', format='pdf', dpi=300) if savefig else plt.show()
