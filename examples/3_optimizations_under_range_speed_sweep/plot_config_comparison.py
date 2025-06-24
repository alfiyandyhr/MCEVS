import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.patches as patches

selection_by = 'selection_by_weight' 			# results from weight-based optimizations
# selection_by = 'selection_by_energy'			# results from energy-based optimizations

battery_energy_density = 250 					# [250,400,550] Wh/kg
battery_energy_density_list = [250, 400, 550] 	# Wh/kg

plot_multi_range_multi_speed_optimal_config_all = True
savefig = False

plot_multi_range_multi_speed_optimal_config = False
plot_multi_range_multi_speed_optimal_config_around_uber_vehicle_requirement = False

plot_multi_range_single_speed = False
at_which_speed = 240  # km/h
plot_multi_range_optimal_speed = False

plot_compare_battery_multi_range_optimal_speed = False

# Mission requirement sweep
range_i, range_f, d_range = 10, 230, 10 	# km
speed_i, speed_f, d_speed = 80, 330, 10 	# km/h
range_array = np.arange(range_i, range_f, d_range)
speed_array = np.arange(speed_i, speed_f, d_speed)

range_grid, speed_grid = np.meshgrid(range_array, speed_array)

if plot_multi_range_optimal_speed:

    data_df = pd.read_csv(f'{selection_by}/multirotor/battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')
    data_df2 = pd.read_csv(f'{selection_by}/liftcruise/battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

    data_df = data_df[data_df['Weight|residual'] < 0.1]
    data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df = data_df[data_df['LiftRotor|Cruise|T_to_P'] < 12.01]
    data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df = data_df[data_df['LiftRotor|Cruise|mu'] < 1.1]
    data_df = data_df[data_df['LiftRotor|Cruise|CT/sigma'] < 0.15]

    data_df2 = data_df2[data_df2['Weight|residual'] < 0.1]
    data_df2 = data_df2[data_df2['Aero|Cruise|CL'] < 0.91]
    data_df2 = data_df2[data_df2['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df2 = data_df2[data_df2['Propeller|Cruise|T_to_P'] < 12.01]
    data_df2 = data_df2[data_df2['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df2 = data_df2[data_df2['Propeller|Cruise|J'] < 3.01]
    data_df2 = data_df2[data_df2['Propeller|Cruise|CT/sigma'] < 0.15]
    data_df2 = data_df2[data_df2['LiftRotor|clearance_constraint'] < 0.1]

    plt.plot(data_df['mission_range'], data_df['Weight|takeoff'], '-o', label='Multirotor')
    plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'], '-o', label='Lift+cruise')
    plt.xlabel('Mission range [km]')
    plt.ylabel('MTOW [kg]')
    plt.legend(loc='upper left')
    plt.title('MTOW vs range at optimal speeds')
    plt.grid()
    plt.show()

    plt.plot(data_df['mission_range'], data_df['Energy|entire_mission'], '-o', label='Multirotor')
    plt.plot(data_df2['mission_range'], data_df2['Energy|entire_mission'], '-o', label='Lift+cruise')
    plt.xlabel('Mission range [km]')
    plt.ylabel('Required energy [kWh]')
    plt.legend(loc='upper left')
    plt.title('Energy vs range at optimal speeds')
    plt.grid()
    plt.show()

    plt.plot(data_df['mission_range'], data_df['mission_range'] / data_df['cruise_speed'] * 60 + 5.34, '-o', label='Multirotor')
    plt.plot(data_df2['mission_range'], data_df2['mission_range'] / data_df2['cruise_speed'] * 60 + 5.34, '-o', label='Lift+cruise')
    plt.xlabel('Mission range [km]')
    plt.ylabel('Mission time [mins]')
    plt.legend(loc='upper left')
    plt.title('Mission time vs range at optimal speeds')
    plt.grid()
    plt.show()

    plt.plot(data_df['mission_range'], data_df['Weight|takeoff'] * 9.8 * data_df['cruise_speed'] * 1000 / 3600 / data_df['Power|segment_3'] / 1000, '-o', label='Multirotor')
    plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'] * 9.8 * data_df2['cruise_speed'] * 1000 / 3600 / data_df2['Power|segment_3'] / 1000, '-o', label='Lift+cruise')
    plt.xlabel('Mission range [km]')
    plt.ylabel(r'$L/D_{e} = W*v/P$')
    plt.legend(loc='upper left')
    plt.title(r'Equivalent $L/D$ vs range at optimal speed')
    plt.grid()
    plt.show()

if plot_multi_range_single_speed:

    data_df = pd.read_csv(f'{selection_by}/multirotor/battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df2 = pd.read_csv(f'{selection_by}/liftcruise/battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')

    data_df = data_df[data_df['Weight|residual'] < 0.1]
    data_df = data_df[data_df['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df = data_df[data_df['LiftRotor|Cruise|T_to_P'] < 12.01]
    data_df = data_df[data_df['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df = data_df[data_df['LiftRotor|Cruise|mu'] < 1.1]
    data_df = data_df[data_df['LiftRotor|Cruise|CT/sigma'] < 0.15]

    data_df2 = data_df2[data_df2['Weight|residual'] < 0.1]
    data_df2 = data_df2[data_df2['Aero|Cruise|CL'] < 0.91]
    data_df2 = data_df2[data_df2['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df2 = data_df2[data_df2['Propeller|Cruise|T_to_P'] < 12.01]
    data_df2 = data_df2[data_df2['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df2 = data_df2[data_df2['Propeller|Cruise|J'] < 3.01]
    data_df2 = data_df2[data_df2['Propeller|Cruise|CT/sigma'] < 0.15]
    data_df2 = data_df2[data_df2['LiftRotor|clearance_constraint'] < 0.1]

    data = data_df[data_df['cruise_speed'] == at_which_speed]
    data2 = data_df2[data_df2['cruise_speed'] == at_which_speed]

    if selection_by == 'selection_by_weight':
        plt.plot(data['mission_range'], data['Weight|takeoff'], '-o', label='Multirotor')
        plt.plot(data2['mission_range'], data2['Weight|takeoff'], '-o', label='Lift+cruise')
        plt.xlabel('Mission range [km]')
        plt.ylabel('Optimal MTOW [kg]')
        plt.legend(loc='upper left')
        plt.title(f'Optimal MTOW at cruise speed= {at_which_speed} km/h\nand at battery energy density= {battery_energy_density} Wh/kg')
        plt.grid()
        plt.show()

    if selection_by == 'selection_by_energy':
        plt.plot(data['mission_range'], data['Energy|entire_mission'], '-o', label='Multirotor')
        plt.plot(data2['mission_range'], data2['Energy|entire_mission'], '-o', label='Lift+cruise')
        plt.xlabel('Mission range [km]')
        plt.ylabel('Required energy [kWh]')
        plt.legend(loc='upper left')
        plt.title(f'Required energy at cruise speed= {at_which_speed} km/h\nand at battery energy density= {battery_energy_density} Wh/kg')
        plt.grid()
        plt.show()

if plot_multi_range_multi_speed_optimal_config or plot_multi_range_multi_speed_optimal_config_around_uber_vehicle_requirement:

    data_df = pd.read_csv(f'{selection_by}/multirotor/battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df2 = pd.read_csv(f'{selection_by}/liftcruise/battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')

    # Filtering
    data_df.loc[data_df['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
    data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    data_df.loc[data_df['LiftRotor|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    data_df.loc[data_df['LiftRotor|Cruise|mu'] > 1, 'Weight|takeoff'] = np.nan
    data_df.loc[data_df['LiftRotor|Cruise|CT/sigma'] > 0.15, 'Weight|takeoff'] = np.nan

    data_df.loc[data_df['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
    data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    data_df.loc[data_df['LiftRotor|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    data_df.loc[data_df['LiftRotor|Cruise|mu'] > 1, 'Energy|entire_mission'] = np.nan
    data_df.loc[data_df['LiftRotor|Cruise|CT/sigma'] > 0.15, 'Energy|entire_mission'] = np.nan

    data_df2.loc[data_df2['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
    data_df2.loc[data_df2['Aero|Cruise|CL'] > 0.91, 'Weight|takeoff'] = np.nan
    data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    data_df2.loc[data_df2['Propeller|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    data_df2.loc[data_df2['Propeller|Cruise|CT/sigma'] > 0.15, 'Weight|takeoff'] = np.nan
    data_df2.loc[data_df2['Propeller|Cruise|J'] > 3.1, 'Weight|takeoff'] = np.nan

    data_df2.loc[data_df2['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
    data_df2.loc[data_df2['Aero|Cruise|CL'] > 0.91, 'Energy|entire_mission'] = np.nan
    data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    data_df2.loc[data_df2['Propeller|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    data_df2.loc[data_df2['Propeller|Cruise|CT/sigma'] > 0.15, 'Energy|entire_mission'] = np.nan
    data_df2.loc[data_df2['Propeller|Cruise|J'] > 3.1, 'Energy|entire_mission'] = np.nan

    # Generate data
    MTOW = data_df['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T
    MTOW2 = data_df2['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T
    Energy = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
    Energy2 = data_df2['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
    data = np.empty((len(speed_array), len(range_array)))
    data.fill(np.nan)
    data2 = np.empty((len(speed_array), len(range_array)))
    data2.fill(np.nan)

    for i in range(len(speed_array)):
        for j in range(len(range_array)):
            if not np.isnan(MTOW[i][j]) and not np.isnan(MTOW2[i][j]):
                if MTOW[i][j] < MTOW2[i][j]:
                    data[i][j] = 0.0
                else:
                    data[i][j] = 1.0
            elif np.isnan(MTOW[i][j]) and not np.isnan(MTOW2[i][j]):
                data[i][j] = 1.0
            elif not np.isnan(MTOW[i][j]) and np.isnan(MTOW2[i][j]):
                data[i][j] = 0.0
            else:
                data[i][j] = 2.0

    for i in range(len(speed_array)):
        for j in range(len(range_array)):
            if not np.isnan(Energy[i][j]) and not np.isnan(Energy2[i][j]):
                if Energy[i][j] < Energy2[i][j]:
                    data2[i][j] = 0.0
                else:
                    data2[i][j] = 1.0
            elif np.isnan(Energy[i][j]) and not np.isnan(Energy2[i][j]):
                data2[i][j] = 1.0
            elif not np.isnan(Energy[i][j]) and np.isnan(Energy2[i][j]):
                data2[i][j] = 0.0
            else:
                data2[i][j] = 2.0

    # print(data)
    # print(data2)

    # Create a custom color map
    colors_list = ['#404788ff', '#55c667ff', '#fde725ff']
    cmap = colors.ListedColormap(colors_list)

    if plot_multi_range_multi_speed_optimal_config:

        if selection_by == 'selection_by_weight':
            fig, ax = plt.subplots()

            # Plot heatmap of optimal config by MTOW
            im = ax.imshow(data, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
                           extent=[range_array[0] - d_range / 2, range_array[-1] + d_range / 2,
                                   speed_array[0] - d_speed / 2, speed_array[-1] + d_speed / 2])

            # Annotate area of interest of Uber Vehicle Requirement
            rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='UberAir Requirement')
            ax.add_patch(rect)

            cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
            cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
            ax.set_title(f'Optimal configuration by MTOW\nat battery energy density= {battery_energy_density} Wh/kg')
            ax.set_xlabel('Mission range [km]')
            ax.set_ylabel('Cruise speed [km/h]')
            ax.legend()
            plt.show()

        if selection_by == 'selection_by_energy':
            fig, ax = plt.subplots()

            # Plot heatmap of optimal config by Energy
            im = ax.imshow(data2, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
                           extent=[range_array[0] - d_range / 2, range_array[-1] + d_range / 2,
                                   speed_array[0] - d_speed / 2, speed_array[-1] + d_speed / 2])

            # Annotate area of interest of Uber Vehicle Requirement
            rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='UberAir Requirement')
            ax.add_patch(rect)

            cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
            cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
            ax.set_title(f'Optimal configuration by required energy\nat battery energy density= {battery_energy_density} Wh/kg')
            ax.set_xlabel('Mission range [km]')
            ax.set_ylabel('Cruise speed [km/h]')
            ax.legend()
            plt.show()

    if plot_multi_range_multi_speed_optimal_config_around_uber_vehicle_requirement:

        if selection_by == 'selection_by_weight':
            fig, ax = plt.subplots()

            # Plot heatmap of optimal config by MTOW
            im = ax.imshow(data, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
                           extent=[range_array[0] - d_range / 2, range_array[-1] + d_range / 2,
                                   speed_array[0] - d_speed / 2, speed_array[-1] + d_speed / 2])

            # Annotate area of interest of Uber Vehicle Requirement
            rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='UberAir Requirement')
            ax.add_patch(rect)

            cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
            cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
            ax.set_title(f'Optimal configuration by MTOW\nat battery energy density= {battery_energy_density} Wh/kg')
            ax.set_xlabel('Mission range [km]')
            ax.set_ylabel('Cruise speed [km/h]')
            ax.set_ylim([160, 320])
            ax.legend()
            plt.show()

        if selection_by == 'selection_by_energy':
            fig, ax = plt.subplots()

            # Plot heatmap of optimal config by MTOW
            im = ax.imshow(data2, cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
                           extent=[range_array[0] - d_range / 2, range_array[-1] + d_range / 2,
                                   speed_array[0] - d_speed / 2, speed_array[-1] + d_speed / 2])

            # Annotate area of interest of Uber Vehicle Requirement
            rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='UberAir Requirement')
            ax.add_patch(rect)

            cbar = fig.colorbar(im, ticks=[0.33, 1, 1.66])
            cbar.ax.set_yticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])
            ax.set_title(f'Optimal configuration by required energy\nat battery energy density= {battery_energy_density} Wh/kg')
            ax.set_xlabel('Mission range [km]')
            ax.set_ylabel('Cruise speed [km/h]')
            ax.set_ylim([160, 320])
            ax.legend()
            plt.show()

if plot_compare_battery_multi_range_optimal_speed:

    data_df_a = pd.read_csv(f'{selection_by}/multirotor/battery_250_Whpkg/results_with_speed_as_design_var.csv')
    data_df2_a = pd.read_csv(f'{selection_by}/liftcruise/battery_250_Whpkg/results_with_speed_as_design_var.csv')
    data_df_b = pd.read_csv(f'{selection_by}/multirotor/battery_400_Whpkg/results_with_speed_as_design_var.csv')
    data_df2_b = pd.read_csv(f'{selection_by}/liftcruise/battery_400_Whpkg/results_with_speed_as_design_var.csv')
    data_df_c = pd.read_csv(f'{selection_by}/multirotor/battery_550_Whpkg/results_with_speed_as_design_var.csv')
    data_df2_c = pd.read_csv(f'{selection_by}/liftcruise/battery_550_Whpkg/results_with_speed_as_design_var.csv')

    data_df_a = data_df_a[data_df_a['Weight|residual'] < 0.1]
    data_df_a = data_df_a[data_df_a['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df_a = data_df_a[data_df_a['LiftRotor|Cruise|T_to_P'] < 12.01]
    data_df_a = data_df_a[data_df_a['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df_a = data_df_a[data_df_a['LiftRotor|Cruise|mu'] < 1.1]
    data_df_a = data_df_a[data_df_a['LiftRotor|Cruise|CT/sigma'] < 0.15]

    data_df2_a = data_df2_a[data_df2_a['Weight|residual'] < 0.1]
    data_df2_a = data_df2_a[data_df2_a['Aero|Cruise|CL'] < 0.91]
    data_df2_a = data_df2_a[data_df2_a['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df2_a = data_df2_a[data_df2_a['Propeller|Cruise|T_to_P'] < 12.01]
    data_df2_a = data_df2_a[data_df2_a['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df2_a = data_df2_a[data_df2_a['Propeller|Cruise|J'] < 3.01]
    data_df2_a = data_df2_a[data_df2_a['Propeller|Cruise|CT/sigma'] < 0.15]
    data_df2_a = data_df2_a[data_df2_a['LiftRotor|clearance_constraint'] < 0.1]

    data_df_b = data_df_b[data_df_b['Weight|residual'] < 0.1]
    data_df_b = data_df_b[data_df_b['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df_b = data_df_b[data_df_b['LiftRotor|Cruise|T_to_P'] < 12.01]
    data_df_b = data_df_b[data_df_b['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df_b = data_df_b[data_df_b['LiftRotor|Cruise|mu'] < 1.1]
    data_df_b = data_df_b[data_df_b['LiftRotor|Cruise|CT/sigma'] < 0.15]

    data_df2_b = data_df2_b[data_df2_b['Weight|residual'] < 0.1]
    data_df2_b = data_df2_b[data_df2_b['Aero|Cruise|CL'] < 0.91]
    data_df2_b = data_df2_b[data_df2_b['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df2_b = data_df2_b[data_df2_b['Propeller|Cruise|T_to_P'] < 12.01]
    data_df2_b = data_df2_b[data_df2_b['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df2_b = data_df2_b[data_df2_b['Propeller|Cruise|J'] < 3.01]
    data_df2_b = data_df2_b[data_df2_b['Propeller|Cruise|CT/sigma'] < 0.15]
    data_df2_b = data_df2_b[data_df2_b['LiftRotor|clearance_constraint'] < 0.1]

    data_df_c = data_df_c[data_df_c['Weight|residual'] < 0.1]
    data_df_c = data_df_c[data_df_c['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df_c = data_df_c[data_df_c['LiftRotor|Cruise|T_to_P'] < 12.01]
    data_df_c = data_df_c[data_df_c['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df_c = data_df_c[data_df_c['LiftRotor|Cruise|mu'] < 1.1]
    data_df_c = data_df_c[data_df_c['LiftRotor|Cruise|CT/sigma'] < 0.15]

    data_df2_c = data_df2_c[data_df2_c['Weight|residual'] < 0.1]
    data_df2_c = data_df2_c[data_df2_c['Aero|Cruise|CL'] < 0.91]
    data_df2_c = data_df2_c[data_df2_c['LiftRotor|HoverClimb|T_to_P'] < 12.01]
    data_df2_c = data_df2_c[data_df2_c['Propeller|Cruise|T_to_P'] < 12.01]
    data_df2_c = data_df2_c[data_df2_c['LiftRotor|HoverDescent|T_to_P'] < 12.01]
    data_df2_c = data_df2_c[data_df2_c['Propeller|Cruise|J'] < 3.01]
    data_df2_c = data_df2_c[data_df2_c['Propeller|Cruise|CT/sigma'] < 0.15]
    data_df2_c = data_df2_c[data_df2_c['LiftRotor|clearance_constraint'] < 0.1]

    plt.plot(data_df_a['mission_range'], data_df_a['Weight|takeoff'], '-.r', label='Multirotor 250 Wh/kg')
    plt.plot(data_df_b['mission_range'], data_df_b['Weight|takeoff'], '-.b', label='Multirotor 400 Wh/kg')
    plt.plot(data_df_c['mission_range'], data_df_c['Weight|takeoff'], '-.g', label='Multirotor 550 Wh/kg')
    plt.plot(data_df2_a['mission_range'], data_df2_a['Weight|takeoff'], '-r', label='Lift+cruise 250 Wh/kg')
    plt.plot(data_df2_b['mission_range'], data_df2_b['Weight|takeoff'], '-b', label='Lift+cruise 400 Wh/kg')
    plt.plot(data_df2_c['mission_range'], data_df2_c['Weight|takeoff'], '-g', label='Lift+cruise 550 Wh/kg')
    plt.xlabel('Mission range [km]')
    plt.ylabel('MTOW [kg]')
    plt.ylim([1000, 5000])
    plt.legend(loc='upper left')
    plt.title('MTOW vs range at optimal speeds')
    plt.grid()
    plt.show()

    plt.plot(data_df_a['mission_range'], data_df_a['Energy|entire_mission'], '-.r', label='Multirotor 250 Wh/kg')
    plt.plot(data_df_b['mission_range'], data_df_b['Energy|entire_mission'], '-.b', label='Multirotor 400 Wh/kg')
    plt.plot(data_df_c['mission_range'], data_df_c['Energy|entire_mission'], '-.g', label='Multirotor 550 Wh/kg')
    plt.plot(data_df2_a['mission_range'], data_df2_a['Energy|entire_mission'], '-r', label='Lift+cruise 250 Wh/kg')
    plt.plot(data_df2_b['mission_range'], data_df2_b['Energy|entire_mission'], '-b', label='Lift+cruise 400 Wh/kg')
    plt.plot(data_df2_c['mission_range'], data_df2_c['Energy|entire_mission'], '-g', label='Lift+cruise 550 Wh/kg')
    plt.xlabel('Mission range [km]')
    plt.ylabel('Required energy [kWh]')
    plt.ylim([0, 600])
    plt.legend(loc='upper left')
    plt.title('Energy vs range at optimal speeds')
    plt.grid()
    plt.show()

    plt.plot(data_df_a['mission_range'], data_df_a['mission_range'] / data_df_a['cruise_speed'] * 60 + 5.34, '-.r', label='Multirotor 250 Wh/kg')
    plt.plot(data_df_b['mission_range'], data_df_b['mission_range'] / data_df_b['cruise_speed'] * 60 + 5.34, '-.b', label='Multirotor 400 Wh/kg')
    plt.plot(data_df_c['mission_range'], data_df_c['mission_range'] / data_df_c['cruise_speed'] * 60 + 5.34, '-.g', label='Multirotor 550 Wh/kg')
    plt.plot(data_df2_a['mission_range'], data_df2_a['mission_range'] / data_df2_a['cruise_speed'] * 60 + 5.34, '-r', label='Lift+cruise 250 Wh/kg')
    plt.plot(data_df2_b['mission_range'], data_df2_b['mission_range'] / data_df2_b['cruise_speed'] * 60 + 5.34, '-b', label='Lift+cruise 400 Wh/kg')
    plt.plot(data_df2_c['mission_range'], data_df2_c['mission_range'] / data_df2_c['cruise_speed'] * 60 + 5.34, '-g', label='Lift+cruise 550 Wh/kg')
    plt.xlabel('Mission range [km]')
    plt.ylabel('Mission time [mins]')
    plt.legend(loc='upper left')
    plt.title('Mission time vs range at optimal speeds')
    plt.grid()
    plt.show()

    plt.plot(data_df_a['mission_range'], data_df_a['Weight|takeoff'] * 9.8 * data_df_a['cruise_speed'] * 1000 / 3600 / data_df_a['Power|segment_3'] / 1000, '-.r', label='Multirotor 250 Wh/kg')
    plt.plot(data_df_b['mission_range'], data_df_b['Weight|takeoff'] * 9.8 * data_df_b['cruise_speed'] * 1000 / 3600 / data_df_b['Power|segment_3'] / 1000, '-.b', label='Multirotor 400 Wh/kg')
    plt.plot(data_df_c['mission_range'], data_df_c['Weight|takeoff'] * 9.8 * data_df_c['cruise_speed'] * 1000 / 3600 / data_df_c['Power|segment_3'] / 1000, '-.g', label='Multirotor 550 Wh/kg')
    plt.plot(data_df2_a['mission_range'], data_df2_a['Weight|takeoff'] * 9.8 * data_df2_a['cruise_speed'] * 1000 / 3600 / data_df2_a['Power|segment_3'] / 1000, '-r', label='Lift+cruise 250 Wh/kg')
    plt.plot(data_df2_b['mission_range'], data_df2_b['Weight|takeoff'] * 9.8 * data_df2_b['cruise_speed'] * 1000 / 3600 / data_df2_b['Power|segment_3'] / 1000, '-b', label='Lift+cruise 400 Wh/kg')
    plt.plot(data_df2_c['mission_range'], data_df2_c['Weight|takeoff'] * 9.8 * data_df2_c['cruise_speed'] * 1000 / 3600 / data_df2_c['Power|segment_3'] / 1000, '-g', label='Lift+cruise 550 Wh/kg')
    plt.xlabel('Mission range [km]')
    plt.ylabel(r'$L/D_{e} = W*v/P$')
    plt.legend(loc='center right')
    plt.title(r'Equivalent $L/D$ vs range at optimal speed')
    plt.grid()
    plt.show()

if plot_multi_range_multi_speed_optimal_config_all:

    data1_list = []
    data2_list = []

    for battery_energy_density in battery_energy_density_list:

        data_df = pd.read_csv(f'{selection_by}/multirotor/battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
        data_df2 = pd.read_csv(f'{selection_by}/liftcruise/battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')

        # Filtering
        data_df.loc[data_df['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
        data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
        data_df.loc[data_df['LiftRotor|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
        data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
        data_df.loc[data_df['LiftRotor|Cruise|mu'] > 1, 'Weight|takeoff'] = np.nan
        data_df.loc[data_df['LiftRotor|Cruise|CT/sigma'] > 0.15, 'Weight|takeoff'] = np.nan

        data_df.loc[data_df['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
        data_df.loc[data_df['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
        data_df.loc[data_df['LiftRotor|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
        data_df.loc[data_df['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
        data_df.loc[data_df['LiftRotor|Cruise|mu'] > 1, 'Energy|entire_mission'] = np.nan
        data_df.loc[data_df['LiftRotor|Cruise|CT/sigma'] > 0.15, 'Energy|entire_mission'] = np.nan

        data_df2.loc[data_df2['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
        data_df2.loc[data_df2['Aero|Cruise|CL'] > 0.91, 'Weight|takeoff'] = np.nan
        data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
        data_df2.loc[data_df2['Propeller|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
        data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
        data_df2.loc[data_df2['Propeller|Cruise|CT/sigma'] > 0.15, 'Weight|takeoff'] = np.nan
        data_df2.loc[data_df2['Propeller|Cruise|J'] > 3.1, 'Weight|takeoff'] = np.nan

        data_df2.loc[data_df2['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
        data_df2.loc[data_df2['Aero|Cruise|CL'] > 0.91, 'Energy|entire_mission'] = np.nan
        data_df2.loc[data_df2['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
        data_df2.loc[data_df2['Propeller|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
        data_df2.loc[data_df2['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
        data_df2.loc[data_df2['Propeller|Cruise|CT/sigma'] > 0.15, 'Energy|entire_mission'] = np.nan
        data_df2.loc[data_df2['Propeller|Cruise|J'] > 3.1, 'Energy|entire_mission'] = np.nan

        # Generate data
        MTOW = data_df['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T
        MTOW2 = data_df2['Weight|takeoff'].to_numpy().reshape(len(range_array), len(speed_array)).T
        Energy = data_df['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
        Energy2 = data_df2['Energy|entire_mission'].to_numpy().reshape(len(range_array), len(speed_array)).T
        data = np.empty((len(speed_array), len(range_array)))
        data.fill(np.nan)
        data2 = np.empty((len(speed_array), len(range_array)))
        data2.fill(np.nan)

        for i in range(len(speed_array)):
            for j in range(len(range_array)):
                if not np.isnan(MTOW[i][j]) and not np.isnan(MTOW2[i][j]):
                    if MTOW[i][j] < MTOW2[i][j]:
                        data[i][j] = 0.0
                    else:
                        data[i][j] = 1.0
                elif np.isnan(MTOW[i][j]) and not np.isnan(MTOW2[i][j]):
                    data[i][j] = 1.0
                elif not np.isnan(MTOW[i][j]) and np.isnan(MTOW2[i][j]):
                    data[i][j] = 0.0
                else:
                    data[i][j] = 2.0

        for i in range(len(speed_array)):
            for j in range(len(range_array)):
                if not np.isnan(Energy[i][j]) and not np.isnan(Energy2[i][j]):
                    if Energy[i][j] < Energy2[i][j]:
                        data2[i][j] = 0.0
                    else:
                        data2[i][j] = 1.0
                elif np.isnan(Energy[i][j]) and not np.isnan(Energy2[i][j]):
                    data2[i][j] = 1.0
                elif not np.isnan(Energy[i][j]) and np.isnan(Energy2[i][j]):
                    data2[i][j] = 0.0
                else:
                    data2[i][j] = 2.0

        data1_list.append(data)
        data2_list.append(data2)

    # Create a custom color map
    colors_list = ['#404788ff', '#55c667ff', '#fde725ff']
    cmap = colors.ListedColormap(colors_list)

    if selection_by == 'selection_by_weight':

        # Create subplots: 1 row, 3 columns
        fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=True)

        for i, ax in enumerate(axes):
            # Plot heatmap of optimal config by MTOW
            im = ax.imshow(data1_list[i], cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
                           extent=[range_array[0] - d_range / 2, range_array[-1] + d_range / 2,
                                   speed_array[0] - d_speed / 2, speed_array[-1] + d_speed / 2])

            # Annotate area of interest of Uber Vehicle Requirement
            rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='UberAir Requirement')
            ax.add_patch(rect)

            # Titles and labels for each subplot
            ax.set_title(f'Battery GED: {battery_energy_density_list[i]} Wh/kg')
            ax.set_xlabel('Mission range [km]')
            if i == 0:
                ax.set_ylabel('Cruise speed [km/h]')

        # Add a single legend for the "Optimal cruise speed path" line
        fig.legend(['Uber Air Mission Requirement'], loc='upper center', bbox_to_anchor=(0.49, 0.94), ncol=1, prop={'size': 12})

        # Add a single colorbar for all heatmaps
        # cbar = fig.colorbar(im, ax=axes, orientation='horizontal', ticks=[0.33, 1, 1.66], shrink=0.9)
        cbar = fig.colorbar(im, ax=axes, orientation='horizontal', ticks=[0.33, 1, 1.66], fraction=0.05, pad=0.1)
        cbar.ax.set_xticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])

        # Adjust layout to prevent overlap
        fig.suptitle('Optimal configuration by MTOW from weight-based optimization', size=16)
        plt.subplots_adjust(bottom=0.25, top=0.82, wspace=0.03)
        plt.savefig('config_selection_by_weight.pdf', format='pdf', dpi=300) if savefig else plt.show()

    if selection_by == 'selection_by_energy':

        fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=True)

        for i, ax in enumerate(axes):
            # Plot heatmap of optimal config by energy
            im = ax.imshow(data2_list[i], cmap=cmap, vmin=0, vmax=2.0, aspect='auto', origin='lower',
                           extent=[range_array[0] - d_range / 2, range_array[-1] + d_range / 2,
                                   speed_array[0] - d_speed / 2, speed_array[-1] + d_speed / 2])

            # Annotate area of interest of Uber Vehicle Requirement
            rect = patches.Rectangle((70, 220), 40, 40, linestyle='--', linewidth=1.5, edgecolor='r', facecolor='none', label='UberAir Requirement')
            ax.add_patch(rect)

            # Titles and labels for each subplot
            ax.set_title(f'Battery GED: {battery_energy_density_list[i]} Wh/kg')
            ax.set_xlabel('Mission range [km]')
            if i == 0:
                ax.set_ylabel('Cruise speed [km/h]')

        # Add a single legend for the "Optimal cruise speed path" line
        fig.legend(['Uber Air Mission Requirement'], loc='upper center', bbox_to_anchor=(0.49, 0.94), ncol=1, prop={'size': 12})

        # Add a single colorbar for all heatmaps
        cbar = fig.colorbar(im, ax=axes, orientation='horizontal', ticks=[0.33, 1, 1.66], fraction=0.05, pad=0.1)
        cbar.ax.set_xticklabels(['Multirotor', 'Lift+Cruise', 'Infeasible'])

        # Adjust layout to prevent overlap
        fig.suptitle('Optimal configuration by energy from energy-based optimization', size=16)
        plt.subplots_adjust(bottom=0.25, top=0.82, wspace=0.03)
        plt.savefig('config_selection_by_energy.pdf', format='pdf', dpi=300) if savefig else plt.show()
