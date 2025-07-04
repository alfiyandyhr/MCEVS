import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

plot_2D = False
plot_contour = False

plot_contour_all = True
savefig = False

plot_surface = False

print_best_speed_for_all_range = False
check_best_speed_for_all_range = False

plot_2D_with_speed_as_design_var = False

battery_energy_density = 250  # [250,400,550]
battery_energy_density_list = [250, 400, 550]

if print_best_speed_for_all_range:
    data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df = data_df[data_df['success']]
    data = data_df[data_df['mission_range'] == 10]
    print(data[['cruise_speed', 'Weight|takeoff', 'LiftRotor|radius', 'Propeller|Cruise|RPM']].sort_values(by='Weight|takeoff'))

if check_best_speed_for_all_range:
    data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df = data_df[data_df['success']]
    data = data_df[data_df['mission_range'] == 10]

    opt_speed_data = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

    for mission_range in range(10, 230, 10):
        weight_at_opt_speed = opt_speed_data[opt_speed_data['mission_range'] == mission_range]['Weight|takeoff'].to_numpy()[0]
        data = data_df[data_df['mission_range'] == mission_range]
        weight_at_best_speed = data[['cruise_speed', 'Weight|takeoff']].sort_values(by='Weight|takeoff').iloc[0]['Weight|takeoff']
        status = 'passed!' if weight_at_opt_speed < weight_at_best_speed else 'NOT passed!'
        print(f'mission_range = {mission_range}; weight_at_opt_speed = {weight_at_opt_speed}; weight_at_best_speed = {weight_at_best_speed}; status = {status}')

if plot_2D:
    data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

    data_df = data_df[data_df['success']]
    data_df2 = data_df2[data_df2['success']]

    range_array = np.unique(data_df['mission_range'].to_numpy())
    speed_array = np.unique(data_df['cruise_speed'].to_numpy())

    for cruise_speed in speed_array:
        data = data_df[data_df['cruise_speed'] == cruise_speed]
        plt.plot(data['mission_range'], data['Weight|takeoff'], '-o', label=f'Cruise speed = {cruise_speed}')

    plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'], 'k-o', label='Cruise speed = optimal')
    plt.xlabel('Mission range (km)')
    plt.ylabel('Optimal MTOW (kg)')
    plt.legend()
    plt.grid()
    plt.show()

if plot_2D_with_speed_as_design_var:
    data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')
    data_df2 = data_df2[data_df2['success']]

    plt.plot(data_df2['mission_range'], data_df2['Weight|takeoff'], '-o', label='Cruise speed = optimal speed')
    # plt.plot(data_df2['mission_range'], data_df2['cruise_speed'], '-o', label=f'Cruise speed = optimal speed')

    plt.xlabel('Mission range (km)')
    plt.ylabel('Optimal MTOW (kg)')
    plt.legend()
    plt.grid()
    plt.show()

if plot_contour:
    data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

    data_df.loc[~data_df['success'], 'Weight|takeoff'] = np.nan
    data_df2.loc[~data_df2['success'], 'Weight|takeoff'] = np.nan

    range_array = np.unique(data_df['mission_range'].to_numpy())
    speed_array = np.unique(data_df['cruise_speed'].to_numpy())

    X, Y = np.meshgrid(range_array, speed_array)
    Z = data_df['Weight|takeoff'].to_numpy().reshape(len(speed_array), len(range_array))
    infeasible_mask = np.isnan(Z)

    fig, ax = plt.subplots()

    cp = ax.contourf(X, Y, Z, 15, cmap='viridis', vmin=np.nanmin(Z), vmax=np.nanmax(Z), extend='min')
    ax.contourf(X, Y, infeasible_mask.astype(float), levels=[0.01, 1.5], colors='white', hatches=['///'], alpha=0.8)
    ax.plot(data_df2['mission_range'], data_df2['cruise_speed'], 'r--')
    cbar = plt.colorbar(cp, ax=ax)
    cbar.set_label('MTOW (kg)')
    ax.set_ylabel('Cruise speed (km/h)')
    ax.set_xlabel('Mission range (km)')
    fig.suptitle('Optimal MTOW (kg)')

    # Legends
    legend_elements = []
    legend_elements.append(Line2D([0], [0], color='red', linestyle='--', linewidth=1.5, label='Optimal cruise speed path'))
    legend_elements.append(Patch(facecolor='white', edgecolor='black', hatch='///', label='Infeasible region'))

    fig.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.49, 0.94), ncol=2)
    plt.subplots_adjust(top=0.85)
    plt.show()

if plot_surface:
    data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
    data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

    data_df.loc[~data_df['success'], 'Weight|takeoff'] = None
    data_df2.loc[~data_df2['success'], 'Weight|takeoff'] = None

    range_array = np.unique(data_df['mission_range'].to_numpy())
    speed_array = np.unique(data_df['cruise_speed'].to_numpy())

    X, Y = np.meshgrid(range_array, speed_array)
    Z = data_df['Weight|takeoff'].to_numpy().reshape(len(speed_array), len(range_array))

    Z_masked = np.ma.masked_where(Z is None, Z)

    # Create a figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the surface
    surf = ax.plot_surface(X, Y, Z_masked, cmap='viridis', vmin=np.nanmin(Z_masked), vmax=np.nanmax(Z_masked), antialiased=False)

    # Plot an optimal line
    ax.plot(data_df2['mission_range'], data_df2['cruise_speed'], data_df2['Weight|takeoff'], color='red', linewidth=5, label='Optimal cruise speed path')

    # Add labels and title
    ax.set_xlabel('Mission range (km)')
    ax.set_ylabel('Cruise speed (km/h)')
    ax.set_zlabel('Optimal MTOW (kg)')
    ax.set_title('Surface Plot')

    fig.colorbar(surf, orientation='horizontal', fraction=0.046, pad=0.04)

    # Show the plot
    plt.legend(loc='lower center', bbox_to_anchor=(0.5, -0.045))
    plt.show()

if plot_contour_all:
    # Create a figure and subplots
    fig, axes = plt.subplots(1, 3, figsize=(14, 6), sharey=True)  # 1 row, 3 columns of subplots

    for i, battery_energy_density in enumerate(battery_energy_density_list):

        # Read data for the current battery energy density
        data_df = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_without_speed_as_design_var.csv')
        data_df2 = pd.read_csv(f'battery_{battery_energy_density}_Whpkg/results_with_speed_as_design_var.csv')

        # Apply filtering conditions
        data_df.loc[~data_df['success'], 'Weight|takeoff'] = np.nan
        data_df2.loc[~data_df2['success'], 'cruise_speed'] = np.nan

        # Prepare data for contour plot
        range_array = np.unique(data_df['mission_range'].to_numpy())
        speed_array = np.unique(data_df['cruise_speed'].to_numpy())

        X, Y = np.meshgrid(range_array, speed_array)
        Z = data_df['Weight|takeoff'].to_numpy().reshape(len(speed_array), len(range_array))
        infeasible_mask = np.isnan(Z)

        # Plot in the corresponding subplot
        ax = axes[i]
        cp = ax.contourf(X, Y, Z, 15, cmap='viridis', vmin=np.nanmin(Z), vmax=np.nanmax(Z), extend='both')
        ax.contourf(X, Y, infeasible_mask.astype(float), levels=[0.01, 1.5], colors='white', hatches=['///'], alpha=0.8)
        ax.plot(data_df2['mission_range'], data_df2['cruise_speed'], 'r--')
        ax.set_title(f'Battery GED: {battery_energy_density} Wh/kg')
        ax.set_xlabel('Mission range (km)')
        if i == 0:
            ax.set_ylabel('Cruise speed (km/h)')

    # Legends
    legend_elements = []
    legend_elements.append(Line2D([0], [0], color='red', linestyle='--', linewidth=1.5, label='Optimal cruise speed path'))
    legend_elements.append(Patch(facecolor='white', edgecolor='black', hatch='///', label='Infeasible region'))
    fig.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, 0.94), ncol=2, prop={'size': 12})

    # Add a horizontal colorbar below the subplots
    cbar = fig.colorbar(cp, ax=axes, orientation='horizontal', fraction=0.05, pad=0.1)
    cbar.set_label('Optimal MTOW (kg)')

    # Adjust layout to prevent overlap
    fig.suptitle('Weight-based optimization results for Lift+Cruise', size=16)
    plt.subplots_adjust(bottom=0.25, top=0.82, wspace=0.03)
    plt.savefig('weight_opt_range_speed_space_liftcruise.pdf', format='pdf', dpi=300) if savefig else plt.show()
