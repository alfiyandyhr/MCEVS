import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

with_fixed_empty_weight = True

plot_objective_all = True
plot_objective_separately = False
plot_constraint_separately = False

plot_compare_optimals = False

battery_energy_density_list = np.arange(250, 725, 25)
selected_battery_energy_density_list = [250, 400, 550]

folder = 'with_fixed_empty_weight' if with_fixed_empty_weight else 'without_fixed_empty_weight'

if plot_objective_all or plot_objective_separately or plot_constraint_separately:

    opt_results_list = []
    opt_results_list.append(pd.read_csv(f'{folder}/selection_by_weight/multirotor/optimization_results.csv'))
    opt_results_list.append(pd.read_csv(f'{folder}/selection_by_energy/multirotor/optimization_results.csv'))
    opt_results_list.append(pd.read_csv(f'{folder}/selection_by_weight/liftcruise/optimization_results.csv'))
    opt_results_list.append(pd.read_csv(f'{folder}/selection_by_energy/liftcruise/optimization_results.csv'))
    test_results_list = []
    test_results_list.append(pd.read_csv(f'{folder}/selection_by_weight/multirotor/test_results.csv'))
    test_results_list.append(pd.read_csv(f'{folder}/selection_by_energy/multirotor/test_results.csv'))
    test_results_list.append(pd.read_csv(f'{folder}/selection_by_weight/liftcruise/test_results.csv'))
    test_results_list.append(pd.read_csv(f'{folder}/selection_by_energy/liftcruise/test_results.csv'))

    metrics = ['Weight|gross_takeoff' if with_fixed_empty_weight else 'Weight|takeoff', 'Energy|entire_mission']
    configs = ['Multirotor', 'Lift+Cruise']

    for i, opt_results in enumerate(opt_results_list):
        for metric in metrics:
            if i in [0, 1]:		# multirotor
                opt_results.loc[opt_results['Weight|residual'] > 0.1, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|Cruise|T_to_P'] > 12.01, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|Cruise|mu'] > 1, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|Cruise|CT/sigma'] > 0.141, metric] = np.nan
            elif i in [2, 3]:  # lift+cruise
                opt_results.loc[opt_results['Weight|residual'] > 0.1, metric] = np.nan
                opt_results.loc[opt_results['Aero|Cruise|CL'] > 0.91, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
                opt_results.loc[opt_results['Propeller|Cruise|T_to_P'] > 12.01, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
                opt_results.loc[opt_results['Propeller|Cruise|J'] > 3.01, metric] = np.nan
                opt_results.loc[opt_results['Propeller|Cruise|CT/sigma'] > 0.141, metric] = np.nan
                opt_results.loc[opt_results['LiftRotor|clearance_constraint'] > 0.1, metric] = np.nan

    for i, test_results in enumerate(test_results_list):
        if i in [0, 2]:
            metric = 'Weight|gross_takeoff' if with_fixed_empty_weight else 'Weight|takeoff'
        if i in [1, 3]:
            metric = 'Energy|entire_mission'
        if with_fixed_empty_weight:
            condition1 = (test_results['Weight|max_takeoff'] < test_results['Weight|gross_takeoff'])
            condition2 = (test_results['battery_energy_density_opt'] != test_results['battery_energy_density_test'])
            test_results[metric] = np.where(condition1 & condition2, np.nan, test_results[metric])
        if i in [0, 1]:		# multirotor
            test_results.loc[test_results['Weight|residual'] > 0.1, metric] = np.nan
            test_results.loc[test_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
            test_results.loc[test_results['LiftRotor|Cruise|T_to_P'] > 12.01, metric] = np.nan
            test_results.loc[test_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
            test_results.loc[test_results['LiftRotor|Cruise|mu'] > 1, metric] = np.nan
            test_results.loc[test_results['LiftRotor|Cruise|CT/sigma'] > 0.141, metric] = np.nan
        elif i in [2, 3]:  # lift+cruise
            test_results.loc[test_results['Weight|residual'] > 0.1, metric] = np.nan
            test_results.loc[test_results['Aero|Cruise|CL'] > 0.91, metric] = np.nan
            test_results.loc[test_results['LiftRotor|HoverClimb|T_to_P'] > 12.01, metric] = np.nan
            test_results.loc[test_results['Propeller|Cruise|T_to_P'] > 12.01, metric] = np.nan
            test_results.loc[test_results['LiftRotor|HoverDescent|T_to_P'] > 12.01, metric] = np.nan
            test_results.loc[test_results['Propeller|Cruise|J'] > 3.01, metric] = np.nan
            test_results.loc[test_results['Propeller|Cruise|CT/sigma'] > 0.141, metric] = np.nan
            test_results.loc[test_results['LiftRotor|clearance_constraint'] > 0.1, metric] = np.nan

    if plot_objective_all:
        fig, axes = plt.subplots(2, 2, figsize=(9, 6), sharex=True)

        for i, metric in enumerate(metrics):
            for j, config in enumerate(configs):
                opt_results = opt_results_list[i + j * 2]  # iterating opt results based on the order of pd.read_csv()
                test_results = test_results_list[i + j * 2]  # iterating test results based on the order of pd.read_csv()
                for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
                    test_results_k = test_results[test_results['battery_energy_density_opt'] == selected_battery_energy_density][metric]
                    axes[i, j].plot(battery_energy_density_list, test_results_k, 'o-', ms=4.0, label=f'Optimized at {selected_battery_energy_density_list[k]} Wh/kg' if i == 0 and j == 0 else None)
                axes[i, j].plot(opt_results['battery_energy_density'], opt_results['Weight|takeoff' if metric == 'Weight|gross_takeoff' or metric == 'Weight|takeoff' else 'Energy|entire_mission'], 'k--', label='Optimized at test battery GED' if i == 0 and j == 0 else None)
                if i == 1:
                    axes[i, j].set_xlabel(r'Test battery GED $[Wh/kg]$')

        axes[0, 0].set_ylabel(r'Gross takeoff weight $[kg]$' if with_fixed_empty_weight else r'Maximum takeoff weight $[kg]$')
        axes[1, 0].set_ylabel(r'Required energy $[kWh]$')
        axes[0, 0].set_title('Weight-minimized multirotor', size=11)
        axes[0, 1].set_title('Weight-minimized lift+cruise', size=11)
        axes[1, 0].set_title('Energy-minimized multirotor', size=11)
        axes[1, 1].set_title('Energy-minimized lift+cruise', size=11)
        fig.legend(loc='upper center', bbox_to_anchor=(0.505, 0.95), ncols=2)
        fig.suptitle('Suboptimal performance evaluation on various test battery GEDs', size=13)
        plt.tight_layout(rect=(0, 0, 1, 0.92))
        plt.show()

    if plot_objective_separately:

        for i, metric in enumerate(metrics):
            for j, config in enumerate(configs):
                opt_results = opt_results_list[i + j * 2]  # iterating opt results based on the order of pd.read_csv()
                test_results = test_results_list[i + j * 2]  # iterating test results based on the order of pd.read_csv()
                for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
                    test_results_k = test_results[test_results['battery_energy_density_opt'] == selected_battery_energy_density][metric]
                    plt.plot(battery_energy_density_list, test_results_k, 'o-', ms=4.0, label=f'Optimized at {selected_battery_energy_density_list[k]} Wh/kg')
                plt.plot(opt_results['battery_energy_density'], opt_results['Weight|takeoff' if metric == 'Weight|gross_takeoff' or metric == 'Weight|takeoff' else 'Energy|entire_mission'], 'k--', label='Optimized at test battery GED')
                plt.xlabel(r'Test battery GED $[Wh/kg]$')
                if metric == 'Weight|takeoff' or metric == 'Weight|gross_takeoff':
                    plt.ylabel(r'Gross takeoff weight $[kg]$' if with_fixed_empty_weight else r'Maximum takeoff weight $[kg]$')
                    plt.title(f'Suboptimal performance evaluation\nfor {config} from weight-based optimization')
                if metric == 'Energy|entire_mission':
                    plt.ylabel(r'Required energy $[kWh]$')
                    plt.title(f'Suboptimal performance evaluation\nfor {config} from energy-based optimization')
                plt.legend()
                plt.show()

    if plot_constraint_separately:

        for i, metric in enumerate(metrics):
            for j, config in enumerate(configs):

                test_results = test_results_list[i + j * 2]  # iterating test results based on the order of pd.read_csv()
                xmin = battery_energy_density_list[0]
                xmax = battery_energy_density_list[-1]

                if 2 * i + j in [0, 2]:

                    # skewed data
                    if 2 * i + j == 0:
                        test_results.loc[(test_results['battery_energy_density_opt'] == 850) & (test_results['battery_energy_density_test'] == 250), 'Weight|residual'] = np.nan

                    ylabels = ['LiftRotor|Cruise|CT/sigma', 'LiftRotor|Cruise|mu', 'Weight|residual',
                               'LiftRotor|HoverClimb|T_to_P', 'LiftRotor|Cruise|T_to_P', 'LiftRotor|HoverDescent|T_to_P']
                    ylabel_names = [r'Blade loading $CT/\sigma$', r'Advance ratio $\mu$', r'Squared residual $[kg^2]$', r'$T/P$ HoverClimb $[g/W]$', r'$T/P$ Cruise $[g/W]$', r'$T/P$ HoverDescent $[g/W]$']

                    fig, axes = plt.subplots(2, 3, sharex=True, figsize=(12, 7))
                    axes = axes.flatten()

                    for l, ylabel in enumerate(ylabels):  # noqa: E741
                        for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
                            test_results_k = test_results[test_results['battery_energy_density_opt'] == selected_battery_energy_density][ylabel]
                            axes[l].plot(battery_energy_density_list, test_results_k, 'o-', ms=3.0, label=f'Optimized at {selected_battery_energy_density} Wh/kg' if l == 0 else None)
                            if ylabel == 'LiftRotor|Cruise|CT/sigma':
                                axes[l].hlines(0.14, xmin=xmin, xmax=xmax, linestyles='--', color='k', label='Constraint upper limit' if k == 2 else None)
                            if ylabel == 'LiftRotor|Cruise|mu':
                                axes[l].hlines(1.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'LiftRotor|HoverClimb|T_to_P':
                                axes[l].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'LiftRotor|Cruise|T_to_P':
                                axes[l].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'LiftRotor|HoverDescent|T_to_P':
                                axes[l].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if l in [3, 4, 5]:
                                axes[l].set_xlabel('Test battery GED [Wh/kg]')
                            axes[l].set_ylabel(ylabel_names[l])
                    axes[2].ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
                    if 2 * i + j == 0:
                        fig.suptitle('Constraint evaluation with varying battery GED for Multirotor from weight-based optimization')
                    if 2 * i + j == 2:
                        fig.suptitle('Constraint evaluation with varying battery GED for Multirotor from energy-based optimization')
                    fig.legend(ncols=2, bbox_to_anchor=(0.49, 0.95), loc='upper center')
                    plt.tight_layout(rect=(0, 0, 1, 0.93))
                    plt.show()

                if 2 * i + j in [1, 3]:

                    ylabels = ['Aero|Cruise|CL', 'Propeller|Cruise|CT/sigma', 'Propeller|Cruise|J', 'Weight|residual',
                               'LiftRotor|HoverClimb|T_to_P', 'Propeller|Cruise|T_to_P', 'LiftRotor|HoverDescent|T_to_P', 'LiftRotor|clearance_constraint']
                    ylabel_names = [r'Lift coeff $C_{L}$', r'Prop blade loading $CT/\sigma$', r'Prop advance ratio $J$', r'Squared residual $[kg^2]$',
                                    r'$T/P$ HoverClimb $[g/W]$', r'$T/P$ Cruise $[g/W]$', r'$T/P$ HoverDescent $[g/W]$', r'Clearance constraint $[m]$']

                    fig, axes = plt.subplots(2, 4, sharex=True, figsize=(14, 7))
                    axes = axes.flatten()

                    for l, ylabel in enumerate(ylabels):  # noqa: E741
                        for k, selected_battery_energy_density in enumerate(selected_battery_energy_density_list):
                            test_results_k = test_results[test_results['battery_energy_density_opt'] == selected_battery_energy_density][ylabel]
                            axes[l].plot(battery_energy_density_list, test_results_k, 'o-', ms=3.0, label=f'Optimized at {selected_battery_energy_density} Wh/kg' if l == 0 else None)

                            if ylabel == 'Aero|Cruise|CL':
                                axes[l].hlines(0.9, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'Propeller|Cruise|CT/sigma':
                                axes[l].hlines(0.14, xmin=xmin, xmax=xmax, linestyles='--', color='k', label='Constraint upper limit' if k == 2 else None)
                            if ylabel == 'Propeller|Cruise|J':
                                axes[l].hlines(3.001, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'LiftRotor|HoverClimb|T_to_P':
                                axes[l].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'Propeller|Cruise|T_to_P':
                                axes[l].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'LiftRotor|HoverDescent|T_to_P':
                                axes[l].hlines(12.0, xmin=xmin, xmax=xmax, linestyles='--', color='k')
                            if ylabel == 'LiftRotor|clearance_constraint':
                                axes[l].hlines(1e-5, xmin=xmin, xmax=xmax, linestyles='--', color='k')

                            if l in [4, 5, 6, 7]:
                                axes[l].set_xlabel('Test battery GED [Wh/kg]')
                            axes[l].set_ylabel(ylabel_names[l])
                    axes[3].ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
                    if 2 * i + j == 1:
                        fig.suptitle('Constraint evaluation with varying battery GED for Lift+Cruise from weight-based optimization')
                    if 2 * i + j == 3:
                        fig.suptitle('Constraint evaluation with varying battery GED for Lift+Cruise from energy-based optimization')
                    fig.legend(ncols=2, bbox_to_anchor=(0.49, 0.95), loc='upper center')
                    plt.tight_layout(rect=(0, 0, 1, 0.93))
                    plt.show()

if plot_compare_optimals:
    opt_results11 = pd.read_csv(f'{folder}/selection_by_weight/multirotor/optimization_results.csv')
    opt_results12 = pd.read_csv(f'{folder}/selection_by_weight/liftcruise/optimization_results.csv')
    opt_results21 = pd.read_csv(f'{folder}/selection_by_energy/multirotor/optimization_results.csv')
    opt_results22 = pd.read_csv(f'{folder}/selection_by_energy/liftcruise/optimization_results.csv')

    # Processing constraints
    opt_results11.loc[opt_results11['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
    opt_results11.loc[opt_results11['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    opt_results11.loc[opt_results11['LiftRotor|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    opt_results11.loc[opt_results11['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    opt_results11.loc[opt_results11['LiftRotor|Cruise|mu'] > 1, 'Weight|takeoff'] = np.nan
    opt_results11.loc[opt_results11['LiftRotor|Cruise|CT/sigma'] > 0.141, 'Weight|takeoff'] = np.nan

    opt_results21.loc[opt_results21['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
    opt_results21.loc[opt_results21['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    opt_results21.loc[opt_results21['LiftRotor|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    opt_results21.loc[opt_results21['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    opt_results21.loc[opt_results21['LiftRotor|Cruise|mu'] > 1, 'Energy|entire_mission'] = np.nan
    opt_results21.loc[opt_results21['LiftRotor|Cruise|CT/sigma'] > 0.141, 'Energy|entire_mission'] = np.nan

    opt_results12.loc[opt_results12['Weight|residual'] > 0.1, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['Aero|Cruise|CL'] > 0.91, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['Propeller|Cruise|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['Propeller|Cruise|J'] > 3.01, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['Propeller|Cruise|CT/sigma'] > 0.141, 'Weight|takeoff'] = np.nan
    opt_results12.loc[opt_results12['LiftRotor|clearance_constraint'] > 0.1, 'Weight|takeoff'] = np.nan

    opt_results22.loc[opt_results22['Weight|residual'] > 0.1, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['Aero|Cruise|CL'] > 0.91, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['LiftRotor|HoverClimb|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['Propeller|Cruise|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['LiftRotor|HoverDescent|T_to_P'] > 12.01, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['Propeller|Cruise|J'] > 3.01, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['Propeller|Cruise|CT/sigma'] > 0.141, 'Energy|entire_mission'] = np.nan
    opt_results22.loc[opt_results22['LiftRotor|clearance_constraint'] > 0.1, 'Energy|entire_mission'] = np.nan

    fig, axes = plt.subplots(1, 2, figsize=(8, 4))

    # Weight-based optimization
    axes[0].plot(opt_results11['battery_energy_density'], opt_results11['Weight|takeoff'], 'r-', label='multirotor')
    axes[0].plot(opt_results12['battery_energy_density'], opt_results12['Weight|takeoff'], 'b-', label='lift+cruise')
    axes[0].set_xlabel(r'Test battery GED $[Wh/kg]$')
    axes[0].set_ylabel(r'Gross takeoff weight $[kg]$' if with_fixed_empty_weight else r'Maximum takeoff weight $[kg]$')
    axes[0].set_title('Weight-based optimization', size=10)

    # Energy-based optimization
    axes[1].plot(opt_results21['battery_energy_density'], opt_results21['Energy|entire_mission'], 'r-')
    axes[1].plot(opt_results22['battery_energy_density'], opt_results22['Energy|entire_mission'], 'b-')
    axes[1].set_xlabel(r'Test battery GED $[Wh/kg]$')
    axes[1].set_ylabel(r'Required energy $[kWh]$', rotation=270, labelpad=15)
    axes[1].yaxis.set_label_position('right')
    axes[1].yaxis.tick_right()
    axes[1].set_title('Energy-based optimization', size=10)

    fig.legend(loc='upper center', bbox_to_anchor=(0.48, 0.94), ncols=2)
    fig.suptitle('Optimal designs with varying battery GED')
    plt.tight_layout(rect=(0, 0, 1, 0.95))
    plt.show()
