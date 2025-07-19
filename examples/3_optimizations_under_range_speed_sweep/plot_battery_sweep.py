import pandas as pd
import matplotlib.pyplot as plt

plot_weight_battery_sweep = True
plot_energy_battery_sweep = True

savefig = False

if plot_weight_battery_sweep:
    data1 = pd.read_csv('battery_sweep/weight_battery_sweep_multirotor.csv')
    data2 = pd.read_csv('battery_sweep/weight_battery_sweep_liftcruise.csv')
    data1 = data1[data1['success']]
    data2 = data2[data2['success']]

    fig, ax = plt.subplots(figsize=(5, 4))

    ax.plot(data1['battery_energy_density'], data1['Weight|takeoff'], 'o-', ms=4, label='Multirotor')
    ax.plot(data2['battery_energy_density'], data2['Weight|takeoff'], 'o-', ms=4, label='Lift+Cruise')
    ax.set_xlabel('Battery GED (Wh/kg)', labelpad=10)
    ax.set_ylabel('Optimal Takeoff Weight (kg)', labelpad=10)
    fig.suptitle('Optimal Takeoff Weight vs Battery GED\nRange= 60 km; Speed= 200 km/h', y=0.94, fontsize=12)
    ax.legend(loc='best', frameon=True)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig('figures/weight_battery_sweep.pdf', format='pdf', dpi=300) if savefig else plt.show()

if plot_energy_battery_sweep:
    data1 = pd.read_csv('battery_sweep/energy_battery_sweep_multirotor.csv')
    data2 = pd.read_csv('battery_sweep/energy_battery_sweep_liftcruise.csv')
    data1 = data1[data1['success']]
    data2 = data2[data2['success']]

    fig, ax = plt.subplots(figsize=(5, 4))

    ax.plot(data1['battery_energy_density'], data1['Energy|entire_mission'], 'o-', ms=4, label='Multirotor')
    ax.plot(data2['battery_energy_density'], data2['Energy|entire_mission'], 'o-', ms=4, label='Lift+Cruise')
    ax.set_xlabel('Battery GED (Wh/kg)', labelpad=10)
    ax.set_ylabel('Optimal Energy (kWh)', labelpad=10)
    fig.suptitle('Optimal Energy vs Battery GED\nRange= 60 km; Speed= 200 km/h', y=0.94, fontsize=12)
    ax.legend(loc='best', frameon=True)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig('figures/energy_battery_sweep.pdf', format='pdf', dpi=300) if savefig else plt.show()

    fig, ax = plt.subplots(figsize=(5, 4))

    ax.plot(data1['battery_energy_density'], data1['Weight|takeoff'] * 9.81 * 200 * 1000 / 3600 / data1['Power|segment_3'] / 1000, 'o-', ms=4, label='Multirotor')
    ax.plot(data2['battery_energy_density'], data2['Weight|takeoff'] * 9.81 * 200 * 1000 / 3600 / data2['Power|segment_3'] / 1000, 'o-', ms=4, label='Lift+Cruise')
    ax.set_xlabel('Battery GED (Wh/kg)', labelpad=10)
    ax.set_ylabel(r'$L/D_\text{e} = W \cdot v \, / \, P$', labelpad=10)
    fig.suptitle(r'$L/D_\text{e}$ vs Battery GED' + '\nRange= 60 km; Speed= 200 km/h', y=0.94, fontsize=12)
    ax.legend(loc='best', frameon=True)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig('figures/energy_battery_sweep_L_by_D.pdf', format='pdf', dpi=300) if savefig else plt.show()
