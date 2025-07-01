import pandas as pd
import matplotlib.pyplot as plt

savefig = False

data1 = pd.read_csv('weight_battery_sweep/weight_battery_sweep_multirotor.csv')
data2 = pd.read_csv('weight_battery_sweep/weight_battery_sweep_liftcruise.csv')

fig, ax = plt.subplots(figsize=(5, 4))

ax.plot(data1['battery_energy_density'], data1['Weight|takeoff'], 'o-', ms=4, label='Multirotor')
ax.plot(data2['battery_energy_density'], data2['Weight|takeoff'], 'o-', ms=4, label='Lift+Cruise')
ax.set_xlabel('Battery GED (Wh/kg)', labelpad=10)
ax.set_ylabel('Optimal Takeoff Weight (kg)', labelpad=10)
fig.suptitle('Optimal Takeoff Weight vs Battery GED\nRange= 60 km; Speed= 200 km/h', y=0.94, fontsize=12)
ax.legend(loc='best', frameon=True)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig('figures/weight_battery_sweep.pdf', format='pdf', dpi=300) if savefig else plt.show()
