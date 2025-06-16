import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Source:
# Tiede, B., O’Meara, C., and Jansen, R., “Battery Key Performance Projections Based on Historical Trends and Chemistries,”
# presented at the 2022 IEEE/AIAA Transportation Electrification Conference and Electric Aircraft Technologies Symposium (ITEC+EATS),
# Anaheim, CA, USA, 2022. https://doi.org/10.1109/ITEC53557.2022.9814008

def Boltzman_Sigmoid_Battery_Pack_GED(t, scenario):

	if scenario == 'conservative':
		models = [9.6, 659.1, 0.0436, 2042.5]
		derating_factor = 0.7
	elif scenario == 'nominal':
		models = [0.0, 801.8, 0.0607, 2030.8]
		derating_factor = 0.8
	elif scenario == 'aggresive':
		models = [32.5, 1009.0, 0.0786, 2030.6]
		derating_factor = 0.9

	a = models[0]; b = models[1]; c = models[2]; tau = models[3]
	res = a + b / (1 + np.exp(c * (tau - t)))
	
	return derating_factor*res

# Battery dictionary
battery_dict = {'conservative':None, 'nominal':None, 'aggresive':None}

# Populating data
year_list = np.arange(1990,2071,1)
for i, scenario in enumerate(battery_dict):
	battery_dict[scenario] = Boltzman_Sigmoid_Battery_Pack_GED(year_list, scenario)

# Current year data
# curr_year = datetime.now().year
curr_year = 2025
conservative_curr_GED = Boltzman_Sigmoid_Battery_Pack_GED(curr_year, 'conservative')
nominal_curr_GED = Boltzman_Sigmoid_Battery_Pack_GED(curr_year, 'nominal')
aggresive_curr_GED = Boltzman_Sigmoid_Battery_Pack_GED(curr_year, 'aggresive')

plt.plot(year_list, battery_dict['conservative'], '-', label='Conservative')
plt.plot(year_list, battery_dict['nominal'], '-', label='Nominal')
plt.plot(year_list, battery_dict['aggresive'], '-', label='Aggresive')
plt.vlines(x=curr_year, ymin=0, ymax=1050, color='k', linestyle='--', alpha=0.5, label=f'Current year ({curr_year})')
plt.plot(curr_year, conservative_curr_GED, 'o', color='#1f77b4', label=f'GED = {np.round(conservative_curr_GED,1)} Wh/kg')
plt.plot(curr_year, nominal_curr_GED, 'o', color='#ff7f0e', label=f'GED = {np.round(nominal_curr_GED,1)} Wh/kg')
plt.plot(curr_year, aggresive_curr_GED, 'o', color='#2ca02c', label=f'GED = {np.round(aggresive_curr_GED,1)} Wh/kg')
plt.ylim([0,1000])
plt.xlabel('Year')
plt.ylabel(r'Pack-level battery GED $[Wh/kg]$')
plt.legend()
plt.grid()
plt.title('Battery technology projection\nby Tiede et al. (2022)')
plt.show()
# plt.savefig('battery_technology_projections.pdf', format='pdf', dpi=300)
