import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plot_power_breakdown = False
plot_type_of_speeds = True

data = pd.read_csv('multirotor_P_vs_v.csv')

if plot_power_breakdown:
	fig, ax = plt.subplots()

	ax.plot(data['cruise_speed'], data['cruise_power'], label='Total cruise power')
	ax.plot(data['cruise_speed'], data['profile_power'], '--', label='Profile power')
	ax.plot(data['cruise_speed'], data['induced_power'], '--', label='Induced power')
	ax.plot(data['cruise_speed'], data['propulsive_power'], '--', label='Propulsive power')
	ax.grid()
	ax.set_ylabel('Power [kW]')
	ax.set_xlabel('Cruise speed [km/h]')
	ax.legend()
	ax.set_title('Multirotor cruise power breakdown')
	plt.show()
	# plt.savefig('cruise_power_breakdown_multirotor.png', format='png', dpi=300)

if plot_type_of_speeds:

	# Polyfit the data
	cruise_power_polycoeffs = np.polyfit(data['cruise_speed'], data['cruise_power'], 6)
	cruise_power_polytrends = np.poly1d(cruise_power_polycoeffs)

	# Compute best endurance speed
	crit = cruise_power_polytrends.deriv().r
	r_crit = crit[crit.imag==0].real
	test = cruise_power_polytrends.deriv(2)(r_crit)
	v_BE = r_crit[test>0]
	P_BE = cruise_power_polytrends(v_BE)

	# Compute best range speed
	m = 1.168
	y_grad, x_grad = m*np.arange(0, 310, 10), np.arange(0, 310, 10)
	roots = np.roots(cruise_power_polycoeffs - [0, 0, 0, 0, 0, m, 0])
	roots = roots[np.isreal(roots)]
	v_BR = (roots[2].real + roots[3].real)/2
	P_BR = cruise_power_polytrends(v_BR)

	fig, ax = plt.subplots()
	ax.plot(data['cruise_speed'], cruise_power_polytrends(data['cruise_speed']), '-', label='Total cruise power')
	ax.plot(x_grad, y_grad, 'k--', linewidth=1.0)

	# Plot best endurance point
	ax.plot(v_BE, P_BE, 'go', label='Best endurance')
	xmin, xmax = ax.get_xlim()
	ymin, ymax = ax.get_ylim()
	ax.axhline(y=P_BE, xmin=0, xmax=(v_BE[0]-xmin)/(xmax-xmin), color='g', linestyle='--', linewidth=1.0)
	ax.axvline(x=v_BE, ymin=0, ymax=(P_BE[0]-ymin)/(ymax-ymin), color='g', linestyle='--', linewidth=1.0)
	ax.annotate(f'({np.round(v_BE[0],2)}, {np.round(P_BE[0],2)})', (np.round(v_BE[0],2)-40.0, np.round(P_BE[0],2)-50.0), textcoords="offset points", xytext=(0,10), ha='center')

	# Plot best range point
	ax.plot(v_BR, P_BR, 'ro', label='Best range')
	ax.axhline(y=P_BR, xmin=0, xmax=(v_BR-xmin)/(xmax-xmin), color='r', linestyle='--', linewidth=1.0)
	ax.axvline(x=v_BR, ymin=0, ymax=(P_BR-ymin)/(ymax-ymin), color='r', linestyle='--', linewidth=1.0)
	ax.annotate(f'({np.round(v_BR,2)}, {np.round(P_BR,2)})', (np.round(v_BR,2)-35.0, np.round(P_BR,2)), textcoords="offset points", xytext=(0,10), ha='center')

	ax.set_ylabel('Power [kW]')
	ax.set_xlabel('Cruise speed [km/h]')	
	ax.grid()
	ax.set_title('Multirotor cruise power vs speed')
	ax.legend()
	# plt.show()
	plt.savefig('cruise_power_vs_speed_multirotor.png', format='png', dpi=300)