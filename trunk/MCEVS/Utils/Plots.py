import numpy as np
import matplotlib.pyplot as plt

def plot_mission_parameters(mission:object, print_info=False, save_fig=False):

	if print_info:
		mission.print_info()

	# Preprocessing data
	t = np.array([])
	x, y = np.array([]), np.array([])
	vx, vy = np.array([]), np.array([])
	ax, ay = np.array([]), np.array([])
	for t_segment in mission.t:
		t = np.concatenate((t, t_segment))
	for x_segment in mission.x:
		x = np.concatenate((x, x_segment))
	for y_segment in mission.y:
		y = np.concatenate((y, y_segment))
	for vx_segment in mission.vx:
		vx = np.concatenate((vx, vx_segment))
	for vy_segment in mission.vy:
		vy = np.concatenate((vy, vy_segment))
	for ax_segment in mission.ax:
		ax = np.concatenate((ax, ax_segment))
	for ay_segment in mission.ay:
		ay = np.concatenate((ay, ay_segment))

	# Plot position
	plt.figure(figsize=(6,3.5))
	plt.plot(x/1000.0, y, '-o')
	plt.xlabel('Range ($km$) in X-dir')
	plt.ylabel('AGL Altitude ($m$) in Y-dir')
	plt.title('Aircraft position y vs x')
	plt.minorticks_on()
	plt.grid(which='major', linewidth=0.8)
	plt.grid(which='minor', linewidth=0.2)
	if save_fig:
		plt.tight_layout()
		plt.savefig('range_vs_altitude.pdf', format='pdf')
	plt.show()

	# Plot bulk
	fig, axs = plt.subplots(nrows=3, ncols=2, sharex=False, figsize=(12,9))
	axs[0,0].plot(t/60.0, x/1000.0, '-o')
	axs[0,0].set_ylabel('Range ($km$) in X-dir')
	axs[0,0].minorticks_on()
	axs[0,0].grid(which='major', linewidth=0.8)
	axs[0,0].grid(which='minor', linewidth=0.2)
	axs[0,1].plot(t/60.0, y, '-o')
	axs[0,1].set_ylabel('AGL Altitude ($m$) in Y-dir')
	axs[0,1].minorticks_on()
	axs[0,1].grid(which='major', linewidth=0.8)
	axs[0,1].grid(which='minor', linewidth=0.2)
	axs[1,0].plot(t/60.0, vx*3.6, '-o')
	axs[1,0].set_ylabel('Velocity x-dir ($km/h$)')
	axs[1,0].minorticks_on()
	axs[1,0].grid(which='major', linewidth=0.8)
	axs[1,0].grid(which='minor', linewidth=0.2)
	axs[1,1].plot(t/60.0, vy, '-o')
	axs[1,1].set_ylabel('Velocity y-dir ($m/s$)')
	axs[1,1].minorticks_on()
	axs[1,1].grid(which='major', linewidth=0.8)
	axs[1,1].grid(which='minor', linewidth=0.2)
	axs[2,0].plot(t/60.0, ax, '-o')
	axs[2,0].set_ylabel('Acceleration x-dir ($m/s^2$)')
	axs[2,0].set_xlabel('Time ($min$)')
	axs[2,0].minorticks_on()
	axs[2,0].grid(which='major', linewidth=0.8)
	axs[2,0].grid(which='minor', linewidth=0.2)
	axs[2,1].plot(t/60.0, ay, '-o')
	axs[2,1].set_ylabel('Acceleration y-dir ($m/s^2$)')
	axs[2,1].set_xlabel('Time ($min$)')
	axs[2,1].minorticks_on()
	axs[2,1].grid(which='major', linewidth=0.8)
	axs[2,1].grid(which='minor', linewidth=0.2)
	if save_fig:
		plt.savefig('flight_params.pdf', format='pdf')
	plt.show()

def plot_performance_by_segments(mission:object, vehicle:object):

	# Preprocessing data
	t = np.array([])
	
	if vehicle.configuration == 'LiftPlusCruise':

		P_propeller, P_lift_rotor = np.array([]), np.array([])
		DL_propeller, DL_lift_rotor = np.array([]), np.array([])

		for t_segment in mission.t:
			t = np.concatenate((t, t_segment))
		for P in mission.P['Propeller']:
			P_propeller = np.concatenate((P_propeller, P))
		for P in mission.P['LiftRotor']:
			P_lift_rotor = np.concatenate((P_lift_rotor, P))
		for DL in mission.DL['Propeller']:
			DL_propeller = np.concatenate((DL_propeller, DL))
		for DL in mission.DL['LiftRotor']:
			DL_lift_rotor = np.concatenate((DL_lift_rotor, DL))

		# Plot bulk
		fig, axs = plt.subplots(nrows=2, ncols=2, sharex=False)
		axs[0,0].plot(t/60.0, P_lift_rotor/1000.0, '-o')
		axs[0,0].set_ylabel('LiftRotor power total ($kW$)')
		axs[0,0].minorticks_on()
		axs[0,0].grid(which='major', linewidth=0.8)
		axs[0,0].grid(which='minor', linewidth=0.2)
		axs[0,1].plot(t/60.0, P_propeller/1000.0, '-o')
		axs[0,1].set_ylabel('Propeller power total ($kW$)')
		axs[0,1].minorticks_on()
		axs[0,1].grid(which='major', linewidth=0.8)
		axs[0,1].grid(which='minor', linewidth=0.2)
		axs[1,0].plot(t/60.0, DL_lift_rotor, '-o')
		axs[1,0].set_ylabel('LiftRotor disk loading each ($N/m^2$)')
		axs[1,0].minorticks_on()
		axs[1,0].grid(which='major', linewidth=0.8)
		axs[1,0].grid(which='minor', linewidth=0.2)
		axs[1,0].set_xlabel('Time ($min$)')
		axs[1,1].plot(t/60.0, DL_propeller, '-o')
		axs[1,1].set_ylabel('Propeller disk loading each ($N/m^2$)')
		axs[1,1].minorticks_on()
		axs[1,1].grid(which='major', linewidth=0.8)
		axs[1,1].grid(which='minor', linewidth=0.2)
		axs[1,1].set_xlabel('Time ($min$)')

		plt.show()

	elif vehicle.configuration == 'Multirotor':

		P_lift_rotor, DL_lift_rotor = np.array([]), np.array([])

		for t_segment in mission.t:
			t = np.concatenate((t, t_segment))
		for P in mission.P['LiftRotor']:
			P_lift_rotor = np.concatenate((P_lift_rotor, P))
		for DL in mission.DL['LiftRotor']:
			DL_lift_rotor = np.concatenate((DL_lift_rotor, DL))

		# Plot bulk
		fig, axs = plt.subplots(nrows=1, ncols=2, sharex=False)
		axs[0].plot(t/60.0, P_lift_rotor/1000.0, '-o')
		axs[0].set_ylabel('Rotor power total ($kW$)')
		axs[0].minorticks_on()
		axs[0].grid(which='major', linewidth=0.8)
		axs[0].grid(which='minor', linewidth=0.2)
		axs[0].set_xlabel('Time ($min$)')
		axs[1].plot(t/60.0, DL_lift_rotor, '-o')
		axs[1].set_ylabel('Rotor disk loading each ($N/m^2$)')
		axs[1].minorticks_on()
		axs[1].grid(which='major', linewidth=0.8)
		axs[1].grid(which='minor', linewidth=0.2)
		axs[1].set_xlabel('Time ($min$)')

		plt.show()










