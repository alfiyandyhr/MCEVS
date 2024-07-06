import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

config = 'multirotor'

# Case reader
cr = om.CaseReader(f'{config}_opt_cases.sql')

driver_cases = cr.list_cases('driver')

# cur_case
hist_array = np.zeros((len(driver_cases),7))
for i, case in enumerate(driver_cases):
	cur_case = cr.get_case(case)
	dv1 = cur_case.get_design_vars()['eVTOL|Cruise_speed'][0]
	dv2 = cur_case.get_design_vars()['Rotor|radius_lift'][0]
	dv3 = cur_case.get_design_vars()['Rotor|mu'][0]
	F = cur_case.get_objectives()["eVTOL|W_takeoff"][0]
	G1 = cur_case.get_constraints()['disk_loading_hover'][0]
	G2 = cur_case.get_constraints()['disk_loading_cruise'][0]
	CV = max(G1-500, 0) + max(G2-500, 0)
	cur_arr = np.array([dv1, dv2, dv3, F, G1, G2, CV])
	hist_array[i] = cur_arr

