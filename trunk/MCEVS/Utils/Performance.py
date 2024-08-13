import numpy as np

def record_performance_by_segments(prob:object, config:str, mission:object):

	constant_segments 		= ['HoverClimbConstantSpeed', 'ClimbConstantVyConstantVx', 'CruiseConstantSpeed', 'DescentConstantVyConstantVx', 'HoverDescentConstantSpeed', 'HoverStay']

	if config == 'LiftPlusCruise':

		mission.P['Propeller'], mission.P['LiftRotor']  	= [], []
		mission.DL['Propeller'], mission.DL['LiftRotor'] 	= [], []

		for i, segment in enumerate(mission.segments):
			if segment.kind in constant_segments:
				P_propeller 	= np.ones_like(mission.t[i]) * prob.get_val(f'Power|Propeller|segment_{i+1}')[0]
				P_lift_rotor 	= np.ones_like(mission.t[i]) * prob.get_val(f'Power|LiftRotor|segment_{i+1}')[0]
				DL_propeller 	= np.ones_like(mission.t[i]) * prob.get_val(f'DiskLoading|Propeller|segment_{i+1}')[0]
				DL_lift_rotor 	= np.ones_like(mission.t[i]) * prob.get_val(f'DiskLoading|LiftRotor|segment_{i+1}')[0]
				
				mission.P['Propeller'].append( P_propeller )
				mission.P['LiftRotor'].append( P_lift_rotor )
				mission.DL['Propeller'].append( DL_propeller )
				mission.DL['LiftRotor'].append( DL_lift_rotor )	

	elif config == 'Multirotor':

		mission.P['LiftRotor'], mission.DL['LiftRotor'] = [], []

		for i, segment in enumerate(mission.segments):
			if segment.kind in constant_segments:
				P_lift_rotor 	= np.ones_like(mission.t[i]) * prob.get_val(f'Power|LiftRotor|segment_{i+1}')[0]
				DL_lift_rotor 	= np.ones_like(mission.t[i]) * prob.get_val(f'DiskLoading|LiftRotor|segment_{i+1}')[0]
				
				mission.P['LiftRotor'].append( P_lift_rotor )
				mission.DL['LiftRotor'].append( DL_lift_rotor )