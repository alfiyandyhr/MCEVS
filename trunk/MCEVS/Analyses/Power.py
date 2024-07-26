import numpy as np

class PowerAnalysis(object):
	"""
	docstring for PowerAnalysis
	"""
	def __init__(self, vehicle:object, mission:object):
		super(PowerAnalysis, self).__init__()
		self.vehicle = vehicle
		self.mission = mission
		self.power = []

	def evaluate(self):

		# Analyze per segment
		for i, segment in enumerate(self.mission.segments):
			if segment.kind == 'HoverClimbConstantSpeed':
				p = evaluate_power_HoverClimbConstantSpeed(self.vehicle, segment, self.mission.t[i])

			elif segment.kind == 'CruiseConstantSpeed':
				p = evaluate_power_CruiseConstantSpeed(self.vehicle, segment, self.mission.t[i])

			elif segment.kind == 'HoverDescentConstantSpeed':
				p = evaluate_power_HoverDescentConstantSpeed(self.vehicle, segment, self.mission.t[i])

			# Appendin power calculation
			self.power.append(p)

		return self.power
		
def evaluate_power_HoverClimbConstantSpeed(vehicle, segment, timestamp):
	rho = 1.225 # kg/m**3
	g = 9.81 # m/s**2

	FM = vehicle.lift_rotor.figure_of_merit
	W = vehicle.weight.max_takeoff * g
	V = segment.speed
	r = vehicle.lift_rotor.radius
	n_rotor = vehicle.lift_rotor.n_rotor
	S_disk = n_rotor * np.pi * r**2

	Vh = np.sqrt(W/(2*rho*S_disk))

	# Power of hover climb under constant speed
	P = (1/FM) * W * ((V/2) + np.sqrt((V/2)**2 + Vh**2))

	P = P * np.ones_like(timestamp)

	return P

def evaluate_power_CruiseConstantSpeed(vehicle, segment, timestamp):
	
	P = np.ones_like(timestamp)

	return P

def evaluate_power_HoverDescentConstantSpeed(vehicle, segment, timestamp):
	rho = 1.225 # kg/m**3
	g = 9.81 # m/s**2

	FM = vehicle.lift_rotor.figure_of_merit
	W = vehicle.weight.max_takeoff * g
	V = segment.speed
	r = vehicle.lift_rotor.radius
	n_rotor = vehicle.lift_rotor.n_rotor
	S_disk = n_rotor * np.pi * r**2

	Vh = np.sqrt(W/(2*rho*S_disk))

	# Power of hover descent under constant speed
	if V < 2*Vh:
		V = 2*Vh # just like hovering

	P = (1/FM) * W * ((V/2) - np.sqrt((V/2)**2 - Vh**2))

	P = P * np.ones_like(timestamp)

	return P












