import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weight.Equipment.Avionics import AvionicsWeight
from MCEVS.Analyses.Weight.Equipment.Flight_Control import FlightControlWeight
from MCEVS.Analyses.Weight.Equipment.Anti_Icing import AntiIcingSystemWeight
from MCEVS.Analyses.Weight.Equipment.Furnishings import FurnishingWeight

class EquipmentWeight(om.Group):
	"""
	Calculates equipment weight
		W_equipment = W_avionics + W_flight_control + W_anti_icing + W_furnishings
	"""
	def setup(self):

		# Avionics weight
		self.add_subsystem('avionics_weight',
							AvionicsWeight(),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|avionics'])

		# Flight control weight
		self.add_subsystem('flight_control_weight',
							FlightControlWeight(),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|flight_control'])

		# Anti icing weight
		self.add_subsystem('anti_icing_weight',
							AntiIcingSystemWeight(),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|anti_icing'])

		# Furnishing weight
		self.add_subsystem('furnishing_weight',
							FurnishingWeight(),
							promotes_inputs=['Weight|takeoff'],
							promotes_outputs=['Weight|furnishings'])

		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weight|equipment',
							input_names=['Weight|avionics', 'Weight|flight_control', 'Weight|anti_icing', 'Weight|furnishings'],
							units='kg',
							scaling_factors=[1., 1., 1., 1.])
		self.add_subsystem('equipment_sum_weight',
							adder,
							promotes_inputs=['Weight|*'],
							promotes_outputs=['Weight|equipment'])