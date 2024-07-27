import numpy as np
import openmdao.api as om

from MCEVS.Analyses.Weights.Equipment.Avionics import AvionicsWeight
from MCEVS.Analyses.Weights.Equipment.Flight_Control import FlightControlWeight
from MCEVS.Analyses.Weights.Equipment.Anti_Icing import AntiIcingSystemWeight
from MCEVS.Analyses.Weights.Equipment.Furnishings import FurnishingWeight

class EquipmentWeight(om.Group):
	"""
	Calculates equipment weight
		W_equipment = W_avionics + W_flight_control + W_anti_icing + W_furnishings
	"""
	def setup(self):

		# Avionics weight
		self.add_subsystem('avionics_weight',
							AvionicsWeight(),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|Avionics'])

		# Flight control weight
		self.add_subsystem('flight_control_weight',
							FlightControlWeight(),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|Flight_control'])

		# Anti icing weight
		self.add_subsystem('anti_icing_weight',
							AntiIcingSystemWeight(),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|Anti_icing'])

		# Furnishing weight
		self.add_subsystem('furnishing_weight',
							FurnishingWeight(),
							promotes_inputs=['eVTOL|W_takeoff'],
							promotes_outputs=['Weights|Furnishings'])

		# Sum up
		adder = om.AddSubtractComp()
		adder.add_equation('Weights|Equipment',
							input_names=['Weights|Avionics', 'Weights|Flight_control', 'Weights|Anti_icing', 'Weights|Furnishings'],
							units='kg',
							scaling_factors=[1., 1., 1., 1.])
		self.add_subsystem('equipment_sum_weight',
							adder,
							promotes_inputs=['Weights|*'],
							promotes_outputs=['Weights|Equipment'])