from MCEVS.Vehicles.Components.Wing import Wing
from MCEVS.Vehicles.Components.Fuselage import Fuselage
from MCEVS.Vehicles.Components.Landing_Gear import LandingGear
from MCEVS.Vehicles.Components.Rotors import LiftRotor, Propeller
from MCEVS.Vehicles.Components.Battery import Battery

from MCEVS.Weights.Container import VehicleWeight

import numpy as np

class LiftPlusCruiseEVTOL(object):
	"""
	docstring for LiftPlusCruiseEVTOL
	This is a type of eVTOL that has a fixed-wing and separate propulsion for hover and cruise.
	"""
	def __init__(self, mtow=None):
		super(LiftPlusCruiseEVTOL, self).__init__()
		
		self.configuration = 'LiftPlusCruise'
		self.n_passenger = None # including the pilot

		# Components
		self.fuselage = None
		self.wing = None
		self.landing_gear = None
		self.empennage = None
		self.lift_rotor = None
		self.propeller = None
		self.battery = None

		# Weight and performance
		self.weight = VehicleWeight(mtow)
		self.performance = None

		# Whether or not this vehicle has been sized
		self.is_sized = False

	def add_component(self, kind:str, **kwargs):

		if kind == 'battery':
			self.battery = Battery(kwargs)
			self.battery._initialize()

		elif kind == 'wing':
			self.wing = Wing(kwargs)
			self.wing._initialize()
			if self.weight.max_takeoff is not None:
				self.wing._calculate_weight_given_mtow(self.weight.max_takeoff)

		elif kind == 'fuselage':
			self.fuselage = Fuselage(kwargs)
			self.fuselage._initialize()
			if self.weight.max_takeoff is not None:
				self.fuselage._calculate_weight_given_mtow(self.weight.max_takeoff)

		elif kind == 'landing_gear':
			self.landing_gear = LandingGear(kwargs)
			self.landing_gear._initialize()
			if self.weight.max_takeoff is not None:
				self.landing_gear._calculate_weight_given_mtow(self.weight.max_takeoff)

		elif kind == 'lift_rotor':
			self.lift_rotor = LiftRotor(kwargs)
			self.lift_rotor._initialize()
			self.lift_rotor._calculate_weight_given_max_power(183068.9599)

		elif kind == 'propeller':
			self.propeller = Propeller(kwargs)
			self.propeller._initialize()
			self.propeller._calculate_weight_given_max_power(35669.31421)

	def print_info(self):
		print('Vehicle type: Lift+Cruise eVTOL')
		print('List of components:')
		print(self.battery._info())
		print(self.fuselage._info())
		print(self.wing._info())
		print(self.landing_gear._info())
		print(self.lift_rotor._info())
		print(self.propeller._info())

class MultirotorEVTOL(object):
	"""
	docstring for MultirotorEVTOL
	This is a type of eVTOL that has a fixed-wing and separate propulsion for hover and cruise.
	"""
	def __init__(self, mtow=None):
		super(MultirotorEVTOL, self).__init__()
		
		self.configuration = 'Multirotor'
		self.n_passenger = None # including the pilot

		# Components
		self.fuselage = None
		self.landing_gear = None
		self.empennage = None
		self.lift_rotor = None
		self.battery = None

		# Weight and performance
		self.weight = VehicleWeight(mtow)
		self.performance = None

		# Whether or not this vehicle has been sized
		self.is_sized = False

	def add_component(self, kind:str, **kwargs):

		if kind == 'battery':
			self.battery = Battery(kwargs)
			self.battery._initialize()

		elif kind == 'fuselage':
			self.fuselage = Fuselage(kwargs)
			self.fuselage._initialize()
			if self.weight.max_takeoff is not None:
				self.fuselage._calculate_weight_given_mtow(self.weight.max_takeoff)

		elif kind == 'landing_gear':
			self.landing_gear = LandingGear(kwargs)
			self.landing_gear._initialize()
			if self.weight.max_takeoff is not None:
				self.landing_gear._calculate_weight_given_mtow(self.weight.max_takeoff)

		elif kind == 'lift_rotor':
			self.lift_rotor = LiftRotor(kwargs)
			self.lift_rotor._initialize()
			self.lift_rotor._calculate_weight_given_max_power(151102.2955)

	def print_info(self):
		print('Vehicle type: Multirotor eVTOL')
		print('List of components:')
		print(self.battery._info())
		print(self.fuselage._info())
		print(self.landing_gear._info())
		print(self.lift_rotor._info())







