from MCEVS.Vehicles.Components.Wing import Wing
from MCEVS.Vehicles.Components.Fuselage import Fuselage
from MCEVS.Vehicles.Components.Landing_Gear import LandingGear
from MCEVS.Vehicles.Components.Rotors import LiftRotor, Propeller
from MCEVS.Vehicles.Components.Battery import Battery

import numpy as np

class LiftPlusCruiseEVTOL(object):
	"""
	docstring for LiftPlusCruiseEVTOL
	This is a type of eVTOL that has a fixed-wing and separate propulsion for hover and cruise.
	"""
	def __init__(self):
		super(LiftPlusCruiseEVTOL, self).__init__()
		
		self.configuration = 'LiftPlusCruise'
		self.n_passenger = None # including the pilot

		# Components
		self.fuselage = None
		self.wing = None
		self.landing_gear = None
		self.empennage = None
		self.lift_rotor = None
		self.cruise_propeller = None
		self.battery = None

		# Weight and performance
		self.mass_properties = None
		self.performance = None

	def add_component(self, kind:str, **kwargs):

		if kind == 'battery':
			self.battery = Battery(kwargs)
			self.battery._initialize()

		elif kind == 'wing':
			self.wing = Wing(kwargs)
			self.wing._initialize()

		elif kind == 'fuselage':
			self.fuselage = Fuselage(kwargs)
			self.fuselage._initialize()

		elif kind == 'landing_gear':
			self.landing_gear = LandingGear(kwargs)
			self.landing_gear._initialize()

		elif kind == 'lift_rotor':
			self.lift_rotor = LiftRotor(kwargs)
			self.lift_rotor._initialize()

		elif kind == 'propeller':
			self.propeller = Propeller(kwargs)
			self.propeller._initialize()

	def print_info(self):
		print('Vehicle type: Lift+Cruise eVTOL')
		print('List of components:')
		print(self.battery._info())
		print(self.fuselage._info())
		print(self.wing._info())
		print(self.landing_gear._info())
		print(self.lift_rotor._info())
		print(self.propeller._info())







