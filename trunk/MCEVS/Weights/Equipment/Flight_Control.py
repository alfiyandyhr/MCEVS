import numpy as np
import openmdao.api as om

class FlightControlWeight(om.ExplicitComponent):
	"""
	Computes flight control system weight
	Parameters:
		tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		eVTOL|W_takeoff : total take-off weight [kg]
	Outputs:
		Weights|Flight_control : weight of all flight control systems [kg]
	Notes:
		> This equation includes all the equipments of the cockpit controls
	Source:
		Prouty, R. W., Helicopter Performance, Stability, and Control, Krieger, 2002.
	"""
	def initialize(self):
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('eVTOL|W_takeoff', units='kg', desc='Total take-off weight')
		self.add_output('Weights|Flight_control', units='kg', desc='Weight of all avionics systems')
		self.declare_partials('Weights|Flight_control', 'eVTOL|W_takeoff')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		W_takeoff = inputs['eVTOL|W_takeoff'] # in [kg]

		# Calculating W_flight_control
		kg_to_lb = 2.20462**0.4
		lb_to_kg = 0.453592

		W_flight_control = 11.5 * (W_takeoff/1000.0)**0.4 * kg_to_lb * lb_to_kg

		outputs['Weights|Flight_control'] = tf * W_flight_control # in [kg]

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		W_takeoff = inputs['eVTOL|W_takeoff'] # in [kg]

		# Calculating dWfc_dWtakeoff
		kg_to_lb = 2.20462**0.4
		lb_to_kg = 0.453592

		dWfc_dWtakeoff = 11.5 * 0.4 * W_takeoff**(-0.6) * (1/1000.0)**0.4 * kg_to_lb * lb_to_kg

		partials['Weights|Flight_control', 'eVTOL|W_takeoff'] = tf * dWfc_dWtakeoff


