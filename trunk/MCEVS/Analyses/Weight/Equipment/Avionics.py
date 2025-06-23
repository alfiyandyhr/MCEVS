import openmdao.api as om


class AvionicsWeight(om.ExplicitComponent):
	"""
	Computes avionics system weight
	Parameters:
		tf 		: technology factor (a reduction due to the use of composites, e.g., 0.8)
	Inputs:
		Weight|takeoff : total take-off weight [kg]
	Outputs:
		Weight|equipment|avionics : weight of all avionics systems [kg]
	Notes:
		> This equation has been checked to be the same as electrical system weight found in Roskam.
	Source:
		1. Kadhiresan, A. R., and Duffy, M. J., “Conceptual Design and Mission Analysis for EVTOL Urban Air Mobility Flight Vehicle Configurations,”
		   presented at the AIAA Aviation 2019 Forum, Dallas, Texas, 2019. https://doi.org/10.2514/6.2019-2873
		2. Roskam, J. Airplane Design - Part V: Component Weight Estimation. Lawrence, Kansas: Analysis and Research Corporation, 2003.
  	"""
	def initialize(self):
		self.options.declare('tf', types=float, default=0.8, desc='Technology factor')

	def setup(self):
		self.add_input('Weight|takeoff', units='kg', desc='Total take-off weight')
		self.add_output('Weight|equipment|avionics', units='kg', desc='Weight of all avionics systems')
		self.declare_partials('Weight|equipment|avionics', 'Weight|takeoff')

	def compute(self, inputs, outputs):
		tf = self.options['tf']
		W_takeoff = inputs['Weight|takeoff']  # in [kg]

		# Calculating W_avionics
		W_avionics = 0.0268 * W_takeoff

		outputs['Weight|equipment|avionics'] = tf * W_avionics  # in [kg]

	def compute_partials(self, inputs, partials):
		tf = self.options['tf']
		partials['Weight|equipment|avionics', 'Weight|takeoff'] = tf * 0.0268
