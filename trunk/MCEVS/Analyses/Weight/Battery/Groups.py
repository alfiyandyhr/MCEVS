import openmdao.api as om


class BatteryWeight(om.ExplicitComponent):
	"""
	Computes battery weight
	Parameters:
		battery_rho, battery_eff, battery_max_discharge
	Inputs:
		required_energy
	Outputs:
		Weight|battery
	Notes:
		> 
	Source:
  	"""
	def initialize(self):
		self.options.declare('battery_rho', types=float, desc='Battery energy density')
		self.options.declare('battery_eff', types=float, desc='Battery efficiency')
		self.options.declare('battery_max_discharge', types=float, desc='Battery maximum discharge')

	def setup(self):
		self.add_input('required_energy', units='W * h', desc='Total required_energy')
		self.add_output('Weight|battery', units='kg', desc='Weight of battery')
		self.declare_partials('Weight|battery', 'required_energy')

	def compute(self, inputs, outputs):
		battery_rho = self.options['battery_rho']
		battery_eff = self.options['battery_eff']
		battery_max_discharge = self.options['battery_max_discharge']
		required_energy = inputs['required_energy']

		outputs['Weight|battery'] = required_energy / (battery_rho * battery_eff * battery_max_discharge)  # in [kg]

	def compute_partials(self, inputs, partials):
		battery_rho = self.options['battery_rho']
		battery_eff = self.options['battery_eff']
		battery_max_discharge = self.options['battery_max_discharge']

		partials['Weight|battery', 'required_energy'] = 1 / (battery_rho * battery_eff * battery_max_discharge)  # in [kg]
