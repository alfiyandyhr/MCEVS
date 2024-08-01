class VehicleWeight(object):
	"""
	docstring for VehicleWeight
	Weight of the vehicle.

	Top level info:
		> Max take-off weight
		> Payload weight
		> Battery weight
		> Propulsion weight
		> Structure weight
		> Equipment weight

	Broken down into:
		> Payload
		> Battery
		> Propulsion:
			- Motor
			- Controller
			- Rotors
		> Structure:
			- Fuselage
			- Landing_Gear
			- Empennage
			- Wing (if any)
		> Equipment:
			- Anti-icing
			- Avionics
			- Flight_Control
			- Furnishings 
	"""
	def __init__(self, mtow=None):
		super(VehicleWeight, self).__init__()
		self.max_takeoff = mtow	# defined, or sized
		self.payload = None
		self.battery = None
		self.propulsion = None
		self.structure = None
		self.equipment = None

	def is_being_sized(self):
		pass



