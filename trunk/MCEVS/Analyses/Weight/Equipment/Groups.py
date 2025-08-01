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
    def initialize(self):
        self.options.declare('tf_equipment', types=float, desc='Technology factor for equipment systems')

    def setup(self):

        tf_equipment = self.options['tf_equipment']

        # Avionics weight
        self.add_subsystem('avionics_weight',
                           AvionicsWeight(tf=tf_equipment),
                           promotes_inputs=['Weight|takeoff'],
                           promotes_outputs=['Weight|equipment|avionics'])

        # Flight control weight
        self.add_subsystem('flight_control_weight',
                           FlightControlWeight(tf=tf_equipment),
                           promotes_inputs=['Weight|takeoff'],
                           promotes_outputs=['Weight|equipment|flight_control'])

        # Anti icing weight
        self.add_subsystem('anti_icing_weight',
                           AntiIcingSystemWeight(tf=tf_equipment),
                           promotes_inputs=['Weight|takeoff'],
                           promotes_outputs=['Weight|equipment|anti_icing'])

        # Furnishing weight
        self.add_subsystem('furnishing_weight',
                           FurnishingWeight(tf=tf_equipment),
                           promotes_inputs=['Weight|takeoff'],
                           promotes_outputs=['Weight|equipment|furnishings'])

        # Sum up
        adder = om.AddSubtractComp()
        adder.add_equation('Weight|equipment',
                           input_names=['Weight|equipment|avionics', 'Weight|equipment|flight_control', 'Weight|equipment|anti_icing', 'Weight|equipment|furnishings'],
                           units='kg',
                           scaling_factors=[1., 1., 1., 1.])
        self.add_subsystem('equipment_sum_weight',
                           adder,
                           promotes_inputs=['Weight|*'],
                           promotes_outputs=['Weight|equipment'])
