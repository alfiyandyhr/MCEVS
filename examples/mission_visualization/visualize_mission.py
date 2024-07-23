import numpy as np
from MCEVS.Missions.Container import Mission
from MCEVS.Missions.Segments.HoverClimb.Constant_Acceleration import HoverClimbConstantAcceleration
from MCEVS.Missions.Segments.HoverDescent.Constant_Deceleration import HoverDescentConstantDeceleration

# kwargs = {'final_speed': 0.0, 'distance': 150.0}
# kwargs = {'final_speed': 0.0, 'duration': 120.0}
# kwargs = {'distance': 150.0, 'duration': 120.0}
# kwargs = {'deceleration': 0.020833333333333332, 'duration': 120.0}
# kwargs = {'deceleration': 0.020833333333333332, 'distance': 150.0}
# kwargs = {'deceleration': 0.020833333333333332, 'final_speed': 0.0}
# segment = HoverDescentConstantDeceleration(id=1, name='HoverDescent', initial_speed=2.5, kwargs=kwargs)
# segment._initialize()
# segment.print_info()

mission = Mission()
mission.add_segment(name='Hover Climb', kind='HoverClimbConstantAcceleration', final_speed=2.5, distance=150.0, n_discrete=10)
# mission.add_segment(name='Hover Climb', kind='HoverClimbConstantSpeed', speed=2.5, distance=150.0, n_discrete=10)
mission.add_segment(name='Cruise', kind='CruiseConstantSpeed', speed=26.8, distance=96500.0, n_discrete=5)
# mission.add_segment(name='Hover Descent', kind='HoverDescentConstantSpeed', speed=1.5, distance=150.0, n_discrete=10)
mission.add_segment(name='Hover Descent', kind='HoverDescentConstantDeceleration', initial_speed=1.5, final_speed=0.0, distance=150.0, n_discrete=10)
# mission.print()
mission.visualize()
# print(mission.t)
# print(mission.vx.shape)

