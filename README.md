# MCEVS: Multi-Configurational E-Vtol Sizing

MCEVS is a framework designed to calculate the feasible weight of multi-configurational eVTOL aircraft based on specific mission requirements and vehicle specifications.

Within this framework, users have the flexibility to tailor their flight profiles by incorporating customized segments, such as takeoff, transition, climb, cruise, descent, hover, and landing. When it comes to the vehicle itself, users can choose from various configurations like multirotor, lift+cruise, tiltrotor, tiltwing, or helicopter. Furthermore, they have the freedom to specify the dimensions of the aircraft and its components, including the fuselage, wing, empennage, landing gear, lift rotor, propeller, boom, and more.

After defining the mission requirements and vehicle specifications, a Newton solver is utilized to perform the sizing process and determine the Maximum Take-Off Weight (MTOW). Once the sizing iteration converges, a suitably sized vehicle is identified that meets the necessary physical criteria, such as aerodynamics and trim, across all flight phases.

Additionally, users can conduct an optimization procedure using either a gradient-based or gradient-free method. This optimization process aims to identify the most efficient design variables, whether geometric or operational, that lead to a lighter takeoff weight for the aircraft.

Author: @alfiyandyhr
