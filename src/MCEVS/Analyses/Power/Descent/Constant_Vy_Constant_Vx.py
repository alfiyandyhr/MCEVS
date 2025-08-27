import openmdao.api as om
from MCEVS.Analyses.Stability.Trim import WingedConstantDescentTrimOfLift, WingedConstantDescentTrimOfThrust
from MCEVS.Analyses.Stability.Trim import MultirotorConstantDescentTrim
from MCEVS.Analyses.Aerodynamics.Parasite import ParasiteDragViaComponentBuildUpApproach, BacchiniExperimentalFixedValueForLPC
from MCEVS.Analyses.Aerodynamics.Empirical import WingedParasiteDragViaWeightBasedRegression, MultirotorParasiteDragViaWeightBasedRegression
from MCEVS.Analyses.Aerodynamics.Induced import WingedAeroDragViaParabolicDragPolar
from MCEVS.Analyses.Aerodynamics.Rotor import ThrustOfEachRotor, ThrustCoefficient, RotorAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import PropellerRevolutionFromAdvanceRatio, RotorRevolutionFromAdvanceRatio
from MCEVS.Analyses.Aerodynamics.Rotor import RotorInflow, InducedVelocity
from MCEVS.Analyses.Power.Rotor import RotorProfilePower, InducedPowerFactor, PowerForwardComp


class PowerDescentConstantVyConstantVxEdgewise(om.Group):
    """
    Computes the power required in edgewise descent phase
    Parameters:
            N_rotor				: number or rotors
            n_blade 			: number of blades per rotor
            Cd0 				: rotor's parasite drag coefficient
            hover_FM			: hover figure of merit
            rho_air				: air density [kg/m**3]
            g 					: gravitational acceleration [m/s**2]
            descent_airspeed	: descent air speed [m/s]
            gamma				: flight path angle during climb/descent
    Inputs:
            Weight|takeoff 			: total take-off weight [kg]
            Mission|cruise_speed	: cruising speed of the eVTOL [m/s]
            Rotor|radius			: rotor radius [m]
            Rotor|mu 	 			: rotor's advance ratio
            Rotor|alpha				: rotor tilt angle [rad]
    Outputs:
            Power|DescentConstantVyConstantVx 	: required power for descent [W]
            Rotor|thrust						: thrust of a rotor [N]
    """

    def initialize(self):
        self.options.declare('vehicle', types=object, desc='Vehicle object')
        self.options.declare('N_rotor', types=int, desc='Number of lifting rotors')
        self.options.declare('n_blade', types=int, desc='Number of blades per rotor')
        self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
        self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
        self.options.declare('rho_air', default=1.225, desc='Air density')
        self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
        self.options.declare('g', types=float, desc='Gravitational acceleration')
        self.options.declare('descent_airspeed', desc='Descent air speed')
        self.options.declare('gamma', desc='Flight path angle during climb/descent')
        self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

    def setup(self):
        vehicle = self.options['vehicle']
        N_rotor = self.options['N_rotor']
        n_blade = self.options['n_blade']
        Cd0 = self.options['Cd0']
        hover_FM = self.options['hover_FM']
        rho_air = self.options['rho_air']
        mu_air = self.options['mu_air']
        g = self.options['g']
        gamma = self.options['gamma']
        descent_airspeed = self.options['descent_airspeed']
        fidelity = self.options['fidelity']

        # Step 1: Calculate the drag for the multirotor in descent
        indep = self.add_subsystem('descent', om.IndepVarComp())
        indep.add_output('descent_airspeed', val=descent_airspeed, units='m/s')
        indep.add_output('gamma', val=gamma, units='rad')

        if fidelity['aerodynamics']['parasite'] == 'WeightBasedRegression':
            self.add_subsystem('parasite_drag',
                               MultirotorParasiteDragViaWeightBasedRegression(N_rotor=N_rotor, rho_air=rho_air),
                               promotes_inputs=['Weight|takeoff', ('Aero|speed', 'descent.descent_airspeed'), ('Rotor|radius', 'LiftRotor|radius')],
                               promotes_outputs=[('Aero|total_drag', 'Aero|Descent|total_drag'), ('Aero|Cd0', 'Aero|Descent|Cd0')])
        elif fidelity['aerodynamics']['parasite'] == 'ComponentBuildUp':
            self.add_subsystem('parasite_drag',
                               ParasiteDragViaComponentBuildUpApproach(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name='descent'),
                               promotes_inputs=['Weight|takeoff', ('Aero|speed', 'descent.descent_airspeed'), ('Rotor|radius', 'LiftRotor|radius')],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Descent|Cd0'), ('Aero|parasite_drag', 'Aero|Descent|total_drag')])

        # Step 2: Calculate thrust required for trim and the body tilt angle
        self.add_subsystem('trim',
                           MultirotorConstantDescentTrim(g=g, gamma=gamma),
                           promotes_inputs=['Weight|takeoff', ('Aero|total_drag', 'Aero|Descent|total_drag')],
                           promotes_outputs=[('Thrust', 'Thrust_all_descent'), ('Body|cos_delta', 'Body|Descent|cos_delta')])

        # Step 3: Convert Body|cos_delta into Rotor|alpha
        self.add_subsystem('delta2alpha',
                           om.ExecComp('alpha = pi/2 - gamma - arccos(cos_delta)', alpha={'units': 'rad'}, gamma={'units': 'rad'}),
                           promotes_inputs=[('cos_delta', 'Body|Descent|cos_delta'), ('gamma', 'descent.gamma')],
                           promotes_outputs=[('alpha', 'LiftRotor|Descent|alpha')])

        # Step 4: Calculate the thrust required by each rotor
        self.add_subsystem('thrust_each',
                           ThrustOfEachRotor(N_rotor=N_rotor),
                           promotes_inputs=[('Thrust_all', 'Thrust_all_descent')],
                           promotes_outputs=[('Rotor|thrust', 'LiftRotor|Descent|thrust')])

        # Step 5: Calculate rotor omega given the advance ratio mu
        self.add_subsystem('rotor_revolution',
                           RotorRevolutionFromAdvanceRatio(),
                           promotes_inputs=[('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|alpha', 'LiftRotor|Descent|alpha'),
                                            ('Rotor|mu', 'LiftRotor|advance_ratio'),
                                            ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('Rotor|omega', 'LiftRotor|Descent|omega')])

        # Step 6: Calculate the thrust coefficient Ct
        self.add_subsystem('Ct',
                           ThrustCoefficient(rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'LiftRotor|Descent|thrust'),
                                            ('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|omega', 'LiftRotor|Descent|omega')],
                           promotes_outputs=[('Rotor|thrust_coefficient', 'LiftRotor|Descent|thrust_coefficient')])

        # Step 7: Calculate profile power
        self.add_subsystem('profile_power',
                           RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
                           promotes_inputs=[('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|chord', 'LiftRotor|chord'),
                                            ('Rotor|mu', 'LiftRotor|advance_ratio'),
                                            ('Rotor|omega', 'LiftRotor|Descent|omega')],
                           promotes_outputs=[('Rotor|profile_power', 'LiftRotor|Descent|profile_power')])

        # Step 8: Calculate induced power
        self.add_subsystem('rotor_inflow',
                           RotorInflow(),
                           promotes_inputs=[('Rotor|mu', 'LiftRotor|advance_ratio'),
                                            ('Rotor|alpha', 'LiftRotor|Descent|alpha'),
                                            ('Rotor|thrust_coefficient', 'LiftRotor|Descent|thrust_coefficient'),],
                           promotes_outputs=[('Rotor|lambda', 'LiftRotor|Descent|lambda')])
        self.add_subsystem('v_induced',
                           InducedVelocity(),
                           promotes_inputs=[('Rotor|radius', 'LiftRotor|radius'),
                                            ('Rotor|alpha', 'LiftRotor|Descent|alpha'),
                                            ('Rotor|omega', 'LiftRotor|Descent|omega'),
                                            ('Rotor|lambda', 'LiftRotor|Descent|lambda'),
                                            ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('v_induced', 'LiftRotor|Descent|v_induced')])
        self.add_subsystem('kappa',
                           InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'LiftRotor|Descent|thrust'),
                                            ('Rotor|profile_power', 'LiftRotor|Descent|profile_power'),
                                            ('Rotor|radius', 'LiftRotor|radius'),],
                           promotes_outputs=[('Rotor|kappa', 'LiftRotor|Descent|kappa')])

        # Step 9: Calculate total power required
        self.add_subsystem('power_req',
                           PowerForwardComp(N_rotor=N_rotor, g=g),
                           promotes_inputs=[('Rotor|thrust', 'LiftRotor|Descent|thrust'),
                                            ('Rotor|profile_power', 'LiftRotor|Descent|profile_power'),
                                            ('Rotor|alpha', 'LiftRotor|Descent|alpha'),
                                            ('Rotor|kappa', 'LiftRotor|Descent|kappa'),
                                            ('v_induced', 'LiftRotor|Descent|v_induced'),
                                            ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('Power|forward', 'Power|DescentConstantVyConstantVx'), ('Rotor|T_to_P', 'LiftRotor|Descent|T_to_P')])


class PowerDescentConstantVyConstantVxWithWing(om.Group):
    """
    Computes the power required in winged descent phase
    Parameters:
            N_propeller			: number of propellers
            n_blade 			: number of blades per rotor
            rho_air				: air density [kg/m**3]
            Cd0 				: rotor's parasite drag coefficient
            hover_FM			: hover figure of merit
            g 					: gravitational acceleration [m/s**2]
            AoA 				: aircraft's angle of attack [deg]
            descent_airspeed	: descent air speed [m/s]
            gamma				: flight path angle during climb/descent
    Inputs:
            Weight|takeoff 		: total take-off weight [kg]
            eVTOL|S_wing 		: wing area [m**2]
            eVTOL|AR_wing		: wing aspect ratio
            Rotor|radius		: rotor radius [m]
            Rotor|J				: propeller's advance ratio
            Rotor|alpha 		: rotor tilt angle [rad]
    Outputs:
            Power|CruiseConstantSpeed 	: required power for cruise [W]
            Rotor|thrust				: thrust of a rotor [N]
    """

    def initialize(self):
        self.options.declare('vehicle', types=object, desc='Vehicle object')
        self.options.declare('N_propeller', types=int, desc='Number of propellers')
        self.options.declare('n_blade', types=int, desc='Number of blades per propeller')
        self.options.declare('Cd0', types=float, desc='Rotor parasite_drag coefficient')
        self.options.declare('hover_FM', types=float, desc='Hover figure of merit')
        self.options.declare('rho_air', types=float, desc='Air density')
        self.options.declare('mu_air', types=float, desc='Air dynamic viscosity')
        self.options.declare('g', types=float, desc='Gravitational acceleration')
        self.options.declare('AoA', desc='Aircraft angle of attack')
        self.options.declare('descent_airspeed', desc='Descent air speed')
        self.options.declare('gamma', desc='Flight path angle during climb/descent')
        self.options.declare('fidelity', types=dict, desc='Fidelity of the analysis')

    def setup(self):
        vehicle = self.options['vehicle']
        N_propeller = self.options['N_propeller']
        n_blade = self.options['n_blade']
        Cd0 = self.options['Cd0']
        hover_FM = self.options['hover_FM']
        rho_air = self.options['rho_air']
        mu_air = self.options['mu_air']
        g = self.options['g']
        AoA = self.options['AoA']
        gamma = self.options['gamma']
        descent_airspeed = self.options['descent_airspeed']
        fidelity = self.options['fidelity']

        # Step 1: Trim analysis
        self.add_subsystem('trim_lift',
                           WingedConstantDescentTrimOfLift(g=g, gamma=gamma),
                           promotes_inputs=['Weight|takeoff'],
                           promotes_outputs=[('Aero|lift', 'Aero|Descent|lift')])

        # Step 2: Calculate drag in descent using simple polar equations
        indep = self.add_subsystem('descent', om.IndepVarComp())
        indep.add_output('descent_airspeed', val=descent_airspeed, units='m/s')

        if fidelity['aerodynamics']['parasite'] == 'WeightBasedRegression':
            self.add_subsystem('parasite_drag',
                               WingedParasiteDragViaWeightBasedRegression(rho_air=rho_air),
                               promotes_inputs=['Weight|takeoff', 'Wing|area', ('Aero|speed', 'descent.descent_airspeed')],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Descent|Cd0'), ('Aero|parasite_drag', 'Aero|Descent|parasite_drag')])

        elif fidelity['aerodynamics']['parasite'] == 'ComponentBuildUp':
            self.add_subsystem('parasite_drag',
                               ParasiteDragViaComponentBuildUpApproach(vehicle=vehicle, rho_air=rho_air, mu_air=mu_air, segment_name='descent'),
                               promotes_inputs=['Weight|takeoff', ('Aero|speed', 'descent.descent_airspeed'), 'Wing|area'],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Descent|Cd0'), ('Aero|parasite_drag', 'Aero|Descent|parasite_drag')])

        elif fidelity['aerodynamics']['parasite'] == 'BacchiniExperimentalFixedValueForLPC':
            self.add_subsystem('parasite_drag',
                               BacchiniExperimentalFixedValueForLPC(rho_air=rho_air),
                               promotes_inputs=[('Aero|speed', 'Mission|cruise_speed'), 'Wing|area'],
                               promotes_outputs=[('Aero|Cd0', 'Aero|Cruise|Cd0'), ('Aero|parasite_drag', 'Aero|Cruise|parasite_drag')])

        if fidelity['aerodynamics']['induced'] == 'ParabolicDragPolar':
            self.add_subsystem('total_drag',
                               WingedAeroDragViaParabolicDragPolar(rho_air=rho_air),
                               promotes_inputs=[('Aero|Cd0', 'Aero|Descent|Cd0'), ('Aero|lift', 'Aero|Descent|lift'), ('Aero|speed', 'descent.descent_airspeed'), 'Wing|*'],
                               promotes_outputs=[('Aero|total_drag', 'Aero|Descent|total_drag'), ('Aero|CL', 'Aero|Descent|CL')])

        # Step 3: Calculate thrust required by each propeller after trimming
        self.add_subsystem('trim_thrust',
                           WingedConstantDescentTrimOfThrust(g=g, gamma=gamma),
                           promotes_inputs=['Weight|takeoff', ('Aero|total_drag', 'Aero|Descent|total_drag')],
                           promotes_outputs=['Thrust_all'])
        self.add_subsystem('thrust_each',
                           ThrustOfEachRotor(N_rotor=N_propeller),
                           promotes_inputs=['Thrust_all'],
                           promotes_outputs=[('Rotor|thrust', 'Propeller|Descent|thrust')])

        # Step 4: Calculate rotor omega given propeller advance ratio J;
        # freestream speed = eVTOL cruise speed
        self.add_subsystem('prop_revolution',
                           PropellerRevolutionFromAdvanceRatio(),
                           promotes_inputs=['Propeller|radius', 'Propeller|advance_ratio', ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('Propeller|omega', 'Propeller|Descent|omega')])

        # Step 5: Calculate rotor advance ratio mu and thrust coefficient Ct
        # Treating propeller as a rotor
        self.add_subsystem('mu',
                           RotorAdvanceRatio(),
                           promotes_inputs=[('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|alpha', 'Propeller|Descent|alpha'),
                                            ('Rotor|omega', 'Propeller|Descent|omega'),
                                            ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('Rotor|mu', 'Propeller|Descent|mu')])
        self.add_subsystem('Ct',
                           ThrustCoefficient(rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'Propeller|Descent|thrust'),
                                            ('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|omega', 'Propeller|Descent|omega')],
                           promotes_outputs=[('Rotor|thrust_coefficient', 'Propeller|Descent|thrust_coefficient')])

        # Step 6: Calculate profile power of a rotor
        self.add_subsystem('profile_power',
                           RotorProfilePower(rho_air=rho_air, n_blade=n_blade, Cd0=Cd0),
                           promotes_inputs=[('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|chord', 'Propeller|chord'),
                                            ('Rotor|mu', 'Propeller|Descent|mu'),
                                            ('Rotor|omega', 'Propeller|Descent|omega')],
                           promotes_outputs=[('Rotor|profile_power', 'Propeller|Descent|profile_power')])

        # Step 7: Calculate induced power
        self.add_subsystem('rotor_inflow',
                           RotorInflow(),
                           promotes_inputs=[('Rotor|mu', 'Propeller|Descent|mu'),
                                            ('Rotor|alpha', 'Propeller|Descent|alpha'),
                                            ('Rotor|thrust_coefficient', 'Propeller|Descent|thrust_coefficient')],
                           promotes_outputs=[('Rotor|lambda', 'Propeller|Descent|lambda')])

        # Assume the rotor tilt angle is 85, or AoA = 5
        self.set_input_defaults('Propeller|Descent|alpha', val=90.0 - AoA, units='deg')

        self.add_subsystem('v_induced',
                           InducedVelocity(),
                           promotes_inputs=[('Rotor|radius', 'Propeller|radius'),
                                            ('Rotor|alpha', 'Propeller|Descent|alpha'),
                                            ('Rotor|omega', 'Propeller|Descent|omega'),
                                            ('Rotor|lambda', 'Propeller|Descent|lambda'),
                                            ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('v_induced', 'Propeller|Descent|v_induced')])
        self.add_subsystem('kappa',
                           InducedPowerFactor(hover_FM=hover_FM, rho_air=rho_air),
                           promotes_inputs=[('Rotor|thrust', 'Propeller|Descent|thrust'),
                                            ('Rotor|profile_power', 'Propeller|Descent|profile_power'),
                                            ('Rotor|radius', 'Propeller|radius'),],
                           promotes_outputs=[('Rotor|kappa', 'Propeller|Descent|kappa')])

        # Step 8: Calculate total power required for winged forward flight
        self.add_subsystem('power_req',
                           PowerForwardComp(N_rotor=N_propeller, g=g),
                           promotes_inputs=[('Rotor|thrust', 'Propeller|Descent|thrust'),
                                            ('Rotor|profile_power', 'Propeller|Descent|profile_power'),
                                            ('Rotor|alpha', 'Propeller|Descent|alpha'),
                                            ('Rotor|kappa', 'Propeller|Descent|kappa'),
                                            ('v_induced', 'Propeller|Descent|v_induced'),
                                            ('v_inf', 'descent.descent_airspeed')],
                           promotes_outputs=[('Power|forward', 'Power|DescentConstantVyConstantVx'), ('Rotor|T_to_P', 'Propeller|Descent|T_to_P')])
