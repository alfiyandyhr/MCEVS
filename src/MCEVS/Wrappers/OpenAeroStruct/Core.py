import numpy as np
import openmdao.api as om
from openaerostruct.meshing.mesh_generator import generate_mesh
from openaerostruct.geometry.geometry_group import Geometry
from openaerostruct.aerodynamics.aero_groups import AeroPoint


class VLMAeroSolverGroup(om.Group):
    """
    Computes aerodynamic coefficients using OpenAeroStruct assuming a rectangular untapered, unswept wing
    Parameter:
            segment_name            : segment name (e.g., cruise)
            surface_name            : surface name (e.g., wing)
            rho_air 				: air density [kg/m**3]
            Re                      : freestream Reynolds number [1/m]
            Mach                    : freestream Mach number (set to 0 if not considering compressibility)
            with_viscous            : whether or not to calculate viscous drag
            with_wave               : whether or not to calculate wave drag
            num_y                   : number of panels in spanwise direction
            num_x                   : number of panels in chord direction
    Inputs:
            Aero|Cd0                : minimum Cd of the polar drag (coefficient of parasitic drag)
            Aero|lift               : aerodynamic lift [N]
            Wing|area               : wing area [m**2]
            Wing|aspect_ratio       : wing aspect ratio
            Aero|speed              : air speed [m/s]
            Aero|AoA                : angle of attack [deg]
    Outputs:
            Aero|total_drag         : aerodynamic drag [N]
            Aero|CL                 : aerodynamic coefficient of lift
            Aero|CD                 : aerodynamic coefficient of drag
            Aero|f_total            : total equivalent flat plate area [m**2]
    Source:
        Jasa, J. P., Hwang, J. T., and Martins, J. R. R. A., “Open-Source Coupled Aerostructural Optimization Using Python,”
        Structural and Multidisciplinary Optimization, Vol. 57, No. 4, 2018, pp. 1815–1827.
        https://doi.org/10.1007/s00158-018-1912-8
    """
    def initialize(self):
        self.options.declare('segment_name', types=str, desc='Segment name (e.g., cruise)')
        self.options.declare('surface_name', types=str, desc='Surface name (e.g., wing)')
        self.options.declare('rho_air', types=float, desc='Air density')
        self.options.declare('Re', types=float, desc='Freestream Reynolds number')
        self.options.declare('Mach', types=float, desc='Freestream Mach number')
        self.options.declare('with_viscous', types=bool, desc='Whether or not to calculate viscous drag')
        self.options.declare('with_wave', types=bool, desc='Whether or not to calculate wave drag')
        self.options.declare('num_y', types=int, desc='Number of panels in spanwise direction')
        self.options.declare('num_x', types=int, desc='Number of panels in chordwise direction')

    def setup(self):

        segment_name = self.options['segment_name']
        surface_name = self.options['surface_name']
        rho_air = self.options['rho_air']
        Re = self.options['Re']
        Mach = self.options['Mach']
        with_viscous = self.options['with_viscous']
        with_wave = self.options['with_wave']
        num_y = self.options['num_y']
        num_x = self.options['num_x']

        # Define flight variables as independent variables of the model
        ivc = om.IndepVarComp()
        ivc.add_output("beta", val=0.0, units="deg")  # Sideslip angle
        ivc.add_output("omega", val=np.zeros(3), units="deg/s")  # Rotation rate
        ivc.add_output("rho", val=rho_air, units="kg/m**3")  # Freestream air density
        ivc.add_output("re", val=Re, units="1/m")  # Freestream Reynolds number
        ivc.add_output("Mach_number", val=Mach)  # Freestream Mach number
        ivc.add_output("cg", val=np.zeros((3)), units="m")  # Aircraft center of gravity
        self.add_subsystem("flight_vars", ivc)

        # Create a planform
        self.add_subsystem("ars2rectwingplanform", ARS2RectWingPlanform(),
                            promotes_inputs=["Wing|aspect_ratio", "Wing|area"],  # noqa: E127
                            promotes_outputs=["Wing|span", "Wing|chord_scale"])  # noqa: E127

        # Create a dictionary to store options about the surface
        mesh_dict = {
            "num_y": num_y,
            "num_x": num_x,
            "wing_type": "rect",
            "symmetry": True,
            "span_cos_spacing": 1.0,
            "chord_cos_spacing": 1.0,
            "span": 10,  # dummy
            "root_chord": 1.0,  # dummy
        }

        # Generate half-wing mesh of rectangular wing
        mesh = generate_mesh(mesh_dict)

        # Define input surface dictionary for our wing
        surface = {

            # Wing definition
            "name": surface_name,  # name of the surface
            "symmetry": True,  # if true, model one half of wing reflected across the plane y = 0
            "S_ref_type": "projected",  # how we compute the wing area, can be 'wetted' or 'projected'
            # "twist_cp": np.zeros(1),  # Define twist using 1 B-spline cp's distributed along span
            "chord_cp": np.array([1]),  # Define chord using 1 B-spline cp's distributed along span
            "mesh": mesh,

            # Aerodynamic performance of the lifting surface at an angle of attack of 0 (alpha=0).
            # These CL0 and CD0 values are added to the CL and CD obtained from aerodynamic analysis
            # of the surface to get the total CL and CD. These CL0 and CD0 values do not vary wrt alpha.
            "CL0": 0.0,  # CL of the surface at alpha=0
            "CD0": 0.0,  # CD of the surface at alpha=0

            # Airfoil properties for viscous drag calculation
            "k_lam": 0.05,  # percentage of chord with laminar flow, used for viscous drag
            # "t_over_c": 0.12,  # thickness over chord ratio (NACA0012)
            "c_max_t": 0.303,  # chordwise location of maximum (NACA0012) thickness

            "with_viscous": with_viscous,  # if true, compute viscous drag,
            "with_wave": with_wave,  # if true, compute wave drag
        }

        # Add geometry group to the problem and add suface as a sub group.
        # These groups are responsible for manipulating the geometry of the mesh.
        self.add_subsystem(surface_name, Geometry(surface=surface))

        # Create the aero point group for this flight condition and add it to the model
        self.add_subsystem(segment_name, AeroPoint(surfaces=[surface], rotational=False),
                           promotes_inputs=[('v', 'Aero|speed'), ('alpha', 'Aero|AoA')],
                           promotes_outputs=[('wing_perf.CL', 'Aero|CL'),
                                             ('wing_perf.CDi', 'Aero|CDi'),
                                             ('wing_perf.CDv', 'Aero|CDv'),
                                             ('wing_perf.CDw', 'Aero|CDw'),
                                             ('wing_perf.CD', 'Aero|CD')])

        self.connect("flight_vars.beta", f"{segment_name}.beta")
        self.connect("flight_vars.Mach_number", f"{segment_name}.Mach_number")
        self.connect("flight_vars.re", f"{segment_name}.re")
        self.connect("flight_vars.rho", f"{segment_name}.rho")
        self.connect("flight_vars.cg", f"{segment_name}.cg")

        # Connect the mesh from the geometry component to the analysis point
        self.connect(surface_name + ".mesh", segment_name + "." + surface_name + ".def_mesh")

        # Perform the connections with the modified names within the 'aero_states' group.
        self.connect(surface_name + ".mesh", segment_name + ".aero_states." + surface_name + "_def_mesh")

        # # Connect user-supplied planform
        self.connect("Wing|span", "wing.mesh.stretch.span")
        self.connect("Wing|chord_scale", "wing.chord_cp")


class ARS2RectWingPlanform(om.ExplicitComponent):
    """
    Convert wing aspect ratio and area to span and chord assuming a rectangular wing
    """
    def setup(self):
        self.add_input("Wing|aspect_ratio", val=8.0)
        self.add_input("Wing|area", val=30.0, units="m**2")
        self.add_output("Wing|span", units="m")
        self.add_output("Wing|chord_scale", units=None)
        self.declare_partials("*", "*")

    def compute(self, inputs, outputs):
        AR = inputs["Wing|aspect_ratio"]
        S = inputs["Wing|area"]
        outputs["Wing|span"] = np.sqrt(AR * S)
        outputs["Wing|chord_scale"] = np.sqrt(S / AR)

    def compute_partials(self, inputs, partials):
        AR = inputs["Wing|aspect_ratio"]
        S = inputs["Wing|area"]
        partials["Wing|span", "Wing|aspect_ratio"] = 0.5 * np.sqrt(S / AR)
        partials["Wing|span", "Wing|area"] = 0.5 * np.sqrt(AR / S)
        partials["Wing|chord_scale", "Wing|aspect_ratio"] = -0.5 * np.sqrt(S) / (AR**1.5)
        partials["Wing|chord_scale", "Wing|area"] = 0.5 / np.sqrt(AR * S)


if __name__ == '__main__':
    
    # Freestream condition
    v = 248.136  # m/s
    AoA = 5.0  # deg

    # Wing geometry (rectangular)
    wing_AR = 10.0
    wing_S = 10.0

    prob = om.Problem(reports=False)

    # Define flight variables as independent variables of the model
    indeps = prob.model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
    indeps.add_output('Aero|speed', v, units='m/s')
    indeps.add_output('Aero|AoA', AoA, units='deg')
    indeps.add_output('Wing|aspect_ratio', wing_AR, units=None)
    indeps.add_output('Wing|area', wing_S, units='m**2')

    # VLM Group
    prob.model.add_subsystem('VLM',
                             VLMAeroSolverGroup(segment_name='cruise',
                                                surface_name='wing',
                                                rho_air=0.38,
                                                Re=1.0E6,
                                                Mach=0.0,
                                                with_viscous=False,
                                                with_wave=False,
                                                num_y=35,
                                                num_x=11),
                             promotes_inputs=['*'],
                             promotes_outputs=['*'])

    # Set up the problem
    prob.setup()

    # Create a n^2 diagram for user to view model connections
    # om.n2(prob)

    # Run analysis
    prob.run_model()

    print(f"span= {prob.get_val('wing.mesh.stretch.span', 'm')[0]} m")
    print(f"chord= {prob.get_val('wing.chord_cp')[0]} m")
    print(f"area= {prob.get_val('cruise.wing.S_ref')[0]} m**2")
    print("CL", prob.get_val("Aero|CL")[0])
    print("CD", prob.get_val("Aero|CD")[0])
    print("CDi", prob.get_val("Aero|CDi")[0])
    print("CDv", prob.get_val("Aero|CDv")[0])
    print("CDw", prob.get_val("Aero|CDw")[0])
