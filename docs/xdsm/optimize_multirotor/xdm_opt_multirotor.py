from pyxdsm.XDSM import XDSM, OPT, SOLVER, FUNC, LEFT

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=True)

x.add_system("opt", OPT, r"\text{Optimizer}")
x.add_system("solver", SOLVER, r"\text{Newton Solver}")
x.add_system("D1", FUNC, r"\text{Aerodynamics}")
x.add_system("D2", FUNC, r"\text{Trim}")
x.add_system("D3", FUNC, r"\text{Climb Power}")
x.add_system("D4", FUNC, r"\text{Cruise Power}")
x.add_system("D5", FUNC, r"\text{Descent Power}")
x.add_system("D6", FUNC, r"\text{Energy}")
x.add_system("D7", FUNC, r"\text{Battery Weight}")
x.add_system("D8", FUNC, r"\text{Propulsion Weight}")
x.add_system("D9", FUNC, r"\text{Structure Weight}")
x.add_system("D10", FUNC, r"\text{Equipment Weight}")
x.add_system("D11", FUNC, r"\text{Residual Weight}")


x.add_input("opt", r"v_{cruise}^{init}, r_{rotor}^{init}, \mu_{rotor}^{init}")
x.add_input("D1", r"\rho_{air}")
x.add_input("D2", r"g")
x.add_input("D3", r"\rho_{air}, g, v_{climb}, N_{rotor}, FM")
x.add_input("D4", r"N_{rotor}, \sigma_{rotor}")
x.add_input("D5", r"\rho_{air}, g, v_{descent}, N_{rotor}, FM")
x.add_input("D6", r"t_{climb}, R_{cruise}, t_{descent}")
x.add_input("D7", r"\rho_{batt}, \eta_{batt}, \eta_{max}")
x.add_input("D8", r"N_{rotor}, N_{blade}, tf")
x.add_input("D9", r"n_{pax}, l_{fuse}, p_{max}, l_{sm}, n_{ult}, tf")
x.add_input("D10", r"tf")
x.add_input("D11", r"W_{payload}")

x.connect("opt", "D3", r"r_{rotor}")
x.connect("solver", "D3", r"W_{takeoff}")
x.connect("D3", "D6", r"P_{climb}")
x.connect("opt", "D1", r"v_{cruise}")
x.connect("solver", "D1", r"W_{takeoff}")
x.connect("D1", "D2", r"Drag")
x.connect("solver", "D2", r"W_{takeoff}")
x.connect("D2", "D4", r"Thrust, \alpha_{rotor}")
x.connect("opt", "D4", r"r_{rotor}, \mu_{rotor}, v_{cruise}")
x.connect("D4", "D6", r"P_{cruise}")
x.connect("opt", "D5", r"r_{rotor}")
x.connect("solver", "D5", r"W_{takeoff}")
x.connect("D5", "D6", r"P_{descent}")
x.connect("opt", "D6", r"v_{cruise}")
x.connect("D6", "D7", r"Energy_{total}")
x.connect("opt", "D8", r"r_{rotor}")
x.connect("D3", "D8", r"P_{climb}")
x.connect("D4", "D8", r"P_{cruise}")
x.connect("D5", "D8", r"P_{descent}")
x.connect("solver", "D9", r"W_{takeoff}")
x.connect("solver", "D10", r"W_{takeoff}")
x.connect("solver", "D11", r"W_{takeoff}")
x.connect("D7", "D11", r"W_{battery}")
x.connect("D8", "D11", r"W_{propulsion}")
x.connect("D9", "D11", r"W_{structure}")
x.connect("D10", "D11", r"W_{equipment}")
x.connect("D11", "solver", r"W_{residual}")

x.connect("solver", "opt", r"W_{takeoff}")
x.connect("D3", "opt", r"DL_{climb}")
x.connect("D4", "opt", r"DL_{cruise}, C_{T}/\sigma_{rotor}")
x.connect("D5", "opt", r"DL_{descent}")


x.add_output("opt", "v_{cruise}^*, r_{rotor}, \mu_{rotor}", side=LEFT)
x.add_output("solver", "W_{takeoff}^*", side=LEFT)
x.add_output("D3", "DL_{climb}^*", side=LEFT)
x.add_output("D4", "DL_{cruise}^*, C_{T}/\sigma_{rotor}^*", side=LEFT)
x.add_output("D5", "DL_{descent}^*", side=LEFT)
x.add_process(["opt", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9", "D10", "D11", "solver", "opt"])
x.write("multirotor_opt")