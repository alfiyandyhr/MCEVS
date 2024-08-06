from pyxdsm.XDSM import XDSM, OPT, SOLVER, FUNC, LEFT

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=True)

x.add_system("opt", OPT, r"\text{Optimizer}")
x.add_system("solver", SOLVER, r"\text{Newton Solver}")
x.add_system("D1", FUNC, r"\text{Aerodynamics}")
x.add_system("D2", FUNC, r"\text{Trim}")
x.add_system("D3", FUNC, r"\text{Power Analysis}")
x.add_system("D4", FUNC, r"\text{Energy Analysis}")
x.add_system("D5", FUNC, r"\text{Weight Analysis}")
x.add_system("D6", FUNC, r"\text{Residual Weight}")


x.add_input("opt", r"DV_{geom}^{init}, DV_{oper}^{init}")
x.add_input("D1", r"\rho_{air}, g")
x.add_input("D2", r"g")
x.add_input("D3", r"\rho_{air}, g, speed, params_{vehicle}")
x.add_input("D5", r"params_{vehicle}")


x.add_input("D4", r"Range, duration")
x.add_input("D6", r"W_{payload}")

x.connect("opt", "D1", r"DV_{geom}, DV_{oper}")
x.connect("solver", "D1", r"W_{takeoff}")
x.connect("opt", "D3", r"DV_{geom}, DV_{oper}")
x.connect("opt", "D4", r"DV_{oper}")
x.connect("solver", "D2", r"W_{takeoff}")
x.connect("solver", "D3", r"W_{takeoff}")
x.connect("solver", "D5", r"W_{takeoff}")
x.connect("D1", "D2", r"Drag")
x.connect("D2", "D3", r"Thrust, \alpha")
x.connect("D3", "D4", r"Power_{segment}")
x.connect("D3", "D5", r"Power_{segment}")
x.connect("D4", "D5", r"Energy_{total}")
x.connect("solver", "D6", r"W_{takeoff}")
x.connect("opt", "D5", r"DV_{geom}")
x.connect("D5", "D6", r"W_{components}")
x.connect("D6", "solver", r"W_{residual}")

x.connect("solver", "opt", r"F: W_{takeoff}")
x.connect("D1", "opt", r"G: CL_{cruise}")
x.connect("D3", "opt", r"G: DL_{segment}, C_{T}/\sigma")


x.add_output("opt", "DV_{geom}^*, DV_{oper}^*", side=LEFT)
x.add_output("solver", "F^*", side=LEFT)
x.add_output("D1", "G^*", side=LEFT)
x.add_output("D3", "G^*", side=LEFT)
x.add_process(["opt", "D1", "D2", "D3", "D4", "D5", "D6", "solver", "opt"])
x.write("general_opt")


