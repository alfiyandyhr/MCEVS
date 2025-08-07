from pyxdsm.XDSM import XDSM, OPT, FUNC, LEFT, SOLVER, IFUNC
import os

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=False)

create_xdsm_general_sizing = False
create_xdsm_general_sizing_and_opt_hybrid = False
create_xdsm_general_sizing_and_opt_MDF = False

if create_xdsm_general_sizing:

    x.add_system("opt", OPT, r"\text{Optimizer}")
    x.add_system("D1", FUNC, r"\text{Aerodynamics}")
    x.add_system("D2", FUNC, r"\text{Trim Calculation}")
    x.add_system("D3", FUNC, r"\text{Rotor Performance}")
    x.add_system("D4", FUNC, r"\text{Energy Calculation}")
    x.add_system("D5", FUNC, r"\text{Battery Weight}")
    x.add_system("D6", FUNC, r"\text{Empty Weight}")
    x.add_system("D7", FUNC, r"\text{Residual Weight}")

    x.add_input("opt", r"W_\text{takeoff}^{(0)}")
    x.add_input("D1", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
    x.add_input("D2", r"g")
    x.add_input("D3", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
    x.add_input("D4", r"\mathbf{p_\text{mission}}")
    x.add_input("D5", r"\rho_\text{battery}")
    x.add_input("D6", r"\mathbf{p_\text{vehicle}}")
    x.add_input("D7", r"W_\text{payload}")

    x.connect("opt", "D1", r"W_\text{takeoff}")
    x.connect("opt", "D2", r"W_\text{takeoff}")
    x.connect("opt", "D6", r"W_\text{takeoff}")
    x.connect("opt", "D7", r"W_\text{takeoff}")
    x.connect("D1", "D2", r"D_\text{cruise}")
    x.connect("D2", "D3", r"\mathbf{T_\text{segments}}, \mathbf{\alpha_\text{segments}}")
    x.connect("D2", "D6", r"\mathbf{T_\text{segments}}")
    x.connect("D3", "D4", r"\mathbf{P_\text{segments}}")
    x.connect("D4", "D5", r"E_\text{required}")
    x.connect("D3", "D6", r"\mathbf{P_\text{segments}}")
    x.connect("D5", "D7", r"W_\text{battery}")
    x.connect("D6", "D7", r"W_\text{empty}")
    x.connect("D7", "opt", r"f: W_\text{residual}^2")

    x.add_output("opt", r"W_\text{takeoff}^{*}", side=LEFT)
    x.add_output("D5", r"W_\text{battery}^{*}", side=LEFT)
    x.add_output("D6", r"W_\text{empty}^{*}", side=LEFT)
    x.add_process(["opt", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "opt"])

    x.write("xdsm_general_sizing")

    # Clean up auxiliary files
    files_to_remove = ["xdsm_general_sizing.aux", "xdsm_general_sizing.log", "xdsm_general_sizing.tex", "xdsm_general_sizing.tikz"]
    for file in files_to_remove:
        try:
            os.remove(file)
            print(f"Removed {file}")
        except FileNotFoundError:
            print(f"{file} not found, skipping")

if create_xdsm_general_sizing_and_opt_hybrid:

    x.add_system("opt", OPT, r"\text{Optimizer}")
    x.add_system("D1", FUNC, r"\text{Aerodynamics}")
    x.add_system("D2", FUNC, r"\text{Trim Calculation}")
    x.add_system("D3", FUNC, r"\text{Rotor Performance}")
    x.add_system("D4", FUNC, r"\text{Energy Calculation}")
    x.add_system("D5", FUNC, r"\text{Battery Weight}")
    x.add_system("D6", FUNC, r"\text{Empty Weight}")
    x.add_system("D7", FUNC, r"\text{Residual Weight}")

    x.add_input("opt", r"W_\text{takeoff}^{(0)}, \mathbf{x}^{(0)}")
    x.add_input("D1", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
    x.add_input("D2", r"g")
    x.add_input("D3", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
    x.add_input("D4", r"\mathbf{p_\text{mission}}")
    x.add_input("D5", r"\rho_\text{battery}")
    x.add_input("D6", r"\mathbf{p_\text{vehicle}}")
    x.add_input("D7", r"W_\text{payload}")

    x.connect("opt", "D1", r"W_\text{takeoff}, \mathbf{x}")
    x.connect("opt", "D2", r"W_\text{takeoff}")
    x.connect("opt", "D3", r"\mathbf{x}")
    x.connect("opt", "D4", r"\mathbf{x}")
    x.connect("opt", "D6", r"W_\text{takeoff}, \mathbf{x}")
    x.connect("opt", "D7", r"W_\text{takeoff}")
    x.connect("D1", "D2", r"D_\text{cruise}")
    x.connect("D2", "D3", r"\mathbf{T_\text{segments}}, \mathbf{\alpha_\text{segments}}")
    x.connect("D2", "D6", r"\mathbf{T_\text{segments}}")
    x.connect("D3", "D4", r"\mathbf{P_\text{segments}}")
    x.connect("D4", "D5", r"E_\text{required}")
    x.connect("D3", "D6", r"\mathbf{P_\text{segments}}")
    x.connect("D5", "D7", r"W_\text{battery}")
    x.connect("D6", "D7", r"W_\text{empty}")
    x.connect("D1", "opt", r"g: C_\text{L,cruise}")
    x.connect("D3", "opt", [r"\mathbf{g}: J_\text{prop}, C_\text{T}/\sigma,", r"\mathbf{PL_\text{segments}}"])
    x.connect("D4", "opt", r"f: E_\text{required}")
    x.connect("D7", "opt", r"h^\text{c}: W_\text{residual}")

    x.add_output("opt", r"W_\text{takeoff}^{*}, \mathbf{x}^{*}", side=LEFT)
    x.add_output("D1", r"g^{*}: C_\text{L,cruise}^{*}", side=LEFT)
    x.add_output("D3", [r"\mathbf{g}^{*}: J_\text{prop}^{*}, C_\text{T}/\sigma^{*},", r"\mathbf{PL_\text{segments}^{*}}"])
    x.add_output("D4", r"f^{*}: E_\text{required}^{*}", side=LEFT)
    x.add_output("D5", r"W_\text{battery}^{*}", side=LEFT)
    x.add_output("D6", r"W_\text{empty}^{*}", side=LEFT)
    x.add_output("D7", r"h^{\text{c*}}: W_\text{residual}^{*}")
    x.add_process(["opt", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "opt"])

    x.write("xdsm_general_sizing_and_opt_hybrid")

    # Clean up auxiliary files
    files_to_remove = ["xdsm_general_sizing_and_opt_hybrid.aux", "xdsm_general_sizing_and_opt_hybrid.log", "xdsm_general_sizing_and_opt_hybrid.tex", "xdsm_general_sizing_and_opt_hybrid.tikz"]
    for file in files_to_remove:
        try:
            os.remove(file)
            print(f"Removed {file}")
        except FileNotFoundError:
            print(f"{file} not found, skipping")

if create_xdsm_general_sizing_and_opt_MDF:

    x.add_system("opt", OPT, r"\text{Optimizer}")
    x.add_system("solver", SOLVER, r"\text{MDA Solver}")
    x.add_system("D1", FUNC, r"\text{Aerodynamics}")
    x.add_system("D2", FUNC, r"\text{Trim Calculation}")
    x.add_system("D3", FUNC, r"\text{Rotor Performance}")
    x.add_system("D4", FUNC, r"\text{Energy Calculation}")
    x.add_system("D5", FUNC, r"\text{Battery Weight}")
    x.add_system("D6", FUNC, r"\text{Empty Weight}")
    x.add_system("D7", FUNC, r"\text{Total Weight}")
    x.add_system("D8", IFUNC, r"\text{Functions}")

    x.add_input("opt", r"\mathbf{x}^{(0)}")
    x.add_input("solver", r"W_\text{takeoff}^{(0)}")
    x.add_input("D1", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
    x.add_input("D2", r"g")
    x.add_input("D3", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
    x.add_input("D4", r"\mathbf{p_\text{mission}}")
    x.add_input("D5", r"\rho_\text{battery}")
    x.add_input("D6", r"\mathbf{p_\text{vehicle}}")
    x.add_input("D7", r"W_\text{payload}")

    x.connect("opt", "D1", r"\mathbf{x}")
    x.connect("opt", "D3", r"\mathbf{x}")
    x.connect("opt", "D4", r"\mathbf{x}")
    x.connect("opt", "D6", r"\mathbf{x}")
    x.connect("solver", "D1", r"\widehat{W}_\text{takeoff}")
    x.connect("solver", "D2", r"\widehat{W}_\text{takeoff}")
    x.connect("solver", "D6", r"\widehat{W}_\text{takeoff}")

    x.connect("D1", "D2", r"D_\text{cruise}")
    x.connect("D2", "D3", r"\mathbf{T_\text{segments}}, \mathbf{\alpha_\text{segments}}")
    x.connect("D2", "D6", r"\mathbf{T_\text{segments}}")
    x.connect("D3", "D4", r"\mathbf{P_\text{segments}}")
    x.connect("D4", "D5", r"E_\text{required}")
    x.connect("D3", "D6", r"\mathbf{P_\text{segments}}")
    x.connect("D5", "D7", r"W_\text{battery}")
    x.connect("D6", "D7", r"W_\text{empty}")
    x.connect("D1", "D8", r"C_\text{L,cruise}")
    x.connect("D3", "D8", [r"J_\text{prop}, C_\text{T}/\sigma,", r"\mathbf{PL_\text{segments}}"])
    x.connect("D4", "D8", r"E_\text{required}")
    x.connect("D7", "solver", r"W_\text{takeoff}")
    x.connect("D8", "opt", r"f, \mathbf{g}")

    x.add_output("opt", r"\mathbf{x}^{*}", side=LEFT)
    x.add_output("D5", r"W_\text{battery}^{*}", side=LEFT)
    x.add_output("D6", r"W_\text{empty}^{*}", side=LEFT)
    x.add_output("D7", r"W_\text{takeoff}^{*}", side=LEFT)
    x.add_output("D8", r"f^{*}, \mathbf{g}^{*}", side=LEFT)
    x.add_process(["opt", "solver", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "solver", "D8", "opt"])

    x.write("xdsm_general_sizing_and_opt_MDF")

    # Clean up auxiliary files
    files_to_remove = ["xdsm_general_sizing_and_opt_MDF.aux", "xdsm_general_sizing_and_opt_MDF.log", "xdsm_general_sizing_and_opt_MDF.tex", "xdsm_general_sizing_and_opt_MDF.tikz"]
    for file in files_to_remove:
        try:
            os.remove(file)
            print(f"Removed {file}")
        except FileNotFoundError:
            print(f"{file} not found, skipping")
