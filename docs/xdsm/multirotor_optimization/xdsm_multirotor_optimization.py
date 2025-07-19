from pyxdsm.XDSM import XDSM, OPT, FUNC, LEFT
import os

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=False)

x.add_system("opt", OPT, r"\text{Optimizer}")
x.add_system("D1", FUNC, r"\text{Aerodynamics}")
x.add_system("D2", FUNC, r"\text{Trim Calculation}")
x.add_system("D3", FUNC, r"\text{Rotor Performance}")
x.add_system("D4", FUNC, r"\text{Energy Calculation}")
x.add_system("D5", FUNC, r"\text{Battery Weight}")
x.add_system("D6", FUNC, r"\text{Empty Weight}")
x.add_system("D7", FUNC, r"\text{Residual Weight}")

x.add_input("opt", [r"W_\text{takeoff}^\text{(0)}, v_\text{cruise}^\text{(0)}", r"r_\text{rotor}^\text{(0)}, RPM_\text{rotor}^\text{(0)}"])

x.add_input("D1", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
x.add_input("D2", r"g")
x.add_input("D3", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
x.add_input("D4", r"\mathbf{p_\text{mission}}")
x.add_input("D5", r"\rho_\text{battery}")
x.add_input("D6", r"\mathbf{p_\text{vehicle}}")
x.add_input("D7", r"W_\text{payload}")

x.connect("opt", "D1", r"W_\text{takeoff}, v_\text{cruise}")
x.connect("opt", "D2", r"W_\text{takeoff}")
x.connect("opt", "D3", r"r_\text{rotor}, RPM_\text{rotor}, v_\text{cruise}")
x.connect("opt", "D4", r"v_\text{cruise}")
x.connect("opt", "D6", r"W_\text{takeoff}, r_\text{rotor}")
x.connect("opt", "D7", r"W_\text{takeoff}")
x.connect("D1", "D2", r"D_\text{cruise}")
x.connect("D2", "D3", r"\mathbf{T_\text{segments}}, \mathbf{\alpha_\text{segments}}")
x.connect("D2", "D6", r"\mathbf{T_\text{segments}}")
x.connect("D3", "D4", r"\mathbf{P_\text{segments}}")
x.connect("D4", "D5", r"E_\text{mission}")
x.connect("D3", "D6", r"\mathbf{P_\text{segments}}")
x.connect("D5", "D7", r"W_\text{battery}")
x.connect("D6", "D7", r"W_\text{empty}")
x.connect("D7", "opt", r"h^\text{c}: W_\text{residual}")
x.connect("D3", "opt", [r"g: \mu_\text{rotor}, C_\text{T}/\sigma,", r"\mathbf{PL_\text{segments}}"])
x.connect("D4", "opt", r"f: E_\text{mission}")

x.add_output("opt", [r"W_\text{takeoff}^{*}, v_\text{cruise}^{*}", r"r_\text{rotor}^{*}, RPM_\text{rotor}^{*}"], side=LEFT)
x.add_output("D3", [r"g^{*}: \mu_\text{rotor}^{*}, C_\text{T}/\sigma^{*},", r"\mathbf{PL_\text{segments}^{*}}"])
x.add_output("D4", r"f^{*}: E_\text{mission}^{*}")
x.add_output("D5", r"W_\text{battery}^{*}")
x.add_output("D6", r"W_\text{empty}^{*}")
x.add_output("D7", r"h^{\text{c*}}: W_\text{residual}^{*}")

x.add_process(["opt", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "opt"])

x.write("xdsm_multirotor_optimization")

# Clean up auxiliary files
files_to_remove = ["xdsm_multirotor_optimization.aux", "xdsm_multirotor_optimization.log", "xdsm_multirotor_optimization.tex", "xdsm_multirotor_optimization.tikz"]
for file in files_to_remove:
    try:
        os.remove(file)
        print(f"Removed {file}")
    except FileNotFoundError:
        print(f"{file} not found, skipping")
