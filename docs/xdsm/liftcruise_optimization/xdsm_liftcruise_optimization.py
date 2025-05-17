from pyxdsm.XDSM import XDSM, OPT, SOLVER, FUNC, LEFT

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=True)

x.add_system("opt", OPT, r"\text{Optimizer}")
x.add_system("D1", FUNC, r"\text{Aerodynamics}")
x.add_system("D2", FUNC, r"\text{Trim Analysis}")
x.add_system("D3", FUNC, r"\text{Rotor Power Analysis}")
x.add_system("D4", FUNC, r"\text{Energy Analysis}")
x.add_system("D5", FUNC, r"\text{Battery Weight}")
x.add_system("D6", FUNC, r"\text{Empty Weight}")
x.add_system("D7", FUNC, r"\text{Residual Weight}")

x.add_input("opt",
			[r"W_\text{takeoff}^\text{init}, v_\text{cruise}^\text{init}", r"S_\text{wing}^\text{init}, AR_\text{wing}^\text{init}",
			r"r_\text{rotor}^\text{init}, r_\text{prop}^\text{init}, RPM_\text{prop}^\text{init}"])

x.add_input("D1", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}")
x.add_input("D2", r"g")
x.add_input("D3", r"\rho_\text{air}, g, \mathbf{p_\text{vehicle}}, \mathbf{p_\text{mission}}")
x.add_input("D4", r"\mathbf{p_\text{mission}}")
x.add_input("D5", r"\rho_\text{battery}")
x.add_input("D6", r"\mathbf{p_\text{vehicle}}")
x.add_input("D7", r"W_\text{payload}")

x.connect("opt", "D1", [r"W_\text{takeoff}, v_\text{cruise}", r"S_\text{wing}, AR_\text{wing}"])
x.connect("opt", "D2", r"W_\text{takeoff}")
x.connect("opt", "D3", [r"r_\text{rotor}, r_\text{prop}", r"RPM_\text{prop}, v_\text{cruise}"])
x.connect("opt", "D4", r"v_\text{cruise}")
x.connect("opt", "D6", [r"W_\text{takeoff}, r_\text{rotor}, r_\text{prop}", r"S_\text{wing}, AR_\text{wing}"])
x.connect("opt", "D7", r"W_\text{takeoff}")
x.connect("D1", "D2", r"D_\text{cruise}")
x.connect("D2", "D3", r"\mathbf{T_\text{segments}}, \mathbf{\alpha_\text{segments}}")
x.connect("D3", "D4", r"\mathbf{P_\text{segments}}")
x.connect("D4", "D5", r"E_\text{total}")
x.connect("D3", "D6", r"\mathbf{P_\text{segments}}")
x.connect("D5", "D7", r"W_\text{battery}")
x.connect("D6", "D7", r"W_\text{empty}")
x.connect("D7", "opt", r"G: W_\text{residual}")
x.connect("D1", "opt", r"G: C_{\text{L,cruise}}")
x.connect("D3", "opt", [r"G: J_\text{prop}, CT/\sigma,", r"\mathbf{T/P_\text{segments}}"])
x.connect("D4", "opt", r"F: E_\text{total}")

x.add_output("opt",
			 [r"W_\text{takeoff}^{*}, v_\text{cruise}^{*}",
			  r"S_\text{wing}^{*}, AR_\text{wing}^{*}",
			 r"r_\text{rotor}^{*}, r_\text{prop}^{*}, RPM_\text{prop}^{*}"],
			 side=LEFT)
x.add_output("D1", r"G^{*}: C_{\text{L,cruise}}^{*}")
x.add_output("D3", [r"G^{*}: J_\text{prop}^{*}, CT/\sigma^{*},", r"\mathbf{T/P_\text{segments}^{*}}"])
x.add_output("D4", r"F^{*}: E_\text{total}^{*}")
x.add_output("D7", r"G^{*}: W_\text{residual}^{*}")

x.add_process(["opt", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "opt"])

x.write("xdsm_liftcruise_optimization")
