from pyxdsm.XDSM import XDSM, FUNC, LEFT
import os

# Change `use_sfmath` to False to use computer modern
x = XDSM(use_sfmath=False)

create_xdsm_weight_model = True

if create_xdsm_weight_model:

    x.add_system("D1", FUNC, r"\text{Battery Weight}")
    x.add_system("D2", FUNC, r"\text{Propulsion Weight}")
    x.add_system("D3", FUNC, r"\text{Structure Weight}")
    x.add_system("D4", FUNC, r"\text{Equipment Weight}")
    x.add_system("D5", FUNC, r"\text{Empty Weight}")
    x.add_system("D6", FUNC, r"\text{Residual Weight}")

    x.add_input("D1", r"E_\text{mission}, \rho_\text{battery}")
    x.add_input("D2", [r"W_\text{takeoff}, \mathbf{p_\text{vehicle}}", r"\mathbf{P_\text{segments}}, \mathbf{T_\text{segments}}"])
    x.add_input("D3", [r"W_\text{takeoff}, \mathbf{p_\text{vehicle}}", r"\mathbf{P_\text{segments}}"])
    x.add_input("D4", r"W_\text{takeoff}")
    x.add_input("D6", r"W_\text{takeoff}, W_\text{payload}")

    x.connect("D2", "D5", r"W_\text{propulsion}")
    x.connect("D3", "D5", r"W_\text{structure}")
    x.connect("D4", "D5", r"W_\text{equipment}")
    x.connect("D1", "D6", r"W_\text{battery}")
    x.connect("D5", "D6", r"W_\text{empty}")

    x.add_output("D6", r"W_\text{residual}", side=LEFT)
    x.add_process(["D1", "D2", "D3", "D4", "D5", "D6"])

    x.write("xdsm_weight_model")

    # Clean up auxiliary files
    files_to_remove = ["xdsm_weight_model.aux", "xdsm_weight_model.log", "xdsm_weight_model.tex", "xdsm_weight_model.tikz"]
    for file in files_to_remove:
        try:
            os.remove(file)
            print(f"Removed {file}")
        except FileNotFoundError:
            print(f"{file} not found, skipping")
