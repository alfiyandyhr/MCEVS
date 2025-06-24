import numpy as np
import pandas as pd
import tempfile
import subprocess
import os
from .Utils import _remove_BL_file, _check_and_warn_solver_parameter, _extract_cl_cd_cm_cdf_cdp_from_stdout_of_single_point_analysis


class XFOIL(object):
    def __init__(self, airfoil_name: str = None):
        super(XFOIL, self).__init__()
        self.airfoil_name = airfoil_name
        self._path_to_xfoilexe = None
        self._path_to_airfoil_data = None

    @property
    def path_to_xfoilexe(self):
        return self._path_to_xfoilexe

    @path_to_xfoilexe.setter
    def path_to_xfoilexe(self, path):
        if not os.path.isfile(path):
            raise ValueError(f"Provided path '{path}' does not exist or is not a file.")
        self._path_to_xfoilexe = path

    @property
    def path_to_airfoil_data(self):
        return self._path_to_airfoil_data

    @path_to_airfoil_data.setter
    def path_to_airfoil_data(self, path):
        if not os.path.isfile(path):
            raise ValueError(f"Provided path '{path}' does not exist or is not a file.")
        self._path_to_airfoil_data = path

    def run_single_point(self, is_viscous: bool, is_compressible: bool, AoA: float, Re: float = None, Mach: float = None, Iter=200, verbose=True):
        """
        Runs a single XFOIL point:
        - AoA: always required (in deg)
        - If is_viscous is False, is_compressible should be False
        - If is_viscous is True, Re is required
        - If is_compressible is True, Mach is required
        Outputs:
                dict1, dict2
        """
        _check_and_warn_solver_parameter(self.airfoil_name, self.path_to_airfoil_data, is_viscous, is_compressible, Re, Mach)

        # Create a temporary filename for Cp (pressure coefficient) data
        with tempfile.NamedTemporaryFile(suffix='.txt', delete=False) as tmp_cp:
            cp_filename = tmp_cp.name

        # ---------- XFOIL RUN ---------- #
        if verbose:
            print(f"Running XFOIL single point with: is_viscous={is_viscous}, is_compressible={is_compressible}, AoA={AoA}, Re={Re}, Mach={Mach}")

        process = subprocess.Popen(
            [self._path_to_xfoilexe],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        if self.airfoil_name[:4] == 'NACA':
            naca_number = self.airfoil_name.replace("NACA", "").strip()
            commands = f"NACA {naca_number}\n"
        else:
            commands = f"LOAD {self.path_to_airfoil_data}\n"
            commands += "PANE\n"
        commands += "OPER\n"
        if is_viscous:
            commands += f"Visc {Re}\n"
        if is_compressible:
            commands += f"Mach {Mach}\n"
        commands += f"Iter {Iter}\n"
        commands += f"Alfa {AoA}\n"
        commands += f"CPWR {cp_filename}\n\n"
        commands += "QUIT"

        # Send commands to XFOIL
        stdout, stderr = process.communicate(commands)
        _remove_BL_file()

        cl, cd, cm, cdf, cdp = _extract_cl_cd_cm_cdf_cdp_from_stdout_of_single_point_analysis(stdout)

        # Check if Cp file was generated
        if not os.path.isfile(cp_filename):
            raise RuntimeError("XFOIL did not produce Cp file. Output:\n" + (stdout or '') + (stderr or ''))

        # Read Cp data
        try:
            data = np.loadtxt(cp_filename)
            x = data[:, 0]
            Cp = data[:, 1]
        except Exception as e:
            raise RuntimeError(f"Could not read Cp file: {e}")

        # Clean temporary Cp data
        os.remove(cp_filename)

        if verbose:
            print(f"Results: CL= {cl}; CD= {cd}; CM= {cm}; L/D= {cl/cd}; CDf= {cdf}; CDp= {cdp}")

        # Results bookkeeping
        res1 = {'cl': cl, 'cd': cd, 'cm': cm, 'ld': cl / cd, 'cdf': cdf, 'cdp': cdp}
        res2 = {'Cp': Cp, 'x': x}

        return res1, res2

    def run_polar(self, is_viscous: bool, is_compressible: bool, AoA_seq: list, Re: float = None, Mach: float = None, Iter=200, save_polar=False, polar_name='airfoil_polar.dat', verbose=True):
        """
        Runs multiple XFOIL points to create a polar:
        - AoA_seq: always required (in deg)
        - If is_viscous is False, is_compressible should be False
        - If is_viscous is True, Re is required
        - If is_compressible is True, Mach is required
        Output:
                pandas.DataFrame
        """
        _check_and_warn_solver_parameter(self.airfoil_name, self.path_to_airfoil_data, is_viscous, is_compressible, Re, Mach)

        if not save_polar:
            # Create a temporary filename for polar data
            with tempfile.NamedTemporaryFile(suffix='.txt', delete=False) as tmp_p:
                p_filename = tmp_p.name
        else:
            p_filename = polar_name

        # Delete existing p_filename so that no duplicate is found
        if os.path.exists(p_filename):
            os.remove(p_filename)

        # ---------- XFOIL RUN ---------- #
        if verbose:
            print(f"Running XFOIL polar with: is_viscous={is_viscous}, is_compressible={is_compressible}, AoA_seq={AoA_seq}, Re={Re}, Mach={Mach}")

        process = subprocess.Popen(
            [self._path_to_xfoilexe],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        if self.airfoil_name[:4] == 'NACA':
            naca_number = self.airfoil_name.replace("NACA", "").strip()
            commands = f"NACA {naca_number}\n"
        else:
            commands = f"LOAD {self.path_to_airfoil_data}\n"
            commands += "PANE\n"
        commands += "OPER\n"
        if is_viscous:
            commands += f"Visc {Re}\n"
        if is_compressible:
            commands += f"Mach {Mach}\n"
        commands += f"Iter {Iter}\n"
        commands += f"Pacc\n{p_filename}\n\n"
        commands += f"ASeq {AoA_seq[0]} {AoA_seq[1]} {AoA_seq[2]}\n\n"
        commands += "QUIT"

        # Send commands to XFOIL
        stdout, stderr = process.communicate(commands)
        _remove_BL_file()

        # Check if polar file was generated
        if not os.path.isfile(p_filename):
            raise RuntimeError("XFOIL did not produce polar file. Output:\n" + (stdout or '') + (stderr or ''))

        # Read polar data
        try:
            with open(p_filename, 'r') as f:
                lines = f.readlines()

            # Find the line number where the data starts
            for i, line in enumerate(lines):
                if line.strip().startswith('alpha'):
                    header_line_idx = i
                    break

            # Read the data into a pandas DataFrame
            polar_df = pd.read_csv(p_filename, sep=r'\s+', skiprows=list(range(header_line_idx)) + [header_line_idx + 1])

        except Exception as e:
            raise RuntimeError(f"Could not read Cp file: {e}")

        # Clean temporary polar data
        if not save_polar:
            os.remove(p_filename)

        if verbose:
            print("A polar created.")

        return polar_df
