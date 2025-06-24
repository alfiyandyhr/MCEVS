import os
import openmdao.api as om
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


class Airfoil(object):
    """
    Airfoil object that stores performance of each section airfoil
    Notes:
            > should be called using "load_airfoil" function
            > alpha is in deg
            > Cl_func and Cd_func use "cubic" spline interpolation
    """
    def __init__(self):
        super(Airfoil, self).__init__()
        self.alpha_data = None
        self.Cl_data = None
        self.Cd_data = None
        self.Cl_func = None
        self.Cd_func = None

    def eval_Cl(self, alpha_deg):
        """
        Evaluate Cl given alpha in deg
        """
        return self.Cl_func(alpha_deg)

    def eval_Cd(self, alpha_deg):
        """
        Evaluate Cd given alpha in deg
        """
        return self.Cd_func(alpha_deg)

    def plot(self):
        """
        Plot the real and interpolation data
        """
        # Prepare interpolation data
        alpha_interp = np.linspace(-180, 180, 100)
        Cl_interp = self.Cl_func(alpha_interp)
        Cd_interp = self.Cd_func(alpha_interp)

        fig, axs = plt.subplots(nrows=2, ncols=1)
        axs[0].plot(self.alpha_data, self.Cl_data, 'o')
        axs[0].plot(alpha_interp, Cl_interp, '-')
        axs[0].set_xlabel('Alpha')
        axs[0].set_ylabel('Cl')
        axs[0].legend(['Real', 'Interpolated'])
        axs[1].plot(self.alpha_data, self.Cd_data, 'o')
        axs[1].plot(alpha_interp, Cd_interp, '-')
        axs[1].set_xlabel('Alpha')
        axs[1].set_ylabel('Cd')
        axs[1].legend(['Real', 'Interpolated'])
        fig.suptitle('Real vs Interpolated Airfoil Data')
        plt.show()


def load_airfoil(name):
    # Load airfoil data
    file_dir = os.path.dirname(os.path.abspath(__file__))
    airfoil_path = os.path.join(file_dir, 'Airfoils', name + '.dat')
    data = np.genfromtxt(airfoil_path, skip_header=14)

    # Instantiate an airfoil object
    a = Airfoil()
    a.alpha_data = data[:, 0]
    a.Cl_data = data[:, 1]
    a.Cd_data = data[:, 2]
    a.Cl_func = interp1d(a.alpha_data, a.Cl_data, kind='quadratic')
    a.Cd_func = interp1d(a.alpha_data, a.Cd_data, kind='quadratic')

    return a


class AirfoilCoeffs(om.ExplicitComponent):
    """
    Parameter: airfoil
    Input: AoA
    Output: Cl, Cd
    """
    def initialize(self):
        self.options.declare('airfoil', types=str)

    def setup(self):
        self.add_input('AoA', units='deg')
        self.add_output('Cl', units=None)
        self.add_output('Cd', units=None)
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        airfoil = self.options['airfoil']
        AoA = inputs['AoA']

        if airfoil == 'CLARKY':
            if AoA < -9.25:
                outputs['Cl'] = 0.00016055 * AoA**2 + 0.03038325 * AoA - 0.12669162
                outputs['Cd'] = 0.00010179 * AoA**2 + 0.01926335 * AoA + 0.25451674
            elif AoA > 17.0:
                outputs['Cl'] = 0.00013341 * AoA**2 - 0.02628272 * AoA + 1.75924935
                outputs['Cd'] = -0.00010646 * AoA**2 + 0.02097336 * AoA - 0.23195915
            else:
                outputs['Cl'] = -0.00022620 * AoA**3 + 0.00035166 * AoA**2 + 0.11501989 * AoA + 0.376
                outputs['Cd'] = 0.00000563 * AoA**3 + 0.00026543 * AoA**2 - 0.00170558 * AoA + 0.00652

        elif airfoil == 'BOEING_VERTOL_VR12_Re5E5':
            if AoA < -9.5:
                outputs['Cl'] = -0.00004972 * AoA**2 - 0.00942282 * AoA - 0.58952914
                outputs['Cd'] = -0.00014875 * AoA**2 - 0.02818766 * AoA - 0.15965827
            elif AoA > 18.5:
                outputs['Cl'] = 0.00069132 * AoA**2 - 0.13722607 * AoA + 3.64317973
                outputs['Cd'] = -0.00020911 * AoA**2 + 0.04150877 * AoA - 0.59881363
            else:
                outputs['Cl'] = -0.00023201 * AoA**3 + 0.00151456 * AoA**2 + 0.11609899 * AoA + 0.17357118
                outputs['Cd'] = -0.00000441 * AoA**3 + 0.00038061 * AoA**2 - 0.00225189 * AoA + 0.00628659

        elif airfoil == 'BOEING_VERTOL_VR12_Re1E6':
            if AoA < -13.25:
                outputs['Cl'] = -0.00037184 * AoA**2 - 0.07185832 * AoA - 2.00984132
                outputs['Cd'] = -0.00013011 * AoA**2 - 0.02514386 * AoA - 0.27188361
            elif AoA > 19.25:
                outputs['Cl'] = 0.00124922 * AoA**2 - 0.24890671 * AoA + 5.71174070
                outputs['Cd'] = -0.00030566 * AoA**2 + 0.06090311 * AoA - 0.95742811
            else:
                outputs['Cl'] = -0.00015145 * AoA**3 + 0.00005240 * AoA**2 + 0.12470884 * AoA + 0.15681273
                outputs['Cd'] = 0.00000612 * AoA**3 + 0.00010165 * AoA**2 - 0.00099125 * AoA + 0.00659008

        elif airfoil == 'BOEING_VERTOL_VR12_Viterna_for_NASA_QR':
            if AoA < -13.25:
                outputs['Cl'] = -0.0000000000083549 * AoA**6 - 0.0000000047840744 * AoA**5 - 0.0000010649366136 * AoA**4 - 0.0001144044407873 * AoA**3 - 0.0059672629119978 * AoA**2 - 0.1473832283222270 * AoA - 2.2751380819795500
                outputs['Cd'] = -0.0000000000012341 * AoA**6 - 0.0000000006745103 * AoA**5 - 0.0000001110146934 * AoA**4 - 0.0000032678211540 * AoA**3 + 0.0003718015047358 * AoA**2 + 0.0000940288215359 * AoA - 0.0292797950805124
            elif AoA > 19.25:
                outputs['Cl'] = 0.0000000000099660 * AoA**6 - 0.0000000055831207 * AoA**5 + 0.0000012239417548 * AoA**4 - 0.0001316531759442 * AoA**3 + 0.0071422773315944 * AoA**2 - 0.1986743118584200 * AoA + 3.3811980767955600
                outputs['Cd'] = 0.0000000059967579 * AoA**6 - 0.0000000635454597 * AoA**5 - 0.0000011663058908 * AoA**4 + 0.0000093163166799 * AoA**3 + 0.0001375011115419 * AoA**2 - 0.0003834737999887 * AoA + 0.0074720850653456
            else:
                outputs['Cl'] = 0.0000000037015648 * AoA**6 - 0.0000001929989587 * AoA**5 - 0.0000019025591953 * AoA**4 - 0.0000754388907332 * AoA**3 + 0.0003190499471639 * AoA**2 + 0.1191538643857040 * AoA + 0.1493914459851730
                outputs['Cd'] = -0.0000000000011680 * AoA**6 + 0.0000000006278587 * AoA**5 - 0.0000000955122211 * AoA**4 + 0.0000005495274894 * AoA**3 + 0.0006085545154665 * AoA**2 - 0.0089360891379086 * AoA + 0.0485758040116397

        elif airfoil == 'BOEING_VERTOL_VR12_Viterna_for_NASA_LPC':
            if AoA < -13.25:
                outputs['Cl'] = -0.0000000000084939 * AoA**6 - 0.0000000048275424 * AoA**5 - 0.0000010684772900 * AoA**4 - 0.0001149958732826 * AoA**3 - 0.0061271552300759 * AoA**2 - 0.1590102301338040 * AoA - 2.4012358386552800
                outputs['Cd'] = -0.0000000000010006 * AoA**6 - 0.0000000005448811 * AoA**5 - 0.0000000891509283 * AoA**4 - 0.0000025152080543 * AoA**3 + 0.0003102440476704 * AoA**2 + 0.0002535330884827 * AoA - 0.0158800585140710
            elif AoA > 19.25:
                outputs['Cl'] = 0.0000000000103108 * AoA**6 - 0.0000000057492830 * AoA**5 + 0.0000012571656448 * AoA**4 - 0.0001359850511096 * AoA**3 + 0.0075612407541648 * AoA**2 - 0.2197185983752470 * AoA + 3.6512025035377400
                outputs['Cd'] = 0.0000000059965836 * AoA**6 - 0.0000000635432698 * AoA**5 - 0.0000011662640125 * AoA**4 + 0.0000093159135849 * AoA**3 + 0.0001374985560142 * AoA**2 - 0.0003834606490549 * AoA + 0.0074721088420790
            else:
                outputs['Cl'] = 0.0000000037013959 * AoA**6 - 0.0000001930000593 * AoA**5 - 0.0000019024789776 * AoA**4 - 0.0000754386864910 * AoA**3 + 0.0003190420283782 * AoA**2 + 0.1191538565164120 * AoA + 0.1493915459140750
                outputs['Cd'] = -0.0000000000007954 * AoA**6 + 0.0000000004143639 * AoA**5 - 0.0000000542459956 * AoA**4 - 0.0000023838353069 * AoA**3 + 0.0006725206782791 * AoA**2 - 0.0127258419497411 * AoA + 0.1108026618073250

        elif airfoil == 'NACA_4412_with_rotation':
            if AoA < -9.5:
                outputs['Cl'] = -0.000000000003 * AoA**6 - 0.000000001559 * AoA**5 - 0.000000328200 * AoA**4 - 0.000029177317 * AoA**3 - 0.000740458597 * AoA**2 + 0.013814985011 * AoA - 0.253270613557
                outputs['Cd'] = 0.000000000001 * AoA**6 + 0.000000000467 * AoA**5 + 0.000000118699 * AoA**4 + 0.000016405925 * AoA**3 + 0.000979956921 * AoA**2 + 0.002528771178 * AoA + 0.035996511980
            elif AoA > 16.25:
                outputs['Cl'] = 0.000000000008 * AoA**6 - 0.000000004094 * AoA**5 + 0.000000858362 * AoA**4 - 0.000085397958 * AoA**3 + 0.004015763272 * AoA**2 - 0.092759651990 * AoA + 1.946132761924
                outputs['Cd'] = -0.0000000000003 * AoA**6 + 0.0000000001824 * AoA**5 - 0.0000000199363 * AoA**4 - 0.0000027391947 * AoA**3 + 0.0003676235077 * AoA**2 + 0.0086769716704 * AoA - 0.1103732730149
            else:
                outputs['Cl'] = -0.00000099 * AoA**5 + 0.00003758 * AoA**4 - 0.00048901 * AoA**3 - 0.00283060 * AoA**2 + 0.13597811 * AoA + 0.34119580
                outputs['Cd'] = -0.000000003 * AoA**6 + 0.000000020 * AoA**5 + 0.000004022 * AoA**4 - 0.000050265 * AoA**3 + 0.000263749 * AoA**2 - 0.000067819 * AoA + 0.026071785

        else:
            raise NotImplementedError(f'The airfoil database for {airfoil} is not found')

    def compute_partials(self, inputs, partials):
        airfoil = self.options['airfoil']
        AoA = inputs['AoA']

        if airfoil == 'CLARKY':
            if AoA < -9.25:
                partials['Cl', 'AoA'] = 2 * 0.00016055 * AoA + 0.03038325
                partials['Cd', 'AoA'] = 2 * 0.00010179 * AoA + 0.01926335
            elif AoA > 17.0:
                partials['Cl', 'AoA'] = 2 * 0.00013341 * AoA - 0.02628272
                partials['Cd', 'AoA'] = -2 * 0.00010646 * AoA + 0.02097336
            else:
                partials['Cl', 'AoA'] = -3 * 0.00022620 * AoA**2 + 2 * 0.00035166 * AoA + 0.11501989
                partials['Cd', 'AoA'] = 3 * 0.00000563 * AoA**2 + 2 * 0.00026543 * AoA - 0.00170558

        elif airfoil == 'BOEING_VERTOL_VR12_Re5E5':
            if AoA < -9.5:
                partials['Cl', 'AoA'] = -2 * 0.00004972 * AoA - 0.00942282
                partials['Cd', 'AoA'] = -2 * 0.00014875 * AoA - 0.02818766
            elif AoA > 18.5:
                partials['Cl', 'AoA'] = 2 * 0.00069132 * AoA - 0.13722607
                partials['Cd', 'AoA'] = -2 * 0.00020911 * AoA + 0.04150877
            else:
                partials['Cl', 'AoA'] = -3 * 0.00023201 * AoA**2 + 2 * 0.00151456 * AoA + 0.11609899
                partials['Cd', 'AoA'] = -3 * 0.00000441 * AoA**2 + 2 * 0.00038061 * AoA - 0.00225189

        elif airfoil == 'BOEING_VERTOL_VR12_Re1E6':
            if AoA < -13.25:
                partials['Cl', 'AoA'] = -2 * 0.00037184 * AoA - 0.07185832
                partials['Cd', 'AoA'] = -2 * 0.00013011 * AoA - 0.02514386
            elif AoA > 19.25:
                partials['Cl', 'AoA'] = 2 * 0.00124922 * AoA - 0.24890671
                partials['Cd', 'AoA'] = -2 * 0.00030566 * AoA + 0.06090311
            else:
                partials['Cl', 'AoA'] = -3 * 0.00015145 * AoA**2 + 2 * 0.00005240 * AoA + 0.12470884
                partials['Cd', 'AoA'] = 3 * 0.00000612 * AoA**2 + 2 * 0.00010165 * AoA - 0.00099125

        elif airfoil == 'BOEING_VERTOL_VR12_Viterna_for_NASA_QR':
            if AoA < -13.25:
                partials['Cl', 'AoA'] = -6 * 0.0000000000083549 * AoA**5 - 5 * 0.0000000047840744 * AoA**4 - 4 * 0.0000010649366136 * AoA**3 - 3 * 0.0001144044407873 * AoA**2 - 2 * 0.0059672629119978 * AoA - 0.1473832283222270
                partials['Cd', 'AoA'] = -6 * 0.0000000000012341 * AoA**5 - 5 * 0.0000000006745103 * AoA**4 - 4 * 0.0000001110146934 * AoA**3 - 3 * 0.0000032678211540 * AoA**2 + 2 * 0.0003718015047358 * AoA + 0.0000940288215359
            elif AoA > 19.25:
                partials['Cl', 'AoA'] = 6 * 0.0000000000099660 * AoA**5 - 5 * 0.0000000055831207 * AoA**4 + 4 * 0.0000012239417548 * AoA**3 - 3 * 0.0001316531759442 * AoA**2 + 2 * 0.0071422773315944 * AoA - 0.1986743118584200
                partials['Cd', 'AoA'] = 6 * 0.0000000059967579 * AoA**5 - 5 * 0.0000000635454597 * AoA**4 - 4 * 0.0000011663058908 * AoA**3 + 3 * 0.0000093163166799 * AoA**2 + 2 * 0.0001375011115419 * AoA - 0.0003834737999887
            else:
                partials['Cl', 'AoA'] = 6 * 0.0000000037015648 * AoA**5 - 5 * 0.0000001929989587 * AoA**4 - 4 * 0.0000019025591953 * AoA**3 - 3 * 0.0000754388907332 * AoA**2 + 2 * 0.0003190499471639 * AoA + 0.1191538643857040
                partials['Cd', 'AoA'] = -6 * 0.0000000000011680 * AoA**5 + 5 * 0.0000000006278587 * AoA**4 - 4 * 0.0000000955122211 * AoA**3 + 3 * 0.0000005495274894 * AoA**2 + 2 * 0.0006085545154665 * AoA - 0.0089360891379086

        elif airfoil == 'BOEING_VERTOL_VR12_Viterna_for_NASA_LPC':
            if AoA < -13.25:
                partials['Cl', 'AoA'] = -6 * 0.0000000000084939 * AoA**5 - 5 * 0.0000000048275424 * AoA**4 - 4 * 0.0000010684772900 * AoA**3 - 3 * 0.0001149958732826 * AoA**2 - 2 * 0.0061271552300759 * AoA - 0.1590102301338040
                partials['Cd', 'AoA'] = -6 * 0.0000000000010006 * AoA**5 - 5 * 0.0000000005448811 * AoA**4 - 4 * 0.0000000891509283 * AoA**3 - 3 * 0.0000025152080543 * AoA**2 + 2 * 0.0003102440476704 * AoA + 0.0002535330884827
            elif AoA > 19.25:
                partials['Cl', 'AoA'] = 6 * 0.0000000000103108 * AoA**5 - 5 * 0.0000000057492830 * AoA**4 + 4 * 0.0000012571656448 * AoA**3 - 3 * 0.0001359850511096 * AoA**2 + 2 * 0.0075612407541648 * AoA - 0.2197185983752470
                partials['Cd', 'AoA'] = 6 * 0.0000000059965836 * AoA**5 - 5 * 0.0000000635432698 * AoA**4 - 4 * 0.0000011662640125 * AoA**3 + 3 * 0.0000093159135849 * AoA**2 + 2 * 0.0001374985560142 * AoA - 0.0003834606490549
            else:
                partials['Cl', 'AoA'] = 6 * 0.0000000037013959 * AoA**5 - 5 * 0.0000001930000593 * AoA**4 - 4 * 0.0000019024789776 * AoA**3 - 3 * 0.0000754386864910 * AoA**2 + 2 * 0.0003190420283782 * AoA + 0.1191538565164120
                partials['Cd', 'AoA'] = -6 * 0.0000000000007954 * AoA**5 + 5 * 0.0000000004143639 * AoA**4 - 4 * 0.0000000542459956 * AoA**3 - 3 * 0.0000023838353069 * AoA**2 + 2 * 0.0006725206782791 * AoA - 0.0127258419497411

        elif airfoil == 'NACA_4412_with_rotation':
            if AoA < -9.5:
                partials['Cl', 'AoA'] = -6 * 0.000000000003 * AoA**5 - 5 * 0.000000001559 * AoA**4 - 4 * 0.000000328200 * AoA**3 - 3 * 0.000029177317 * AoA**2 - 2 * 0.000740458597 * AoA + 0.013814985011
                partials['Cd', 'AoA'] = 6 * 0.000000000001 * AoA**5 + 5 * 0.000000000467 * AoA**4 + 4 * 0.000000118699 * AoA**3 + 3 * 0.000016405925 * AoA**2 + 2 * 0.000979956921 * AoA + 0.002528771178
            elif AoA > 16.25:
                partials['Cl', 'AoA'] = 6 * 0.000000000008 * AoA**5 - 5 * 0.000000004094 * AoA**4 + 4 * 0.000000858362 * AoA**3 - 3 * 0.000085397958 * AoA**2 + 2 * 0.004015763272 * AoA - 0.092759651990
                partials['Cd', 'AoA'] = -6 * 0.0000000000003 * AoA**5 + 5 * 0.0000000001824 * AoA**4 - 4 * 0.0000000199363 * AoA**3 - 3 * 0.0000027391947 * AoA**2 + 2 * 0.0003676235077 * AoA + 0.0086769716704
            else:
                partials['Cl', 'AoA'] = -5 * 0.00000099 * AoA**4 + 4 * 0.00003758 * AoA**3 - 3 * 0.00048901 * AoA**2 - 2 * 0.00283060 * AoA + 0.13597811
                partials['Cd', 'AoA'] = -6 * 0.000000003 * AoA**5 + 5 * 0.000000020 * AoA**4 + 4 * 0.000004022 * AoA**3 - 3 * 0.000050265 * AoA**2 + 2 * 0.000263749 * AoA - 0.000067819


if __name__ == '__main__':
    airfoil = load_airfoil('CLARKY')
    airfoil.plot()
