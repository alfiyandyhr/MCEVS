import openmdao.api as om


class MeanChord(om.ExplicitComponent):
    """
    Computes mean chord from "mean_c_to_R"
    Inputs:
            R			: rotor's radius [m]
            mean_c_to_R : mean chord to R
    Outputs:
            mean_chord
    """
    def setup(self):
        self.add_input('R', units='m', desc='Rotor radius')
        self.add_input('mean_c_to_R', units=None, desc='Mean chord to radius ratio')
        self.add_output('mean_chord', units='m', desc='Mean chord')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        outputs['mean_chord'] = inputs['mean_c_to_R'] * inputs['R']

    def compute_partials(self, inputs, partials):
        partials['mean_chord', 'mean_c_to_R'] = inputs['R']
        partials['mean_chord', 'R'] = inputs['mean_c_to_R']
