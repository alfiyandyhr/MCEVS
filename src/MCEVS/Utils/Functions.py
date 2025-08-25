import numpy as np
import openmdao.api as om
import warnings


class SoftMax(om.ExplicitComponent):
    """
    Compute the soft (smooth) max function by KS aggregation
    Inputs:
            f1
            f2
    Outputs:
            fmax
    """

    def initialize(self):
        self.options.declare('rho', types=int, default=10, desc='KS Factor')

    def setup(self):
        self.add_input('f1')
        self.add_input('f2')
        self.add_output('fmax')
        self.declare_partials('*', '*')

    def compute(self, inputs, outputs):
        f1 = inputs['f1']
        f2 = inputs['f2']
        rho = self.options['rho']

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            tmp = np.exp(rho * f1) + np.exp(rho * f2)

        outputs['fmax'] = (1 / rho) * np.log(tmp)

    def compute_partials(self, inputs, partials):
        f1 = inputs['f1']
        f2 = inputs['f2']
        rho = self.options['rho']
        tmp = np.exp(rho * f1) + np.exp(rho * f2)

        partials['fmax', 'f1'] = np.exp(rho * f1) / tmp
        partials['fmax', 'f2'] = np.exp(rho * f2) / tmp


class BoltzmanSigmoid(om.ExplicitComponent):
    """
    Compute the Boltzman Sigmoid function
    Input:
            t
    Output:
            f
    """

    def initialize(self):
        self.options.declare('a', types=float, desc='Boltzman sigmoid coefficient "a"')
        self.options.declare('b', types=float, desc='Boltzman sigmoid coefficient "b"')
        self.options.declare('c', types=float, desc='Boltzman sigmoid coefficient "c"')
        self.options.declare('tau', types=float, desc='Boltzman sigmoid coefficient "tau"')

    def setup(self):
        self.add_input('t')
        self.add_output('f')
        self.declare_partials('f', 't')

    def compute(self, inputs, outputs):
        a = self.options['a']
        b = self.options['b']
        c = self.options['c']
        tau = self.options['tau']
        t = inputs['t']

        outputs['f'] = a + b / (1 + np.exp(c * (tau - t)))

    def compute_partials(self, inputs, partials):
        b = self.options['b']
        c = self.options['c']
        tau = self.options['tau']
        t = inputs['t']

        partials['f', 't'] = b * c * np.exp(c * (tau - t)) / (np.exp(c * (tau - t)) + 1)**2
