import numpy as np
import openmdao.api as om


class ARS2RectWingPlanform(om.ExplicitComponent):
    """
    Convert wing aspect ratio and area to span and chord assuming a rectangular wing
    """
    def setup(self):
        self.add_input("Wing|aspect_ratio", val=8.0)
        self.add_input("Wing|area", val=30.0, units="m**2")
        self.add_output("Wing|span", units="m")
        self.add_output("Wing|chord", units="m")
        self.declare_partials("*", "*")

    def compute(self, inputs, outputs):
        AR = inputs["Wing|aspect_ratio"]
        S = inputs["Wing|area"]
        outputs["Wing|span"] = np.sqrt(AR * S)
        outputs["Wing|chord"] = np.sqrt(S / AR)

    def compute_partials(self, inputs, partials):
        AR = inputs["Wing|aspect_ratio"]
        S = inputs["Wing|area"]
        partials["Wing|span", "Wing|aspect_ratio"] = 0.5 * np.sqrt(S / AR)
        partials["Wing|span", "Wing|area"] = 0.5 * np.sqrt(AR / S)
        partials["Wing|chord", "Wing|aspect_ratio"] = -0.5 * np.sqrt(S) / (AR**1.5)
        partials["Wing|chord", "Wing|area"] = 0.5 / np.sqrt(AR * S)
