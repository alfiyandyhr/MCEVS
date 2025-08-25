from setuptools import setup

setup(
    name="MCEVS",
    version="0.0.1",
    description="MCEVS: Multi-configurational eVTOL Sizing Codes",
    license="",
    packages=["MCEVS"],
    package_dir={"": "src"},
    install_requires=["numpy>=2.2.5", "pandas>=2.2.3", "matplotlib>=3.10.1", "scipy>=1.15.2", "openmdao>=3.38.0", "pymoo>=0.6.1.3", "openvsp>=3.42.3"]
)
