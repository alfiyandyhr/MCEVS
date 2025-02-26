from setuptools import setup

setup(
	name = "MCEVS",
	version = "0.0.1",
	description = "MCEVS: Multi-configurational eVTOL Sizing Codes",
	license="",
	packages=["MCEVS"],
	package_dir={"" : "trunk"},
	install_requires=["numpy", "pandas", "matplotlib", "scipy", "openmdao>=3.16.0", "pymoo>=0.6.0.1"]
	)