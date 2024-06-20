from setuptools import setup

setup(
	name = "MCEVS",
	version = "0.0.1",
	description = "MCEVS: Alfi eVTOL; Multi-configurational eVTOL Sizing Codes",
	license="",
	packages=["MCEVS"],
	package_dir={"" : "trunk"},
	install_requires=["numpy", "openmdao>=3.16.0", "matplotlib"]
	)