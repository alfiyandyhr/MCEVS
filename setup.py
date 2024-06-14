from setuptools import setup

setup(
	name = "AVTOL",
	version = "0.0.1",
	description = "AVTOL: Alfi eVTOL; Multi-configurational eVTOL Sizing Codes",
	license="",
	packages=["AVTOL"],
	package_dir={"" : "trunk"},
	install_requires=["numpy", "openmdao>=3.16.0", "matplotlib"]
	)