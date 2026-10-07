# FPVDrone
Pydrake simulation files for FPV Drone for MAE 5810 project.

### PYDRAKE CONFIGURATION:
First, you must run the following command in terminal to establish pydrake environment variable prior to running script (taken from [Drake: Installation via APT](https://drake.mit.edu/apt.html)):
```bash
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
```
- Note: this is for standard Ubuntu WSL installation/implementation, it may differ for other distros.
