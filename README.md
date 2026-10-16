# FPVDrone
Pydrake simulation files for FPV Drone for MAE 5810 project.

## IMPLEMENTATION:
- Ubuntu-26.04 (WSL)
- Python 3.14.4
- 

## PYDRAKE CONFIGURATION:
First, you must run the following command in terminal *or* include it in your ```~/.bashrc``` file to establish pydrake environment variable prior to running script (taken from [Drake: Installation via APT](https://drake.mit.edu/apt.html)):
```bash
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
```

You must *also* install ```xacro``` so that the script an parse the ```.xacro.urdf``` file. To do this:
```bash
pipx install xacro
```

- Note: this configuration is for standard Ubuntu-26.04 WSL installation/implementation. It may differ for other distros.
