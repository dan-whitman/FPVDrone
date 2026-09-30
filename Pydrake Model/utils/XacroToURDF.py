import subprocess

##### XACRO FUNCTION (inputs xacro file outputs urdf string)
def XacroToURDF(filepath: str) -> str:
    cmd = ["xacro", filepath] # base command line

    # running result
    rawurdf = subprocess.run(cmd, capture_output=True, text=True, check=True)

    return rawurdf.stdout # return urdf string
