import os
import subprocess
ld_library_path = (
    os.path.dirname(__file__)
    + "/arm_control:"
    + os.path.dirname(__file__)
    + "/arm_control/FK:"
    + "/lib/x86_64-linux-gnu" + ":"
    + "/usr/local/lib"
    # + os.environ["LD_LIBRARY_PATH"]
    + ":$LD_LIBRARY_PATH"
)

print(f"export LD_LIBRARY_PATH={ld_library_path}")