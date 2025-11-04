
import sys
import os

venv=os.getenv("VIRTUAL_ENV")
print("Virtual env (python3.%d): "%sys.version_info.minor + str(venv))


sys.path.insert(0,venv+"/lib/python3.%d/site-packages"%sys.version_info.minor)