import os
# bevel/__init__.py

lib_dir = os.path.dirname(__file__)
current_ld = os.environ.get("LD_LIBRARY_PATH", "")
new_ld = lib_dir + (":" + current_ld if current_ld else "")
os.environ["LD_LIBRARY_PATH"] = new_ld

__version__ = "0.1.0"