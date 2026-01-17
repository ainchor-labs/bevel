python3 -m pip uninstall --break-system-packages bevel -y
rm -rf build/ dist/ *.egg-info/ __pycache__/
pip install --break-system-packages --no-cache-dir -e /mnt/Projects/Ainchor/bevel