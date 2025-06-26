import sys
import os

what_you_can_do = """
Here is what you can do with your new bevel project!

bevel build: build project to run locally
bevel export <platform>: Export to web (more platforms coming soon!)
"""

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)


def send_completion_message(proj_name, path):
    output_message = f"""
Project {proj_name} created at {path} successfully!

To start working in your project, run `cd {proj_name}` and start developing!
    """
    return output_message