from setuptools import setup, find_packages

setup(
    name="bevel",
    version="0.1.0",
    description="Bevel Game Dev CLI",
    author="Your Name",
    packages=find_packages(),
    include_package_data=True,
    package_data={
        "bevel": ['project_template/**/*']
    },
    install_requires=[
        "raylib-py>=5.0.0",
        "pyyaml"
    ],
    entry_points={
        "console_scripts": [
            "bevel=bevel.cli:main"
        ]
    },
    python_requires=">=3.8",
)
