# Bevel

## About
Bevel is an open source cli-based tool that uses raylib as the main framework, with the goal of having a unity or godot-type feel in terms of end-user development

Currently only 2D, but 3D is desired

## Install instructions

Instructions were tested on linux and should work on mac

To use currently compiled version:
```
sudo cp dist/bevel /usr/local/bin
sudo chmod +x /usr/local/bin/bevel
```

To compile from scratch
```
pip install -r requirements.txt
pyinstaller \
    --clean \
    --noconfirm \
    --onefile \
    --add-data "project_template:project_template" \
    --add-data "raylibpy/bin/64bit:raylibpy/bin/64bit" \
    bevel.py
sudo cp dist/bevel /usr/local/bin
sudo chmod +x /usr/local/bin/bevel
```

## Future Desired additions

- State Machine for animation
- LDtk integration for level design
- Box2D for physics
- Collisions (Physics-based and non-physics based)
- Shaders with GLSL

More features to come

## Working commands

`bevel init <project_name>`: Create a project
`bevel run`: Runs project locally

## Demo
See example folder for example project