# Bevel

## About
Bevel is an open source cli-based tool that uses raylib as the main framework, with the goal of having a unity or godot-type feel in terms of development

Currently only 2D

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
- Ability to have 3d as well
- LDtk integration for level design
- Box2D for physics

More features to come