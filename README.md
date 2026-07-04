# Pirkka-R2-D2

An interactive miniature robot developed as a group project for the Principles of Digital Fabrication course at the University of Oulu (2025). Inspired by R2-D2, the robot is controlled gesture-based by moving objects in front of its optical sensors. The project integrates custom 3D-printed mechanics, embedded electronics, and Python-based control logic.

## Goal and Overview
The objective was to design and build a fully functional physical prototype combining mechanical engineering, electronics design, and software development. Built around a microcontroller, the system satisfies core robotics requirements by integrating sensory input (detecting hand/object movement) with physical actuators (motor control).

### Tools and Software Used:
* **Software:** Python (`main.py`)
* **Electronics:** Schematics and circuit design created using KiCad (`Schematics.kicad_sch`)
* **3D Modeling & Mechanics:** Chassis and legs modeled in Autodesk Fusion 360 (`.f3d` and `.svg` files included for 3D printing and laser cutting)

## 📁 Project Structure

```text
├── src/
│   └── main.py              # Main Python control script for the robot
├── 3D-models/
│   ├── Foot_V2.f3d          # 3D models for the robot's legs
│   ├── Middle_Section...    # 3D models for the middle section of the chassis
│   └── *.svg                # Vector files for laser-cut chassis components
├── electronics/
│   └── Schematics.kicad_sch # Complete hardware wiring and circuit schematic
└── images/
    ├── Diagram_1.jpg        # System architecture diagram
    └── Diagram_2.jpg        # Structural drawing
