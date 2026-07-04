# Pirkka-R2-D2

An interactive miniature robot developed as a group project for the Principles of Digital Fabrication course at the University of Oulu (2025). Inspired by R2-D2, the robot is controlled gesture-based by moving objects in front of its optical sensors. The project integrates custom 3D-printed mechanics, embedded electronics, and Python-based control logic.

## Goal and Overview
The objective was to design and build a fully functional physical prototype combining mechanical engineering, electronics design, and software development. Built around a microcontroller, the system satisfies core robotics requirements by integrating sensory input (detecting hand/object movement) with physical actuators (motor control).

### Tools and Software Used:
* **Software:** Python (`main.py`)
* **Electronics:** Schematics and circuit design created using KiCad (`Schematics.kicad_sch`)
* **3D Modeling & Mechanics:** Chassis and legs modeled in Autodesk Fusion 360 (`.f3d` and `.svg` files included for 3D printing and laser cutting)

## My Role in the Project

Even though we all had specific assigned roles at the start and I wasn't officially the leader, I naturally stepped up to coordinate a lot of the practical work to keep the project moving forward. 

My main contributions and tasks included:

* **Project Coordination:** I supported teammates, gave instructions, and made sure everyone knew what to do next so our progress didn't stall.
* **Mechanical Design:** My main personal task was designing the robot's legs to work properly with the internal actuator and attached wheel. I started by crafting simple cardboard prototypes before designing the final vector files in Inkscape.
* **Digital Fabrication:** I personally handled the manufacturing side, taking full care of operating the 3D printers and laser cutters for the team.
* **Problem Solving:** The biggest challenge was making the leg structure simple yet sturdy enough. After multiple iterations, I ended up using laser-cut parts assembled with interlocking finger joints. It worked well for our timeline, though with more time, I would have iterated further for even better reliability.

## Project Structure

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
    ├── Diagram_1.jpg                                 # System architecture diagram
    ├── Diagram_2.jpg                                 # Structural drawing
    ├── chassis_3d_print_detailed.jpg                 # Close-up of 3D-printed chassis components
    ├── electronics_breadboard_motor_driver_test.jpg  # Motor driver testing and breadboard wiring
    ├── electronics_breadboard_sensor_integration.jpg # Microcontroller and optical sensor integration
    └── robot_prototype_chassis_leg_assembly.jpg      # Assembled physical prototype with chassis and legs
└── videos/
    └── Pirkka-R2-D2.mp4   # Video of the robot in action
```

![robot](images/robot_prototype_chassis_leg_assembly.jpg) **
