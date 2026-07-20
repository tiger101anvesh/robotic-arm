2-DOF Planar Robotic Arm (Version 1)

Overview

This repository documents the first version (V1) of a custom-built, two-degree-of-freedom (2-DOF) planar robotic arm. The system is driven by a Raspberry Pi Pico and standard hobby servos. The primary focus of V1 is establishing a robust mathematical foundation for precise spatial manipulation within a 2D Cartesian plane, bypassing rudimentary angle-based control in favour of direct coordinate targeting.

Mechanical Design & CAD
The physical hardware was designed with standardisation and rigid geometry in mind. The current physical configuration relies on two primary linkages:
 - Shoulder to Elbow (Bicep): 100.0 mm
 - Elbow to Wrist (Forearm): 100.0 mm

Hardware Limitations and Software Calibration
Due to the discrete spline tooth count on standard servo motors, achieving a perfectly flush mechanical zero-degree alignment is geometrically improbable. Rather than compromising the physical design, this inaccuracy is resolved in software. The system utilises hardcoded calibration offsets (shoulderOffset and elbowOffset) to translate the pure mathematical angles into physically corrected hardware commands, ensuring the physical linkages align perfectly with the calculated geometry.

Computer-Aided Design (CAD) Files
The custom 3D printable components designed for this assembly can be accessed here:
(https://cad.onshape.com/documents/a808d79a63986f19cc0dd3e8/w/269c81dbee1a74ea6a06b10f/e/55a99f86cc88f248928f3927)

Software Architecture
The control software is written in C++ via the Arduino IDE and is structured around three core operational pillars:

1. Inverse Kinematics (IK)
   - At the core of the system is the calculateIK() function, which utilises the Law of Cosines to convert target Cartesian coordinates (X, Y in millimetres) into requisite joint angles.
   - Extended Range: The servos are attached using custom pulse widths (500μs to 2500μs) to unlock their true 180-degree physical range, overcoming the limitations of the default <Servo.h> library.
   - Kinematic Flipping: The maths includes a geometric safeguard. If a negative X coordinate is parsed, the system recognises the need to reach backwards and dynamically flips the trigonometric functions to calculate the correct reverse geometry without causing mechanical collisions.
   - Elbow-Down Posture: The equations have been tuned to naturally default to an "elbow-down" configuration when reaching forward, mimicking organic movement.

2. Linear Interpolation
Standard servo commands result in arced movements. To achieve precise industrial behaviour, the moveLine() function employs linear interpolation. It calculates the hypotenuse between the current coordinate and the target coordinate, divides this path into individual 1-millimetre segments, and rapidly calculates the required IK maths for every segment. This forces the end effector to draw a perfectly straight line in physical space.

3. Serial Command Parser
The system does not require hardcoded routines. It actively listens to the Serial Monitor (115200 baud) for live user inputs. The custom parser distinguishes between two types of commands:
   - Arc Snap: Entering X Y (e.g., 150 100) executes an immediate, unconstrained movement to the target, allowing the servos to move at maximum velocity along their natural arcs.
   - Linear Movement: Entering line X Y (e.g., line 150 100) triggers the linear interpolation protocol, moving the arm to the target in a highly controlled, straight path.

Usage Instructions:
   - Flash the provided .ino / .uf2 file to the Raspberry Pi Pico.
   - Ensure a stable power supply is connected to the servos (do not power them directly from the Pico's logic pins).
   - Open the Serial Monitor and configure it to 115200 baud with Newline enabled.
   - Upon startup, the system will automatically home itself to 100 100 to synchronise physical space with digital memory.
   - Input commands to control the arm.

Command Examples:
   - 200 0 (Maximum forward extension, arc movement)
   - line 100 100 (Straight line movement back to the home diagonal)

-100 100 (Backward reach via kinematic flip)

Known Limitations and System Evaluation
While the V1 architecture successfully validates the core kinematic maths and Cartesian control, several physical and mathematical limitations have been identified during operation:
 - Mechanical Backlash and Jitter: The utilisation of standard hobby servos introduces inherent mechanical tolerances, resulting in minor gear backlash and electronic jittering at resting coordinates.
 - Linear Interpolation Inaccuracies: Consequently, the physical execution of the moveLine() function exhibits slight spatial deviations from a mathematically perfect straight line due to these gear tolerances.
 - Pathing Constraints: The current software suite is restricted to linear interpolation and unconstrained point-to-point sweeps. It lacks the advanced mathematical functions required to calculate and trace defined curves, specific arcs, or circular trajectories.
 - Kinematic Dead Zones: Due to the initial hardware configuration—where the servo's 90-degree resting position maps to a 180-degree straight arm—the elbow is mechanically prevented from folding into tight, acute angles. This creates a spatial blind spot, prohibiting the end effector from reaching or manipulating objects situated in close proximity to the base pivot.

Strategic Evaluation and Rapid Iteration:
Following an evaluation of the V1 system's performance, I have decided that attempting to engineer out these physical and algorithmic imperfections would require a disproportionate expenditure of time, financial resources, and complex software compensation logic.

Rather than succumbing to diminishing returns by over-optimising a foundational proof-of-concept, I decided to do rapid iteration instead. The minor planar discrepancies in V1 will be accepted as known mechanical constraints, and development resources will immediately pivot to Version 2. This allows the project to maintain momentum and focus on larger, more impactful architectural advancements.

Future Development (V2)

Version 1 establishes pure Cartesian control within a two-dimensional plane. Version 2 will expand this into three-dimensional space by integrating a rotating base turret, upgrading the kinematics from 2D planar to a full 3D spherical coordinate system.
