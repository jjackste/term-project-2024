# term-project-2024

1.0 Introduction
Robotic systems are being developed to collect objects of interest from environments that are presently too dangerous for
humans (Figure 1). The primary objective of these robots is to traverse unknown terrains and collect objects of high commercial
value. To minimize transportation costs and maximize efficiency, the robots must be compact and lightweight. They must also
have sufficient autonomy to ensure the safety of the robot at all times, particularly in the event of communication dropouts.
Figure 1. Scavenger robot that can navigate in an unknown environment to collect high-value objects.
With the robot safely deployed near an area of interest, the objective is to have it collect uncontaminated objects that are deemed
to be valuable based on specific physical properties, while ensuring that no low-value or contaminated objects are collected.
Each team is required to design and construct a semi-autonomous scavenger robot that can depart from a base location, find
and gather the objects of interest from the ground, return to the base location, and deposit only the retrieved valuable objects
into a collection container. Objects are deemed to be valuable based on their reflectance/absorption of specific wavelengths
(i.e., their colour). It can be assumed that the robot is operating in a relatively flat area without any major obstacles.
The robot must be self-powered and all parts must be brought back once the task is complete. The robot and operator control
station must use an ESP32-based microcontroller. The unstructured environment will not be directly visible by the remote
operator; instead, a wireless camera mounted on the robot will provide visual feedback. The object identification process and
object pickup and/or sorting actions must be automated. The use of the motors, drives, sensors, and other components included
in the MME 4487 Lab Kit is encouraged.
Each team will create a working prototype of their design for a demonstration at the end of the academic term. The exploratory
robot should function under a large variety of conditions, including the lab, hallways, and homes. Prior to the final
demonstration of the prototype, the robot and operator control station must be thoroughly tested for operating range, accuracy,
repeatability, reproducibility, sensitivity and any other relevant performance measures. All performance tests must be
documented with clearly stated goals, structured methodology, thorough analyses, and detailed assessment of the experimental
results.

2.0 Additional Constraints
From a design perspective, the semi-autonomous robotic system can consist of two or more separate interfaced (i.e., tethered)
modules. At the beginning of the timed mission, the entire robot must fit in a 25 cm x 25 cm x 25 cm box. However, it must be
self-powered and all parts that start the task must be brought back upon completion. While the robot may use multiple batteries,
every team must use the same batteries supplied in the MME 4487 Lab Kit without any modifications. Further, an operator
can provide interactive guidance to the robot through remote control. The ESP32-based remote control system must
communicate with the robot’s ESP32 microcontroller. Given the nature of the task, the remote environment will not be directly
visible by the remote operator. A single digital camera can be mounted on the robot to provide visual feedback to the operator.
While there are a number components and other parts available in SEB 1068, each team is encouraged to design and fabricate
as many components as possible from lightweight and easily machined materials. A laser cutter is available in the Design and
MME 4487a — Mechatronic System Design Term Project 2024 Page 2
Manufacturing Studio (SEB 1071) for cutting parts from sheet goods (paper, cardboard, corrugated plastic, acrylic, MDF, etc.).
There are also two 3D printers in SEB 1068, along with number of additional 3D printers in the 3D Printing Lab (SEB 1063)
that may be used to print parts with complex geometries. At least 50% of the robot should be made from designed components.
The use of VEX structural components is strongly discouraged. All parts that are fabricated or machined must have correctly
dimensioned detail drawings. Appropriate engineering analysis, including failure analysis, is required for critical fabricated
components. Detailed assembly drawings and parts lists are required for all components.

3.0 Milestones (20 Marks)
Four project milestones are required to be submitted electronically on OWL during the term. These are as follows:
1. Concept Generation and Selection, due Monday, October 21, 2024. This report should include concepts generated for
individual subsystems, along with the overall design of the robot. The most promising overall design should be
selected and justified.
2. Detail Design documentation, due Monday, November 4, 2024. This includes a complete CAD model (eDrawing +
SolidWorks/NX) and the bill of materials for the robot.
3. Drive System implementation, due Friday, November 15, 2024. A brief video that demonstrates the fully functional
robot drive system, along with corresponding microcontroller code.
4. Pickup and/or Sorting Mechanism implementation, due Friday, November 22, 2024. A brief video that demonstrates
the fully functional pickup and/or sorting mechanism, along with corresponding microcontroller code.
Each milestone is worth 5%. Failure to submit a milestone will result in a project grade of 0.
4.0 Showcase Demonstration and Prototype (15 Marks)
Each team is required to submit a brief digital video to the instructor showing the capabilities of their fully functional design
by 11:59 pm on Wednesday, November 27, 2024. The video should demonstrate the efficacy of their design with respect to
each of the tasks outlined in the problem description and clearly show the functionality of the complete system. The semi-
autonomous robot will be demonstrated during the lecture time on Thursday, November 28, 2024. The robot will be given
three minutes to maneuver through the unstructured environment, collect objects, return to the point of origin, and deposit the
objects into the collection container. Points will be awarded based on completion time, the number of valuable samples
collected, and the weight of the robot, as follows:
Following the showcase, the overall mechanical and electrical build quality of the prototype will also be assessed. The prototype
must be placed in SEB 1068 on, or before, Friday, December 6, 2024.

5.0 Technical Report (30 Marks)
As part of the product development process, the team will prepare a technical design report that provides a thorough
description of the semi-autonomous exploratory robot and target recovery system. The final report must summarize the key
requirements and engineering specifications. The main body of the report should also briefly describe viable alternative
concepts and extensive engineering details on the final proposed solution. The solution must be evaluated in terms of product
function, performance requirements, and technology readiness. The format of the main report must follow the guidelines
presented in the document Preparation of Papers for IEEE Transactions and Journals. A template for the report is available
at: http://www.ieee.org/web/publications/authors/transjnl/index.html
In addition, all detailed drawings, assembly drawings, C code, and supportive supplementary material (including original
experimental data) must be submitted in a separate Appendix. The completed Technical Report and Appendix must be
submitted electronically on OWL on, or before, Friday, December 6, 2024.
6.0 Mark Distribution
It will be assumed that all design team members have made an equal contribution to the project, unless the team decides to
submit individual performance assessments.
Note: The above requirements are subject to adjustments and changes as needed.