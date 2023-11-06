# About Telebot's Systems
The Telebot-4R is broken up into systems, levels, and subsystems. This division allows for a fine-grained control of dependency between subsystems.
## System
Represents *where* the subsystem exists on a conceptual level. For example, the **Motion** subsystem is a part of the **Telebot** system. The **Unity** subsystem is part of the **Control** system.
## Level
Denoted "L_\<n\>" where *n* is where it sits in the hierarchy of subsystems. A subsystem of level *n* can only depend on subsystems of level *n-1* or below.

Examples:
- An L_2 system can rely on an L_1 system
- An L_4 system can rely on L_3, L_2, and L_1 systems
- An L_1 system cannot rely on an L_2 system or other L_1 systems, but could hypothetically rely on L_0 systems
## Subsystem
A subsystem is a collection of nodes and the data that flows between them to accomplish a common goal or functionality. Subsystems should be contained in a single package.