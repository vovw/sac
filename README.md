### README

**SRA Board Line Follower**

A fast and efficient line follower robot built using the [SRA Board](https://sravjti.in).

**Features:**
- Line following using PID control.
- Intersection and obstacle detection.
- 180-degree turn capability.
- Configurable PID constants via HTTP server.

**Dependencies:**
- [SRA Board](https://github.com/SRA-VJTI/sra-board-hardware-design)
- ESP-IDF
- FreeRTOS

**Build Instructions:**
1. Clone the repository.
2. Set up your ESP-IDF environment.
3. Build and flash the firmware to the SRA Board.

**Usage:**
- Power on the robot.
- Access the PID tuning interface via your browser at the robot's IP address.
- Place the robot on the line and watch it follow.

**Website:**
- [SRA VJTI](https://sravjti.in)
