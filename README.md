# EE Bachelor's Capstone Project - Cartesian 3D Printer

## Project Overview
This project involves the ground-up design and programming of a custom Cartesian 3D printer. This capstone project focuses on developing a hardware and software ecosystem — spanning from custom PCB design to a custom firmware.


## Core Objectives
*   **Mechanical Systems:** Design and assemble a rigid Cartesian frame optimized for precise XYZ axis motion control.
*   **Hardware Design:** Architect and route a custom mainboard Printed Circuit Board (PCB) from scratch, managing power distribution, stepper driver interfaces, and sensor inputs.
*   **Firmware Development:** Write dedicated, low-latency control firmware to handle G-code parsing, thermal regulation (PID loops), and precise motor acceleration profiles.


## System Architecture
*    **[HW & SW Architecture Diagram](./docs/assets/)** - Architectural block diagrams


## Technical Stack & Tools
*   **Hardware / PCB Design:** Altium Designer
*   **Firmware Framework:** ESP-IDF, C


## Key Engineering Challenges
*   **Thermal Management:** Implementing accurate PID control for the heated bed and hotend while ensuring hardware-level overtemperature protection.
*   **Signal Integrity:** Mitigating high-frequency switching noise from stepper drivers on the custom mainboard to ensure clean sensor data.
*   **Real-Time Execution:** Ensuring deterministic step generation and avoiding timing jitter during complex multi-axis moves.
