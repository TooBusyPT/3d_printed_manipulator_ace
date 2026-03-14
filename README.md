# 3D-Printed Robotic Arm — ACE Final Project

A color-sorting robotic manipulator built with a **Raspberry Pi Pico (RP2040)**, 4 servo motors, a ToF distance sensor (VL53L0X), and an RGB color sensor (TCS34725).
Developed as the final project for the Embedded Computing Architectures course at FEUP (University of Porto).

The arm computes inverse kinematics to reach target positions in cylindrical coordinates and operates across three autonomous modes: **single pick-and-drop**, **3×3 grid scanning**, and **autonomous object detection via ToF sweep**.
All parameters are configurable at runtime through a terminal interface and persisted to flash memory — no recompilation needed.

## 📹 Demo
👉 [Watch on YouTube](https://youtu.be/tKDbqyOoan8)

## 📁 Repository Contents
- `src/` — Source code (C++, Raspberry Pi Pico SDK)
- `report.pdf` — Full project report (PDF)

## Authors
João Nuno Borges Ruivo & Pedro Miguel Oliveira Rodrigues