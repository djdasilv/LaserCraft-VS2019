# Haptic Interface Program

An interactive program for controlling a robotic system via a haptic device, keyboard, and mouse. Designed for semi-automated sample scanning, real-time visualization, and 3D mesh reconstruction from THG signal data.

---

## 🔧 Features

- 🎮 **Haptic-controlled movement** for intuitive robot navigation  
- ⌨️ **Keyboard & mouse support** for full control over scanning, movement restriction, and visualization  
- 🔄 **Semi-automated scanning** in X, Y, or Z direction with output to `.csv`  
- 🖼️ **3D sample rendering** and multi-angle visualization  
- 🧩 **Mesh generation** using the TetGen library from collected THG point clouds  
- ⚙️ **Live mesh filtering** by triangle area and edge length for quality control  

---

## 🕹 Controls Overview

> For complete layout:  
> ![Program Instructions](Haptic%20program%20instructions.png)

### Movement Control

- `x`, `y`, `z`: Lock/unlock movement on each axis (can combine for 1D constraint)
- `0`–`4`: Set scale factor between haptic device and robot (e.g., `0` = 0.001×)

### Scanning

- `G`, `H`, `J`: Start/stop scanning along X, Y, Z respectively
- Output: CSV file with robot position and THG signal

### Visualization & Interaction

- Arrow keys: Move camera around the center of the sampling cube
- Mouse wheel: Zoom in/out
- `.` (dot): Reset robot origin to current position (e.g., under laser)

### Sample Rendering

- `U`: Toggle sample rendering  
  - Displays test sample if <12 points  
  - Uses real sample after 12+ points from FemtoTouch setup  
- Limit: 5000 points max for real-time mesh

### Mesh Filtering (Post-Processing)

- Triangle area filter:
  - `W`: Increase area threshold
  - `S`: Decrease area threshold
- Triangle edge length filter:
  - `E`: Increase length threshold
  - `D`: Decrease length threshold
- Filtering is done using the **most restrictive** of both conditions

---

## 🧪 Notes on Mesh Generation

The mesh is built from a point cloud where the THG signal exceeds a preset threshold. TetGen is used to convert this point cloud into a tetrahedral mesh. However:

- TetGen tries to close the mesh, sometimes adding unrealistic large triangles
- Use mesh filters (`W`, `S`, `E`, `D`) to remove such artifacts
- Increasing point count improves quality, but increases processing time  
  > ⚠️ Current real-time setup limited to 5000 points

---

## 💼 License & Usage

This software is **publicly viewable** but **restricted for use only within the GALATEA at EPFL**.

