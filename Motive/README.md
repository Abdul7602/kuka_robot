# Motive Calibration Guide

This guide explains how to calibrate the ground plane and set the correct height offset in OptiTrack Motive.

---

## Calibration Steps

### 1. Change Ground Plane
- Open Motive and navigate to **Reconstruction → Ground Plane**.
- Select **Change Ground Plane** to redefine the floor reference.

### 2. Select L-Shape Markers
- Place the calibration object (L-frame with markers) on the capture area.
- In Motive, select the markers forming the **L-shape**.
- This defines the X and Z axis orientation for the ground plane.

### 3. Set Height Offset
- Go to the **Ground Plane Settings**.
- Choose **Custom** mode.
- Enter the **Vertical Height = 6 cm** (or the exact measured offset).
- Apply the **vertical offset** to match the true height of the markers above the ground.

### 4. Apply and Confirm
- Confirm the changes and apply the calibration.
- Check the 3D view to ensure the ground plane is aligned correctly with the floor.
- Verify that the height offset has been correctly applied.

---

## Notes
- Always ensure the L-shape calibration frame is flat and stable on the ground.
- Double-check the height offset value before finalizing.
- Recalibrate if the markers are moved or if the capture volume setup changes.

---

## Troubleshooting
- **Markers not detected properly**: Clean the reflective markers and ensure cameras are calibrated.
- **Ground plane looks tilted**: Reselect the L-shape markers carefully and repeat calibration.
- **Height mismatch**: Verify the measured offset value and reapply.
