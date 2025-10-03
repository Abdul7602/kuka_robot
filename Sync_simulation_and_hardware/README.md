# MoveIt via RViz - Hardware (Raspberry Pi 5 Ethernet Setup & Robot Power-Up Guide)

This guide walks you through configuring a static IP for your Raspberry Pi 5 over Ethernet, setting up multicast routing, powering up the robot, and connecting via SSH.

---

## 1. Manual IP Configuration (Ethernet)

> **Important:**  
> Before proceeding, ensure your Raspberry Pi is physically connected to your network via a wired Ethernet cable. This is essential for static IP assignment and successful SSH access.

1. **Assign a Static IP to Your Raspberry Pi:**

   - Open your network settings.
   - Manually set the following values for your Ethernet interface:
     - **Address:** `172.31.1.149`
     - **Netmask:** `255.255.0.0`
   - Save and apply the settings.

   ![Manual IP Configuration](../images/manual_ip.png)

2. **Verify the Network Configuration:**

   - Open a terminal on your Raspberry Pi.
   - Run the following command to view network interfaces:
     ```bash
     ifconfig
     ```
   - Look for your Ethernet interface (commonly `eth0`, `enpXsY`, etc.).
   - Confirm the assigned IP address is correct.

   ![Checking IP](../images/checking_ip.png)

---

## 2. Add IP Route to Multicast Group

To enable multicast communication required by the robot, add the following route:

```bash
sudo ip route add 224.0.0.0/4 dev <your_interface>
```
- Replace `<your_interface>` with the exact interface name found in the previous step (e.g., `enp0s25`, `eth0`, etc.).

---

## 3. Power Up the Robot

Turn on the robot. The Raspberry Pi should remain connected to the network and is now ready to receive commands.

---

## 4. SSH into the Raspberry Pi

SSH into your Pi from your workstation (replace with your credentials if needed):

```bash
ssh ubuntu@172.31.1.148
```
## 4.1. When prompted add the pwd

Password:
```bash
humeaIiwa14!
```

> **Note:**  
> - Double-check that your Ethernet cable is connected before attempting to SSH.
> - Ensure credentials are correct (`ubuntu` as the username by default).
> - If you can't connect, re-verify the static IP assignment and cable connection.

---

 Client side configurations:
            Configure the ```client_command_mode``` to ```position``` in lbr_system_config.yaml

Set the ```update_rate``` to ```100``` in lbr_controllers.yaml

Remote side configurations:

Select:        
    
                FRI send period: 10 ms

                IP address: your configuration

                FRI control mode: POSITION_CONTROL or JOINT_IMPEDANCE_CONTROL

                FRI client command mode: POSITION

Proceed with steps 1 and 2 from MoveIt via RViz - Simulation but with ros2 launch lbr_bringup hardware.launch.py in step 1.


ubuntu@ubuntu:~/lbr-stack$ 

```
ros2 launch lbr_bringup hardware.launch.py     model:=iiwa14 
```

rosuser@rosuser:~/ros2_ws$ 

```
ros2 launch lbr_bringup move_group.launch.py     mode:=mock     rviz:=true     model:=iiwa14
```

---

Launch the OptiTrack System

This command starts nodes responsible for motion capture marker visualization using OptiTrack.

```
ros2 launch combined_launch optitrack_bringup.launch.py
```
This will bring up nodes related to the OptiTrack system and publish marker transforms.

## Troubleshooting

- **Can't connect via SSH?**
  - Confirm Ethernet cable is securely plugged in.
  - Re-run `ifconfig` to ensure the correct static IP is active.
  - Check your workstation is on the same subnet (`172.31.0.0/16`).
