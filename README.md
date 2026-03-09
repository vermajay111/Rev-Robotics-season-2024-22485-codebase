# FTC Robot Controller - Rav Robotics (Centerstage Season)

This project contains the codebase I developed during my time as the coding lead for the **Rav Robotics** team in my high school robotics club. It was my first year in robotics, and I led the coding for an autonomous robot used in competition.

## Problem Overview

Before I joined the team, our robot used simple for-loops to control motor power for a few seconds, hoping that the motors would drive the robot as expected. However, due to factors like friction (from wheels, ground, and motor gearbox), timing issues, and motor lag, the robot’s positioning was inconsistent. This meant that every autonomous run was different, leading to unreliable performance.

### My Approach

I focused on making the robot’s movement predictable and repeatable by implementing a more accurate control system using sensors and encoders. Here's how I tackled it:

1. **Mecanum Wheel Drive Setup**  
   I implemented mecanum drive based on [this documentation](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html), allowing the robot to move in all directions with precise control.

2. **Driver Assistance Features**  
   I added an automatic slow-down feature that reduced speed when the robot got too close to walls or obstacles. However, this caused significant lag, so I reduced the frequency of the calculation to prevent slowdowns. After receiving feedback from the team and drivers, I ultimately removed the feature.

3. **Artificial Gears for Speed Control**  
   To provide the drivers with more control, I created a system for adjusting the robot’s maximum speed via "artificial gears." This allowed the driver to set the max speed for different joystick input thresholds. The drivers liked this feature, so it was kept.

4. **Field-Centric Control**  
   I implemented field-centric control, so the robot’s movements always respected the driver’s perspective. This way, pushing the joystick backward would always move the robot backward, regardless of its orientation. This removed the mental overhead of figuring out which way to turn based on robot orientation.

---

## Moving to Autonomous Navigation

### The Next Challenge: Autonomous Path Following

The real challenge came when I realized we could use autonomous navigation with object detection to score more points by accurately identifying and interacting with props placed on the field. Here's how I approached it:

1. **Object Detection Setup**  
   I started by collecting and labeling a dataset of custom props used by the team. After figuring out how to use computer vision, I faced the dilemma of which object detection model to use: YOLO (known for accuracy but computationally heavy) or FPN Lite (more lightweight but potentially less powerful).  
   
   I trained a **YOLO v7 small model**, but ran into issues with the built-in FTC Vision library, which wasn’t compatible with YOLO’s input/output format. After significant debugging, I had to switch to the **FPN Lite** model, which worked with the FTC Vision library and was compatible with the robot’s environment.

2. **Path Following with Road Runner**  
   To make the robot follow precise paths, I integrated **Road Runner**, a path-following library that required significant tuning to work properly. This allowed for smooth trajectory execution and minimized the impact of friction, motor inconsistencies, and encoder errors.

3. **Simulation with MeepMeep**  
   Given the limited time for real-world testing, I used a virtual robot simulation called **MeepMeep** to pre-run complex trajectories before deploying them on the physical robot. MeepMeep allowed me to test code in a simulated environment with different setups, reducing the need for constant real-world testing.  
   
   I had to figure out the correct versions of Java and SDKs to use with MeepMeep, which required some trial and error, but once it was set up, it saved us countless hours.

---

## Getting Started

To get started with this project, follow these instructions:

1. Clone the repository to your local machine.
2. Install the necessary **FTC SDK** and dependencies.
3. Follow the **Road Runner** and **MeepMeep** setup guides to configure the simulation environment.
4. Use the provided code for the robot’s drive system, autonomous path following, and object detection.

---

## Features

- **Mecanum Drive**: Full omni-directional movement using mecanum wheels.
- **Field-Centric Control**: Movement based on driver orientation, not robot orientation.
- **Artificial Gears**: Adjustable speed thresholds for more precise driver control.
- **Object Detection**: Uses machine learning to detect props and adjust navigation accordingly.
- **Autonomous Navigation**: Path following and decision-making to navigate around obstacles.
- **Simulation with MeepMeep**: Test paths and behaviors in a virtual environment before deploying to the robot.

---

## Challenges & Solutions

- **Motor Lag and Inconsistent Movement**:  
  Implemented precise control with sensors and encoders to reduce friction’s impact and increase consistency.
  
- **Lag from Driver Assistance Features**:  
  Reduced the frequency of the slowdown calculations to avoid lag. The feature was ultimately removed based on driver feedback.

- **Object Detection Model Compatibility**:  
  Had to switch to a compatible FPN Lite model after encountering issues with YOLO and the FTC Vision library.

- **Time Constraints**:  
  Used MeepMeep for virtual simulation to test paths and debug without needing physical testing time.

---

## Future Work

- **More advanced object detection and decision-making**  
- **Optimizing autonomous path-following algorithms**  
- **Driver Assistance Features**  

---

## Conclusion

Through trial and error, research, and perseverance, I was able to transform the robot into a much more capable and reliable system. The knowledge I gained from this project laid the foundation for my continued work in robotics.

---

### Links

- [Road Runner Documentation](https://learnroadrunner.com/introduction.html#frequently-asked-questions)  
- [MeepMeep Demo](https://learnroadrunner.com/assets/trajectorybuilder-functions/spline-to-spline-heading.mp4)  
- [Rav Robotics - Centerstage Season Preview](https://www.youtube.com/watch?v=6e-5Uo1dRic)
