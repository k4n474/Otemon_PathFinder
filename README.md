# Otemon PathFinder

<p align="center">
  <img src="images/Team_images/team_photo.JPG" alt="team_photo" width="45%">
  <img src="images/Team_images/funny_photo.JPG" alt="funny_photo" width="45%">
</p>



# Robot

## 1. Robot Introduction

<p align="center">
  <img src="images/Robot_images/IMG_R1.JPG" alt="Robot1" width="45%">
  <img src="images/Robot_images/IMG_R2.JPG" alt="Robot2" width="45%">
</p>

<p align="center">
  <img src="images/Robot_images/IMG_R3.JPG" alt="Robot3" width="45%">
  <img src="images/Robot_images/IMG_R4.JPG" alt="Robot4" width="45%">
</p>

<p align="center">
  <img src="images/Robot_images/IMG_R5.JPG" alt="Robot5" width="45%">
  <img src="images/Robot_images/IMG_R6.JPG" alt="Robot6" width="45%">
</p>

<p align="center">
  <img src="images/Robot_images/IMG_R7.JPG" alt="Robot7" width="45%">
  <img src="images/Robot_images/IMG_R8.JPG" alt="Robot8" width="45%">
</p>

<p align="center">
  <img src="images/Robot_images/IMG_R9.JPG" alt="Robot9" width="45%">
  <img src="images/Robot_images/IMG_R10.JPG" alt="Robot10" width="45%">
</p>

## Controller

Our robot uses a **Raspberry Pi 5** as its main controller.

To accurately recognize objects, the robot must process camera images while simultaneously handling data from multiple sensors, including a LiDAR and a gyroscope. Therefore, we selected the Raspberry Pi because of its high computing performance and excellent expandability.

Unlike a conventional microcontroller, the Raspberry Pi is a compact computer that runs Linux. As a result, it is less suitable for applications requiring strict real-time control, such as motor control, and it also consumes more power. However, its powerful processing capability allows it to handle computationally intensive tasks, including image processing and LiDAR point cloud processing. Since these functions are essential for our robot, we concluded that the Raspberry Pi was the most suitable controller.

At the beginning of development, we used a **Raspberry Pi 4**. However, image processing required more computational power than expected, making real-time control difficult in some situations. After upgrading to the Raspberry Pi 5, image processing became significantly faster, resulting in much more stable driving performance.

## LiDAR

Our robot uses a **LiDAR** for wall detection, distance measurement, and wall following.

We initially considered using ultrasonic sensors. However, ultrasonic sensors can only measure distance in a single direction, whereas the LiDAR can obtain distance information from almost the entire surrounding area, although part of its field of view is blocked by components such as the Raspberry Pi. This allows the robot to detect obstacles in front while simultaneously measuring the distances to both side walls, enabling more accurate self-localization.

One of the greatest challenges during development was designing an algorithm capable of accurately detecting walls using LiDAR point cloud data. In addition, because the robot must process a large amount of distance data in real time, optimizing the processing speed was also a major challenge. Through repeated testing and refinement, we continuously improved both the algorithm and the software until the current system was completed.

![LiDAR](images/Other_images/LiDAR_screen.png)

## Camera

At the beginning of development, we used a **HuskyLens** because it allowed us to implement object recognition relatively easily. However, we found limitations in both recognition accuracy and flexibility, so we decided to develop our own image recognition program from scratch.

We use a **Raspberry Pi Camera Module 3 Wide** because of its excellent compatibility with the Raspberry Pi and its ability to capture images at high speed. Initially, we used the standard **Raspberry Pi Camera Module 3**, but its horizontal field of view was only about **66°**, causing obstacles to occasionally move outside the camera image. As a result, reliable obstacle avoidance was difficult. After replacing it with the **Camera Module 3 Wide**, the horizontal field of view increased to approximately **102°**, allowing the robot to detect obstacles much more reliably.

The captured RGB image is first converted into the **HSV color space**. HSV consists of **Hue, Saturation, and Value,** and is less affected by changes in lighting conditions than RGB. This enables the robot to distinguish red and green objects more reliably by using hue information.

<p align="center">
  <img src="images/Other_images/RGB_image.png" alt="Robot9" width="45%">
  <img src="images/Other_images/HSV_image.jpg" alt="Robot10" width="45%">
</p>

After the conversion, separate color masks are applied to the red and green objects, followed by binary thresholding to extract only the target objects. Contour detection is then performed to obtain the coordinates of the four corners of each object. These coordinates are used to determine each object’s position and color, and the resulting information is used for navigation.

![Camera](images/Other_images/camera_screen.png)

## Chassis

Most of the robot’s mechanical components, excluding electronic parts, were designed by our team and manufactured using **3D printers**. This allowed us to create custom parts that would have been difficult or impossible to produce using commercially available components, enabling a structure optimized specifically for our robot.

We selected **PETG** as the printing material. Although PLA is easy to print, its durability is limited. ABS provides excellent strength, but it is prone to warping and requires more demanding printing conditions. PETG offers an excellent balance between printability and mechanical strength, making it the most suitable material for our robot.

We use both a **Bambu Lab A1** and a **Bambu Lab X2D** for manufacturing. At the beginning of development, all parts were produced using only the A1. However, because nearly all of the robot’s mechanical components are 3D printed, a single printer required an excessive amount of production time. Therefore, we introduced the X2D, allowing multiple parts to be printed simultaneously.

Compared with the A1, the X2D provides higher printing speed and better print quality, enabling us to manufacture high-quality components in a shorter amount of time. We currently use both printers according to the requirements of each part.

## Steering Mechanism

Initially, our robot used a conventional steering mechanism. However, it could not achieve sufficient turning performance when negotiating sharp corners.

To improve cornering performance, we adopted an **Ackermann steering mechanism** for the front wheels. This allows each front wheel to follow the appropriate turning radius while sharing the same turning center, reducing tire slip and enabling smoother cornering.

![Ackermann](images/Robot_images/ackermann_image.png)

In addition, we selected **high-torque drive motors**, providing sufficient driving force even under heavy loads and ensuring stable vehicle movement.

A **differential gear** is also installed on the rear axle. During cornering, it absorbs the rotational speed difference between the left and right wheels, reducing mechanical stress on the tires and enabling smoother turns.

![Gear](images/Robot_images/gear_image.png)


## Electrical System

To prevent voltage drops and electrical noise generated by the drive motors from affecting the control system, our robot uses **separate power supplies for the drive system and the control system.**

The drive motors are powered by a battery pack consisting of **three 18650 lithium-ion batteries connected in series**. Since the fully charged battery voltage is approximately **12 V**, the motors are powered directly without additional voltage conversion.

The steering servo is powered through a **buck converter (DC-DC converter)**, which steps the voltage down to **5 V** and provides a stable power supply.

The Raspberry Pi is powered by a **5,000 mAh USB Power Delivery (PD) power ban**k. Separating the motor and control power supplies reduces malfunctions caused by voltage fluctuations and electrical noise, significantly improving the overall stability of the robot.

To simplify wiring as additional functions were added, we also designed and manufactured a **custom Raspberry Pi HAT board**. This board organizes the wiring, simplifies assembly and maintenance, and improves the overall maintainability of the robot.

# Software

## Open Challenge

In the **Open Challenge**, the robot navigates primarily using **LiDAR**. As described earlier, we selected LiDAR because it provides distance information from almost the entire surrounding area, allowing the robot to perceive its environment with high accuracy.

During operation, the robot continuously measures the distances to the front wall and both side walls using the LiDAR. The distances to the side walls are used as the input for a **PID controller**, enabling the robot to maintain a stable position near the center of the course. In addition, the robot’s heading is continuously corrected using a **gyroscope**, allowing it to maintain a stable orientation while driving.

When the distance to the front wall falls below a predefined threshold, the robot determines that it has reached a corner. It then performs a **90-degree turn** while continuously monitoring its heading with the gyroscope. This feedback-based control allows the robot to achieve consistent and accurate turns.

The driving direction is determined by monitoring changes in the distances to the side walls. If the distance to one side suddenly increases, the robot determines that the wall on that side has ended and turns in that direction. This method enables the robot to adapt automatically to different course layouts and driving directions used in the competition.

## Obstacle Challenge

In the **Obstacle Challenge**, the robot uses a **Raspberry Pi Camera** to detect obstacles and control its movement accordingly.

First, the robot detects the contours of all visible objects in the camera image and obtains the coordinates of their four corners together with their colors (**red** or **green**). When multiple objects are detected simultaneously, the object that appears lowest in the image is regarded as the closest obstacle and is given the highest priority for avoidance.

During obstacle avoidance, the robot passes **to the left of red obstacles** and **to the right of green obstacles**. The steering angle is continuously calculated so that the center of the detected object moves toward a predefined target position in the camera image.

Once the center of the object passes a predefined reference position, the robot determines that the obstacle has been successfully cleared. The steering is then gradually returned to the neutral position, allowing the robot to smoothly return to the center of the course and continue driving straight.

By continuously repeating this process, the robot is able to avoid multiple obstacles in sequence while maintaining stable and reliable navigation around the course.
