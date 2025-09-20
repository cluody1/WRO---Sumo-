# Engineering Design Document – LEGO EV3 Sumo Robot


This engineering document describes the full design of a LEGO EV3-based Sumo robot intended for WRO Sumo competitions. 
The aim is to maximize pushing strength, stability, and reliability under competition rules. 
The design integrates mechanical, electrical, and programming strategies into a single robust solution.



## Concept of the Design
The robot is built around a compact and stable chassis with a wedge-shaped front (~20° angle). 
This wedge is the main offensive tool, sliding under opponents and lifting them slightly to reduce their traction. 
The structural frame is symmetrical and uses LEGO Technic beams reinforced for stiffness. 
The EV3 Intelligent Brick is positioned in the middle to optimize balance and maintain a low center of gravity. 
Every design choice is made to maximize the score on the WRO rubric: clear documentation, effective mechanics, stable programming, and visible strategy.



## Mechanical Design and Wheels
- **Rear drive wheels (70 mm rubber tires):** Powered by EV3 Large Motors, these wheels are the main source of traction. 
  Using torque τ = F × r, the motors (stall torque ~40 N·cm, rated ~20 N·cm at ~160 rpm) deliver strong pushing force. 
  Larger wheels provide a good balance between pushing torque and forward speed.
- **Front wheels (30 mm silicone rollers):** Small passive wheels at the front reduce wedge height, lower the overall chassis front profile, 
  and allow smooth pivoting during turns. Their silicone surface increases stability without generating excessive rolling resistance.
- **Why large rear and small front wheels?**  
  The rear wheels maximize pushing force and grip, while the front small wheels reduce turning resistance and ensure the wedge slides under the opponent effectively. 
  The slope created by this wheel arrangement improves the effectiveness of the wedge.



## Sensors and Their Placement
- **Color sensors (2x, rear):** Detect the white boundary line of the ring to avoid falling out. 
  They continuously scan the surface and trigger immediate retreat when the edge is detected.
- **Ultrasonic/Distance sensors (2x):** One mounted at the front to detect opponents and one at the rear to detect threats from behind. 
  They enable active searching and strategic repositioning.
- **Touch sensors (2–3x, front bumpers):** Provide physical confirmation of contact with an opponent before engaging full pushing force.
- **Gyro sensor (IMU):** Monitors orientation and prevents the robot from spinning uncontrollably.



## EV3 Intelligent Brick Placement
The EV3 Brick (≈215 g) is mounted exactly in the center of the chassis. 
This balances the weight distribution across all wheels and lowers the center of gravity. 
By placing the heaviest component centrally, the robot is less likely to tip over during sudden pushes or collisions. 
The battery pack is also integrated near the bottom, keeping mass as low as possible. 
Together, this ensures both longitudinal and lateral stability in the ring.



## Strategy and Control
The control program is structured as a finite state machine:
- **Search:** Rotate in place scanning for opponents using distance sensors.  
- **Lock-on:** When an opponent is detected, align direction using sensor feedback.  
- **Approach:** Drive forward at controlled speed.  
- **Push:** When bump sensors confirm contact, apply maximum motor power.  
- **Edge Check:** At all times, color sensors monitor the ring’s edge to prevent accidental exits.  
- **Recovery:** If edge is detected, back up and rotate to re-engage.

This layered strategy ensures the robot is both aggressive and safe, maximizing winning probability.



## Bill of Materials (BOM)
1. EV3 Intelligent Brick – 1× (≈215 g)
2. EV3 Large Motors – 2× (stall torque ~40 N·cm, 160 rpm)
3. Rear drive wheels – 2× (70 mm diameter, rubber high grip)
4. Front support wheels – 2× (30 mm diameter, silicone)
5. EV3 Color Sensors – 2×
6. EV3 Ultrasonic Sensors – 2× (front & rear)
7. LEGO Technic chassis beams and plates – custom assembly
8. EV3 Rechargeable Battery Pack – 1×
9. Screws, connectors, reinforcement parts – set



## Conclusion
This EV3 Sumo robot design combines mechanical strength, intelligent sensor use, and strategic control. 
The large rear wheels provide pushing power, the small front wheels enhance wedge function, 
the sensor suite enables precise reactions, and the central EV3 Brick placement ensures stability. 
The overall structure is fully aligned with WRO requirements and aims to achieve maximum scoring in all rubric categories.
