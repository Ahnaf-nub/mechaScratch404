# wro_fe_2023 Engineering Documentation 
## Team Members:
* A. N. M. Noor - email: nafis.noor.202012@gmail.com
* Mir Muhammad Abidul Haq (Ahnaf) - email: abidulhaqahnaf@gmail.com
## Overview of our Repository
 * chasis - this folder contains the pictures of the chasis that we used on our robot.  
 * models - the 3d printable files used in our robot.
 * schematic - contains the schematic of the electrical system of our robot.
 * src - contains the main code of our robot.
 * experiments - codes that were used to do experiments.
 * video - contains the video link of youtube where our robot can be seened in action.
 * t-photos - contains one serious and one funny photo of our team.
 * v-photos - contains the photos of the robot from all required directions
 * others - other essential photos
# Program infrastructure and explanation of algorithm.
### Qualifying Round
The robot has five ultrasonic distance sensors, each oriented in a different direction: left, front-left, front, front-right, and right. Each sensor pair (left and right) has its own minimum and maximum detection range to ensure optimal performance. Detecting the wall at the maximum distance has no impact on the steering value, whereas detecting it at the minimum distance exerts the maximum influence on the steering value. Sensors are also fine-tuned for optimal performance in terms of their ability to influence steering values. Specific steering values are shifted to the right by the left sensors, while the right sensors are shifted to the left by the right sensors. The left and right sensors cancel each other out when they detect the wall at the same distance. Based on proximity to the wall, the front sensor determines the throttle value. The throttle value is reduced if the wall is detected too close.

The lap count is monitored using a gyroscope sensor (MPU-6050). When there is a change in the robot's angle exceeding ninety degrees compared to the last recorded angle, it indicates that the robot has navigated a corner on the track. To complete three laps, the robot needs to make a total of twelve turns. Consequently, the robot is programmed to halt after a predefined time interval following the completion of its twelfth turn. 

The robot harnesses the processing power of both cores of the ESP32 microcontroller by using FreeRTOS, a real-time operating system. The primary core handles all logical operations and calculations, ensuring the swift execution of tasks. Simultaneously, a separate core is dedicated to acquiring sensor data, allowing for rapid data retrieval. This dual-core configuration enables the robot to perform calculations and make decisions with remarkable speed and efficiency.
### Obstacle Round
Initially, the robot computes steering and throttle values using distance sensors, following the same methodology applied during the qualifying round. Subsequently, these values are adjusted based on the presence of obstacles in front of the robot.

For the detection of red and green towers, the robot relies on the HuskyLens. This device identifies the red and green towers and transmits relevant data to the ESP32, including the position, width, and height of the towers on the HuskyLens' screen. The distance to the target tower is calculated using the on-screen width of the object. The ESP32 then prioritizes these detected towers based on their proximity to the robot. The robot is programmed to position green towers towards the right side of the screen and red towers towards the left side. Two values, each ranging from zero to one, are generated based on the object's screen position and distance. These values are multiplied to yield another value ranging from zero to one. This value is then added to or subtracted from the existing steering value, depending on the color of the target tower.

To prevent collisions, if any tower approaches closer than a predefined distance threshold, the robot halts momentarily and reverses its direction while adjusting its steering left or right based on the color of the target tower. Afterward, it resumes forward movement as usual.

Once again, the ESP32's primary core handles the calculations and decision-making processes, while the secondary core is responsible for collecting data from the distance sensors and the Husky Lens. This configuration enables the robot to react quickly and avoid collisions with walls or obstacles.
## Electrical design of our robot.
In order to achieve the highest possible efficiency and reliability, we have spent several hundred hours researching and developing the parts. The following paragraphs provide detailed information about electrical systems design.
### Parts list
* Esp32 Development Board
* 300 rpm 25GA 12V DC Gear Motor
* MG91 Servo
* Huskylens: Embedded Vision Sensor
* 5xHC-SR04 Ultrasonic Sensors
* TB6612FNG Motor Driver
* MPU6050
* 3.0 USB Type-C Buck Boost module
* XL6009 Buck Boost Module
* 3S LiPo battery input through XT60 Connector
## Mechanical Design
We've made our robot totally from scratch. Most of the parts of our robot are 3D printed, starting from chasis, wheels and so on.
### Design Decisions
* We've designed a sonar mount which is mounted at the front and the side of the robot where each sonar sensors are mounted at an angle of 52.5 deg. Based on our testing, this is the optimal angle for the sonars to detect walls ahead of time, giving the bot enough time to react. 4 sonar sensors were used to detect walls.
* 
* We used Ackerman steering as it is a vital steering geometry concept that enhances the handling and cornering performance of vehicles, making them safer and more efficient on the road. Ackerman steering, also known as Ackerman geometry or Ackerman principle, is a steering mechanism which is used in vehicles in order to enable proper turning of the wheels while maintaining optimal geometry and minimizing tire scrubbing during turns. It is commonly used in most modern vehicles, including cars, trucks, and other wheeled vehicles.
The fundamental idea behind Ackerman steering is that the inner and outer wheels of a vehicle must follow different turning radii during a turn, considering the varying angles of the front wheels. When a vehicle turns, the inner wheel needs to negotiate a tighter radius than the outer wheel to maintain a smooth turn without dragging or scuffing the tires. The Ackerman steering mechanism achieves this by using a steering linkage that connects the wheels. Typically, it consists of two tie rods connected to the steering arms on each wheel. The tie rods are connected to a central steering mechanism, such as a steering rack or a pitman arm, which is controlled by the driver. As the driver turns the steering wheel, the steering mechanism transfers the motion to the tie rods, causing the wheels to rotate accordingly. The geometry of the Ackerman steering mechanism ensures that the inner wheel turns at a sharper angle compared to the outer wheel, allowing both wheels to trace their respective turning radii accurately. By implementing Ackerman steering, vehicles can achieve better maneuverability, stability, and reduced tire wear during turns. It ensures that all wheels maintain proper contact with the road surface and minimizes the likelihood of skidding or slipping during turns.
![image](https://github.com/Ahnaf-nub/Mecha-404/assets/76505613/5aab9af5-65b7-4ce1-a794-1a9a6564b4d6)
