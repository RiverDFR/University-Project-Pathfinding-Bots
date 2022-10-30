# University-Project-Pathfinding-Bots

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


***- IMPORTANT - Please note this is my personal project during university, you have my permission to use this work to aid in your own personal/academic projects, but by using my project you agree any consequences from plagerism or academic malpractice is your own responsibily and I will not be resposible for said consequences. Thank you for your understanding.***


------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Robocones: Robotic Traffic Cone

Third Year Individual Project

May 2022

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Contents

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. Introduction 

1.1. Introduction

1.2. Motivation

1.3. Scope

1.4. Aims

1.5. Objectives

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2. Literature Review

2.1. Assumptions

2.2. Obstacle Detection Using an Ultrasonic Sensor - HC-SR04

2.3. MONA Platform

2.4. Wireless Communication

2.5. Pathfinding Algorithm 

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3. Robocone Development and Testing (Implementation) 

3.1. Wiring Diagrams and Parts List

3.2. A* Pathfinding for the Arduino 

3.3. Wireless Communication using the Zigbee Wireless Network Protocol 

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.1. Configuration within the XCTU Software

3.3.2. XBee Transparent Mode

3.3.3. XBee API Mode

3.3.4. Integration within the Project

3.4. Motor Control

3.4.1. The reliability of the MONA Educational Robot traveling in a straight line in Closed and Open Loop design 

3.4.2. The reliability of the MONA Educational Robot rotating from stand still using closedloop motor control

3.5. Obstacle Detection

3.5.1. The reliability of measuring distance using HC-SR04

3.5.2. Ultrasonic Implementation

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4. Robocone Performance Review

4.1. No Obstacle Environment

4.2. Known Environments

4.2.1. Example A

4.2.2. Example B

4.3. Unknown Environments

4.3.1. Example C

4.3.2. Example D

4.4. Discussion

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5. Analysis and Conclusions

5.1. Analysis

5.2. Personal Limitations

5.3. Project Limitations

5.4. Possible Improvements

5.5. Summary

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

6. References

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

List of Figures

Figure 1 – Ultrasonic sensor sending and receiving a signal. [15]

Figure 2 – MONA Education Robot [16]

Figure 3 – Zigbee Star and Mesh Network [17]

Figure 4 – Graph Theory Nodes and Links [18]

Figure 5 – Simple example of A* Pathfinding at work [19]

Figure 6 – Front, side, top and isometric view of the Robocone

Figure 7 – Overall System View

Figure 8 – Flowchart of how the Appendix C Code/A* Pathfinding works

Figure 9 – Test Area

Figure 10 – Wire Diagram for the Robocone

Figure 11 - Wire Diagram of the Coordinator - PC

Figure 12 – Arduino Serial Monitor of the A* Pathfinding

Figure 13 – Occupancy Map

Figure 14 – Occupancy

Figure 15 – Xbee to USB Adapter

Figure 16 – Configuring Coordinator and Router in the XCTU Software Suite

Figure 17 - Communication between the Coordinator and Router through the XCTU Console

Figure 18 – XBee API Reference Guide [20]

Figure 19 – Measurement of distance using the MONA Bot [21] 

Figure 20 – Measurement of Rotation of the MONA [21]

Figure 21 – Mean %Error of Open and Closed Loop [21]

Figure 22 – Mean of Angular Error of Open and Closed Loop [21] 

Figure 23 – Angular Error of rotation using the MONA [21]

Figure 24 - %Error of Rotation using the MONA [21]

Figure 25 – Travel Test [21]

Figure 26 – Ultrasonic Test of different shapes and materials

Figure 27 - Ultrasonic Sensor Test on Different Material and Shapes

Figure 28 – No Obstacle Environment Test Area

Figure 29 – Known environment example A + Final Occupancy Map 

Figure 30 – Known environment example B + Final Occupancy Map 

Figure 31 – Unknown environment example C + Final Occupancy Map

Figure 32 – Unknown environment example D + Final Occupancy Map

Figure 33 - Arduino MEGA2560 [22]

Figure 34 – Rasberry Pi 4 8GB [23]

Figure 35 – TurtleBot3 [24]

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

List of Figures

Table 1 - Open-Loop

Table 2 - Closed-Loop

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. Introduction

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1.1. Introduction

“Robots are not going to replace humans; they are going to make their jobs much more humane. Difficult, demeaning, demanding, dangerous, dull – these are the jobs robots will be taking.” [1] - Sabine Hauert, Co-founder of Robohub.org

In our evolving and modern world, the use of robots has become more common to replace jobs that are seen as repetitive and dangerous for humans. The automotive industry was the first to adopt robot in the 1960s using single-minded robots to do repetitive tasks on the assembly line. As robotics and communication technology has advanced, robots are now able to communicate with each other while performing tasks, creating a ‘Swarm intelligence’. These swarming robots can be simple and cheaply produced and can perform tasks more efficiently than a single robot with more advance and expensive components.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1.2. Motivation

The need to control the movement of traffic in the event of road works or accidents is of great importance, not just for safety of road users but for the safety of road workers. The occupation of working on road is consider quite dangerous, especially on motor ways where vehicles could be moving up to 70mph (112.65 kmh). “The US’s bureau of labor statistics states that from 2003-2017, 1844 workers lost their lives, roughly 60% of those deaths are due to a worker being struck by a vehicle while on a work site.” [2]

The purpose of the Robocones swarm is to automate the initial phase of the road worksite, considered the most dangerous phase, placing traffic cones. If this phase is automated, it will lower the risk significantly of any potential impact (vehicular or debris) from injuring workers. Not only will the Robocones swarm lower the risk to workers, but they will also offload the repetitive and labour-intensive task of placing cones on site, allowing for other works to be prioritized to other task and potentially save a road maintenance company labour cost, therefore saving revenue in the long run.

The Robocones will also be able to work as a group to find their respective ‘best’ path by sharing data such as obstacles to reach their respective goal. The Robocone swarm would also be scalable,varying on the size of the worksite and how many worksites there are. 10 Robocones operating on one site would be the same as 100 Robocones working on another allowing for huge time and monetary savings on manpower. Due to the repetitive and low relative cost of construction of these robots, scaling up production using economy of scale techniques would be easily achievable.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1.3. Scope

Due to time constraints the main scope for this project will be specialised in pathfinding and if possible, swarming to provide data for multiple MONAs to operate effectively. The MONAs will be able to provide the best path for its environment and have a “swarm intelligence” by sharing data between them like their position relative to each other to a reference point (the deploymentarea/start zone). The Robocones will be able to share potential obstacles (debris/cones) and be able to change their path to navigate around these obstacles to a central microcontroller, a brain of the network.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1.4. Aims

The aim of this project is to develop a pathfinding algorithm that will work in tandem with a swarming algorithm for the MONA module robots to produce a working Robocones concept, in the effort to reduce lives loss and creating a safer work environment for road maintenance crew and emergency responders. This algorithm will allow multiple MONAs to communicate with each other, such as their position relative to each other and their respective deployment area. The algorithm would also allow for environment updates with the use of auxiliary sensors to detect unexpected obstacles such as debris from accidents and traffic cones placed by other MONAs and then share this information between the MONAs to form a type of “swarm intelligence”. The MONAs would be able to perform these tasks individually, but while in formation with the aid of “swarm intelligence” the MONAs collectively would gain a greater understanding of their environment and be able to reform their ‘best’ path to work more efficiently due to the extra information shared between them.

“The field of swarm robotics concerns the coordination of multi-robot systems composed by a large number of robots, where the collective behaviour emerges from simple local interaction among teammates and between the robots and the environment.” [3]

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1.5. Objectives

The initial objectives will be to review and test potential components/devices to provide the most effective system for multiple MONA robots to be able to perform effective swarming behaviour. The middle stage will be developing/constructing the software and hardware required for the MONAs to operate. The final stages will be debugging and proof of concept.

• To test ultrasound sensors for obstacle detection to see what their effective range is and to test to see if there is any interference created when the MONAs are in formation. 

• Design and implement a sensor board schematic and then construct an auxiliary platform onto the MONAs for the sensor board. This is because MONAs lack these sensors.

• To develop code for an obstacle sensing algorithm for the MONAs to avoid collision and for swarming.

• To create a user interface displaying how the MONAs are operating while within the worksite

• Reviewing a suitable swarm communication device for data to be shared between the MONAs wirelessly.

• To review pathfinding algorithm and then implement effective swarm formation control into the MONAs by sharing data between them to improve on their pathfinding

• Testing the MONAs in a controlled environment with obstacles randomly placed and random deployment areas. The user will define where the cones need to be placed in this controlled environment and this will display if the Robocones is effective

• To then present any shortfalls of the project and possible improvement to the design of the Robocone

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2. Literature Review

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.1. Assumptions

Within the scope of the project, there are assumptions already made and therefore will not need to be included in the method. One of these assumptions is that the “worksite” will be able to bepre-mapped and known, therefore the Robocones would already know where obstacles will be relative to the start and end position. However, the final design will also be capable of dealing with an unknown environment such as an accident and therefore potential unknown debris is littered on the road. This aspect will be discussed further in the sensors section.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.2. Obstacle Detection Using an Ultrasonic Sensor - HC-SR04

On the roads, there is the potential of debris and obstacles that the Robocones may need to navigate around. As such within the project, the HC-SR04 ultrasonic sensor will be used. The reason for using the HC-SR04 is due to its cost-efficiency for the purpose of its objective, which is to perceive obstacles that are within 2cm – 400cm according to the datasheet [8]. although reallab testing on various object shape shows the sensor is reliable to 5cm – 80cm (See chapter 3.5.1),any distance higher than this shows a drop off in accuracy. This affordability makes it easily replaced if a part is broken, but also allows the Robocones cost to be reduced. 

<p align="center"> <img src="Images/Figures/Figure 1 – Ultrasonic sensor sending and receiving a signal.JPG"> </p>

The HC-SR04 works by having two ultrasound sensors, this is known as a bistatic transducer, one ultrasound sensor acts as a ‘transmitter’, and the other acts as a ‘receiver’. The ultrasound functions by transmitting a beam-like acoustic wave from the ‘transmitter’ side of the device, which then travels at the speed of sound until it hit a solid surface. This signal will then bounce back towards the HC-SR04 and be received by the ‘receiver’. The data that is transmitted to the microcontroller will be the time difference between the sending and receiving of a signal. With the knowledge that the speed of sound in air is 343 ms-1 @ 20 oC, we can use an algorithm on a microcontroller to calculate the distance by using…

<p align="center">  <img src="Images/Equations/Equation 1.JPG"> </p>

The reason for the division of 2 is that the signal must travel back and forth.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.3. MONA Platform

<p align="center"> <img src="Images/Figures/Figure 2 – MONA Education Robot.JPG"> </p>

“The MONA robotic platform is an open-hardware/open-source robotic platform that has been developed to be used in swarm robotics research, to understand research topics such as perpetual swarms, multi-agent interaction and human-interaction/control with a swarm of robotic platforms.” [4] This robot provides a great foundation to test out swarming behaviours, due to its high availability at the university and the fact that the MONA uses the open-source Arduino architecture as a basis for programming, it was chosen as the best platform to use for the project. The MONA also provides a suite of onboard sensors that can be used for positioning/obstacle avoidance purposes, such as its IF sensor surrounding the front of the outer body however the range is limited to about 5cm max and can be unreliable at detecting dark coloured objects. The other sensor is hall-effect encoders, which could be used for positing relative to the start point by measuring the number of rotations done by the wheel, this is known as dead-reckoning. An equation can be used within the algorithm to find out the distance travelled by the MONA… 

<p align="center">  <img src="Images/Equations/Equation 2.JPG"> </p>


The reliability of using this system for localization in short distances is reliable, however, as the MONA travels further this system tends to become inaccurate. A way to solve this problem is by restarting the count of N when the MONA returns to the start position, which may be needed for the router MONA

Other systems could be used but they would have to be added and implemented externally, such as internal measurement units (IMU) and global position systems (GPS). Both components are considerably expensive for the purpose of the project. GPS would be unreliable as civilian GPS is only accurate to about 2-5 meters which could cause the Robocones to move into oncoming traffic and the use of military GPS is unlikely. IMU could also be used as it uses acceleration and time to measure distance.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.4. Wireless Communication

There are two main forms of wireless communication that will be the focus of this section. First will be Bluetooth network and second will be Zigbee Protocol. 

Bluetooth is a low-cost device (between £10 - £20 depending on the device type) [13] and requires a moderate amount of power to operate when compared to a XBee device. The Bluetooth device I’ll be using to compare is the HC-05, datasheet link here [14]. It’s unable to go into sleep mode to conserve energy like a XBee device, but it can output a high data rate, about 1Mbits/s. Latency is between 3-10s and the number of nodes connected to a master Bluetooth modules can only be up to 7, which for the purpose this project is not ideal, especially as scalability will need to be a factor when potentially moving forward within the project. 

Zigbee is an affordable device (between £16 – £27 depending on device type) [12] and low-powered wireless communication protocol that allows for mesh networks which is advantageous for swarm communication. This protocol functions with a hierarchy of 3 device classes. These classes are…

Zigbee Coordinator (ZC): The coordinator is the root of the network tree and must always be on for the network to operate. The network only requires one coordinator, and this coordinator will be where all the information of the networking will be consolidated. Within the project the coordinator will be a laptop with an XBee Pro S2 connected to through a USB connection, however, in the field, there is also the possibility of using a smartphone or tablet that is connected to an XBee device to act as a coordinator.

Zigbee Router (ZR): The Router acts as a middleman connection sharing data collected by itself and 
the end devices to the coordinator.

Zigbee End Device (ZED): The end devices are unable to share data except to their respective router. These devices can be low memory and low powered and so can save on battery life as well as cost. In the project, these could be the actual cones that are put into position, as they wouldn’t require any operation while at rest, this saves battery life and could be made from cheaper materials and batteries. The only operation these devices will do while at rest is to send a confirmation signal to check for further instructions from the router.

<p align="center"> <img src="Images/Figures/Figure 3 – Zigbee Star and Mesh Network.JPG"> </p>

Another interesting aspect of the Zigbee protocol is the ability to create different types of networks, varying on how the XBee are configured in the XCTU software. A mesh formation allows for communication between all XBee within a network. While a star formation allows for a central XBee device to receive all incoming data from all other XBee within the network. It also allows for this central XBee (Coordinator) to then broadcast data back to either all XBee within a network or just specific XBee if using API mode (AT and API mode is explained further in chapter 3.3.2 and 3.3.3 respectively)

Within this project, we’ll be using the XBee Pro S2 due to its availability at the university and its ability to use the Zigbee protocol, compared to Bluetooth, the scalability of the Zigbee is far more favourable for the goals of this project, as well as for the future expansion of increasing the amount of Robocone that can operate at one time. However, as of 2022, this line of XBee has been discontinued and the currently available model is the S3 line. The S3 can use any protocols from the S2 and S1 lines and therefore any code is written in this project would be easy to implement the S3 once they’re configured for Zigbee.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.5. Pathfinding Algorithm

<p align="center"> <img src="Images/Figures/Figure 4 – Graph Theory Nodes and Links.JPG"> </p>

Pathfinding is the ability for an algorithm to autonomously find the ‘best’ path from an origin node to a goal node. These pathfinding algorithms are usually based on graph theory and are created with node connected by links, look at figure (4) for an example. These can also be represented as a 2D grid and like the node and link example Instead of searching every node within a network/grid map to find the goal node, they are capable of finding the most cost-effective method of searching through these network/grid-based map from an origin node to the goal node. 

The foundation to most pathfinding algorithms is Dijkstra’s algorithm, which works by computing the cost of going through a node, and then computes all the neighbouring nodes until the goal node has been reached. For the purpose of this project, this would be an overly costly method towards the RAM of most microcontrollers, as calculating the cost of a node and then exploring all the adjacent node and continuing this process could become an exponential issue. Especially if the grid map is large and the goal node was at the furthest point from the origin node. 

Instead, an easier and RAM safe solution for pathfinding is the use of an A* Pathfinding algorithm. Instead of searching every node and it neighbours, A* adds a heuristic constant h(n) that will direct the searching towards the goal node. The h(n) starts at the goal node and increases in value the further away it is from the goal node. This is further improved with the use of g(n) constant that start from the origin and increases in value the further away from the origin node. By adding the two values this can create an f(n) value which is then place onto a heap with the lowest cost at the top. Any equivalent f(n) nodes are then compared with the cheapest h(n) to find the shortest path to the goal node. [10]

A general cost formula with respect to node (n) can be used to display this function:

<p align="center">  <img src="Images/Equations/Equation 3.JPG"> </p>

<p align="center"> <img src="Images/Figures/Figure 5 – Simple example of A Pathfinding at work.JPG"> </p>


Figure (19) is a simple example of how A* works and how the heuristic guides towards the goal node B. starting from node A the most cost-effective f(n) node is 42, notice how the h(n) value is also the ‘cheapest’. The next step is also the same, the ‘cheapest’ f(n) is 42 and the h(n) is also the ‘cheapest’. Finally, the goal has been reached and can be checked by the value of h(n) = 0. The ‘best’ isnow found as shown by the last frame.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3. Robocone Development and Testing (Implementation)

The Robocone seen in figure [6] is built with the goal of being able to traverse an unknown area, it capable to do this with the following components/software. 

<p align="center"> <img src="Images/Figures/Figure 6 – Front, side, top and isometric view of the Robocone.JPG" width=420> </p>

• A XBee Series 2 for wireless communication to act as a router which is then able to communicate to a coordinator connected to the PC. This allows for data to be received/sent by an operator.

• A* Pathfinding Algorithm. This allows the Robocone to find the optimal path either through a known or unknown environment.

• An ultrasonic sensor (HC-SR04) to detect for potential obstacles that may arise in the path used by the Robocone.

• The MONA Educational Robot. This allows the Robocone the ability to traverse the environment due the wheels of the MONA. The MONA is built with an onboard Arduino Pro mini and encoders which allows for a PI closed-loop controller for a more accurate traversal.

• An Arduino Uno for acting as the central brain of the MONA cone to calculate the optimum path using A* Pathfinding and receive and send data to/from the other components that are used.

<p align="center"> <img src="Images/Figures/Figure 7 - Overall System View.JPG"> </p>

<p align="center"> <img src="Images/Figures/Figure 8 – Flowchart of how the Appendix C Code and A Star Pathfinding works.JPG"> </p>

<p align="center"> <img src="Images/Figures/Figure 9 – Test Area.JPG"> </p>

Figure (8) shows how the code in Appendix C operates, more specifically how the A* pathfinding algorithm work with the obstacle avoidance system. 

Figure (9) is the test area that will be used in testing the fully developed Robocone, each number is a node that the user can input as the goal destination for the Robocone to traverse. The total grids on the test area are 8 by 8, which equal to 64 nodes and each individual grid is 12cm by 12cm. This means the total size of the test area is 96cm by 96cm.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.1. Wiring Diagrams and Parts List

Parts required to build 1 Robocone:

1. Mona Educational Platform Robot x1

2. Arduino UNO x1

3. XBee Series 2 Pro x1

4. Arduino XBee Shield V1.1 x1

5. Ultrasonic Sensor – HC-SR04 x1

6. Servo - FS90R x1 (screws included)

7. Spacers/Standoffs at 30mm x9

8. Spacers/Standoffs at 20mm x4

9. 3mm Screws x18

10. 100mm by 100mm plate to act as a platform x2

11. 30mm by 30mm plate to be a platform for HC-SR04 x1

12. 9V battery with 5.5mm/2.1mm DC Barrel Jack x1 

<p align="center"> <img src="Images/Figures/Figure 10 – Wire Diagram for the Robocone.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Parts required to build 1 Coordinator - PC interface:

1. Arduino UNO x1

2. XBee Series 2 Pro x1

3. Breadboard x1

<p align="center"> <img src="Images/Figures/Figure 11 - Wire Diagram of the Coordinator - PC.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.2. A* Pathfinding for the Arduino

Within the project A* Pathfinding is used for searching the optimal path for the Robocone to traverse an unknown or known environment. The way this is implemented is by creating an 8 by 8 (64 cell) node grid map that is numbered from 0 – 63 to represent the environment and a second 8 by 8 (64 cell) node grid that will be a binary occupancy map. The occupancy grid map will represent obstacles and the optimal path for each map build, if a new obstacle is detected then the map is rebuilt with the new obstacle and the ‘best’ path is recalculated from the current position of the Robocone. This process can be seen in Figure (12).

An issue with using this method is that the size of the grid map is limited on how much ram memory a device has, as many nodes and variables must be stored for the calculations to work. Every h(n) and g(n) and f(n) value must be stored, including the Robocone current virtual position in the grid and which direction the Robocone is facing and lastly the obsticles and path within the grid enviroment must be stored. 

Below is an example of the grid map and binary occupancy map as represented in the serial monitor in the Arduino IDE. (See figure 12)

<p align="center"> <img src="Images/Figures/Figure 12 – Arduino Serial Monitor of the A Star Pathfinding.JPG" width=800> </p>

When an obstacle is detected, the map is rebuilt with the old path removed, essentially restarting the build map function at a new position with the unknown obstacle now being place in itsrespective position where it was detected. 

Obstacles can also be pre-written by the user within the code to represent known obstacles by updating line 277 in the Appendix C with ‘|| PF.Map[i][k].gridNom == X’ (X representing the grid node number that the user would like to place an obstacle) after PF.Map[i][k].gridNom == 7. However, the Robocone can find the best path in a completely unknown environment with obstacles not registered.

It should also be noted that diagonal movement is not possible for simplicity purpose. Instead, leftor right turns are considered a costly move within the algorithm, only being done when required due to obstacles or reaching the edge of the grid map. In this example the Robocone uses A* Pathfinding Algorithm to find an optimal path in an unknown environment with a single obstacle that is not declared to the algorithm. The user has inputted 63 as the goal grid and after calculating the optimal f(n) value path, the algorithm returns the path[i] that the Robocone will use. 

<p align="center"> <img src="Images/Figures/Figure 13 and 14 - Occupancy Maps.JPG" width=800> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3. Wireless Communication using the Zigbee Wireless Network Protocol

For communication to be possible between the user and the Robocones, a form of wireless communication is required to allow the Robocones independent movement without the bot being tethered to a PC. The first steps to understand how the XBee devices work was by creating a simple one-way communication between 2 XBee modules, a sender and receiver. This can be done 
directly through the XCTU software created by Digi, the creators of the Zigbee protocol and XBee 
devices. But can also be done through code, however the XBee device must be configured first in 
the XCTU software to make sure they share the same network (Pan ID).

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.1. Configuration within the XCTU Software

<p align="center"> <img src="Images/Figures/Figure 15 – Xbee to USB Adapter.JPG"> </p>

To be able to configure the XBee, the device must be connected to the PC. There are two ways to do this. First is using a XBee to USB adapter as seen in Figure (15) and simply connect to the PC. The second is to use a microcontroller (for this example, an Arduino is used) with USB capabilities and bypass the chip on board the microcontroller. To do this, the GND and RESET pins on the Arduino must be connect which allows for the bypass, then the TX pin (Pin:1) on the Arduino and DOUT pin (Pin:2) on the XBee must be connected and the RX pin (Pin:0) on the Arduino must be connected DIN pin (Pin:3) on the XBee. The XBee must also be powered during this process by connecting the Arduino 3.3V pin to the XBee VCC pin (Pin:1) and the Arduino GND to the XBee GND (Pin:10). Below is an image of the XCTU software when a XBee has connected successfully.

<p align="center"> <img src="Images/Figures/Figure 16 – Configuring Coordinator and Router in the XCTU Software Suite.JPG"> </p>

As seen in Figure (16) there are highlighted/numbered parameters that are of importance to allow successful communication between two XBee devices. These are…

[1] - Function set: This is the firmware placed onto the XBee device, all devices in the network must be on the same family of firmware to work. In the project, Zigbee Protocol is being used and therefore this is also how the devices are designated as either a Coordinator, Router or End Device. However, if IEEE XBee 802.15.4 is used (an alternative network protocol), then an option called CE would have to enabled to the coordinator and all other devices in the network would have this setting disabled. In the case of Figure (16) a Coordinator and Router is designated using the Zigbee Protocol.

[2] – Pan ID: This is the unique personal network area ID, this is an identifier used by all XBee devices on the network to make sure that they are all connected to the same network. In Figure (16) the Pan ID used is 2185, however, any value can be used as long as all XBee in the network are designated the same value in the Pan ID.

[3] – DH: This is the destination address high, this is the first half of the address to the XBee device that the source XBee would like to communicate too. In figure (16) the router uses the serial high (SH) of the coordinator. In a complex network with multiple routers, designating this as 0 will communicate directly to the coordinator.

[4] – DL: This is the destination address low, this is the last half of the address to the XBee device the source XBee would like to communicate too. In figure (16) the router uses the serial low (SL) of the coordinator. In a complex network with multiple routers, designating this as 0 on the router will communicate directly to the coordinator. Designating this as FFFF on the coordinator will broadcast to all routers in the network.

<p align="center"> <img src="Images/Figures/Figure 17 - Communication between the Coordinator and Router through the XCTU Console.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.2. XBee Transparent Mode

The AT mode allows for point-to-point communication between two XBee, though more than two XBee can be connected to an AT network, the issue is traffic would overwhelm the coordinator causing data to be lost and overlap each other. The coordinator is unable to select a single XBee to communicate with, instead the coordinator would be able to broadcast the same command to all XBee, which in the case of the Robocone would cause two Robocone to arrive at the same grid location. As seen in figure (17) the coordinator sends a simple string of ‘Hello from the Coordinator, which is received by the router. The Router return the message ‘Hello from the Router’ back and this message is received by the coordinator. The code for Appendix A allows for this communication to be interpreted by the serial monitor of the Arduino UNO. So when the user is prompted to enter a grid goal, the coordinator is able to send a message over the Zigbee network to the router in order to update the Robocone goal grid.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.3. XBee API Mode

<p align="center"> <img src="Images/Figures/Figure 18 – XBee API Reference Guide.JPG"> </p>

The API mode is more complex and requires following figure (18) format exactly in order to send and receive data. API mode also allows for changing the properties of the pin on XBee device, acting as a wireless switch to those pins. This also allows for specific communication if the respective XBee DH and DL are inputted in section designated as the destination network address. Due to time constraints and issues with hardware, getting the API mode to fully work was unsuccessful.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.4. Integration within the Project

Due to limitations of time and unforeseen issues with hardware, AT mode was chosen for sending commands to a single Robocone through the Arduino IDE serial monitor. The XBee also allowed for the receiving of data on to the serial monitor that could be displayed for the user, such as it’s current ‘virtual’ position as well as how the A* Pathfinding algorithm is calculating the ‘best’ path.The SoftwareSerial library for Arduino also allows for the XBee to be configured to use specific digital pin on the Arduino for TX and RX communication. In the appendix C code, the TX and RX pins are configured to send and receive on pin 2 and 3 on the Arduino.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.4. Motor Control

The MONA Educational Robot in this project can use an open-loop code structure or a closed-loop code structure design relying on the encoder built in to give feedback as to how far the MONA bot has travelled. The next few pages will illustrate tests done on the MONA bot to see how reliable the bot is while traversing in a straight line and while rotating from a stand still position.

<p align="center"> <img src="Images/Figures/Figure 19 – Measurement of distance using the MONA Bot.JPG"> </p>

The test for the straight line was done using a 1-centimeter grid paper with designated ‘expected’ markings at 5, 10 and 15 cm to test to see how accurate open-loop and closed-loop are. Not only will distance be measured but also the drift when the MONA bot reaches its target location. This is done by marking a line at the centre of the paper and measuring the distance from the centre line to the small LED at the front of the MONA bot. 

<p align="center"> <img src="Images/Figures/Figure 20 – Measurement of Rotation of the MONA.JPG"> </p>

In the rotation test, only the closed loop will be used (see the end of chapter 3.4.1 for the reason why) to see how well the MONA bot rotate at 45, 90 and 180 degrees. This is done to see if these respective values are reflective of their real-world result. The K and I values have been kept the same as in the original code and so adjusting the angle to compensate maybe required.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.4.1. The reliability of the MONA Educational Robot traveling in a straight line in Closed and 
Open Loop design

<p align="center"> <img src="Images/Figures/Figure 21 – Mean %25Error of Open and Closed Loop.JPG"> </p>

<p align="center"> <img src="Images/Figures/Figure 22 – Mean of Angular Error of Open and Closed Loop.JPG"> </p>

<p align="center"> <img src="Images/Tables/Table of Open and Closed loop.JPG"> </p>

In figure (21) and figure (22), both graphs show that the closed-loop is more reliable than the open-loop. The optimal PWM value for the open-loop was found by counting how many rotation per minute were done by the wheel vs the PWM value and plotting these values onto a graph, allowing the gradient (𝛼) and constant (𝛽) to be found. Then find the optimal RPM (Nr) that would achieve 5cm/s using this equation: 

<p align="center"> <img src="Images/Equations/Equation 4.JPG"> </p>

However as seen In figure (21) and figure (22) the closed-loop has less error compared to the open-loop counterpart. The drift is also less in the closed-loop and could be because the PWM in the open-loop is not tuned enough for accurate results. The open-loop could be improved by tuning the PWM but the same can be said for the closed-loop. However, even if both systemswere perfectly tuned, there would still be errors because of slip and mechanical backlash, this isunavoidable unless other more accurate and potentially more expensive wheel bot is used.Therefore, the closed-loop will be used in this project and the distance in the code will not be adjusted, keeping the distance at 12cm, the central distance between of each grid within the test area. The closed-loop will also be used for the next test, the rotation test. 

However as seen In figure (21) and figure (22) the closed-loop has less error compared to the open-loop counterpart. The drift is also less in the closed-loop and could be because the PWM in the open-loop is not tuned enough for accurate results. The open-loop could be improved by tuning the PWM but the same can be said for the closed-loop. However, even if both systemswere perfectly tuned, there would still be errors because of slip and mechanical backlash, this isunavoidable unless other more accurate and potentially more expensive wheel bot is used.Therefore, the closed-loop will be used in this project and the distance in the code will not be adjusted, keeping the distance at 12cm, the central distance between of each grid within the test area. The closed-loop will also be used for the next test, the rotation test.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.4.2. The reliability of the MONA Educational Robot rotating from stand still using closed-loop motor control

<p align="center"> <img src="Images/Figures/Figure 23 – Angular Error of rotation using the MONA.JPG"> </p>

<p align="center"> <img src="Images/Figures/Figure 24 - %25Error of Rotation using the MONA.JPG"> </p>

<p align="center"> <img src="Images/Tables/Rotationl Table.JPG"> </p>

In this test, the MONA bot was place on top of a piece of paper with a compass printed on the paper, then the MONA wasrotated at an angle of 45, 90 and 180 degrees and was measured with the expected degrees as seen in figure (20). The results in the table above and the corresponding graphs of figure (23) and figure (24) shows that percentage error does decrease with larger value. However, these are an unacceptable %error for the Robocone to be able to traverse within the test area. To resolve this issue, different degrees were tested while the MONA bot traverse and rotate until the MONA bot performs 2 perfect laps as seen in figure (25) the final ‘best’ degrees found for the closed-loop code was 97 degrees. This could be improved to 90 degrees, but this would require the K and I to be tuned.

<p align="center"> <img src="Images/Figures/Figure 25 – Travel Test.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.5. Obstacle Detection

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.5.1. The reliability of measuring distance using HC-SR04

Using a paper ruler with an increment of 2 cm, the different objects of varying shapes were placed centre of the ultrasonic component. It was discovered that flat surfaces provide a very reliable measurement of distance while circular objects provide some error at some distances and rightangled-shaped objects were the most unreliable to measure. This can be explained due to how the ultrasonic sensor works. The ‘transmitter’ side transmits an acoustic wave that bounces off the object and is received by the receiver ultrasound. However, if the object provides a surface in which the acoustic wave bounces off in a direction that is not towards the receiver side, then it’s assumed there’s nothing there and an output of max distance is sent to the algorithm.

<p align="center"> <img src="Images/Figures/Figure 26 – Ultrasonic Test of different shapes and materials.JPG"> </p>

Another interesting find during the experiment was regardless of the surface used, was that the further the object was to the ultrasonic components there was drop-off in accuracy compared to the real-world measured distance. This could be because of hardware deficiency/voltage fluctuation due to low tolerance components used, but also could be that temperature within the room was not considered and so the speed of sound was not calibrated to the room’s temperature that the experiment was performed in. I provided 343 ms-1 @ 20 oC but the room was considerably colder and could cause a drift inaccuracy. For e.g. the speed of sound is 331.3 ms-1 @ 12 oC.

<p align="center"> <img src="Images/Figures/Figure 27 - Ultrasonic Sensor Test on Different Material and Shapes.JPG"> </p>

In figure (27) an ideal scenario for all surfaces and shapes would be a perfect gradient where expected and measured distances are equal. However, as seen in the graph above, the flat sidedand rounded surfaces produce a reliable gradient for the ultrasonic sensor. Although after 80 cm, the rounded measured surface start to plateau. The corner (45 degree surface) is highly irregular, due to the ‘transmitter’ signal being bounced away from the direction of the ‘receiver’ side of the ultrasonic sensor, causing gaps where the ‘receiver’ side reported abnormally large distances, meaning the signal never returned to the ‘receiver’. Though some signals do return for the corner shape, these are still higher than the expected and could also be due to potentially picking up background noise. The lab where this test was done had other within the room also using ultrasonic sensor and more than likely picked up other ultrasonic sensor ‘transmitter’ signal.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.5.2. Ultrasonic Implementation

This test allowed for the code to be tested for reliability to then be implemented into the larger code of Appendix C. As well as helping with selection of objects to be used within the test area shown in figure (26) to prove the concept of the Robocone.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4. Robocone Performance Review

With the development and testing phase complete, the Robocone system is now ready for deployment in the grid-based testing environment. The Robocone can now traverse an unknown and known environments. The A* Pathfinding code in Appendix C will send commands (‘orders’) to the MONA motor control code in Appendix D to then facilitate movement in the MONA’s wheels. 

Three Robocone were produced, however unforeseen hardware issues dropped that to two Robocone and time issue only allowed for XBee communication to function between only one Robocone to the PC. Meaning the other Robocone would not be sending data to the coordinator and would act independently if used. However, the overall system on one Robocone will be demonstrated below to show that a Robocone can traverse it’s environment successfully even if the environment is known or unknown.

It should be noted that only the final occupancy map frame will be shown. In the case of the unknown environments, every time an obstacle is detected, the old map is cleared and rebuilt with the current position of the Robocone and the new obstacles is place in it respective grid that the obstacle was detected, all former obstacles are saved for any new map build.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.1. No Obstacle Environment

In this test, no obstacles were placed, and no obstacles was preplaced in the code. The Robocone initially started at 0 (Red) and the goal grid is 63 (green). The Robocone travel down the grid map towards grid number 56 and then turn towards the goal grid and travels until the current position = the goal grid. This is to prove that the Robocone can travel the simplest grid map possible. An issue that was prevalent in this simple test is that there is no localisation, the Robocone tends to drift slightly, to the point where at that when the Robocone reached the end goal, half of its chassis was off the edge of map while the other half was on the goal grid. 

<p align="center"> <img src="Images/Figures/Figure 28 – No Obstacle Environment Test Area.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.2. Known Environments

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.2.1. Example A

In the known environments test, the Arduino UNO code inAppendix C on line 277 is populated with obstacles to inform the Robocone of these obstacles within the environment. The occupancy map on the right of figure (29) shows the obstacles are present, there is also a continuous line from start to finish that can be seen as no new obstacles were detected, therefore no rebuild of the map was required during the Robocones travels from the origin grid to the goal grid. 

Though the Robocone was successful at traversing the environment, issues did arise such as drifting slightly off track, rotation not being exactly 90 degrees causing more drifting, as well asslight contact due to the chassis being slightly larger in certain circumstances such as rotating.

<p align="center"> <img src="Images/Figures/Figure 29 – Known environment example A + Final Occupancy Map.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.2.2. Example B

In this example, the Roboconeinitially start at grid 27 and is surrounded by obstacles. The goal grid is 30 and the obstacles are pre-written into the Appendix C code at line 277. The Robocone is able to traverse to the goal grid with success but did have an issue with the obstacle at grid 18 as the edge body of the Robocone did get stuck while the Robocone was trying to move from grid 9 to grid 10 and so had to be recentred to grid 10. Drifting was also observed due to the lack of localisation. A point to take note of is that the Robocone does take a path that take it further from the goal grid at grid 17 to 16 and then backtracks to grid 17. This is a consequence of how A* pathfinding operates, as the algorithm would have calculated the node f(n) cost of grid 16 and 9 are equal, but then the ‘cheaper’ h(n) cost would be for grid 16, meaning the ‘best’ path is to travel to grid 16, hence why it’s the ‘best’ path and not the optimal path.

<p align="center"> <img src="Images/Figures/Figure 30 – Known environment example B + Final Occupancy Map.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.3. Unknown Environments

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.3.1. Example C

In the unknown environments test, no obstacles were pre-written and therefore initially the occupancy map empty of obstacles with only the path being shown. The initial path was similar to that of figure (31) occupancy map, but as obstacles were detected the map was rebuilt with the detected obstacle placed in the updated map and the new ‘best’ path is calculated. The coloured lines represent each time the map is rebuilt due to a new obstacle being detected, as can be seen in the occupancy map, the coloured squares correspond to which coloured path detected that specific obstacle.

When comparing example A and example C occupancy grid map, two obstacles are not present in example C occupancy grid map, these obstacles are on grid 20 and grid 34. There are two reasonsfor this, first is that these obstacles are not within any of the ‘best’ paths when the Robocone is traversing. The second is that ultrasonic sensor is also not active while the Robocone is rotating or traversing and so the obstacles are not detected.

Another comparison between the known and unknown version of this grid-based environment is that the unknown version takes significantly longer move through the environment, as the Robocone must recalculate every time an obstacle is detected. While in the known environment the Robocone only needs to focus on movement as no new obstacles are present.

<p align="center"> <img src="Images/Figures/Figure 31 – Unknown environment example C + Final Occupancy Map.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.3.2. Example D

In this final example, the Robocone will traverse the same environment as example B but with no pre-written obstacles. The Robocone’s initial path is to travel from the origin grid 27 and travel to 28, 29 and the goal grid 30. However, the Roboconedetects and obstacle on grid 28 and recalculates the path to go around this obstacle only to be stopped again by more obstacles. Similar to example C some obstacles were not detected due to these obstacles not being within the path of the Robocone, these obstacles are on grid 44, 40 and 24. Furthermore the backtracking observed in example B is not present in this version as the f(n) cost for grid 9 is cheaper when recalculating at position 17 to reach the goal at grid 30. However, the overall time it took for the Robocone to traverse was significantly longer than the original due to the recalculating take roughly 10-20 seconds for each obstacle detected, taking roughly 3 minutes longer than example B.

<p align="center"> <img src="Images/Figures/Figure 32 – Unknown environment example D + Final Occupancy Map.JPG"> </p>

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

4.4. Discussion

The above examples have shown that the Robocone is capable of traversing unknown and known environments, as well as being able to adapt and create new ‘best’ path when traversing, showing that the Robocone can handle dynamic environment. This means that the Robocone has successfully reached the goals noted in chapter 1. However, certain limitation has presented themselves during this phase of testing, such as the lack of localisation and therefore the ability to recentre the Robocone to the respective grid that the Robocone assumes it should be in. Finer tuning of the closed-loop code could improve this but would not be a viable long-term solution. Another issue is how dependant the A* Pathfinding is with ram memory, originally it was intended for a 10 by 10 grid to be used but with initial tests showing memory instability issues due to low memory availability, the grid was reduced 8 by 8 as well optimisation techniques were used, such as storing any stings into flash memory to called when needed. However, It can’t be said that the Robocone is the most ideal solution for traversing known or unknown environments, as shown in example B when the Robocone backtracks, as well as the long wait time between recalculation, but it does show the overall system is independent and capable of traversing these environment.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5. Analysis and Conclusions

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5.1. Analysis 

Overall, the Robocone system has shown to be capable at navigating through unknown and known environment, detecting obstacle, and creating an interface between a PC and the Roboconesystem allow the user to visualise how the Robocone is operating as well as sending commands to the Robocone. However certain issues have made it obvious that the system can be improved if more time was available, such as localisation, communication between the Robocone to create a swarm intelligence and allow multiple Robocone to operate at the same time, as well as potential upgrades to the system to make it more reliable i.e., better wheel encoders for the motor controller.

Certain aspects of the project were not completed during the academic year such as the mentioned swarming, as there was difficulty in getting the XBee modules to work for a number of weeks, even basic AT communication was difficult to get working until mid easter. As well as difficulty getting the A* Pathfinding to function due to memory limitations, as larger grid dimensions would cause memory instability and cause the Robocone to reach a memory ram overload halfway through pathing in its environments.

However, after optimising the code by reducing the original 10 by 10 grid size to 8 by 8 and storing any strings into flash memory of the Arduino UNO. The A*pathfinding showed great promise with the result presented in chapter 4. As for the real-world applications, the Robocone is not viable for reason mentioned before, but also due to the size of the Robocone as well and the reliability of the motor controller. Comparing the test area with a real road is incomparable, as the test area is almost perfectly flat, while a road will have bump and dips that will cause the drift seen in testing to be even more significant.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5.2. Personal Limitations

A huge barrier to the unsuccessful completion of the swarming aspect was largely a result of time constraint, as much of the work had focused on getting the A* Pathfinding to work as well as the XBee to able to communicate. In regards of the XBee, I was able to receive them by luck as some departments of the University was relocating at the time and so I was given them, as their fate would have been to the recycling centre. However, this became a more of a time sink as some of the components were either not working or their online resources had been taken down. For e.g.,one USB to XBee adapter was pre-solider in the wrong orientation, which wasted a few weeks until the issue was noticed. Another USB to XBee adapter had an extra set of pins which wasn’t noticed for a few weeks as well… luckily the XBee device used in this adapter was not damaged.

Lastly, some software required more time to understand and so took longer than was originally expected such as the XCTU software suite, as getting two XBee devices to communicate took longer than expected.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5.3. Project Limitations

As stated in chapter 4, there are certain issue that arose that due to lack of time was not fixed. One of these limitations was a lack of localisation, which is the ability for the Robocone to location itself in the environment. The Robocone after traveling multiple grids would drift either due to traveling further than intended or rotated more or less than intended. This would certainly be because the motor controller was not finely tuned, but also because the encoders on the MONA educational bot are not ideal for extremely precise movements as the wheels would constantly display symptoms of slip. In some cases, while the Robocone was not moving or rotating, the Robocone would jerk/sharply rotate about 3-7 degrees as the encoder realigned.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5.4. Possible Improvements

Due to the A* Pathfinding reliance on large amounts ofmemory of the SRAM to store nodes of the map and variables, the limited memory on the Arduino UNO (2kB of SRAM) is a huge barrier in scalability. Improvement to this would allow for larger grid map and therefore larger work environment. This could be achieved by using a microcontroller like an upgraded version of the Arduino UNO, such as the Arduino MEGA2560 (8kB of SRAM). [5]However, eventually this upgraded version will also be limited by its size. Instead, a recommendation of using a Raspberry Pi 4 would be ideal for the Robocone as the SRAM can be upgraded varying on the model chosen, from 1GB, 2GB, 4GB up to 8GB. This would allow for grid row and column sizes of up to the hundreds, potentially thousands.

<p align="center"> <img src="Images/Figures/Figure 33 - Arduino MEGA2560.JPG"> </p>

<p align="center"> <img src="Images/Figures/Figure 34 – Rasberry Pi 4 8GB.JPG"> </p>

Another potential Improvement would be to use a smarter robot pre-built like the turtle bot 3 waffle, which was the inspiration for the Robocones design in this project but in the hopes to make it cheaper. The turtle bot 3 contains a built-in Raspberry Pi 4, a 360-degreeLIDAR and has SLAM built-in as part of its navigation to create 2-D representation of the world in the Turtlebot software suite. The use of SLjgiuyAM would be ideal as it would solve the localisation issue as well as map the local area at the same time, as the robot would be scanning the local environment at high speed with the built-in LIDAR. The Turtlebot is easily customisable with 3D-printed CAD files available online. Allowing for the possibility of using current traffic cones to be placed on top with some customisable 3D printed part. The device can communicate with a PC, joypad or even an android device with the respective app. [6] However there are two barriers for this choice, first is the cost, being roughly £1447.10 [7] for 1 unit and second is the fragility of this robot. The turtle bot 3 is not intended for outdoor use, but it would be perfect for progression of this project.

<p align="center"> <img src="Images/Figures/Figure 35 – TurtleBot3.JPG"> </p>

Lastly another improvement could be to use a more efficient pathfinding algorithm, such as D* or even AD*. While using A* Pathfinding is more than suitable for this project, the recalculating becomes a time sink and extends the operation of reaching the goal node by a factor of 2 – 4 times varying on how many obstacles are detected within a path. In chapter 4, example D when compared to example B was took 3 minutes longer to reach it’s end goal. D* works by using a second heuristic which allows for quicker recalculation, while AD* uses an inflation factor that will get a result even quicker to an almost real-time state but may produce a less than desirable path. [9]. But these two systems are both plagued with the same issue as A* has on the Arduino UNO and would both require an upgrade of the RAM to be effective.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

5.5. Summary

The project report attempts to show the progression of how and why building a robot for road work is an asset to the traffic accident and road works industry. The report starts by discussing the fundamental concept that will achieve the goals set in the introduction. While the latter half of the report shows how effective those principles are when used in a test area environment. The overall report attempts to act as a guide to allow others to build/improve on the design and avoid potential pitfalls learned through the process. While there was success in some department such as the A* Pathfinding and the designing of the Robocone, as well as the successful testing in the small environments that the Robocone did navigate. There were however some limitations of time, which showed flaws that future work could improve upon, such as implementing communication between multiple Robocone to allow a swarm intelligence such as multiple Robocone building a map together. In the end, the overall project is not enough to say that the Robocone built in this project is ideal for the purpose of road use but could be if future work is built on top of the work done in this project.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

6. References

[1] Balaganur, S. (2020). Ten Famous Quotes About Artificial Intelligence. [online] Analytics India Magazine. Available at: https://analyticsindiamag.com/ten-famous-quotes-about-artificial-intelligence/. [Accessed 26 Feb. 2022].

[2] Anon, (2019). CDC - Highway Work Zone Safety - NIOSH Workplace Safety and Health Topic. [online] Available at: https://www.cdc.gov/niosh/topics/highwayworkzones/default.html[Accessed 26 Feb. 2022].

[3] Dejan Milutinović and Lima, P. (2012). Chapter 6 - biological cell inspired stochastic models and control. In: M. Kim, A. Agung Julius and E. Steager, eds. [online] William Andrew Publishing, pp.145–161. [online] Available at: https://www.sciencedirect.com/science/article/pii/B9781455778911000062. [Accessed 26 Feb. 2022].

[4] uomrobotics.com. (n.d.). MONA. [online] Available at: https://uomrobotics.com/robots/mona.html [Accessed 26 Feb. 2022].

[5] www.arduino.cc. (n.d.). Arduino - Memory. [online] Available at: https://www.arduino.cc/en/pmwiki.php?n=Tutorial/Memory [Accessed 3 May 2022].

[6] IEEE Spectrum. (2017). Hands-on With TurtleBot 3, a Powerful Little Robot for Learning ROS. [online] Available at: https://spectrum.ieee.org/review-robotis-turtlebot-3 [Accessed 2 May 2022].

[7] robosavvy.co.uk. (n.d.). TurtleBot3 Waffle Pi. [online] Available at: https://robosavvy.co.uk/turtlebot3-waffle-pi.html [Accessed 3 May 2022].

[8] ElecFreaks (2011). Ultrasonic Ranging Module HC -SR04. [online] Available at: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf. [Accessed 3 May 2022].

[9] Murillo, M., Sánchez, G., Genzelis, L. and Giovanini, L. (2017). A Real-Time Path-Planning Algorithm based on Receding Horizon Techniques. Journal of Intelligent & Robotic Systems, 91(3-4), pp.445–457. doi:10.1007/s10846-017-0740-1.

[10] Allaire, F.C.J., Tarbouchi, M., Labonté, G. and Fusina, G. (2008). FPGA Implementation of Genetic Algorithm for UAV Real-Time Path Planning. Unmanned Aircraft Systems, 54(1), pp.495–510. doi:10.1007/978-1-4020-9137-7_26.

[11] brilliant.org. (n.d.). A* Search | Brilliant Math & Science Wiki. [online] Available at: https://brilliant.org/wiki/a-star-search/#:~:text=The%20A%2A%20Algorithm%201%20f%20%28%20n%29%20f [Accessed 4May 2022].

[12] www.sparkfun.com. (n.d.). Search Results for XBee - SparkFun Electronics. [online] Available at: https://www.sparkfun.com/search/results?term=XBee [Accessed 4 May 2022].

[13] www.sparkfun.com. (n.d.). Search Results for Bluetooth - SparkFun Electronics. [online] Available at: https://www.sparkfun.com/search/results?term=Bluetooth [Accessed 4 May 2022].

[14] etechnophiles.com. (2020). HC-05 pinout, specifications, datasheet and HC05 Arduino connection. [online] Available at: https://www.etechnophiles.com/hc-05-pinout-specifications-datasheet/ [Accessed 5 May 2022].

[15] Santos, R. (2019). Complete Guide for Ultrasonic Sensor HC - SR04. [online] Random Nerd Tutorials. Available at: https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/. [Accessed 5 May 2022].

[16] Liu, Z., West, C., Lennox, B. and Arvin, F. (2020). Local bearing estimation for a swarm of low-cost miniature robots. Sensors (Basel, Switzerland), 20. doi:10.3390/s20113308.

[17] Sparkfun.com. (2013). Connectivity of the Internet of Things - learn.sparkfun.com. [online] Available at: https://learn.sparkfun.com/tutorials/connectivity-of-the-internet-of-things/all.[Accessed 5 May 2022].

[18] Chopra, C. (2019). Pathfinding Algorithms. [online] Medium. Available at: https://medium.com/swlh/pathfinding-algorithms-6c0d4febe8fd. [Accessed 5 May 2022].

[19] Lague, S. (2014). A* Pathfinding (E01: algorithm explanation). [online] www.youtube.com. Available at: https://www.youtube.com/watch?v=-L-WgKMFuhE. [Accessed 5 May 2022].

[20] www.tunnelsup.com. (n.d.). XBee S2 Quick Reference Guide/Cheat Sheet and Video Tutorials to Getting Started - TunnelsUP. [online] Available at: https://www.tunnelsup.com/xbee-guide/ [Accessed 6 May 2022]

[21] Fox-Ratola, R.D. (2022). Coursework 2 - MONA Lab. [online] www.youtube.com. Available at: https://youtu.be/GRLzOxhOqGM. [Accessed 5 May 2022].

[22] Arduino Official Store. (n.d.). Arduino Mega 2560 Rev3. [online] Available at: https://store.arduino.cc/products/arduino-mega-2560-rev3. [Accessed 5 May 2022].

[23] Skroutz. (n.d.). Raspberry Pi 4 Model B 8GB. [online] Available at: https://www.skroutz.gr/s/23794351/Raspberry-Pi-4-Model-B-8GB.html [Accessed 6 May 2022]

[24]Name, Y. (n.d.). ROBOTIS e-Manual. [online] ROBOTIS e-Manual. Available at: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/.[Accessed 6 May 2022].

[25] Erome, J. (2021). A-Path-Finding---Makeblock-Mbot. [online] GitHub. Available at: https://github.com/jeremiahe00/A-star-PathFinding---Makeblock-Mbot---Arduino/blob/master/Path_Finding/Path_Finding.ino [Accessed 4 May 2022].

[26]learn.sparkfun.com. (n.d.). XBee Shield Hookup Guide - learn.sparkfun.com. [online] Available at: https://learn.sparkfun.com/tutorials/xbee-shield-hookup-guide/example-communication-test [Accessed 4 May 2022].

[27]Arvin, F. (2022). Mona_Encoder.ino [online] Manchester.ac.uk. Available at: https://online.manchester.ac.uk/webapps/blackboard/content/listContent.jsp?course_id=_68652_1&content_id=_13558384_1 [Accessed 4 May 2022].

