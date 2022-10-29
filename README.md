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

The HC-SR04 works by having two ultrasound sensors, this is known as a bistatic transducer, one ultrasound sensor acts as a ‘transmitter’, and the other acts as a ‘receiver’. The ultrasound functions by transmitting a beam-like acoustic wave from the ‘transmitter’ side of the device, which then travels at the speed of sound until it hit a solid surface. This signal will then bounce back towards the HC-SR04 and be received by the ‘receiver’. The data that is transmitted to the microcontroller will be the time difference between the sending and receiving of a signal. With the knowledge that the speed of sound in air is 343 ms-1 @ 20 oC, we can use an algorithm on a microcontroller to calculate the distance by using…

Distance(m) = ( 𝑇𝑖𝑚𝑒(𝑠) ∗ 𝑆𝑝𝑒𝑒𝑑 𝑜𝑓 𝑠𝑜𝑢𝑛𝑑 (𝑚𝑠−1) ) / 2      𝐸𝑞𝑢𝑎𝑡𝑖𝑜𝑛 (1)

The reason for the division of 2 is that the signal must travel back and forth.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.3. MONA Platform

“The MONA robotic platform is an openhardware/open-source robotic platform that has been developed to be used in swarm robotics research, to understand research topics such as perpetual swarms, multi-agent interaction and humaninteraction/control with a swarm of robotic platforms.” [4] This robot provides a great foundation to test out swarming behaviours, due to its high availability at the university and the fact that the MONA uses the open-source Arduino architecture as a basis for programming, it was chosen as the best platform to use for the project. The MONA also provides a suite of onboard sensors that can be used for positioning/obstacle avoidance purposes, such as its IF sensor surrounding the front of the outer body however the range is limited to about 5cm max and can be unreliable at detecting dark coloured objects. The other sensor is hall-effect encoders, which could be used for positing relative to the start point by measuring the number of rotations done by the wheel, this is known as dead-reckoning. An equation can be used within the algorithm to find out the distance travelled by the MONA… 

Distance travelled (m) = (𝑁 / 𝑁𝑃𝑃𝑅) ∗ 𝜋 ∗ 𝐷 (𝑚)     𝐸𝑞𝑢𝑎𝑡𝑖𝑜𝑛 (2)

N: the total number of pulses since the start of operation.

𝑁𝑃𝑃𝑅: the number of pulses per revolution.

D: the diameter of the wheel 

The reliability of using this system for localization in short distances is reliable, however, as the MONA travels further this system tends to become inaccurate. A way to solve this problem is by restarting the count of N when the MONA returns to the start position, which may be needed for the router MONA

Other systems could be used but they would have to be added and implemented externally, such as internal measurement units (IMU) and global position systems (GPS). Both components are considerably expensive for the purpose of the project. GPS would be unreliable as civilian GPS is only accurate to about 2-5 meters which could cause the Robocones to move into oncoming traffic and the use of military GPS is unlikely. IMU could also be used as it uses acceleration and time to measure distance.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.4. Wireless Communication

There are two main forms of wireless communication that will be the focus of this section. First will be Bluetooth network and second will be Zigbee Protocol. 

Bluetooth is a low-cost device (between £10 - £20 depending on the device type) [13] and requires a moderate amount of power to operate when compared to a XBee device. The Bluetooth device I’ll be using to compare is the HC-05, datasheet link here [14]. It’s unable to go into sleep mode to conserve energy like a XBee device, but it can output a high data rate, about 1Mbits/s. Latency is between 3-10s and the number of nodes connected to a master Bluetooth modules can only be up to 7, which for the purpose this project is not ideal, especially as scalability will need to be a factor when potentially moving forward within the project. 

Zigbee is an affordable device (between £16 – £27 depending on device type) [12] and lowpowered wireless communication protocol that allows for mesh networks which is advantageous for swarm communication. This protocol functions with a hierarchy of 3 device classes. These classes are…

Zigbee Coordinator (ZC): The coordinator is the root of the network tree and must always be on for the network to operate. The network only requires one coordinator, and this coordinator will be where all the information of the networking will be consolidated. Within the project the coordinator will be a laptop with an XBee Pro S2 connected to through a USB connection, however, in the field, there is also the possibility of using a smartphone or tablet that is connected to an XBee device to act as a coordinator.

Zigbee Router (ZR): The Router acts as a middleman connection sharing data collected by itself and 
the end devices to the coordinator.

Zigbee End Device (ZED): The end devices are unable to share data except to their respective router. These devices can be low memory and low powered and so can save on battery life as well as cost. In the project, these could be the actual cones that are put into position, as they wouldn’t require any operation while at rest, this saves battery life and could be made from cheaper materials and batteries. The only operation these devices will do while at rest is to send a confirmation signal to check for further instructions from the router.

Another interesting aspect of the Zigbee protocol is the ability to create different types of networks, varying on how the XBee are configured in the XCTU software. A mesh formation allows for communication between all XBee within a network. While a star formation allows for a central XBee device to receive all incoming data from all other XBee within the network. It also allows for this central XBee (Coordinator) to then broadcast data back to either all XBee within a network or just specific XBee if using API mode (AT and API mode is explained further in chapter 3.3.2 and 3.3.3 respectively)

Within this project, we’ll be using the XBee Pro S2 due to its availability at the university and its ability to use the Zigbee protocol, compared to Bluetooth, the scalability of the Zigbee is far more favourable for the goals of this project, as well as for the future expansion of increasing the amount of Robocone that can operate at one time. However, as of 2022, this line of XBee has been discontinued and the currently available model is the S3 line. The S3 can use any protocols from the S2 and S1 lines and therefore any code is written in this project would be easy to implement the S3 once they’re configured for Zigbee.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

2.5. Pathfinding Algorithm

Pathfinding is the ability for an algorithm to autonomously find the ‘best’ path from an origin node to a goal node. These pathfinding algorithms are usually based on graph theory and are created with node connected by links, look at figure (4) for an example. These can also be represented as a 2D grid and like the node and link example Instead of searching every node within a network/grid map to find the goal node, they are capable of finding the most cost-effective method of searching through these network/grid-based map from an origin node to the goal node. 

The foundation to most pathfinding algorithms is Dijkstra’s algorithm, which works by computing the cost of going through a node, and then computes all the neighbouring nodes until the goal node has been reached. For the purpose of this project, this would be an overly costly method towards the RAM of most microcontrollers, as calculating the cost of a node and then exploring all the adjacent node and continuing this process could become an exponential issue. Especially if the grid map is large and the goal node was at the furthest point from the origin node. 

Instead, an easier and RAM safe solution for pathfinding is the use of an A* Pathfinding algorithm. Instead of searching every node and it neighbours, A* adds a heuristic constant h(n) that will direct the searching towards the goal node. The h(n) starts at the goal node and increases in value the further away it is from the goal node. This is further improved with the use of g(n) constant that start from the origin and increases in value the further away from the origin node. By adding the two values this can create an f(n) value which is then place onto a heap with the lowest cost at the top. Any equivalent f(n) nodes are then compared with the cheapest h(n) to find the shortest path to the goal node. [10]

A general cost formula with respect to node (n) can be used to display this function:

“𝑓(𝑛) = 𝑔(𝑛) + ℎ(𝑛) 𝐸𝑞𝑢𝑎𝑡𝑖𝑜𝑛(3)

Where

• f(n) = total estimated cost of path through node n

• g(n) = cost so far to reach node n

• h(n) = estimated cost from n to goal. This is the heuristic part of the cost function, so it is like a guess.” [11]

Figure (19) is a simple example of how A* works and how the heuristic guides towards the goal node B. starting from node A the most cost-effective f(n) node is 42, notice how the h(n) value is also the ‘cheapest’. The next step is also the same, the ‘cheapest’ f(n) is 42 and the h(n) is also the ‘cheapest’. Finally, the goal has been reached and can be checked by the value of h(n) = 0. The ‘best’ isnow found as shown by the last frame.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3. Robocone Development and Testing (Implementation)

The Robocone seen in figure [6] is built with the goal of being able to traverse an unknown area, it capable to do this with the following components/software. 

• A XBee Series 2 for wireless communication to act as a router which is then able to communicate to a coordinator connected to the PC. This allows for data to be received/sent by an operator.

• A* Pathfinding Algorithm. This allows the Robocone to find the optimal path either through a known or unknown environment.

• An ultrasonic sensor (HC-SR04) to detect for potential obstacles that may arise in the path used by the Robocone.

• The MONA Educational Robot. This allows the Robocone the ability to traverse the environment due the wheels of the MONA. The MONA is built with an onboard Arduino Pro mini and encoders which allows for a PI closed-loop controller for a more accurate traversal.

• An Arduino Uno for acting as the central brain of the MONA cone to calculate the optimum path using A* Pathfinding and receive and send data to/from the other components that are used.

Figure (8) shows how the code in Appendix C operates, more specifically how the A* pathfinding algorithm work with the obstacle avoidance system. 

Figure (9) is the test area that will be used in testing the fully developed Robocone, each number is a node that the user can input as the goal destination for the Robocone to traverse. The total grids on the test area are 8 by 8, which equal to 64 nodes and each individual grid is 12cm by 12cm. This means the total size of the test area is 96cm by 96cm.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.1. Wiring Diagrams and Parts List

Parts required to build 1 Robocone: **(See Figure 10 – Wire Diagram for the Robocone)**

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

Parts required to build 1 Coordinator - PC interface: **(See Figure 11 - Wire Diagram of the Coordinator - PC)**

1. Arduino UNO x1

2. XBee Series 2 Pro x1

3. Breadboard x1

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.2. A* Pathfinding for the Arduino

Within the project A* Pathfinding is used for searching the optimal path for the Robocone to traverse an unknown or known environment. The way this is implemented is by creating an 8 by 8 (64 cell) node grid map that is numbered from 0 – 63 to represent the environment and a second 8 by 8 (64 cell) node grid that will be a binary occupancy map. The occupancy grid map will represent obstacles and the optimal path for each map build, if a new obstacle is detected then the map is rebuilt with the new obstacle and the ‘best’ path is recalculated from the current position of the Robocone. This process can be seen in Figure (12).

An issue with using this method is that the size of the grid map is limited on how much ram memory a device has, as many nodes and variables must be stored for the calculations to work. Every h(n) and g(n) and f(n) value must be stored, including the Robocone current virtual position in the grid and which direction the Robocone is facing and lastly the obsticles and path within the grid enviroment must be stored. 

Below is an example of the grid map and binary occupancy map as represented in the serial monitor in the Arduino IDE. (See figure 12)

When an obstacle is detected, the map is rebuilt with the old path removed, essentially restarting the build map function at a new position with the unknown obstacle now being place in itsrespective position where it was detected. 

Obstacles can also be pre-written by the user within the code to represent known obstacles by updating line 277 in the Appendix C with ‘|| PF.Map[i][k].gridNom == X’ (X representing the grid node number that the user would like to place an obstacle) after PF.Map[i][k].gridNom == 7. However, the Robocone can find the best path in a completely unknown environment with obstacles not registered.

It should also be noted that diagonal movement is not possible for simplicity purpose. Instead, leftor right turns are considered a costly move within the algorithm, only being done when required due to obstacles or reaching the edge of the grid map. In this example the Robocone uses A* Pathfinding Algorithm to find an optimal path in an unknown environment with a single obstacle that is not declared to the algorithm. The user has inputted 63 as the goal grid and after calculating the optimal f(n) value path, the algorithm returns the path[i] that the Robocone will use. **(See Figure 13/14 Occupancy Map)**

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3. Wireless Communication using the Zigbee Wireless Network Protocol

For communication to be possible between the user and the Robocones, a form of wireless communication is required to allow the Robocones independent movement without the bot being tethered to a PC. The first steps to understand how the XBee devices work was by creating a simple one-way communication between 2 XBee modules, a sender and receiver. This can be done 
directly through the XCTU software created by Digi, the creators of the Zigbee protocol and XBee 
devices. But can also be done through code, however the XBee device must be configured first in 
the XCTU software to make sure they share the same network (Pan ID).

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.1. Configuration within the XCTU Software

To be able to configure the XBee, the device must be connected to the PC. There are two ways to do this. First is using a XBee to USB adapter as seen in Figure (15) and simply connect to the PC. The second is to use a microcontroller (for this example, an Arduino is used) with USB capabilities and bypass the chip on board the microcontroller. To do this, the GND and RESET pins on the Arduino must be connect which allows for the bypass, then the TX pin (Pin:1) on the Arduino and DOUT pin (Pin:2) on the XBee must be connected and the RX pin (Pin:0) on the Arduino must be connected DIN pin (Pin:3) on the XBee. The XBee must also be powered during this process by connecting the Arduino 3.3V pin to the XBee VCC pin (Pin:1) and the Arduino GND to the XBee GND (Pin:10). Below is an image of the XCTU software when a XBee has connected successfully.

As seen in Figure (16) there are highlighted/numbered parameters that are of importance to allow successful communication between two XBee devices. These are…

[1] - Function set: This is the firmware placed onto the XBee device, all devices in the network must be on the same family of firmware to work. In the project, Zigbee Protocol is being used and therefore this is also how the devices are designated as either a Coordinator, Router or End Device. However, if IEEE XBee 802.15.4 is used (an alternative network protocol), then an option called CE would have to enabled to the coordinator and all other devices in the network would have this setting disabled. In the case of Figure (16) a Coordinator and Router is designated using the Zigbee Protocol.

[2] – Pan ID: This is the unique personal network area ID, this is an identifier used by all XBee devices on the network to make sure that they are all connected to the same network. In Figure (16) the Pan ID used is 2185, however, any value can be used as long as all XBee in the network are designated the same value in the Pan ID.

[3] – DH: This is the destination address high, this is the first half of the address to the XBee device that the source XBee would like to communicate too. In figure (16) the router uses the serial high (SH) of the coordinator. In a complex network with multiple routers, designating this as 0 will communicate directly to the coordinator.

[4] – DL: This is the destination address low, this is the last half of the address to the XBee device the source XBee would like to communicate too. In figure (16) the router uses the serial low (SL) of the coordinator. In a complex network with multiple routers, designating this as 0 on the router will communicate directly to the coordinator. Designating this as FFFF on the coordinator will broadcast to all routers in the network.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.2. XBee Transparent Mode

The AT mode allows for point-to-point communication between two XBee, though more than two XBee can be connected to an AT network, the issue is traffic would overwhelm the coordinator causing data to be lost and overlap each other. The coordinator is unable to select a single XBee to communicate with, instead the coordinator would be able to broadcast the same command to all XBee, which in the case of the Robocone would cause two Robocone to arrive at the same grid location. As seen in figure (17) the coordinator sends a simple string of ‘Hello from the Coordinator, which is received by the router. The Router return the message ‘Hello from the Router’ back and this message is received by the coordinator. The code for Appendix A allows for this communication to be interpreted by the serial monitor of the Arduino UNO. So when the user is prompted to enter a grid goal, the coordinator is able to send a message over the Zigbee network to the router in order to update the Robocone goal grid.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.3. XBee API Mode

The API mode is more complex and requires following figure (18) format exactly in order to send and receive data. API mode also allows for changing the properties of the pin on XBee device, acting as a wireless switch to those pins. This also allows for specific communication if the respective XBee DH and DL are inputted in section designated as the destination network address. Due to time constraints and issues with hardware, getting the API mode to fully work was unsuccessful.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.3.4. Integration within the Project

Due to limitations of time and unforeseen issues with hardware, AT mode was chosen for sending commands to a single Robocone through the Arduino IDE serial monitor. The XBee also allowed for the receiving of data on to the serial monitor that could be displayed for the user, such as it’s current ‘virtual’ position as well as how the A* Pathfinding algorithm is calculating the ‘best’ path.The SoftwareSerial library for Arduino also allows for the XBee to be configured to use specific digital pin on the Arduino for TX and RX communication. In the appendix C code, the TX and RX pins are configured to send and receive on pin 2 and 3 on the Arduino.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.4. Motor Control

The MONA Educational Robot in this project can use an open-loop code structure or a closed-loop code structure design relying on the encoder built in to give feedback as to how far the MONA bot has travelled. The next few pages will illustrate tests done on the MONA bot to see how reliable the bot is while traversing in a straight line and while rotating from a stand still position.

The test for the straight line was done using a 1-centimeter grid paper with designated ‘expected’ markings at 5, 10 and 15 cm to test to see how accurate open-loop and closed-loop are. Not only will distance be measured but also the drift when the MONA bot reaches its target location. This is done by marking a line at the centre of the paper and measuring the distance from the centre line to the small LED at the front of the MONA bot. 

In the rotation test, only the closed loop will be used (see the end of chapter 3.4.1 for the reason why) to see how well the MONA bot rotate at 45, 90 and 180 degrees. This is done to see if these respective values are reflective of their real-world result. The K and I values have been kept the same as in the original code and so adjusting the angle to compensate maybe required.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

3.4.1. The reliability of the MONA Educational Robot traveling in a straight line in Closed and 
Open Loop design

(See Figure 21 – Mean %Error of Open and Closed Loop [21])

(See Figure 22 – Mean of Angular Error of Open and Closed Loop [21])

(See Table 1 Open-Loop)

(See Table 2 Closed-Loop)

In figure (21) and figure (22), both graphs show that the closed-loop is more reliable than the open-loop. The optimal PWM value for the open-loop was found by counting how many rotation per minute were done by the wheel vs the PWM value and plotting these values onto a graph, allowing the gradient (𝛼) and constant (𝛽) to be found. Then find the optimal RPM (Nr) that would achieve 5cm/s using this equation: 

(𝑣∗60) / (2∗π∗r) = 𝑁𝑟 (𝑟𝑝𝑚) 𝐸𝑞𝑢𝑎𝑡𝑖𝑜𝑛 (4)

Key: 

V = velocity in (cm/s) 

1.6 = r = radius of the wheel in (cm)

Then gradient (𝛼) and constant (𝛽) as well as the Nr are plug into the formula below:

𝑃𝑟 (𝑂𝑝𝑡𝑖𝑚𝑎𝑙 𝑃𝑊𝑀) = (𝑁𝑟 − 𝛽) / 𝛼 = 𝑁𝑟 (𝑟𝑝𝑚) 𝐸𝑞𝑢𝑎𝑡𝑖𝑜𝑛 (4)

The resulting Pr is 155.93 for the optimal PWM.

However as seen In figure (21) and figure (22) the closed-loop has less error compared to the open-loop counterpart. The drift is also less in the closed-loop and could be because the PWM in the open-loop is not tuned enough for accurate results. The open-loop could be improved by tuning the PWM but the same can be said for the closed-loop. However, even if both systemswere perfectly tuned, there would still be errors because of slip and mechanical backlash, this isunavoidable unless other more accurate and potentially more expensive wheel bot is used.Therefore, the closed-loop will be used in this project and the distance in the code will not be adjusted, keeping the distance at 12cm, the central distance between of each grid within the test area. The closed-loop will also be used for the next test, the rotation test. 

However as seen In figure (21) and figure (22) the closed-loop has less error compared to the 
open-loop counterpart. The drift is also less in the closed-loop and could be because the PWM in 
the open-loop is not tuned enough for accurate results. The open-loop could be improved by 
tuning the PWM but the same can be said for the closed-loop. However, even if both systems
were perfectly tuned, there would still be errors because of slip and mechanical backlash, this is
unavoidable unless other more accurate and potentially more expensive wheel bot is used.
Therefore, the closed-loop will be used in this project and the distance in the code will not be 
adjusted, keeping the distance at 12cm, the central distance between of each grid within the test 
area. The closed-loop will also be used for the next test, the rotation test.
