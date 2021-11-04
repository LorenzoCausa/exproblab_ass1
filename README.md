# Experimental Robotics Laboratory assignment

## Introduction
This is a ROS package that simulates a simple Cluedo-like investigation. It uses ARMOR as reasoner and container of the ontology of the game and smach for the Finite State Machine behavior.

## Software Architecture
### Component Diagram
In the diagram below you can see all the nodes necessary for the correct functioning of the code. smach_viewer is not there as it is only useful for displaying the states while the code is running (so it is not necessary but optional).

* **update_ontology**: This node loads "cluedo_ontology" into "armor" and adds all Cluedo hints. I add all individuals at the beginning for two reasons:
  1) Doing it only once at the beginning of everything decreases the number of REASON that ARMOR has to do, in fact every time an individual is added it is necessary to REASON twice: ADD, REASON, DISJOINT and REASON again.
  2) It makes the system more flexible, in fact, by doing so "update_ontology" is the only node dependent on the hints, all the others nodes take them from the ontology. In this way it is possible to change, add and modify the hints by modifying only this node.

    "update_ontology" also loads possible hint sources in rosparam: PERSON, WEAPON and PLACE. This node must be run after "armor" after it is completed "oracle" and "FSM" can be run.

* **armor_service**: This is the node provided to us to manage ontologies. It is a powerful and versatile management system for single and multi-ontology architectures under ROS. It allows to load, query, and modify multiple ontologies. Despite its ease of use, ARMOR provides a large share of OWL APIs functions and capabilities in a simple server-client architecture and offers increased flexibility compared to previous ontological systems running under ROS.

* **oracle**:  This node is the oracle of the game, it generates a finite number of hypotheses and provides two services to interact with them:
  * hint_generator: It request a hint of an hypothesis. 
  * try_hypothesis: Check if my guess is correct.

* **FSM**: Implement a finite state machine that simulates the investigation using smach. It has four clients that it uses to interact with other nodes. More information about the behavior of this node will be provided in the State Diagram section of the README.

* **turtle_controller**: This node provides two services to command the turtlebot which simulates the investigator in the environment:
  * move_turtle_service: Move the turtle to the room you want.
  * search_hint_service: Spin the turtle a random time to simulate searching for clues in a room.

* **turtlesim_node**: Standard node of ros.

![Alt text](/images/component.PNG?raw=true)

### State Diagram
In the diagram below you can see all the states of the finite state machine implemented in the node "FSM" thanks to smach.

* **EXPLORE**: In this state, the investigator moves to a random room to look for hints

* **SEARCH_HINTS**: The investigator looks for a hint, once it is found if this ends the hypothesis and if the hypothesis is valid (complete and consistent) then it goes to the TRY_HYPOTHESIS state otherwise it goes back to the EXPLORE state. The cycle continues until a valid hypothesis is found.

* **TRY_HYPOTHESIS**: The valid hypothesis found is tested, if it is correct the game ends, if wrong it returns to the EXPLORE state.

* **Case solved**: Congratulation! You found the solution!

![Alt text](/images/state.PNG?raw=true)

### Temporal Diagram
In the diagram below you can see the behavior of the code over time.
 
1) "update_ontology" is executed.
2) Once the ontology is ready (loaded and updated) the Oracle node generates a finite set of random hypotheses: 3 valid, 3 incomplete and 3 inconsistent. From the set of complete hypotheses it will choose a solution.
3) Now that all the services are ready also "FSM" can start and with it the investigation, which will end once the state machine has managed to find the hypothesis that "oracle" has chosen as the solution.

![Alt text](/images/temporal.PNG?raw=true)

### RQT Graph
Graph generated by:
```
rosrun rqt_graph rqt_graph
```
Highlight all active nodes and subscriptions to related topics.

![Alt text](/images/rqt_graph.jpeg?raw=true)

### Cluedo Map
This image represents the cluedo map superimposed on the world of turtlesim. Unfortunately, there is no way with turtlesim to load an image in the background so the turtle moves in the rooms highlighted in this image however while the code runs the background of the map will remain blue without making explicit the room in which the turtle has just entered . To remedy this problem just read from the terminal, every time the investigator (the turtle) enters a room, the room in which she entered is written. Another solution is to keep this map within reach and see where the turtle is moving by making the comparison.

![Alt text](/images/turtle_map.PNG?raw=true)

### List of msgs, srvs and rosparameters
#### msgs
* **Twist**: to publish the speeds and control the turtle
* **Pose**: to subscribe and know the current position of the turtle
#### srvs
* **armor_interface_srv**: to interact with the armor node and the ontology
* **hint_generator**: to request a hint from the oracle
* **try_hypothesis**: to check if the hypothesis is correct
* **move_service**: to move the turtle in rooms
* **search_hint_service**: to move the turtle in a way that simulates searching for hints in a room
#### rosparameters
* **current_ID_hypothesis**: it gives you the ID of the current hypothesis
* **current_murderer_hypothesis**: it gives you the murderers (if plural clearly the hypothesis is not valid) of the current hypothesis
* **current_murder_weapon_hypothesis**: it gives you the murder weapons (if plural clearly the hypothesis is not valid) of the current hypothesis
* **current_murder_place_hypothesis**: it gives you the murder places (if plural clearly the hypothesis is not valid) of the current hypothesis
* **source1**: it gives you the first source of informations (PERSON)
* **source2**: it gives you the second source of informations (WEAPON)
* **source3**: it gives you the third source of informations (PLACE)

## Installation and running procedure
### Installation
The installation is straightforward, just clone this repository to your ros workspace and do the catkin_make:
```
git clone https://github.com/LorenzoCausa/exproblab_ass1
```
```
catkin_make
```
### Requirements
To use this code some external packages are needed:
* armor
* smach
* turtlesim

### Put Correct Paths
Before you can run the code it is necessary to enter the correct paths for the cluedo_ontology and for where you want to save the finished ontology with all the hypotheses and the solution loaded.

Go in "update_ontology.py" and put the path in which there is your cluedo_ontology in the global variable "path_ontology"

Go in "FSM.py" and put the path in which you want to save your solution_ontology in the global variable "path_save_ontology"

**Note**: If you use the docker that was provided to us the paths are already written correctly.

### Running procedure
Just use the launcher:
```
roslaunch exproblab_ass1 cluedo.launch 
```
If you prefer you can also run a node at a time. In this case it is good to run them in this order:
1) armor
2) turtlesim_node
3) turtle_controller
4) update_ontology
5) wait that update_ontology finish
6) oracle
7) FSM 

**Note**: smach_viewer is optional and it can be run in any moment 

## Behavior
1) The investigator (the turtle) moves into a random room.
2) Look for hints in the room (turn on itself)
3) If with the last hint found concludes a valid hypothesis then it passes to 4. otherwise it restarts from 1.
4) Go to the Cluedo Room and test his hypothesis. If it is correct the case is solved, on the contrary it goes back to 1.

**Note**: This cycle continues until the case is solved, however the execution is never too long as each time a hypothesis is tested it is removed from the finite list of possible hypotheses.

![Alt Text](/images/my_turtle_gif.gif?raw=true) 

In the figure below you can also see all the windows of interest of the system:
* the terminal
* the turtlesim_node node
* the smach_viewer node

**Note**: It was not possible to make the GIF with all the windows because the reduced resolution made the writings unreadable.

![Alt Text](/images/screenshot.PNG?raw=true)

## Video demo
You can take a look at the complete video demo of the project from here:

[Video demo here](https://drive.google.com/file/d/16Gz2Lt-EQ11GA9QHAHYPvvqo3uU54BXt/view?usp=sharing)

## Working hypothesis and environment
The system is designed for maximum flexibility and modularity. With very few modifications, for example, both ontology and simulation can be changed.To avoid that the code lasts too long, it has been made so that every time a hypothesis is made, it is eliminated from the set of possible hypotheses. This is done also to simulate an intelligent investigator who does not repeat the same hypotheses. In the current code, a set of 9 randomly generated possible hypotheses is initialized: 3 valid, 3 incomplete, 3 inconsistent. The number of hypotheses can be easily expanded. In an earlier version of the code, the hypotheses were generated in a completely random way (not from a finite starting set) to better simulate the cluedo's investigation. With this version, however, the code ran for a very long time so to make it testable and debuggable the finite set of hypotheses was implemented.

## System’s features
* Elimination of already tried hypotheses.
* Simulated motion in the environment with turtlesim.
* Modularity and flexibility.
* Random-based generation of hints and the validation of hypothesis through armor.
* Saving the ontology with the history of all the hypotheses made and the solution.

Another feature of the system is the ability to retrieve information about the investigation while the code is running.

With:
```
rosparam get /current_ID_hypothesis 
```
you can get the ID of the hypothesis that the investigator is currently making:
* HP0, HP1, HP2 are valid hyotheses.
* HP3, HP4, HP5 are incomplete hypotheses.
* HP6, HP7, HP8 are inconsistent hypotheses.

Instead with:
```
rosparam get /current_murderer_hypothesis 

rosparam get /current_murder_weapon_hypothesis

rosparam get /current_murder_place_hypothesis
```
you can get the murderer, the murder weapon and the crime scene of the hypothesis that the investigator is currently making, in case of invalid hypotheses these could also be multiple.

## Improvements
Possible improvements for the system are:
* Simulation in a more precise environment: a map with walls and rooms to better emulate the Cluedo game
* "intelligent" investigator who manages to immediately discard wrong hypotheses: now he has to collect the whole hypothesis before being able to ask armor if it is valid, an important improvement could be to make it immediately discard a hypothesis as soon as it has a double field (two who or two what or two where).
* Generation of totally random hypotheses. At the moment they are pseudorandom (from a predetermined finite set) to limit the execution time of the system. Note that if we had the "intelligent" investigator of the previous point we could generate random hypotheses and still maintain a not too long execution time.

## Documentation
All the doxygen documentation is in the Doc folder or you can access it by clicking here:
[click Here!](https://lorenzocausa.github.io/exproblab_ass1/)

**Note**: For some obscure reason, if from the github page you go to file and try to access the documentation of the FSM.py file, this will not be found. The problem is easily avoided by looking at the FSM documentation from the namespaces or by opening the documentation directly from the Doc folder after downloading the package locally.

## Author and contacts
The system was developed entirely by me, below my credentials and contacts:

**Lorenzo Causa, 4519089**

**mail**: lorenzo.causa@libero.it

**phone number**: 3247427182  
