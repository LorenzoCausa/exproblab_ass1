# Experimental Robotics Laboratory assignment

## Introduction
This is a ROS package that simulates a simple Cluedo-like investigation. It uses ARMOR as reasoner and container of the ontology of the game and smach for the Finite State Machine behavior.

## Software Architecture
### Component Diagram
In the diagram below you can see all the nodes necessary for the correct functioning of the code. smach_viewer is not there as it is only useful for displaying the states while the code is running (so it is not necessary but optional).

* **update_ontology**: This node loads the "cluedo_ontology" into "armor" and adds all cluedo hints. This is done only once for the beginning of everything to decrease the number of REASON that armor has to do, in fact every time an individual is added it is necessary to reason twice: ADD, REASON, DISJOINT and REASON again. It also loads possible hint sources into rosparam: PERSON, WEAPON and PLACE. This node must be run after "armor" after it is completed "oracle" and "FSM" can be run.

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
![Alt text](/images/temporal.PNG?raw=true)

### RQT Graph
![Alt text](/images/rqt_graph.jpeg?raw=true)

### Cluedo Map
![Alt text](/images/turtle_map.PNG?raw=true)

### List og msgs and srv

## Put Correct Paths
Go in "update_ontology.py" and put the path in which there is your cluedo_ontology in the global variable "path_ontology"

Go in "FSM.py" and put the path in which you want to save your solution_ontology in the global variable "path_save_ontology"
