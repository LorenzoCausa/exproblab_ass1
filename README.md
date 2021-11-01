# Experimental Robotics Laboratory assignment

## Introduction
This is a ROS package that simulates a simple Cluedo-like investigation. It uses ARMOR as reasoner and container of the ontology of the game and smach for the Finite State Machine behavior.

## Software Architecture
### Component Diagram
In the diagram below you can see all the nodes necessary for the correct functioning of the code. smach_viewer is not there as it is only useful for displaying the states while the code is running (so it is not necessary but optional).

* **update_ontology**: This node loads the "cluedo_ontology" into "armor" and adds all cluedo hints. This is done only once for the beginning of everything to decrease the number of REASON that armor has to do, in fact every time an individual is added it is necessary to reason twice: ADD, REASON, DISJOINT and REASON again. It also loads possible hint sources into rosparam: PERSON, WEAPON and PLACE. This node must be run after "armor" after it is completed "oracle" and "FSM" can be run.

* **armor_service**: This is the node provided to us to manage ontologies. It is a powerful and versatile management system for single and multi-ontology architectures under ROS. It allows to load, query, and modify multiple ontologies. Despite its ease of use, ARMOR provides a large share of OWL APIs functions and capabilities in a simple server-client architecture and offers increased flexibility compared to previous ontological systems running under ROS.

* **oracle**:  This node is the oracle of the game, it generates a finite number of hypotheses and provides two services to interact with them:
  * hint_generator to request a hint of a hypothesis 
  * try_hypothesis to check if my guess is correct

![Alt text](/images/component.PNG?raw=true)

### State Diagram
![Alt text](/images/state.PNG?raw=true)

### Temporal Diagram
![Alt text](/images/temporal.PNG?raw=true)

### RQT Graph
![Alt text](/images/rqt_graph.jpeg?raw=true)

## Cluedo Map
![Alt text](/images/turtle_map.PNG?raw=true)

## Put Correct Paths
Go in "update_ontology.py" and put the path in which there is your cluedo_ontology in the global variable "path_ontology"

Go in "FSM.py" and put the path in which you want to save your solution_ontology in the global variable "path_save_ontology"
