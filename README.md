# Experimental Robotics Laboratory assignment

## Introduction
This is a ROS package that simulates a simple Cluedo-like investigation. It uses ARMOR as reasoner and container of the ontology of the game and smach for the Finite State Machine behavior.

## Software Architecture
### Component Diagram
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
