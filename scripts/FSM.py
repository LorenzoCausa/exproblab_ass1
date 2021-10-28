#!/usr/bin/env python


## @package exproblab_ass1
#   \file FSM.py
#   \brief This node provides the finite state machine
#   \author lorenzo Causa
#   \version 1.0
#   \date 28/10/2021
#
#   \details
#
#   Clients : <BR>
#        /hint_generator
#
#        /try_hypothesis
#
#        /armor_interface_srv
#
#        /move_turtle_service
#
#        /search_hints_service
#          
# Description:    
# 
# This node implements the finite state machine thanks to smach and manages the 
# investigation. It is the node to be run for last (smach_viewer node is optional 
# and it can be run in any moment).
#  

import roslib
import rospy
import smach
import smach_ros
import time
import random
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exproblab_ass1.srv import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# GLOBAL VARIABLES
armor_client=None
hint_gen_client=None
check_HP_client=None
move_turtle_client=None
search_hints_client=None
HP_counter=0
## Path in which you want to save the solution ontology.
path_save_ontology='/root/ros_ws/src/exproblab_ass1/ontology_case_solved.owl'
current_pose = Pose()
rooms=[]
HP=None

class Hypothesis:
    """ This class contains the hypothesis of the game """
    
    def __init__(self):
        """ Initializer of the class."""
        self.murderer = []
        self.murder_weapon = []
        self.murder_place = []
        
def cut_IRI(my_ind):
    """ Very simple function that cut the IRI from a string received  from armor."""
    start = my_ind.rfind('#')
    end=len(my_ind)-1
    my_ind=my_ind[(start+1) : end]
    return my_ind
    
def cut_all_IRI(ind_array):
    """ Very simple function that cut the IRI from an array of strings received  from armor."""
    for i in range(len(ind_array)):
        ind_array[i] = cut_IRI(ind_array[i])
    return ind_array
  
def add_HP(HP):
    """Add an hypothesis in to the ontology"""	
    # ADD  HYPOTHESIS
    global HP_counter
    req = ArmorDirectiveReq()
    req.client_name = 'FSM'
    req.reference_name = 'my_ontology'
    HP_counter = HP_counter+1
    hypothesis = 'Hypothesis'+str(HP_counter)
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS' 
    req.args = [hypothesis,'HYPOTHESIS']
    res = armor_client(req)
    # add properties to new ind.
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND' 
    
    for i in range(len(HP.murderer)):
        req.args = ['who',hypothesis,HP.murderer[i]]
        res = armor_client(req)
    for i in range(len(HP.murder_weapon)):
        req.args = ['what',hypothesis,HP.murder_weapon[i]]
        res = armor_client(req)
    for i in range(len(HP.murder_place)):
        req.args = ['where',hypothesis,HP.murder_place[i]]
        res = armor_client(req)        

    # Reason
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['']
    res = armor_client(req)	
    #print('Hypothesis'+str(HP_counter),'added')
    return
    
def check_HP():
    """Check with the ontologi if the last hypothesis is completed and consistent"""	
    ok=False
    req = ArmorDirectiveReq()
    req.client_name = 'FSM'
    req.reference_name = 'my_ontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS' 
    req.args = ['COMPLETED']   
    res = armor_client(req).armor_response.queried_objects 
    hypotheses=cut_all_IRI(res)
    #print('completed HP: ',hypotheses)
    if('Hypothesis'+str(HP_counter) in hypotheses):
        req.args = ['INCONSISTENT']   
        res = armor_client(req).armor_response.queried_objects   
        hypotheses=cut_all_IRI(res)
        #print('inconsistent HP: ',hypotheses)
        if(('Hypothesis'+str(HP_counter) in hypotheses)==False):
            ok=True
    return ok            			
         		

def query_ontology(ont_class):
    """It get stuff from my ontology"""
    req = ArmorDirectiveReq()
    req.client_name = 'FSM'
    req.reference_name = 'my_ontology'   
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS' 
    req.args = [ont_class]
    res = armor_client(req).armor_response.queried_objects
    return res    
    
def save_ontology():    
    """save the ontology_solution in the path in which you want the ontology"""
    req = ArmorDirectiveReq()
    req.client_name = 'FSM'
    req.reference_name = 'my_ontology'  
    req.command = 'SAVE'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = [path_save_ontology]
    res = armor_client(req)	
    return

def put_hint_in_HP(hint,HP):
    if(hint.which_hint=='who'):
        HP.murderer.append(hint.hint)
    if(hint.which_hint=='what'):
        HP.murder_weapon.append(hint.hint)
    if(hint.which_hint=='where'):
        HP.murder_place.append(hint.hint)
    return
                
# define state explore
class Explore(smach.State):
    """ State in which the investigator move through  rooms. """	
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, outcomes=['enter_room'])
        
    def execute(self, userdata):
        # choose next room  
        print('Exploring')
        room = random.choice(rooms) 
        move_turtle_client(room)
        print('Enter in ', room)
        return 'enter_room'
    
# define state make_hypothesis
class Search_hints(smach.State):
    """ State in which the investigator search for hints and think a hypothesis. """
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_cluedo_room','search_another_room'],
                                   input_keys=['first'],
                                   output_keys=['first'])

    def execute(self, userdata):
        global HP
        search_hints_client('')
        hint = hint_gen_client(userdata.first)
        put_hint_in_HP(hint,HP)
        print(userdata.first)
        if(userdata.first):
            userdata.first=False

        if(hint.last==False):
            print('More hints needed to finish the hypothesis ')
            return 'search_another_room' 
            
        else:    
            #print(HP)
            userdata.first=True
            add_HP(HP )
            good_HP=check_HP()
            if(good_HP):
                print('Valid hypothesis')
                return 'go_cluedo_room'
                
            else:
                print('Inconsistent or incomplete hypoyhesis')
                HP.murderer=[]
                HP.murder_weapon=[]
                HP.murder_place=[]
                return 'search_another_room'    

                    			    
# define state make_hypothesis
class Try_hypothesis(smach.State):
    """ State in which the investigator tries its hypothesis. """
    def __init__(self):
        smach.State.__init__(self, outcomes=['wrong_hypothesis','right_hypothesis'])

    def execute(self, userdata):
        global HP
        move_turtle_client('Cluedo Room')
        print('Hypothesis: ')
        print('Who: ',HP.murderer[0])
        print('What: ',HP.murder_weapon[0])
        print('Where: ',HP.murder_place[0])
        res=check_HP_client(HP.murderer[0],HP.murder_weapon[0],HP.murder_place[0])
        if res.ok:
            save_ontology()
            print('Right solution!!!')
            return 'right_hypothesis'
        else:
            print('wrong hypothesis')
            HP.murderer=[]
            HP.murder_weapon=[]
            HP.murder_place=[]
            return 'wrong_hypothesis'    
        
        
def main():
    """ main of the node, it creates ros clients and initialize the state machine for the investigation."""
    global armor_client,hint_gen_client,check_HP_client,rooms,move_turtle_client,search_hints_client,HP
    rospy.init_node('FSM')
    HP=Hypothesis()
    print("Wait for all services needed..")
    rospy.wait_for_service('armor_interface_srv')
    rospy.wait_for_service('hint_generator')
    rospy.wait_for_service('try_hypothesis')
    rospy.wait_for_service('move_service')
    rospy.wait_for_service('search_hint_service')
    
    # Create services client
    armor_client = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)  
    hint_gen_client = rospy.ServiceProxy('hint_generator', hint_gen) 
    check_HP_client = rospy.ServiceProxy('try_hypothesis', solution_HP)
    move_turtle_client = rospy.ServiceProxy('move_service', turtle_controller_srv)
    search_hints_client = rospy.ServiceProxy('search_hint_service', turtle_controller_srv)
    
    rooms=query_ontology('PLACE')
    rooms=cut_all_IRI(rooms)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['CASE_SOLVED'])
    sm.userdata.first=True

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('EXPLORE', Explore(), 
                               transitions={'enter_room':'SEARCH_HINTS'})

        smach.StateMachine.add('SEARCH_HINTS', Search_hints(), 
                               transitions={'go_cluedo_room':'TRY_HYPOTHESIS',
                                            'search_another_room':'EXPLORE'})

        smach.StateMachine.add('TRY_HYPOTHESIS', Try_hypothesis(), 
                               transitions={'wrong_hypothesis':'EXPLORE', 
                                            'right_hypothesis':'CASE_SOLVED'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
