#!/usr/bin/env python

## @package exproblab_ass1
#   \file oracle.py
#   \brief This node choose a random hypoyheses for the game and provide a service to check it
#   \author lorenzo Causa
#   \version 1.0
#   \date 28/10/2021
#
#   \details
#
#   Clients : <BR>
#        /armor_interface_srv
#
#   Services : <BR>
#        /try_hypothesis
#        /hint_generator
#          
# Description:  
#  
# This node is the oracle of the game, it generates a finite number of Hypotheses
# and provides services to interact with them in the game.
#

import roslib
import rospy
import time
import random
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exproblab_ass1.srv import *



class Hypothesis:
    """ This class contains the hypothesis of the game """
    
    def __init__(self):
        """ Initializer of the class."""
        self.murderer = []
        self.murder_weapon = []
        self.murder_place = []
        self.ID=''

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
            
def random_HP(hypotheses):
    """Generate finite random set of hipotheses, some correct, some wrong. Needed otherwhise to many possibilities would lead to a very long game """
    global solution
    source1=rospy.get_param("source1")
    source2=rospy.get_param("source2")
    source3=rospy.get_param("source3")
    
    persons=query_ontology(source1)
    persons=cut_all_IRI(persons)
    weapons=query_ontology(source2)
    weapons=cut_all_IRI(weapons)
    places=query_ontology(source3)
    places=cut_all_IRI(places)
    
    for i in range(3):
        HP = Hypothesis()
        hypotheses.append(HP)
        
    #correct HP
    hypotheses[0].murderer.append(random.choice(persons))
    hypotheses[0].murder_weapon.append(random.choice(weapons))
    hypotheses[0].murder_place.append(random.choice(places)) 
    hypotheses[0].ID='HP0' 
     
    hypotheses[1].murderer.append(random.choice(persons))
    hypotheses[1].murder_weapon.append(random.choice(weapons))
    hypotheses[1].murder_place.append(random.choice(places)) 
    hypotheses[1].ID='HP1'
    
    hypotheses[2].murderer.append(random.choice(persons))
    hypotheses[2].murder_weapon.append(random.choice(weapons))
    hypotheses[2].murder_place.append(random.choice(places)) 
    hypotheses[2].ID='HP2'
    
    #choose solution
    solution = random.choice(hypotheses)	        
    load_solution_in_ontology(solution)
    
    for i in range(6):
        HP = Hypothesis()
        hypotheses.append(HP)
        
    # wrong HP not complete
    hypotheses[3].murderer.append(random.choice(persons))
    hypotheses[3].murderer.append(random.choice(persons))
    hypotheses[3].murderer.append(random.choice(persons))
    hypotheses[3].ID='HP3'
    
    hypotheses[4].murder_weapon.append(random.choice(weapons))
    hypotheses[4].murder_weapon.append(random.choice(weapons))
    hypotheses[4].murderer.append(random.choice(persons))
    hypotheses[4].ID='HP4'

    hypotheses[5].murder_place.append(random.choice(places))
    hypotheses[5].murder_place.append(random.choice(places))
    hypotheses[5].murder_weapon.append(random.choice(weapons))  
    hypotheses[5].ID='HP5'
    
    # wrong HP inconsistent
    hypotheses[6].murderer.append(random.choice(persons))
    hypotheses[6].murderer.append(random.choice(persons))
    hypotheses[6].murder_weapon.append(random.choice(weapons))
    hypotheses[6].murder_place.append(random.choice(places))  
    hypotheses[6].ID='HP6'
    
    hypotheses[7].murderer.append(random.choice(persons))
    hypotheses[7].murderer.append(random.choice(persons))
    hypotheses[7].murder_weapon.append(random.choice(weapons))
    hypotheses[7].murder_weapon.append(random.choice(weapons))
    hypotheses[7].murder_place.append(random.choice(places)) 
    hypotheses[7].murder_place.append(random.choice(places))
    hypotheses[7].ID='HP7'

    hypotheses[8].murderer.append(random.choice(persons))
    hypotheses[8].murder_weapon.append(random.choice(weapons))
    hypotheses[8].murder_place.append(random.choice(places)) 
    hypotheses[8].murder_place.append(random.choice(places)) 
    hypotheses[8].murder_place.append(random.choice(places)) 
    hypotheses[8].ID='HP8'
    
    return        
    
def load_solution_in_ontology(solution):  
    """ Method for loading the solution of the game into ontology.""" 
    armor_client = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)  
    req = ArmorDirectiveReq()
    req.client_name = 'case_solution'
    req.reference_name = 'my_ontology'
        
    # ADD SOLUTION HYPOTHESIS
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS' 
    req.args = ['SOLUTION','HYPOTHESIS']
    res = armor_client(req)
    # ADD PROPERTIES TO THE NEW INDIVIDUAL
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND' 
    req.args = ['who','SOLUTION',solution.murderer[0]]
    res = armor_client(req)
    req.args = ['what','SOLUTION',solution.murder_weapon[0]]
    res = armor_client(req)
    req.args = ['where','SOLUTION',solution.murder_place[0]]
    res = armor_client(req)
    # REASON
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['']
    res = armor_client(req)
    return
        
def query_ontology(ont_class):
    """It get stuff from my ontology"""
    armor_client = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)  
    req = ArmorDirectiveReq()
    req.client_name = 'oracle'
    req.reference_name = 'my_ontology'
    
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS' 
    req.args = [ont_class]
    res = armor_client(req).armor_response.queried_objects
    return res
    
def handle_try_HP(req):
    """ Handle of the try_hypothesis service."""
    res = solution_HPResponse()
    res.ok=False
    
    if(req.who==solution.murderer[0] and req.what==solution.murder_weapon[0] and req.where==solution.murder_place[0]):
        res.ok=True
  
    return res  

def handle_hint_gen(req):
    """ Handle of the hint_gen service"""	
    global hypotheses,murderer,murder_weapon,murder_place
    res = hint_genResponse() 
    res.last=False
    # If first take a new HP
    if req.first:
        HP=random.choice(hypotheses)        
        murderer=HP.murderer[:]
        murder_weapon=HP.murder_weapon[:]
        murder_place=HP.murder_place[:]
        ID=HP.ID
        hypotheses.remove(HP)
        # Put in rosparam the current hypothesis
        rospy.set_param('current_murderer_hypothesis', murderer)
        rospy.set_param('current_murder_weapon_hypothesis', murder_weapon)
        rospy.set_param('current_murder_place_hypothesis', murder_place)
        rospy.set_param('current_ID_hypothesis', ID)
        # print for debug
        #print(murderer)
        #print(murder_weapon)
        #print(murder_place)
        
    # The hints found go in order: who, what, where. This is just for simplify the code, the FSM is independent from this and could take hints in any order.
    if(murderer!=[]):
        res.which_hint='who'
        res.hint=murderer[0]
        murderer.remove(murderer[0])
        
    elif(murder_weapon!=[]):
        res.which_hint='what'
        res.hint=murder_weapon[0]
        murder_weapon.remove(murder_weapon[0])
           
    elif(murder_place!=[]):
        res.which_hint='where'
        res.hint=murder_place[0]
        murder_place.remove(murder_place[0]) 
        
    if(murderer==[] and murder_weapon==[] and murder_place==[]):
        res.last=True
        
    #print (res)
    return res
    
    
#GLOBAL VARIABLES
solution = Hypothesis()
hypotheses=[] 
murderer=[]
murder_weapon=[]
murder_place=[]  
  	        
def main():
    """ Main of the node, it creates and provides the services. """
    global hypotheses
    rospy.init_node('oracle')
    time.sleep(15) #wait that ontology is updated
    random_HP(hypotheses)
     
    # PRINT SOLUTION
    print('SOLUTION')
    print('murderer: ',solution.murderer[0])
    print('murder weapon: ',solution.murder_weapon[0])
    print('murder place: ',solution.murder_place[0])
    

    # create services
    try_hypothesis = rospy.Service('try_hypothesis', solution_HP, handle_try_HP)
    hint_serv = rospy.Service('hint_generator', hint_gen, handle_hint_gen)
    
    rospy.spin()

if __name__ == '__main__':
    main()        
