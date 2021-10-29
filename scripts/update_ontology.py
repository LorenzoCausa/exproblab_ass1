#!/usr/bin/env python

## @package exproblab_ass1
#   \file update_ontology.py
#   \brief This node load the cluedo ontology and add all individuals needed for the game
#   \author lorenzo Causa
#   \version 1.0
#   \date 28/10/2021
#
#   \details
#
#   Clients : <BR>
#        /armor_interface_srv
#          
# Description:    
# 
# This simple node loads the cluedo ontology and adds all the hints of Cluedo, 
# also it set all the sources in the ros parameters. This node needs to be  
# run after armor, since it has a client that use its service. All hints are 
# uploaded from the start for avoid to reason each time we add a new hint 
# (and reason again for the disjoint).
#

import roslib
import rospy
import time
import random
from armor_msgs.srv import * 
from armor_msgs.msg import * 

# GLOBAL VARIABLES
## ## Path in which there is the cluedo ontology,it is important to set it correctly.
path_ontology = '/root/ros_ws/src/exproblab_ass1/cluedo_ontology.owl'

def main():
    """ Main of the update_ontology node, it is the only function of the script and does what is described in the Description."""
    rospy.init_node('update_ontology')
    print("Wait ARMOR service..")
    rospy.wait_for_service('armor_interface_srv')
    armor_client = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    req = ArmorDirectiveReq()
    req.client_name = 'update_ontology'
    req.reference_name = 'my_ontology'
    
    # load ontology
    req.command = 'LOAD'
    req.primary_command_spec = 'FILE'
    req.secondary_command_spec = '' 
    # ATTENTION! NEED TO PUT THE CORRECT PATH!
    req.args = [path_ontology, 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true'] # NEED TO PUT THE CORRECT PATH OF THE ONTOLOGY
    res = armor_client(req)
    if(res.armor_response.success==0):
        print('WRONG PATH OF THE ONTOLOGY ')
        exit()
        
    
    #add individuals
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    
    # persons
    req.args = ['Rev. Green','PERSON']
    res = armor_client(req)
    req.args = ['Prof. Plum','PERSON']
    res = armor_client(req)
    req.args = ['Col. Mustard','PERSON']
    res = armor_client(req)
    req.args = ['Mrs. Peacock','PERSON']
    res = armor_client(req)
    req.args = ['Miss. Scarlett','PERSON']
    res = armor_client(req)
    req.args = ['Mrs. White','PERSON']
    res = armor_client(req)
    
    # weapons
    req.args = ['Candlestick','WEAPON']
    res = armor_client(req)
    req.args = ['Dagger','WEAPON']
    res = armor_client(req)
    req.args = ['Lead Pipe','WEAPON']
    res = armor_client(req)
    req.args = ['Revolver','WEAPON']
    res = armor_client(req)
    req.args = ['Rope','WEAPON']
    res = armor_client(req)
    req.args = ['Spanner','WEAPON']
    res = armor_client(req)
    
    # places
    req.args = ['Conservatory','PLACE']
    res = armor_client(req)
    req.args = ['Lounge','PLACE']
    res = armor_client(req)
    req.args = ['Kitchen','PLACE']
    res = armor_client(req)
    req.args = ['Library','PLACE']
    res = armor_client(req)
    req.args = ['Hall','PLACE']
    res = armor_client(req)
    req.args = ['Study','PLACE']
    res = armor_client(req)
    req.args = ['Ballroom','PLACE']
    res = armor_client(req)
    req.args = ['Dining Room','PLACE']
    res = armor_client(req)
    req.args = ['Billiard Room','PLACE']
    res = armor_client(req)
    #req.args = ['Cluedo Room','PLACE']
    #res = armor_client(req)
    
    # Reason
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['']
    res = armor_client(req)
    
    # Disjoint all (need to disjoint after the reason than reason again)
    req.command = 'DISJOINT'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['PERSON']
    res = armor_client(req)
    req.args = ['WEAPON']
    res = armor_client(req)
    req.args = ['PLACE']
    res = armor_client(req)
    
    # Reason
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['']
    res = armor_client(req)
    
    
    #add all hints in ros param
    rospy.set_param('who', 'PERSON')
    rospy.set_param('what', 'WEAPON')
    rospy.set_param('where', 'PLACE')
    
    print('Finished!')

if __name__ == '__main__':
    main()
