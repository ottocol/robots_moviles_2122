#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
import sys


class ClienteMoveBase:
    def __init__(self):
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

    #para probar esto desde una terminal: rostopic pub comando std_msgs/String STOP
    def escucharComando(self,datos):
	if datos.data == "STOP":
   	    print("PARANDO!!!")
	    self.client.cancel_goal()
     		

    def moveTo(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        #nos ponemos a escuchar en el topic "comando" por si nos dan orden de parar
	    rospy.Subscriber("comando", String, self.escucharComando)

        self.client.send_goal(goal)
        #En lugar de quedarnos bloqueados esperando que termine el goal
        #nos metemos en un bucle que nos da la oportunidad de escuchar mensajes
	    state = self.client.get_state()
	    while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
	       rospy.Rate(10)
	       state = self.client.get_state()
        return self.client.get_result()

if __name__ == "__main__":
    if len(sys.argv) <= 2:
	    print("Uso: " + sys.argv[0] + " x_objetivo y_objetivo")
	    exit()		
    rospy.init_node('prueba_clientemovebase')
    cliente = ClienteMoveBase()
    result = cliente.moveTo(float(sys.argv[1]), float(sys.argv[2]))
    if result:
        rospy.loginfo("Goal conseguido!")

