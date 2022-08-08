# server_odom.py

# Ros imports
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Sys imports
import sys
import socket
import struct
import threading
from time import sleep

class ClientThread(threading.Thread):
    def __init__(self,clientAddress,clientsocket):
        threading.Thread.__init__(self)
        self.csocket = clientsocket
        print ("New connection added: ", clientAddress)
        rospy.init_node('serverOdom', anonymous=True)
        rospy.Subscriber('odom',Odometry, self.callback)
        print ("New subscriber 'severOdom' added: ")

    def run(self):
        rospy.spin()

    def callback(self, msg):
        string=str(msg.pose.pose.position.x)+"\n"+str(msg.pose.pose.position.y)+"\n"
        try:
            self.csocket.send(string.encode())
            print (string)
        except socket.error:
            print ("Error client lost")
            return

if __name__ == '__main__':
    LOCALHOST = ""
    PORT = 9876
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LOCALHOST, PORT))
    print("Server started")
    print("Waiting for client request..")
    while True:
        sleep(2)
        server.listen(5)
        clientsock, clientAddress = server.accept()
        newthread = ClientThread(clientAddress, clientsock)
        newthread.start()