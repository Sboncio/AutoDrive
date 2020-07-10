#! /usr/bin/env python3

import rospy
import tkinter as tk
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import os
import time
import subprocess

current_state = "Idle"

def publishCommand(xpos, ypos, start):

    if xpos != ' ' and ypos != ' ':
        msg = Vector3()
        msg.x = xpos
        msg.y = ypos
        msg.z = start

        pub.publish(msg)


## End Publish

def test():
    
    num1 = float(entry.get())
    num2 = float(entry1.get())
    publishCommand(num1, num2, 1)

## End Test

def stop():
    num1 = float(entry.get())
    num2 = float(entry1.get())
    publishCommand(num1, num2, 0)

def launch_simulation():
    if os.environ.get('KDE_FULL_SESSION') == 'true':
        terminal = "konsole"
    elif os.environ.get('GNOME_DESKTOP_SESSION_ID') == 'true':
        terminal = "gnome-terminal"
    else:
        terminal = "gnome-terminal"

    command = terminal + " -e roslaunch turtlebot3_gazebo turtlebot3_world.launch &"
    os.system(command)

def exit_system():
    exit()

def subscriber(msg):
    current_state = msg.data


rospy.init_node('Test')
rate = rospy.Rate(10)

pub = rospy.Publisher("interface", Vector3, queue_size=10)

sub = rospy.Subscriber("state", String, subscriber)

while not rospy.is_shutdown():

    ##Windows
    window=tk.Tk()
    window.title("Auto drive control panel")
    window.minsize(800,200)
    window.iconphoto(False, tk.PhotoImage(file="/home/sam/catkin_ws/src/AutoDrive/Assets/rosLogo.png"))

    ##Labels
    greeting = tk.Label(text="AutoDrive", fg="black", bg="white")
    greeting.pack()

    ##Entry
    label = tk.Label(text="X-coordinate")
    entry = tk.Entry()
    label.place(x=10,y=0)
    entry.place(x=0,y=20)

    ##Entry
    label = tk.Label(text="Y-coordinate")
    entry1 = tk.Entry()
    label.place(x=10, y=50)
    entry1.place(x =0, y = 70)

    state_entry = tk.Entry()
    state_entry.place(x = 0, y = 100)
    state_entry.delete(0, len(current_state))
    state_entry.insert(0,current_state)
    print(current_state)


    ##Buttons
    button1 = tk.Button(text="Begin Simulator",width=20,height=10,fg="#000000",bg="#f5a70c", command=launch_simulation, relief="groove")
    button1.place(x = 200, y = 20)

    button = tk.Button(text="Begin System",width=20,height=10,fg="#000000",bg="#00ff00", command=test, relief="groove")
    button.place(x = 385, y = 20)

    stop_button = tk.Button(text="Stop System", width=20,height=10,fg="black",bg="red",command=stop,relief="groove")
    stop_button.place(x = 570, y = 20)

    exit_button = tk.Button(text="Exit", width=5,height=1,fg="white",bg="red",command=exit_system,relief="groove")
    exit_button.place(x=0, y=175)

    window.mainloop()
    rospy.spin()
    rate.sleep()
