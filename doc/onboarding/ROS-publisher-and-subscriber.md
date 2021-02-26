# How to write a simple ROS publisher and subscriber

We are going to create 2 ROS nodes.
A "node" in ROS is a small program that is used to accomplish a specific task.
One node will be a publish messages, and the other will listen to those messages.

As a side note, there is a full ROS tutorial [here](http://wiki.ros.org/ROS/Tutorials).
You can go through that if you wish, or you can simply follow the instructions below.
This tutorial has been adapted from the ["Writing a Simple Publisher and Subscriber (Python)"](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) page.

We will be writing these nodes in python for simplicity's sake.
You can find a tutorial for C++ on the ROS website.

## Publisher

In the directory `software/spear_rover/nodes`, create a new file called `talker.py`.
You can create and edit this file outside the docker container (in your virtual machine if you are using one or in your host machine if you have a native linux install).
In this file, paste the following code:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('my_topic', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = "Current time: %s " % rospy.get_time()
        msg += "insert your own message here!"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

Once you have created the file `talker.py`, run this command in the `software/spear_rover/nodes` directory to make sure it is executable:

```bash
chmod +x talker.py
```

## Explanation of publisher code

I'll go line by line and explain the code above.

```python
#!/usr/bin/env python3
```

This tells the computer to execute the file using the python interpreter (as opposed to the interpreter for some other language like bash).

```python
import rospy
from std_msgs.msg import String
```

The first `import` line imports the base ROS python library.
Every ROS node in python must do this.
The next line imports the `String` message type.
ROS has a bunch of message types but right now we will just be publishing `String` type messages.

```python
def talker():
```

Here we are defining a python function that will publish messages for us.

```python
    pub = rospy.Publisher('my_topic`, String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
```

This tells ROS to create a new publisher that will publish to a topic called `my_topic`.
Every ROS message must be assigned a topic to be published under.
This line also tells ROS that the node will publish `String` type.
The `queue_size=10` argument tells ROS to queue up to 10 messages if a subcribing node is not reading them fast enough.
The line `rospy.init_node('talker', anonymous=True)` registers this ROS node with the ROS Master under the name `talker` so that it can communicate with other ROS nodes.
The argument `anonymous=True` ensures this node has a unique name.

```python
    rate = rospy.Rate(10) # 10hz
```

This line defines the rate at which the node will publish messages.
The variable `rate` will be used later in the loop.

```python
    while not rospy.is_shutdown():
```

This line begins a while loop so that our node can repeatedly publish messages.
The bit `rospy.is_shutdown()` is to make sure the loop exits when the user presses Ctrl-C or otherwise stops the node.

```python
        msg = "Current time: %s " % rospy.get_time()
        msg += "insert your own message here!"
        rospy.loginfo(msg)
        pub.publish(msg)
```

These lines create a new message and publish it.
You can go ahead and change the string `"insert you own message here!"` to say whatever you want!

Note that the line `rospy.loginfo(msg)` is not neccessary.
It just prints out the message you created to the terminal.

```python
        rate.sleep()
```

This line causes the loop to wait momentarily so that the node does not publish faster than the rate we specified earlier.

```python
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

These lines simply run the `talker` function we wrote.
Don't worry too much about them right now.

## Subscriber

Now we will create another node to listen to messages from the talker node.

Create another file in `software/spear_rover/nodes` called `listener.py`.

Paste in the following code:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("my_topic", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
```

Again, make sure the file is executable by running this:

```bash
chmod +x listener.py
```

## Explanation of subscriber code

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
```

These lines do the same things as they did in the publisher.

```python
def callback(data):
```

This line defines a function called `callback`.
"Callback" functions are functions that are called after something happens.
In the case of ROS subscribers, callbacks are called after every message is received.


```python
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
```

All our callback function will do it print out messages it hears.
That's what that line does.

```python
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("my_topic", String, callback)

    rospy.spin()
```

This function called `listener` creates and registers a new ROS node and subscribes it to the topic called `my_topic`.
Note that this must be the same topic name that we published to.
The call to `rospy.spin()` keeps the node running and listening for messages (since a python program would normally just exit immediately).

```python
if __name__ == '__main__':
    listener()
```

This simply runs the `listener` function.

## Running everything

This next section assumes you are using Docker. If you are using a virtual machine or a native install, open a terminal to the main `software` directory and skip to the bit where you run `roscore`. Instead of using `tmux`, you can just open new terminal windows.

Now that we've finished editing the files outside of the Docker container, we are ready to run everything inside the container.

First, start the Docker container the following in the main `software` directory:

```bash
docker-compose run spear
```

We will use tmux to allow us to run multiple terminal windows inside the Docker container.
Create a new tmux session simply by running this command in the container:

```bash
tmux
```

After you do that, you should see a yellow bar appear at the bottom of your terminal.

Here is a crash course on tmux:
* In tmux, we can create new terminal windows by hitting "Ctrl-b" then "c".
* To switch between windows, hit "Ctrl-b", then the number of the window.
  * For example, to go to the first window (usually numbered 0), hit "Ctrl-b" then "0".
* To close a tmux window, simply hit "Ctrl-d"

Feel free to google tmux tutorials if you would like to learn more.
Also see the "Further Resources" section of the main new member activity page.

In this first tmux window, run the following command:

```bash
roscore
```

This starts the base part of a ROS system.
We must run `roscore` before running our own nodes.

Leave `roscore` running and open a new tmux window.
In this window, run the publisher as follows:

```bash
rosrun spear_rover talker.py
```

Leave that running.
Open a third tmux window.
Now run the subscriber:

```bash
rosrun spear_rover listener.py
```

You should see the listener node print out the messages it received from the talker node.

You can stop any ROS node by hitting "Ctrl-c" in the terminal in which you ran the node.

Congratulations!
You have written and run your first 2 ROS nodes!
