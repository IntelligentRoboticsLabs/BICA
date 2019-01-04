# BICA

[![Build Status](https://travis-ci.com/fmrico/BICA.svg?branch=master)](https://travis-ci.com/fmrico/BICA)

**BICA (Behavior-based Iterative Component Architecture) provides a way to create ROS applications that implements complex behaviors in robots.**

BICA has a long history. It was designed and implemented in 2008 for the RoboCup SPL competition, being the behavioral architecture that allowed a Nao robot to play soccer in this competition. It was executed inside a NaoQi module and allowed to implement behaviors that arose from the concurrent execution of many components. There were perceptual components (image processing, sound ...), acting (walk, kick, leds, attention ...) or reasoning. Component hierarchies were created, where some components explicitly activated others, avoiding synchronization problems and saving computation time.

With the arrival of ROS, the implementation changed, with each component being a ROS node that could define what other components needed for its operation. BICA executed the dependencies of each component. With this architecture we have participated in competitions such as RoCKIn (robot MYRAbot), RoboCup @Home (robot RB-1 Robotnik) and we continue using it, whith (ROSPlan)[https://github.com/KCL-Planning/ROSPlan], in the RoboCup SSPL with the robot Pepper. We also use it in our industrial projects. It provides a mechanism to create independent components that only consume processing time when a component requires the result of its computation.

You can find more references in these papers:

* "Planning-centered Architecture for RoboCup SSPL @Home". Francisco Martín, Jonathan Ginés, David Vargas, Francisco J. Rodríguez-Lera, Vicente Matellán. WAF2018, publisehd in Advances in Intelligent Systems and Computing series. November 2018.
* "A Simple, Efficient, and Scalable Behavior-based Architecture for Robotic Applications", Francisco Martín Rico, Carlos E. Agüero Durán. ROBOT'2015 Second Iberian Robotics Conference.
* "Humanoid Soccer Player Design", Francisco Martín, Carlos Agüero, José María Cañas y Eduardo Perdices. Robot Soccer. Ed: Vladan Papic, pp 67-100. IN-TECH, ISBN, 978-953-307-036-0. 2010.
* "Behavior-based Iterative Component Architecture for soccer applications with the Nao humanoid". Carlos E. Agüero, Jose M. Canas, Francisco Martin and Eduardo Perdices 5Th Workshop on Humanoids Soccer Robots. Nashville, TN, USA. Dic.

Now we have released it so that anyone can take advantage of its robotic applications.

## How it works

Each bica component is executed in a ROS node. Its life cycle is as follows:

* Each component has its own frequency of execution.
* When you launch your application, your nodes will be running, but they will be inactive until some other component activates them. Thus hierarchies of components can be formed.
* A component declares what other components need to be running when it is active. They are its dependecies.
* Un componente se activa cuando algún  componente que lo declaró como depencia se activa. También se activa si este componente se declara como raiz de una jerarquía de ejecución.
* A component is deactivated when there is no active component that has declared it as a dependency.

Example:

In this video we can show the execution of the nodes in bica_examples folder of this repo.

* Component A depends on B and C
* Component C depends con D
* Components log a message when they are active.

In this video we will:

1. Activate A, so B,C and D will be also active. For activating A we will usa a bica tool called launcher by executing `rosrun bica launcher node_A`
1. Activate C, so D will be also active.
1. Activate B.
1. Activate A, so B,C and D will be also active. Then we will close C, so D will be inactive until we execute C again, which inmediatelly is active.

[![BICA Example](https://img.youtube.com/vi/ozYrQdCbGA4/0.jpg)](https://www.youtube.com/watch?v=ozYrQdCbGA4)

The code of a node that implements a BICA component is very simple:

```
#include <ros/ros.h>
#include <ros/console.h>

#include <bica/Component.h>

class TestA: public bica::Component
{
public:
	TestA()
	{
    addDependency("node_B");
    addDependency("node_C");
	}

	void step()
	{
		if(!isActive()) return;

		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_A");

	TestA test_a;

	ros::Rate loop_rate(10);
	while(test_a.ok())
	{
		test_a.step();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
```
* A BICA component inherits from bica :: Component
* Declare its dependencies with `addDependency (id)`. `id` is the name of the node that implements this component (you can match it with the name of your component for clarity)
* In your `main()`, create an instance of this class and call the frequency you want to your step () method. The first thing you do in this method is to check if it is active.

You can also implement your components in Python:

```
#!/usr/bin/env python

import rospy
from bica.bica import Component

class TestP(Component):
  def step(self):
    rospy.loginfo("[" +  rospy.get_name() + "] step")

if __name__ == '__main__':
  rospy.init_node('node_P', anonymous=False)
  rate = rospy.Rate(10) # 10hz

  test_p = TestP(10)

  while test_p.ok():
    pass
```

## FSM and BICA-GUI

Any of your components can be a finite state machine, even creating hierarchical state machines. We have developed a tool to create these components graphically. This tool automatically generates the C ++ code.

![alt text](https://raw.githubusercontent.com/fmrico/BICA/images/images/hfsm.png)
