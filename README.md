# BICA

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

[(https://img.youtube.com/vi/ozYrQdCbGA4/0.jpg)](https://www.youtube.com/watch?v=ozYrQdCbGA4)

