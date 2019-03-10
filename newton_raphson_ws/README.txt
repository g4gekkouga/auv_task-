Implementing Newton Raphson Method

Compilation :
	
1) Create a catkin workspace

2) Copy the enitre NewtonRaphson Directory present in src/ to your catkin_workspace/src directory

3) Build the package using catkin_make from the workspace directory

Note :

1) rosrun NewtonRaphson NewtonRaphson_client – To run the client

2) rosrun NewtonRaphson NewtonRaphson_server – To run the server

3)  rostopic echo /newton_raphson/feedback – To get the feedback status (Server name is newton_raphson) and similarly for result.

Changes :

1) Initial Guess can be changed in the NewtonRaphson_client.cpp by changing the #define tag value 

2) The Cubic Function can also be changed by changing the #define tag values in  NewtonRaphson_server.cpp
