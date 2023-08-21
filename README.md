# Planning_Control_Carla
Planning-Control Test With Carla-Simulator
if you have your own planning alogrithm, you can use similiar loggic to test your planning alogrithm at simulator before going to test with the real car.

branch -- main test

Map: Town03

Tested Object: Planning and Control Algorithms

have obstacle: yes 

If you wish to test you planning and control algorithm in carla-simulator, then you need prepare the input of your planning algorithms from carla
(like get host state information function include x,y,v,a,heading) by implment some function using carla api, and you need to prarpe a scene where 
your alogrithms will be test.

and also explore in a real time system,how to schedule planning and control in order to let them work togther,  control run 10ms every time, and planning 100ms,and how to arrange them

