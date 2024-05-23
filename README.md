# ROS2 Robotarm Simulation

## Code compileren
Om deze code te bouwen moet `colcon build` in deze directory uitgevoerd worden. Dus, als je `ls` uitvoert moet je `robotarm_interface` moeten zien.

## Dependencies
Om missende dependencies te voorkomen is het aangeraden om `rosdep install --from-paths src -y --ignore-src` uit te voeren vóór het bouwen van de code.

## Voorbeeld commando's
`ros2 topic pub --once /commands std_msgs/msg/String '{data: "#0P2000T1500\r"}'`