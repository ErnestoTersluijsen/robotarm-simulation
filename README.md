# ROS2 Robotarm Simulation

## Code compileren
Om deze code te bouwen moet `colcon build` in deze directory uitgevoerd worden. Dus, als je `ls` uitvoert moet je `robotarm_interface` en/of `RUN COLCON BUILD HERE.txt` zien.

## Dependencies
Om missende dependencies te voorkomen is het aangeraden om `rosdep install --from-paths src -y --ignore-src` uit te voeren vóór het bouwen van de code.

## Demo draaien
Om de demo te draaien, draai het `demo_script.sh` script. Deze compileerd, sourced en draait de applicatie. Deze voert ook een paar commando's uit om de functionaliteit van het programma te testen. Zo wordt de mok opgepakt en laat de robotarm deze weer vallen.

Hieronder staat beschreven wat je moet typen in je terminal om het script uit te voeren:

`. demo_script.sh`

## Robotarm bewegen
Om de robotarm te bewegen moet eerst de code gecompileerd zijn, waarna het setup.sh script gesourced kan worden. Zodra dit gedaan is kan de launch file opgestart worden. Hieronder zijn de commando's genoteerd:

`. install/setup.sh`

`ros2 launch robotarm_simulation robotarm_simulation.launch.py`

Om commando's te sturen naar de robotarm start een nieuwe terminal op en stuur een commando voor de robotarm tussen de aanhalingstekens. Hieronder staat een voorbeeld om de base van de robotarm te draaien:

`ros2 topic pub --once /commands std_msgs/msg/String '{data: "#0P2000T1500\r"}'`

Het stop commando voor de robotarm is als volgt:

`ros2 topic pub --once /commands std_msgs/msg/String '{data: "STOP 0\r"}'`

Hierbij is de 0 het ID van de servo die gestopt moet worden.