# Cosa fare
Dare un'occhiata al [tutorial ROS](http://wiki.ros.org/navigation/Tutorials) sulla navigazione. Da sviluppare nella pratica quattro cose:
- registrare dati
- creare una mappa
- iniziare la localizzazione
- problemi comandi di posizione

# Homework
### registrazione dati
- Load a synthetic environment with stage (**willow-erratic.world**);
```shell
roscd stage_ros
rosrun stage_ros stageros world/willow-erratic.world 
```

- Move around th robot and record a *rosbag*, per muovere con joystick usare
```shell
rosrun srrg_joystick_teleop joy_teleop_node
```

per registrare la bag (bag prova `dataset/homework3.bag`).
```shell
rosbag record TOPIC1 [TOPIC2 ...] -O <nome_bag.bag>
```
Controllare con **Rviz** che la registrazione abbia funzionato.

### creazione mappa
- Generate a map using [Gmapping](http://wiki.ros.org/gmapping)
Mentre la bag sta runnando, lanciare *Gmapping* per generare la mappa nel topic `/map`
```shell
rosrun gmapping slam_gmapping scan:=base_scan
```

Per salvare la mappa usare il nodo *map_server* (lanciare il comando una volta terminata la bag),
```shell
rosrun map_server map_saver -f <map_name>
```
Mappa già creata con la bag di prova, in `dataset/map`.

- Publish the map using the *map_server* node, nel topic */map*
```shell
rosrun map_server map_server <map_name>.yaml
```

### localization 
- Publish a static transform between `/map` and `/odom`
- Start the localizer (**srrg_localizer2d_ros**)
```shell
rosrun srrg_localizer2d_ros srrg_localizer2d_node
```
Il nodo *srrg_localizer2d_ros* si occupa di calcolare la trasformata fra /map e /odom, e di pubblicarla sul topic /tf. 

### TODO
- Start the planner (**thin_navigation**)
```shell
rosrun thin_navigation thin_planner_node
```

- Open Rviz and add the map, the particle cloud and the path to the viewer
- From Rviz give an initial estimate of the robot position (2D pose estimate in the toolbar)
- From Rviz set a goal position on the map (2D nav goal in the toolbar)
- Hints: Give to the system a good initial guess about the robot location
