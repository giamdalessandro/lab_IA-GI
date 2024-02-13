# Cosa fare
In questo homework dobbiamo creare un nostro `node`.

- [x] Copiare la cartella `laser_scanner` da `lab_ai_gi` alla nostra repo
- Inserire il link simbolico di questa cartella nella `workspaces` di `catkin`,
lanciando il comando

```shell
cd ~/workspaces/labaigi_ws/src
ln -s ~/university/laboratorio_ia/homework/hw_2/laser_mapper/ laser_mapper
```

quindi creando il link simbolico riusciamo ad avere i file sulla nostra repo e versionarli
ed in più li compiliamo dentro il workspaces **suggerito da Grisetti**.

- Compilare il tutto utilizzando il comando

```shell
catkin build
```

## Esecuzione 
Dopo la compilazione eseguire lo script `setup.bash` nella cartella *devel/*, dentro il workspace 
labaigi_ws;

```shell
cd ~/workspaces/labaigi_ws
source devel/setup.bash
```

una volta eseguito verificare la visibilità del nuovo package con il comando *rospack list*.

### Testing soluzione
- Usare un robot reale
	
- Usare l'emulatore 2D `stage`, (il file *.world* può essere cambiato):

```shell
roscd stage_ros
rosrun stage_ros stageros world/willow-erratic.world 
```
	
- opppure utilizzare `bag` creata da *Tiziano Guadagnino*: **sol. più facile**
	- runnare *roscore*;
	- runnare la bag con il comando *rosbag play <path/to/bag_name>* (quella dell' homework è in `lab_ai_gi/datasets`, vedere README);
	- runnare il nodo con *rosrun <package_name> <node_name>*

```shell
roscore
rosbag play lab_ai_gi/datasets/diag_corridor.bag
rosrun laser_mapper laser_mapper_node
```
