# Info 
## repo labIAGI
slide e codice delle lezioni [qui](https://gitlab.com/tizianoGuadagnino/lab_ai_gi)

## contatti
mail assistente laboratorio: <guadagnino@diag.uniroma1.it>

# Setup srrg2_orazio
## software arduino
Per la scheda installare le librerie di **arduino**,
```shell
sudo apt-get install arduino-mk
```

ed aggiungere l'utente al gruppo `dialout` per leggere la seriale
```shell
sudo adduser <nome-utente> dialout
```

## orazio_host
Clonare la repository [srrg2_oratio](https://gitlab.com/srrg-software/srrg2_orazio), su gitlab.
```shell
cd <qualunque-cartella-sul-pc>
git clone https://gitlab.com/srrg-software/srrg2_orazio.git
git pull
```

Installare le seguenti librerie per compilare *orazio_host*
```shell
sudo apt-get install libwebsockets-dev libreadline-dev
```

compilare il *Makefile* nella cartella `host_build/`
```shell
cd host_build/
make
```

per info sui comandi disponibili, l'eseguibile orazio è in `host_build/`
```shell
./orazio -h
```

i comandi per avviare l'interfaccia html, dalla root della repo (controllare con *dmesg* che la scheda sia ttyACM0)
```shell
cd ../
./host_build/orazio -serial-device /dev/ttyACM0 -resource-path html
```

Per controllare il file della periferica
```shell
dmesg
```

# GDB E Debug

Per avviare `gdb` da `rosrun` usare il comando:

```s
rosrun --prefix 'gdb -ex run --args' laser_mapper laser_mapper_node
```

e per compilare con l'opzione `-g`

```s
catkin build laser_mapper --cmake-args -DCMAKE_BUILD_TYPE=Debug
``` 
