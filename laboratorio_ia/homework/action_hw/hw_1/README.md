# homework 1 - branch azione labIAGI
Esempi actionserver ros:
- http://wiki.ros.org/actionlib
- http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
- messaggi: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
- action serve: http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29

## Creare il package
Nel workspace di catkin eseguire:
```
$ catkin_create_pkg <nome> roscpp actionlib actionlib_msgs message_generation message_runtime std_msgs
```

Aggiungere nel `package.xml` 
```text
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>message_runtime</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <exec_depend>message_generation</exec_depend>
```

Modificare il `CMakeLists.txt` come segue:
```text
find_package(catkin REQUIRED COMPONENTS
  genmsg    #add by me
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)

add_action_files(
  DIRECTORY action
  FILES <NOME>.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs message_generation message_runtime roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

```

Scrivere inoltre il file .action e compilare; la compilazione genererà:
- file `.msg`, che si trovano in `${CATKIN_WORKSPACE}/devel/share/PackageName/msg`.
- file `.h`, che si trovano in `${CATKIN_WORKSPACE}/devel/include/PackageName`.

A questo punto scrivere i file `*_node.cpp` richiesti e aggiungere al `CMakeLists.txt`:
```
add_executable(
    ${PROJECT_NAME}_node
    src/*_node.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_node_node
    ${catkin_LIBRARIES}
)
```
Prolly darà errori di undefined reference a ogni cosa possibile quando farai `catkin build`, in quel caso prova a controllare i `target_link_libraries` nel `CMakeLists.txt`, devono essere uno per ogni eseguibile che generi.

~~Lancia `catkin clean --yes` e poi ribuilda daccapo, dovrebbe risolvere.~~

~~Guarda -> https://answers.ros.org/question/226719/undefined-reference-to-rosinit/~~
