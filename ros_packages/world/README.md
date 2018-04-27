#World

Este pacote possui dois programas executáveis, o nó "map creator" e o nó "world".

##Map Creator:
Este nó permite a criação de um mapa para uso com o navigation_stack do ROS. É utilizado uma única vez para o setup da tarefa/mapa.
### Instruções:
1 - Rode-o com: roslaunch world map_creator.launch
2 - Verifique se o mapa está correto com o RVIZ.
3 - rode rosrun map_server map_saver

##World node:
Este nó publica o estado (posições no mapa, posições no goal, cor do(s) puck(s) de cada máquina e DC, ou seja, condições de inicialização), para uso em conjunto com o resto dos pacotes.

###Instruções:
Apenas rode esse nó antes do machine learning no arquivo .launch que inicializa o robô.
Utilize-o no nó machine learning para fazer a inicialização do primeiro estado
(apenas subscreva e atribua as informações relevantes para as variáveis privadas do node machine_learning).

##Dependências:
#### ROS:
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  cv_bridge
  filters
  tf
  actionlib_msgs
  message_generation

#### Third Party:

Eigen3
Boost
octomap

