cmake_minimum_required(VERSION 3.10)
project(rover_description)

find_package(ament_cmake REQUIRED)

# Instalación de archivos .xacro
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Instalación de meshes (.dae)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Instalación de archivos de configuración
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Instalación de archivos de lanzamiento
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Instalación de configuraciones de RViz
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Instalación de descripciones de robots si hay una carpeta específica (opcional)
install(DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots
)

ament_package()
