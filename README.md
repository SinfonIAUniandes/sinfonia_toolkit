<<<<<<< HEAD
# Robot Toolkit para Pepper

Documentación en https://gitlab.com/sinfonia-sspl-robocup-2020/sinfonia-robot-toolkit/robot_toolkit/-/wikis/home

## Documentación Antigua...

### Compilar con Kdevelop

1. En una consola ejecutar `kdevelop`
2. En Project, click en Open / Import Project...
3. Seleccionar el CMakeLists.txt de la carpeta src, click en siguiente
4. Cambiar src por robot_toolkit
5. En Extra Arguments poner: `-DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install`
6. Con el boton de build comprobar que la compilacion sea exitosa. 

### Uso de `robot_toolkit.sh`

Usando el bash `robot_toolkit.sh`  usted podra:
1. Instalar sshpass: `sudo apt-get install sshpass`
2. Compilar el codigo fuente en la maquina virtual `virtual_nao`
3. Instalar los binarios sobre el robot
4. Ejecutar el robot_toolkit en el robot
5. Detener el robot_toolkit en el robot

Para mayor informacion ejecute `./robot_toolkit.sh --help`
=======
# sinfonia_toolkit
ROS package to manage and communicate with the hardware components of the Pepper robot
>>>>>>> 47f5e8a907ae6390e7dd377a1fabd77d18ab027c
