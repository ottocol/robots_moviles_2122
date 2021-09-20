#dependencias de paquetes de Ubuntu
sudo apt install libusb-dev libftdi-dev python-is-python3 pyqt5-dev-tools

#dependencias de paquetes de Noetic
sudo apt install ros-noetic-openslam-gmapping ros-noetic-joy ros-noetic-base-local-planner ros-noetic-move-base

#directorio para los fuentes
mkdir $HOME/tb2_ws
cd $HOME/tb2_ws
#se baja los fuentes de los repos especificados en el .rosinstall
wstool init src ../tb2.rosinstall

#Ñapa para arreglar un problema de dependencias
rm -rf src/ar_track_alvar/ar_track_alvar

#Compilar.Tardará! :)
catkin_make_isolated
