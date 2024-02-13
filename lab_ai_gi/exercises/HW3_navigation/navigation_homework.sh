sudo apt install ros-kinetic-gmapping ros-kinetic-map-server -y
cd ~/sources/srrg
git clone https://gitlab.com/srrg-software/srrg_localizer2d
git clone https://gitlab.com/srrg-software/srrg_localizer2d_ros
cd ~/workspaces/labaigi_ws/src
ln -s ~/sources/srrg/srrg_localizer2d .
ln -s ~/sources/srrg/srrg_localizer2d_ros .
cd ..
catkin build
