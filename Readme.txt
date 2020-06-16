
######################################################################################################
################# Package : ROS(kinetic) + SOEM(ROS) + ELMO + Xenomai-3.0.5 +Eigen ###################
######################################################################################################



< Packaged Command order >
 - typing the codes on termianl in order
 
(0) roscore
(1) cm
(2) cd src/PongBotQ_One_Leg/bin/ && sudo setcap cap_net_admin,cap_net_raw=eip elmo_pkgs.exe
(3) rosrun elmo_pkgs elmo_pkgs.exe


##############################################################################

< Solution for Error(error while loading shared libraries: libalchemy.so.0) >

 cd /etc/ld.so.conf.d        : Move to "/etc/ld.so.conf.d" directory.
 sudo gedit tspi.conf        : Make "tspi.conf" file

 /usr/xenomai/lib            : typing the location of directory where "libalchemy.so.0 file" exist in 

 sudo ldconfig               : apply the changes


###############################################################################

ctrl+c : program exit
ctrl+z : program stop
