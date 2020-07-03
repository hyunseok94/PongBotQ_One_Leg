
######################################################################################################
################# Package : ROS(kinetic) + SOEM(ROS) + ELMO + Xenomai-3.0.5 +Eigen ###################
######################################################################################################



< Package Execution Command order >
 - typing the codes on termianl in order
___________________________________________________________________________________ 
Main. execution
(0) roscore
(1) cm
(2) cd ~/catkin_ws/src/PongBotQ_One_Leg/bin/ && sudo setcap cap_net_admin,cap_net_raw=eip elmo_pkgs.exe
(3) rosrun elmo_pkgs elmo_pkgs.exe
___________________________________________________________________________________
Example. xenomai_test execution
(0) cm
(1) cd ~/catkin_ws/src/PongBotQ_One_Leg/examples/xenomai_test/bin/ && sudo setcap cap_net_admin,cap_net_raw=eip xenomai_test.exe
(2) sudo ./xenomai_test.exe
___________________________________________________________________________________
Example. soem_test execution
(0) cm
(1) cd ~/catkin_ws/src/PongBotQ_One_Leg/examples/soem_test/bin/ && sudo setcap cap_net_admin,cap_net_raw=eip soem_test.exe
(2) sudo ./soem_test.exe


##############################################################################

Error : libalchemy.so.0: cannot open shared object file: No such file or directory

Solution : 
 ldd ./<file_name>           : Confirm the header file in the list. (Run the codes )
                               => ldd ./libalchemy.so.0 이 없음을 확인?

 cd /etc/ld.so.conf.d        : Move to "/etc/ld.so.conf.d" directory.
 sudo gedit tspi.conf        : Make "tspi.conf" file

 /usr/xenomai/lib            : typing the location of directory where "libalchemy.so.0 file" exist in 

 sudo ldconfig               : apply the changes


###############################################################################

ctrl+c : program exit
ctrl+z : program stop
