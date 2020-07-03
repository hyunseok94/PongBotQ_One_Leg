#! /bin/bash
read -s -p "Enter Password for sudo: " sudoPW
echo $sudoPW | sudo -S ifconfig eth1 down
echo $sudoPW | sudo -S ifconfig eth0 down
echo $sudoPW | sudo -S ifconfig rteth0 down
echo $sudoPW | sudo -S /usr/xenomai/sbin/rtifconfig rteth0 down
echo $sudoPW | sudo -S modprobe -r e1000e
echo $sudoPW | sudo -S modprobe -r igb
echo $sudoPW | sudo -S modprobe -r rtcap
echo $sudoPW | sudo -S modprobe -r rtpacket
echo $sudoPW | sudo -S modprobe -r rt_igb
echo $sudoPW | sudo -S modprobe -r rt_e1000e
echo $sudoPW | sudo -S modprobe -r rtnet
echo $sudoPW | sudo -S modprobe rtnet
echo $sudoPW | sudo -S modprobe rt_igb
echo $sudoPW | sudo -S modprobe rt_e1000e
echo $sudoPW | sudo -S modprobe rtpacket
echo $sudoPW | sudo -S modprobe rtcap
echo $sudoPW | sudo -S /usr/xenomai/sbin/rtifconfig rteth0 up
echo $sudoPW | sudo -S ifconfig rteth0 up
echo $sudoPW | sudo -S lsmod | grep rt
echo $sudoPW | sudo -S ifconfig

