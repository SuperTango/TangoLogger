#!/bin/sh
/mnt/sda1/TangoLoggerUploader/on.py
/sbin/uci set  network.lan=interface
/sbin/uci set  network.lan.proto=dhcp
/sbin/uci delete  network.lan.ipaddr
/sbin/uci delete  network.lan.netmask
/sbin/uci set wireless.@wifi-iface[0].mode=sta
/sbin/uci set wireless.@wifi-iface[0].ssid=StartX
/sbin/uci set wireless.@wifi-iface[0].encryption=none
/sbin/uci commit wireless; /sbin/wifi
/etc/init.d/network  restart
/mnt/sda1/TangoLoggerUploader/off.py
