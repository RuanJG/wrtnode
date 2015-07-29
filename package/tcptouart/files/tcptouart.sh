#!/bin/sh /etc/rc.common
# Copyright (C) 2006-2011 OpenWrt.org

START=99
USE_PROCD=1

start_service() {
	local ip=$(ifconfig br-lan  | grep 'inet addr')
	ip=${ip#*'inet addr:'}
	ip=${ip%' Bcast:'*}
        echo "start tcpuart server, in $ip"
        /bin/tcptouart $ip 6666 /dev/ttyUSB0 57600 &
}
stop_service() {
        echo "stop tcpuart server"
        killall tcptouart
}

