#!/bin/sh

while [ '1' == '1' ]
do
	gateway=$(route -n |grep '192.168.1.1')
	if [ "$gateway"'x' == 'x' ];then
		echo try to set gateway
	else
		sleep 10
		echo recheck gateway
		continue
	fi
	ip=$(ifconfig apcli0 | grep '192.168.1.108')
	if [ "$ip"'x' == 'x' ];then
		sleep 2
	else
		echo set gateway 192.168.1.1
		route add default gw 192.168.1.1 
		#route add -net 0.0.0.0 gw 192.168.0.1；
	fi
	sleep 1
done
