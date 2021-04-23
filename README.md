# melex_os
ROS driver and interface for melex at PUT
on x86 machines start by downloading wget https://github.com/joan2937/pigpio/archive/master.zip and placing pigpio.h in usr/include
install libmodbus from apt repository

# 2. Connect Raspberry to wifi using command
* `sudo nano /etc/network/interfaces`

add the following lines to the file
```bash
auto wlan0
iface wlan0 inet dhcp
        wpa-ssid {ssid}
        wpa-psk  {password}

```

then Ctrl+o to save, Ctrl+x to close , then run 
* `sudo dhclient -v wlan0`

to check the connectivity run 
* `ping google.pl`
