To make
------
$make

$cp tty2comport /usr/bin

Edit .bash_profile to add,

case $DIRECTPORT in
 "USB0") /usr/bin/tty2comport $DIRECTPORT;;
esac


note: "S0" Can be use of it is for ttyS0. The example is for ttyUSB0


