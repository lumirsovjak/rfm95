#!/bin/bash

SHELL=/bin/sh
PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin:/home/pi/script

wget -q -O /dev/null www.sovjak.cz/mail.php?predmet=RPi_kotelna_se_restartovalo

echo "poslal jsem mail pres sovjak.cz php mail ?predmetz rc.local kotelna"
