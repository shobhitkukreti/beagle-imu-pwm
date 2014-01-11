direc=/sys/devices/ocp.3
pwm1=$direc/pwm_test_P8_13.17
pwm2=$direc/pwm_test_P8_19.18
pwm3=$direc/pwm_test_P9_14.15
pwm4=$direc/pwm_test_P9_16.16

#echo 0>$pwm1/run
echo 0 >$pwm1/polarity

#echo 0>$pwm2/run
echo 0 >$pwm2/polarity

#echo 0>$pwm3/run
echo 0 >$pwm3/polarity

#echo 0>$pwm4/run
echo 0 >$pwm4/polarity

echo 1000000 >$pwm1/duty
echo 1000000 >$pwm2/duty
echo 1000000 >$pwm3/duty
echo 1000000 >$pwm4/duty

#echo 1>$pwm1/run
#echo 1>$pwm2/run
#echo 1>$pwm3/run
#echo 1>$pwm4/run


sleep 2

#echo 1200000 >$pwm1/duty
#echo 1200000 >$pwm2/duty
echo  1000000 >$pwm3/duty
#echo 1200000 >$pwm4/duty

