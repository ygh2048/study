### code
#### PWM output in c51
***
    while(1)
    {
        if(!Inc)
        speed = speed > 0 ? speed - 1 : 0;
        if(!Dec)
        speed = speed < 500 ? speed +1 : 500;
        PWM = 1;
        delay(speed);
        PWM = 0;
        delay(500 - speed);
    }
***