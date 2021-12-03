//replace this with the normal !opModeIsActive() loop
//Tiernan Lindauer
int ones = 0;
int twos = 0;
int threes = 0;

while (!opModeIsActive()) {
    int bLevel = getLevel();
    if(bLevel == 1)
        ones++;
    else if(bLevel == 2)
        twos++;
    else
        threes++;
}

if(ones > 1)
    level = 1;
else if(twos > 1)
    level = 2;
else
    level = 3;

telemetry.addData("DETECTED LEVEL: ",level);
telemetry.update();
