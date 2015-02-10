function main () {
    if (sensors.keys.W) {
        setSpeed(1.7);
    } else if (sensors.keys.S) {
        setSpeed(-1.7);
    } else {
        setSpeed(0);
    }
    if (sensors.keys.A) {
        startTurningLeft(1);
    } else if (sensors.keys.D) {
        startTurningRight(1);
    } else {
        startTurningLeft(0);
    }
}

