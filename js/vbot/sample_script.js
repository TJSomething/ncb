var speed = 0;
var w = 0;
function main () {
    if (sensors.keys.W) {
        speed = 1.7;
    } else if (sensors.keys.S) {
        speed = -1.7
    } else {
        speed = 0;
    }
    if (sensors.keys.A) {
        w = 1;
    } else if (sensors.keys.D) {
        w = -1;
    } else {
        w = 0;
    }
    if (w !== sensors.angularVelocity) {
        startTurningLeft(w);
    }
    if (speed !== sensors.speed) {
        setSpeed(speed);
    }
}

