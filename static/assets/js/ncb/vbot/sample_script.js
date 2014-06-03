function start () {
    turnTowards('portable_table').then('goToTable');
    next('wait');
}

function wait() {}

function goToTable () {
    setSpeed(1.0);
    next('goingToTable');
}

function goingToTable () {
    // Check if we've hit anything yet
    if (sensors.collision.front === true) {
        setSpeed(0.0);
        next('pointToTable');
    }
}

function pointToTable () {
    pointRightArmAt('portable_table').then('grabTable');
    next('wait');
}

function grabTable () {
    grabWithRightArm().then('didWeGrabTheTable');
    next('wait');
}

function didWeGrabTheTable () {
    if (sensors.arms.right.held === 'portable_table') {
        // If we have the table, then let's raise it to the sky,
        // smile, then search for the road
        pointRightArm(0.0, 1.0, 0.0). // Point arm up
            over(1.0).                // Over a second
            then('search for road');
        setExpression('smile');
    }
    next('wait');
}

function isGray(pixel) {
    return pixel.lightness < 0.3 && pixel.saturation < 0.3;
}

function searchForRoad () {
    // When both of the bottom pixels are road, then we're good
    var leftPixel = getPixel(-0.1, -1.0),
        rightPixel = getPixel(0.1, -1.0),
        leftIsGray = isGray(leftPixel),
        rightIsGray = isGray(rightPixel);
    if (leftIsGray && rightIsGray) {
        setSpeed(1.0);
        next('travelDownTheRoad');
    }
}

function travelDownTheRoad () {
    // If the pixels at the bottom of the camera aren't dark gray,
    // then we need to find the road again
    var leftPixel = getPixel(-0.1, -1.0),
        rightPixel = getPixel(0.1, -1.0),
        leftIsGray = isGray(leftPixel)
        rightIsGray = isGray(rightPixel);
    if (leftIsGray)  {
        if (rightIsGray) {
            // If they're both gray, everything is fine
        } else {
            // If only the left is road, then we need to turn left
            startTurningLeft(1.0);
            next('searchForRoad');
        }
    } else {
        if (rightIsGray) {
            // If only the right is road, then we need to turn right
            startTurningRight(1.0);
            next('searchForRoad');
        } else {
            // If neither are road, we'll search in a random
            // direction
            if (Math.random() < 0.5) {
                startTurningRight(1.0);
                next('searchForRoad');
            } else {
                startTurningLeft(1.0);
                next('searchForRoad');
            }
        }
    }
}
