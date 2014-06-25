/* jslint browser: true */
/* global THREE, $:false, _:false, console, mod */

// Note that this depends on the the robot being loaded
(function (global) {
    'use strict';
    
    global.VBOT = global.VBOT || {};
    var VBOT = global.VBOT;
    
    var capabilities,
        robot,
        actionQueue = [],
        worker,
        actionsCompleted = [];    

    function init() {
        robot = VBOT.robot;
        capabilities = getCapabilities();
    }

    // Tests if an object has all of a list of methods
    function methodTest(obj, methods) {
        return methods.every(function (method) {
            return typeof obj[method] === 'function';
        });
    }

    // We need the robots capabilities
    function getCapabilities() {
        var capabilities = {};

        // 2d motion
        if (methodTest(robot, [
                'instantForward',
                'instantBackward',
                'instantLeft',
                'instantRight',
                'simultaneousTurnMove',
                'move'
            ])) {
            capabilities.motion = true;
        }

        // Grabbing
        if (methodTest(robot, [
                'pickUpAtHand'
            ])) {
            capabilities.grab = true;
        }

        // Arm manipulation
        if (methodTest(robot, [
                'changeArmAngle',
                'flexShoulder',
                'adductShoulder',
                'rotateShoulder',
                'setArmAngle',
                'getArmAngle'
            ])) {
            capabilities.arms = true;
        }

        // Automatically pick stuff up
        if (methodTest(robot, [
                'pickUpNearest'
            ])) {
            capabilities.autoPickUp = true;
        }

        // If we can change expressions
        if (methodTest(robot, [
                'changeExpression'
            ])) {
            capabilities.expressions = true;
        }
        
        return capabilities;
    }

    // Makes an object containing all "sensor" data
    function sense() {
        var sensors = {};

        if (capabilities.motion) {
            sensors.speed = robot.speed;
            sensors.angularVelocity = robot.angularVelocity;
            // TODO: Add odometer
            //sensors.odometer = robot.odometer;
            sensors.compass = mod(90 - (robot.rotation.y * 180 / Math.PI), 360);
        }

        if (capabilities.pickUp) {
            // TODO: Make held objects public
            //sensors.leftHandObject = robot.leftHandObject.name;
            //sensors.rightHandObject = robot.rightHandObject.name;
        }

        if (capabilities.arms) {
            sensors.arms = {
                left: {
                    held: robot.leftHandObject && robot.leftHandObject.name
                },
                right: {
                    held: robot.rightHandObject && robot.rightHandObject.name
                }
                
            };

            // TODO: add arm velocities
        }

        if (capabilities.expressions) {
            sensors.expression = robot.expression;
        }

        sensors.camera = {
            data: VBOT.cameraData,
            width: VBOT.cameraWidth,
            height: VBOT.cameraHeight
        };

        sensors.collision = detectCollisions();

        return sensors;
    }

    /** A helper method that takes an object of functions that mirrors
     * the possible structure of an incoming object, tests for that
     * structure and executes code on the condition that that part
     * of the structure exists, with the found bit as the parameter
     *
     * Expected usage:
     *   executeFromStructure({
     *       walk: function (speed) { robot.walk(speed); },
     *       turn: function (angle) { robot.turn(angle); },
     *       jump: function () { robot.jump(); }
     *   }, { walk: 1.0, jump: true });
     *
     *
     * @param template the expected structure
     * @param obj the object being tested
     */
    function executeFromStructure(template, obj) {
        for (var key in template) {
            if (obj.hasOwnProperty(key)) {
                if (typeof template[key] === 'object' &&
                    typeof obj[key] === 'object') {
                    executeFromStructure(template[key], obj[key]);
                } else if (typeof template[key] === 'function') {
                    template[key](obj[key]);
                }
            }
        }
    }

    /*function initActuationTemplate() {
        var template = {};

        if (capabilities.motion) {
            template.speed = function (speed) {
                robot.speed = speed;
            };
            template.angularVelocity = function (omega) {
                robot.angularVelocity = omega;
            };
        }

            console.log(sense());
        if (capabilities.arms) {
            template.arms = {
                left: {
                    flex: function (speed) {
                        robot.arms.left.flex = speed;
                    },
                    adduct: function (speed) {
                        robot.arms.left.adduct = speed;
                    },
                    rotate: function (speed) {
                        robot.arms.left.rotate = speed;
                    }
                },
                right: {
                    flex: function (speed) {
                        robot.arms.right.flex = speed;
                    },
                    adduct: function (speed) {
                        robot.arms.right.adduct = speed;
                    },
                    rotate: function (speed) {
                        robot.arms.right.rotate = speed;
                    }
                }
            };
        }

        if (capabilities.grab) {
            template.arms.left
        }
    }*/

    function test() {
        var sandbox = new Worker('assets/js/ncb/vbot/worker.js'),
            exampleReq = new XMLHttpRequest();
        
        init();

        exampleReq.addEventListener('load', function () {
            console.log('loaded example');
            sandbox.postMessage( { script: this.responseText, start: true } );
        });
        exampleReq.open('GET', 'assets/js/ncb/vbot/sample_script.js', true);

        function start(e) {
            console.log(e.data);

            sandbox.removeEventListener('message', start, false);
            sandbox.addEventListener('message', actuate, false);
        }

        sandbox.addEventListener('message', start, false);

        exampleReq.send();

        worker = sandbox;

        return sandbox;
    }
    
    function removeCompletedAction(actionId) {
        actionQueue = actionQueue.filter(function (action) {
            return action.id !== actionId;
        });
    }

    function step(dt) {
        if (worker !== undefined) {
            // Delete all the completed actions from the action queue
            actionsCompleted.forEach(removeCompletedAction);

            worker.postMessage({
                sensors: sense(),
                actionsCompleted: actionsCompleted,
                step: dt
            });

            // Clear completed actions
            actionsCompleted = [];
        }
    }

    function actuate(e) {
        if (e.data.error) {
            console.log(e.data.error);
        }
        if (e.data !== undefined &&
            e.data.hasOwnProperty('step')) {
            var step = e.data.step;

            if (e.data.actions) {
                e.data.actions.forEach(function (action) {
                    console.log(action.action);
                });
                actionQueue = actionQueue.concat(e.data.actions);
            }

            actionQueue.forEach(function (action) {
                stepAction(action, e.data.step);
            });
        }
    }

    function stepAction(action, dt) {
        var remainingAngle,
            targetLoc,
            target,
            stepAngle,
            stepAmount,
            done;

        // If the action has no speed or duration, then let's just set the
        // speed to 1
        if (!action.hasOwnProperty('speed') &&
            !action.hasOwnProperty('time')) {
            action.speed = 1;
        }

        switch (action.action) {
            case 'turnTowards':
                // Calculate how much to turn
                target = VBOT.scene.getObjectByName(action.objName, true);
                // Target location in local space
                targetLoc = VBOT.robot.worldToLocal(
                    target.positionWorld());
                remainingAngle = Math.atan2(targetLoc.x, targetLoc.z);
                // Calculate how much we need to turn
                if (action.hasOwnProperty('speed')) {
                    stepAngle = action.speed * dt * Math.sign(remainingAngle);
                } else if (action.hasOwnProperty('timeLeft')) {
                    stepAngle = remainingAngle * dt / action.timeLeft;
                    action.timeLeft -= dt;
                } else if (action.hasOwnProperty('time')) {
                    stepAngle = remainingAngle * dt / action.time;
                    action.timeLeft = action.time - dt;
                } else {
                    throw "Logic error in turnTowards";
                }
                // Make sure that we don't over turn
                if (Math.sign(remainingAngle - stepAngle) !== 
                    Math.sign(remainingAngle)) {
                    stepAngle = remainingAngle;
                }
                // Turn it
                VBOT.robot.rotateY(stepAngle);
                // If we're done, note that
                if (remainingAngle - stepAngle === 0) {
                    actionsCompleted.push(action.id);
                }
                break;
            case 'setSpeed':
                VBOT.robot.speed = action.speed;
                actionsCompleted.push(action.id);
                break;
            case 'turn':
                VBOT.robot.angularVelocity = action.speed;
                actionsCompleted.push(action.id);
                break;
            case 'pointArmAt':
                target = VBOT.scene.getObjectByName(action.objName, true);
                targetLoc = target.centerWorld();
                done = false;
                if (action.hasOwnProperty('speed')) {
                    stepAngle = action.speed * dt;
                    done = VBOT.robot.rotateArmTowards(action.arm,
                                                       targetLoc,
                                                       stepAngle,
                                                       true,
                                                       true);
                } else if (action.hasOwnProperty('timeLeft') ||
                           action.hasOwnProperty('time')) {
                    if (action.hasOwnProperty('timeLeft')) {
                        stepAmount = dt / action.timeLeft;
                        action.timeLeft -= dt;
                    } else if (action.hasOwnProperty('time')) {
                        stepAmount = dt / action.time;
                        action.timeLeft = action.time - dt;
                    } else {
                        throw "Insane logic error in pointArmAt";
                    }
                    done = VBOT.robot.rotateArmTowards(action.arm,
                                                       targetLoc,
                                                       stepAmount,
                                                       false,
                                                       true);
                } else {
                    throw "pointArmAt is missing any indication of speed or duration";
                }
                if (done) {
                    actionsCompleted.push(action.id);
                }
                break;
            case 'grab':
                console.log('go');
                robot.pickUpAtHand(action.arm);
                actionsCompleted.push(action.id);
                break;
            case 'pointArm':
                targetLoc = THREE.Vector3.prototype.clone.call(action.direction);
                done = false;
                if (action.hasOwnProperty('speed')) {
                    stepAngle = action.speed * dt;
                    done = VBOT.robot.rotateArmTowards(action.arm,
                                                       targetLoc,
                                                       stepAngle,
                                                       true,
                                                       false);
                } else if (action.hasOwnProperty('timeLeft') ||
                           action.hasOwnProperty('time')) {
                    if (action.hasOwnProperty('timeLeft')) {
                        stepAmount = dt / action.timeLeft;
                        action.timeLeft -= dt;
                    } else if (action.hasOwnProperty('time')) {
                        stepAmount = dt / action.time;
                        action.timeLeft = action.time - dt;
                    } else {
                        throw "Insane logic error in pointArmAt";
                    }
                    done = VBOT.robot.rotateArmTowards(action.arm,
                                                       targetLoc,
                                                       stepAmount,
                                                       false,
                                                       false);
                } else {
                    throw "pointArm is missing any indication of speed or duration";
                }
                if (done) {
                    actionsCompleted.push(action.id);
                }
                break;
            default:
                throw action.action + ' is not available';
        }
    }

    function detectCollisions() {
        var collisions = {
                front: false,
                back: false,
                left: false,
                right: false,
                top: false,
                bottom: false
            };

        function setCollision(collision) {
            var axis,
                maxAxis,
                maxAxisLength = -Infinity,
                normal = collision.contactNormal.clone();

            normal.transformDirection(VBOT.robot.matrixWorld);

            for (axis = 0; axis < 3; axis += 1) {
                if (Math.abs(normal.getComponent(axis)) >
                        maxAxisLength) {
                    maxAxis = axis;
                    maxAxisLength = Math.abs(
                        normal.getComponent(axis));
                }
            }

            if (maxAxis === 0) {
                if (normal.x > 0) {
                    collisions.right = true;
                } else {
                    collisions.left = true;
                }
            } else if (maxAxis === 1) {
                if (normal.y > 0) {
                    collisions.bottom = true;
                } else {
                    collisions.top = true;
                }
            } else if (maxAxis === 2) {
                if (normal.z > 0) {
                    collisions.front = true;
                } else {
                    collisions.back = true;
                }
            }
        }

        VBOT.robot.collisions.forEach(setCollision);

        return collisions;
    }
    
    VBOT.controller = {
        init: init,
        test: test,
        step: step
    };
}(this));