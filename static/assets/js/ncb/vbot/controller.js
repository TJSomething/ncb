/* jslint browser: true */
/* global THREE, $:false, _:false, console, mod */

'use strict';

var VBOT = VBOT ||  {};

// Note that this depends on the the robot being loaded
VBOT.controller = (function () {
    var capabilities,
        robot,
        actionQueue,
        worker;

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
        var capabilities = [];

        // 2d motion
        if (methodTest(robot, [
                'instantForward',
                'instantBackward',
                'instantLeft',
                'instantRight',
                'simultaneousTurnMove',
                'move',
                'getCollisionVector'
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
    }

    // Makes an object containing all "sensor" data
    function sense() {
        var sensors = {};

        if (capabilities.motion) {
            sensors.speed = robot.speed;
            sensors.angularVelocity = robot.angularVelocity;
            // TODO: Add collision resolution to robot
            sensors.collision = robot.collisionResolution;
            // TODO: Add odometer
            sensors.odometer = robot.odometer;
            sensors.compass = mod(90 - (robot.rotation.y * 180 / Math.PI), 360);
        }

        if (capabilities.pickUp) {
            // TODO: Make held objects public
            sensors.leftHandObject = robot.leftHandObject.name;
            sensors.rightHandObject = robot.rightHandObject.name;
        }

        if (capabilities.arms) {
            sensors.arms = {};
            sensors.arms.left = robot.getArmAngle('l');
            sensors.arms.right = robot.getArmAngle('r');

            // TODO: add arm velocities
        }

        if (capabilities.expressions) {
            sensors.expression = robot.expression;
        }

        // TODO: add vision

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

    function initActuationTemplate() {
        var template = {};

        if (capabilities.motion) {
            template.speed = function (speed) {
                robot.speed = speed;
            };
            template.angularVelocity = function (omega) {
                robot.angularVelocity = omega;
            };
        }

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
    }

    function test() {
        var sandbox = new Worker('assets/js/ncb/vbot/worker.js'),
            exampleReq = new XMLHttpRequest();

        exampleReq.addEventListener('load', function () {
            console.log('loaded example');
            sandbox.postMessage( { script: this.responseText, start: true } );
        });
        exampleReq.open('GET', 'assets/js/ncb/vbot/sample_script.js', true);

        function start(e) {
            console.log(e.data);

            sandbox.removeEventListener('message', start, false);
            sandbox.addEventListener('actuate', actuate, false);
        }

        sandbox.addEventListener('message', start, false);

        exampleReq.send();

        worker = sandbox;

        return sandbox;
    }

    function step(dt) {
        worker.postMessage({
            sensors: {},
            actionsCompleted: []
        });
    }

    function actuate(e) {
        console.log(e);
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

    return {
        init: init,
        test: test,
        step: step
    };
}());