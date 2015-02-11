/* jslint browser: true */
/* global THREE, $:false, _:false, console, mod */

define(['vbot/three', 'vbot/utils'],
function (THREE, utils) {
    'use strict';

    /** @exports vbot/controller */

    var capabilities,
        robot,
        actionQueue = [],
        worker,
        actionsCompleted = [],
        app,
        keys = {},
        sensors = {};

    /**
     * Loads some useful objects into the controller scope.
     *
     * @memberof module:vbot/controller
     * @param {Object} vbotApp the virtual robot application state
     */
    function init(vbotApp) {
        if (!app) {
            app = vbotApp;
            robot = app.robot;
            capabilities = getCapabilities();
            initKeyboardListener();
        }
    }

    /**
     * Tests if an object has all of an array of methods.
     *
     * @memberof module:vbot/controller~
     * @param  {Object} obj             the object being tested
     * @param  {Array.<string>} methods the methods to check for
     * @return {Boolean}                if all of those methods exist
     */
    function methodTest(obj, methods) {
        return methods.every(function (method) {
            return typeof obj[method] === 'function';
        });
    }

    /**
     * Tests if the robot loaded has certain capabilities.
     *
     * Tested capabilities are:
     * - motion
     * - grab
     * - arms
     * - autoPickUp
     * - expressions
     *
     * @memberof module:vbot/controller~
     * @return {Object.<string,boolean>} an object where the capabilities are
     *                                   the keys and the values are all true
     */
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
                'rotateArmTowards',
                'instantPointArm'
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

    /**
     * Holds all of the sensor data.
     *
     * @typedef SenseData
     * @memberof module:vbot/controller~
     * @property {number} speed the current speed in m/s
     * @property {number} angularVelocity the current angular velocity in
     *                                    radians per second
     * @property {number} odometer the distance travelled in meters
     * @property {number} compass the current faced angle in degrees as a
     *                            header
     * @property {object} arms
     * @property {object} arms.left
     * @property {string} arms.left.held the name of the object in the left
     *                                   hand
     * @property {object} arms.right
     * @property {string} arms.right.held the name of the object in the left
     *                                    hand
     * @property {string} expression the current expression
     * @property {object} camera
     * @property {Int8Buffer} camera.data the raw RGBA camera data
     * @property {number} camera.height the height of the camera image
     * @property {number} camera.width the width of the image
     * @property {object} collision the collision data for each side
     *                              of the robot
     * @property {boolean} collision.top whether there is a collision on the
     *                                   top
     * @property {boolean} collision.bottom whether there is a collision on the
     *                                      bottom
     * @property {boolean} collision.left whether there is a collision on the
     *                                    left
     * @property {boolean} collision.right whether there is a collision on the
     *                                     right
     * @property {boolean} collision.front whether there is a collision on the
     *                                     front
     * @property {boolean} collision.back whether there is a collision on the
     *                                    back
     * @property {object} keys a map from keys to whether they are pressed.
     *                         The unpressed value may be undefined or false.
     */

    /**
     * Builds an object containing all the sensor data.
     *
     * Existing sensors are:
     * - speed
     * - angularVelocity
     * - compass
     * - arms
     *   + left and right
     *     * held: the name of the held object in a given hand
     * - expression
     * - camera
     *   + data: Int32Buffer of camera data
     *   + height
     *   + width
     * - collision
     *   + top
     *   + bottom
     *   + front
     *   + back
     *   + left
     *   + right
     *
     * @memberof module:vbot/controller~
     * @return {object} the sensor data
     */
    function sense() {
        function calcCompass() {
            var forward = new THREE.Vector4(1,0,0,0).applyMatrix4(robot.matrixWorld);
            var rad = Math.atan2(forward.x, forward.z);
            return rad/Math.PI * 180 + 180;
        }

        var sensors = {};

        if (capabilities.motion) {
            sensors.speed = robot.speed;
            sensors.angularVelocity = robot.angularVelocity;
            sensors.odometer = robot.odometer;
            sensors.compass = calcCompass();
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
            data: app.cameraData,
            width: app.cameraWidth,
            height: app.cameraHeight
        };

        sensors.collision = detectCollisions();

        sensors.keys = keys;

        return sensors;
    }

    /**
     * A helper method that takes an object of functions that mirrors
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
     * @memberof module:vbot/controller~
     * @param {object} template the expected structure
     * @param {object} obj the object being tested
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

    /**
     * Starts up the controller
     *
     * @memberof module:vbot/controller
     * @param {Object} vbotApp the virtual robot application state
     * @param {string} script the controller script code
     * @return {Worker} The web worker executing the script
     */
    function start(vbotApp, script) {
        var sandbox = new Worker('assets/js/vbot_worker.js');

        init(vbotApp);

        function start(e) {
            console.log(e.data);

            sandbox.removeEventListener('message', start, false);
            sandbox.addEventListener('message', actuate, false);
        }

        sandbox.addEventListener('message', start, false);
        sandbox.postMessage( { script: script, start: true });

        worker = sandbox;

        return sandbox;
    }

    /**
     * Removes an action from the list of actions in progress. This is used
     * when an action is completed.
     *
     * @memberof module:vbot/controller~
     * @param  {string} actionId the ID number of the action
     */
    function removeCompletedAction(actionId) {
        actionQueue = actionQueue.filter(function (action) {
            return action.id !== actionId;
        });
    }

    /**
     * Steps the worker by:
     * - Removing completed actions from the list of actions in progress
     * - sending the completed actions, the sensor data, and the time since the
     *   last step to the controller
     *
     * @memberof module:vbot/controller
     * @param  {Number} dt the time step length
     */
    function step(dt) {
        sensors = sense();
        if (worker !== undefined) {
            // Delete all the completed actions from the action queue
            actionsCompleted.forEach(removeCompletedAction);

            worker.postMessage({
                sensors: sensors,
                actionsCompleted: actionsCompleted,
                step: dt
            });

            // Clear completed actions
            actionsCompleted = [];
        }
    }

    /**
     * Actuates the robot based on what the controller says to do.
     *
     * @memberof module:vbot/controller~
     * @param  {Object} e the event from the controller worker
     */
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

    function findGraspingLocation(arm, target) {
        var isLeft = arm.toLowerCase()[0] === 'l',
            armObj = isLeft ? robot.larm : robot.rarm,
            targetObj = robot.app.scene.getObjectByName(target, true),
            wristPos = robot.calculateHandLocation(armObj),
            palmSphere = new THREE.Box3().
                setFromObject(targetObj).
                getBoundingSphere(),
            wristSphere = new THREE.Sphere(palmSphere.center,
                palmSphere.radius + 0.03),
            targetPalmPos = palmSphere.clampPoint(wristPos),
            targetWristPos = wristSphere.clampPoint(wristPos);

        return {
            wrist: targetWristPos,
            palm: targetPalmPos,
            center: palmSphere.center
        };
    }

    /**
     * Given an action in progress, actuate the robot over one time step,
     * executing a portion of that action.
     *
     * @memberof module:vbot/controller~
     * @param  {scripting~Action} action the action in progress
     * @param  {Number} dt     the length of the time step
     */
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
                target = app.scene.getObjectByName(action.objName, true);
                // Target location in local space
                targetLoc = app.robot.worldToLocal(
                    target.centerWorld());
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
                app.robot.rotateY(stepAngle);
                // If we're done, note that
                if (remainingAngle - stepAngle === 0) {
                    actionsCompleted.push(action.id);
                }
                break;
            case 'setSpeed':
                app.robot.speed = action.speed;
                actionsCompleted.push(action.id);
                break;
            case 'turn':
                app.robot.angularVelocity = action.speed;
                actionsCompleted.push(action.id);
                break;
            case 'pointArmAt':
                target = app.scene.getObjectByName(action.objName, true);
                targetLoc = target.centerWorld();
                done = false;
                if (action.hasOwnProperty('speed')) {
                    stepAngle = action.speed * dt;
                    done = app.robot.rotateArmTowards(action.arm,
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
                    done = app.robot.rotateArmTowards(action.arm,
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
                robot.pickUpAtHand(action.arm);
                actionsCompleted.push(action.id);
                break;
            case 'pointArm':
                targetLoc = THREE.Vector3.prototype.clone.call(action.direction);
                done = false;
                if (action.hasOwnProperty('speed')) {
                    stepAngle = action.speed * dt;
                    done = app.robot.rotateArmTowards(action.arm,
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
                    done = app.robot.rotateArmTowards(action.arm,
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
            case 'reachForAndGrab':
                targetLoc = findGraspingLocation(action.arm, action.target);
                done = robot.stepHandTowardsTarget(action.arm,
                                                   0.1,
                                                   action.speed * dt,
                                                   targetLoc.wrist,
                                                   targetLoc.palm,
                                                   targetLoc.center);
                if (done === 'success') {
                    app.robot.grab(app.scene.getObjectByName(action.target, true),
                                   action.arm);
                    console.log(app.robot.rightHandObject);
                }
                if (done === 'success' ||
                    done === 'failure') {
                    console.log(done);
                    actionsCompleted.push(action.id);
                }
                break;
            default:
                throw action.action + ' is not available';
        }
    }

    /**
     * Makes an object consisting of whether or not there is a collision in
     * any of the six directions for the robot.
     *
     * These directions are:
     * - top
     * - bottom
     * - left
     * - right
     * - front
     * - back
     *
     * @memberof module:vbot/controller~
     * @return {module:vbot/controller~RobotCollisions} an object where each direction is a key and
     *                           the values are whether there is a collision
     *                           from that direction
     */
    function detectCollisions() {
        var collisions = {
                front: false,
                back: false,
                left: false,
                right: false,
                top: false,
                bottom: false
            };

        /**
         * Given a collision that the robot is experiencing, set that that
         * collision is occuring on the RobotCollisions object that we're
         * building.
         *
         * @param {module:vbot/physics~Collision} collision a collision the robot is
         *                                   experiencing

         */
        function setCollision(collision) {
            var axis,
                maxAxis,
                maxAxisLength = -Infinity,
                normal = collision.contactNormal.clone();

            normal.transformDirection(app.robot.matrixWorld);

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

        app.robot.collisions.forEach(setCollision);

        return collisions;
    }

    function initKeyboardListener() {
        var otherKeys = {
            8: "backspace", //  backspace
            9: "tab", //  tab
            13: "enter", //  enter
            16: "shift", //  shift
            17: "ctrl", //  ctrl
            18: "alt", //  alt
            19: "pause/break", //  pause/break
            20: "caps lock", //  caps lock
            27: "escape", //  escape
            33: "page up", // page up, to avoid displaying alternate character and confusing people
            34: "page down", // page down
            35: "end", // end
            36: "home", // home
            37: "left arrow", // left arrow
            38: "up arrow", // up arrow
            39: "right arrow", // right arrow
            40: "down arrow", // down arrow
            45: "insert", // insert
            46: "delete", // delete
            91: "left window", // left window
            92: "right window", // right window
            93: "select key", // select key
            96: "numpad 0", // numpad 0
            97: "numpad 1", // numpad 1
            98: "numpad 2", // numpad 2
            99: "numpad 3", // numpad 3
            100: "numpad 4", // numpad 4
            101: "numpad 5", // numpad 5
            102: "numpad 6", // numpad 6
            103: "numpad 7", // numpad 7
            104: "numpad 8", // numpad 8
            105: "numpad 9", // numpad 9
            106: "multiply", // multiply
            107: "add", // add
            109: "subtract", // subtract
            110: "decimal point", // decimal point
            111: "divide", // divide
            112: "F1", // F1
            113: "F2", // F2
            114: "F3", // F3
            115: "F4", // F4
            116: "F5", // F5
            117: "F6", // F6
            118: "F7", // F7
            119: "F8", // F8
            120: "F9", // F9
            121: "F10", // F10
            122: "F11", // F11
            123: "F12", // F12
            144: "num lock", // num lock
            145: "scroll lock", // scroll lock
            186: ";", // semi-colon
            187: "=", // equal-sign
            188: ",", // comma
            189: "-", // dash
            190: ".", // period
            191: "/", // forward slash
            192: "`", // grave accent
            219: "[", // open bracket
            220: "\\", // back slash
            221: "]", // close bracket
            222: "'" // single quote
        };

        window.addEventListener('keydown', function (e) {
            var keyName = otherKeys.hasOwnProperty(e.which) ?
                    otherKeys[event.keyCode] :
                    String.fromCharCode(e.which);
            console.log(keyName + ' down');
            keys[keyName] = true;
        });
        window.addEventListener('keyup', function (e) {
            var keyName = otherKeys.hasOwnProperty(e.which) ?
                    otherKeys[event.keyCode] :
                    String.fromCharCode(e.which);
            console.log(keyName + ' up');
            delete keys[keyName];
        });
        window.addEventListener('blur', function (e) {
            keys = {};
        });
    }

    return {
        init: init,
        start: start,
        step: step,
        get sensors() {return sensors;}
    };
});
