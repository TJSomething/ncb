/* jslint worker: true, evil: true */
/* global esprima, loadScript, getAction, next, completeAction, run,
          newActions, clearNewActions, _ */

/**
 * These are the commands used to script the robot's actions.
 *
 * @namespace scripting
 */

/**
 * Holds all the sensor data.
 * @var {module:vbot/controller~SenseData} sensors
 */

importScripts('../../lib/esprima.js');
importScripts('../../lib/underscore.js');

(function () {
    'use strict';

    var state = null;
    var runningActions = {};
    var newActions = [];
    var states = null;

    /**
     * Makes an action.
     *
     * @class
     * @memberof scripting~
     * @param {string} action the action to be performed
     * @param {object} params parameters for the action
     */
    var Action = function (action, params) {
        var param;

        this.action = action;
        this.id = '' + Math.random();

        if (params) {
            for (param in params) {
                if (params.hasOwnProperty(param)) {
                    this[param] = params[param];
                }
            }
        }

        // Queue it too
        newActions.push(this);

        return this;
    };

    /**
     * Creates an extended action and puts it in the running actions list.
     *
     * @constructs ExtendedAction
     * @memberof scripting~
     * @augments {scripting~Action}
     * @param {string} action the action to be performed
     * @param {object} params parameters for the action
         */
        var ExtendedAction = function (action, params) {
        Action.call(this, action, params);

        runningActions[this.id] = this;

        return this;
    };

    ExtendedAction.prototype =
    /** @lends scripting~ExtendedAction.prototype */
    {
        /**
         * Sets a time period for the action to take place over.
         *
         * @param  {Number} seconds the time period in seconds
         * @return {scripting~ExtendedAction} this object
         */
        over: function (seconds) {
            // This conflicts with at, so whatever happens last
            // will take precedence
            if (this.speed !== undefined) {
                delete this.speed;
            }
            this.time = seconds;
            return this;
        },

        /**
         * Switch to a new state on the completion of this action
         *
         * @param  {(string|function)} newState the new state; if a function is
         *                                      used, it needs to be one of the
         *                                      top-level state functions
         *                                      defined in the script
         * @return {scripting~ExtendedAction}             this object
         */
        then: function (newState) {
            this.newState = newState;
            return this;
        },

        /**
         * Sets a speed for the action to performed at, which is going to either
         * be in meters per second or radians per second, depending on the
         * action.
         *
         * @param  {Number} speed
         * @return {scripting~ExtendedAction} this action
         */
        at: function (speed) {
            // This conflicts with over, so whatever happens last
            // will take precedence
            if (this.time !== undefined) {
                delete this.time;
            }
            this.speed = speed;
            return this;
        }
    };

    /**
     * Converts from RGB to HSV. All parameters are assumed to be in [0,255].
     *
     * @param  {Number} red
     * @param  {Number} green
     * @param  {Number} blue
     * @return {{h: Number,
     *           s: Number,
     *           v: Number}} the color in HSV representation
     */
    function rgb2hsv (red, green, blue) {
        var rr, gg, bb,
            r = red / 255,
            g = green / 255,
            b = blue / 255,
            h, s,
            v = Math.max(r, g, b),
            diff = v - Math.min(r, g, b),
            diffc = function(c){
                return (v - c) / 6 / diff + 1 / 2;
            };

        if (diff === 0) {
            h = s = 0;
        } else {
            s = diff / v;
            rr = diffc(r);
            gg = diffc(g);
            bb = diffc(b);

            if (r === v) {
                h = bb - gg;
            }else if (g === v) {
                h = (1 / 3) + rr - bb;
            }else if (b === v) {
                h = (2 / 3) + gg - rr;
            }
            if (h < 0) {
                h += 1;
            }else if (h > 1) {
                h -= 1;
            }
        }
        return {
            h: Math.round(h * 360),
            s: Math.round(s * 100),
            v: Math.round(v * 100)
        };
    }

    /**
     * A color with multiple convenient representations.
     *
     * @typedef {Object} scripting~Color
     *
     * @property {Number} red red level from 0 to 255
     * @property {Number} green green level from 0 to 255
     * @property {Number} blue blue level from 0 to 255
     * @property {Number} hue hue angle from 0 to 360
     * @property {Number} saturation saturation from 0 to 100
     * @property {Number} value value from 0 to 100
     */

    /**
     * Gets the pixel value at x and y, where x and y are in [-1,1] on a
     * Euclidean plane.
     *
     * @memberof scripting
     * @param {Number} x
     * @param {Number} y
     * @return {scripting~Color} the color at that point, with keys red,
     *                                green, blue, alpha, hue, saturation, and
     *                                value
     */
    function getPixel(x, y) {
        var pixelX = ((x + 1.0) * 0.5 * (self.sensors.camera.width - 1))|0,
            pixelY = ((1.0 - y) * 0.5 * (self.sensors.camera.height - 1))|0,
            index = (pixelY * self.sensors.camera.width + pixelX) * 4,
            r = self.sensors.camera.data[index],
            g = self.sensors.camera.data[index + 1],
            b = self.sensors.camera.data[index + 2],
            a = self.sensors.camera.data[index + 3],
            hsv = rgb2hsv(r, g, b);

        return {
            red: r,
            green: g,
            blue: b,
            alpha: a,
            hue: hsv.h,
            saturation: hsv.s,
            value: hsv.v
        };
    }

    /**
     * Turns towards the named object.
     *
     * @param {string} objName the object's name
     * @memberof scripting
     * @return {scripting~ExtendedAction} the turning action
     */
    function turnTowards(objName) {
        var action = new ExtendedAction('turnTowards', {
            objName: objName
        });

        return action;
    }

    /**
     * Set the walking speed. Can be negative to walk backwards.
     *
     * @memberof scripting
     * @param {Number} speed the speed in m/s
     * @return {scripting~Action} the action of setting the walking speed
     */
    function setSpeed(speed) {
        return new Action('setSpeed', {
            speed: speed
        });
    }

    /**
     * Set the turning speed in radians per second. Can be negative to turn
     * right.
     *
     * @memberof scripting
     * @param {Number} speed the turning speed
     * @return {scripting~Action}      the action of beginning to turn left
     */
    function startTurningLeft(speed) {
        return new Action('turn', {
            speed: speed
        });
    }

    /**
     * Set the turning speed in radians per second. Can be negative to turn
     * left.
     *
     * @memberof scripting
     * @param {Number} speed the turning speed
     * @return {scripting~Action}      the action of beginning to turn right
     */
    function startTurningRight(speed) {
        return new Action('turn', {
            speed: -speed
        });
    }

    /**
     * Turns left by the given number of degrees.
     *
     * @memberof scripting
     * @param {Number} degrees  the number of degrees to turn
     * @return {scripting~ExtendedAction} the action of turning left
     */
    function turnLeft(degrees) {
        return new ExtendedAction('turnLeft', {
            degrees: degrees
        });
    }

    /**
     *  Turns right by the given number of degrees.
     *
     * @param {Number} degrees   the number of degrees to turn
     * @return {scripting~ExtendedAction} the action of turning right
     */
    function turnRight(degrees) {
        return new ExtendedAction('turnRight', {
            degrees: degrees
        });
    }

    /**
     *  Go forward by the given distance.
     *
     * @memberof scripting
     * @param {Number} distance the distance to go forward by
     * @return {scripting~ExtendedAction} this action
     */
    function goForward(distance) {
        return new ExtendedAction('goForward', {
            distance: distance
        });
    }

    /**
     *  Go backwards by the given distance.
     *
     * @memberof scripting
     * @param {Number} distance the distance to go backward by
     * @return {scripting~ExtendedAction}
     */
    function goBackward(distance) {
        return new ExtendedAction('goForward', {
            distance: -distance
        });
    }

    /**
     *  Tries to point the right arm at the given object.
     *
     * @memberof scripting
     * @param {string} objName the name of the object to point at
     * @return {scripting~ExtendedAction} this action
     */
    function pointRightArmAt(objName) {
        return new ExtendedAction('pointArmAt', {
            arm: 'right',
            objName: objName
        });
    }

    /**
     *  Tries to point the left arm at the given object.
     *
     * @memberof scripting
     * @param {string} objName  the name of the object to point at
     * @return {scripting~ExtendedAction} this action
     */
    function pointLeftArmAt(objName) {
        return new ExtendedAction('pointArmAt', {
            arm: 'left',
            objName: objName
        });
    }

    /**
     *  Tries to point the left arm in the direction of the given vector of the
     *  given coordinates. This vector starts at the shoulder joint.
     *
     * @memberof scripting
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @return {scripting~ExtendedAction} this action
     */
    function pointRightArm(x, y, z) {
        return new ExtendedAction('pointArm', {
            arm: 'right',
            direction: {
                x: x,
                y: y,
                z: z
            }
        });
    }

    /**
     * Tries to point the left arm in the direction of the given vector of the
     * given coordinates. This vector starts at the shoulder joint.
     *
     * @memberof scripting
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     * @return {scripting~ExtendedAction} this action
     */
    function pointLeftArm(x, y, z) {
        return new ExtendedAction('pointArm', {
            arm: 'right',
            direction: {
                x: x,
                y: y,
                z: z
            }
        });
    }

    /**
     * Changes the expression.
     *
     * @memberof scripting
     * @param {string} expression the expression to change to
     * @return {scripting~ExtendedAction} this action
     */
    function changeExpression(expression) {
        return new ExtendedAction('changeExpression', {
            expression: expression
        });
    }

    /**
     *  Grabs the object that is intersecting with the hand
     *  with the right arm
     *
     * @memberof scripting
     * @return {scripting~Action} this action
     */
    function grabWithRightArm() {
        return new Action('grab', {
            arm: 'right'
        });
    }

    /**
     *  Grabs the object that is intersecting with the hand
     *  with the left arm
     *
     * @memberof scripting
     * @return {scripting~Action} this action
     */
    function grabWithLeftArm() {
        return new Action('grab', {
            arm: 'left'
        });
    }

    /**
     * Reaches for and grabs an object with the left hand.
     *
     * @param objName
     * @returns {ExtendedAction}
     */
    function reachForAndGrabWithLeftHand(objName) {
        return new ExtendedAction('reachForAndGrab', {
            arm: 'left',
            target: objName
        });
    }

    /**
     * Reaches for and grabs an object with the right hand.
     *
     * @param objName
     * @returns {ExtendedAction}
     */
    function reachForAndGrabWithRightHand(objName) {
        return new ExtendedAction('reachForAndGrab', {
            arm: 'right',
            target: objName
        });
    }

    /**
     *  Sets the state.
     * @memberof scripting
     */
    function next(newState) {
        if (_.isString(newState) &&
            states.hasOwnProperty(newState)) {
            state = newState;
        } else if (typeof newState === 'function') {
            state = newState.name;
        } else {
            throw 'Invalid next state ' + newState;
        }
    }

    /**
     * Logs an error.
     *
     * @memberof scripting
     * @param  {string} error an error message
     */
    function log(error) {
        postMessage({error: error});
    }

    /**
     * Loads a script and puts its functions into a state machine.
     *
     * @param  {string} script the script code
     */
    function loadScript(script) {
        var functionBody,
            ast,
            stateNames = [],
            tempFunc;

        // Only bother with loading the script if we haven't done
        // it yet
        if (states === null) {
            // Find the functions in the given script and make them
            // states for the state machine
            ast = esprima.parse(script);

            ast.body.forEach(function (astNode) {
                // Only use it if it's a function
                if (astNode.type === 'FunctionDeclaration') {
                    // If this i$s the first function, then it's the starting
                    // state
                    if (state === null) {
                        state = astNode.id.name;
                    }

                    stateNames.push(astNode.id.name);
                }
            });

            // Export the functions that we're using as states from
            // the state machine
            functionBody = script + '\n';
            functionBody += 'return {\n';
            functionBody += stateNames.map(function (stateName) {
                return '    ' + stateName + ': ' + stateName;
            }).join(',\n');
            functionBody += '\n}\n';

            // Run the script as code
            tempFunc = new Function(functionBody);
            states = tempFunc();
        }
    }

    Object.defineProperties(self,
        {
            'getPixel': {value: getPixel},
            'turnTowards': {value: turnTowards},
            'setSpeed': {value: setSpeed},
            'startTurningLeft': {value: startTurningLeft},
            'startTurningRight': {value: startTurningRight},
            'turnLeft': {value: turnLeft},
            'turnRight': {value: turnRight},
            'goForward': {value: goForward},
            'goBackward': {value: goBackward},
            'pointRightArmAt': {value: pointRightArmAt},
            'pointLeftArmAt': {value: pointLeftArmAt},
            'pointRightArm': {value: pointRightArm},
            'pointLeftArm': {value: pointLeftArm},
            'changeExpression': {value: changeExpression},
            'grabWithLeftArm': {value: grabWithLeftArm},
            'grabWithRightArm': {value: grabWithRightArm},
            'reachForAndGrabWithLeftHand': {value: reachForAndGrabWithLeftHand},
            'reachForAndGrabWithRightHand': {value: reachForAndGrabWithRightHand},
            'next': {value: next},
            'state': {
                get: function () { return state; }
            },
            'dequeue': {
                value: newActions.shift.bind(newActions)
            },
            'getAction': {
                value: function (actionId) {
                    return runningActions[actionId];
                    }
            },
            'completeAction': {
                value: function (actionId) {
                    delete runningActions[actionId];
                }
            },
            'run': {
                get: function () { return states[state]; }
            },
            'newActions': {
                get: function () { return newActions; }
            },
            'clearNewActions': {
                value: function () {
                    while (newActions.length > 0) {
                        newActions.pop();
                    }
                }
            },
            'log': {
                value: log
            }
        }
    );

    self.addEventListener('message', function (oEvent) {
        var reply = {};

        // If this is our first message
        if (oEvent.data.start) {
            // Load up the script as a state machine
            try {
                loadScript(oEvent.data.script);
                postMessage({loaded: true});
            } catch (e) {
                // If there's an error in their script, send them a message
                reply.error = e.message;
                postMessage(reply);
            }
        } else if (oEvent.data.sensors) {
            // Make sensor data available
            self.sensors = oEvent.data.sensors;
            // Check if any of the current actions have completed
            if (oEvent.data.actionsCompleted) {
                oEvent.data.actionsCompleted.forEach(function (actionId) {
                    if (getAction(actionId)) {
                        // If the action has a state to set after it completes,
                        // then set that
                        if (getAction(actionId).newState) {
                            next(getAction(actionId).newState);
                        }
                        completeAction(actionId);
                    }
                });
            }
            // Run the current state
            try {
                run();
                // Prep a message to send back
                // Send over the newly queued actions
                reply.actions = newActions;
                // We actually end up threading the time interval into our
                // actuation call
                reply.step = oEvent.data.step;
                // Send the message back
                postMessage(reply);
                clearNewActions();
            } catch (e) {
                // If there was an error running their stuff, tell them
                reply.error = {
                    message: e.message,
                    stack: e.stack
                };
                postMessage(reply);
            }
        }
    });
}());
