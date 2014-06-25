/* jslint worker: true, evil: true */
/* global esprima, loadScript, getAction, next, completeAction, run,
          newActions, clearNewActions, _ */

importScripts('../../lib/esprima.js');
importScripts('../../lib/underscore.js');

(function (global) {
    'use strict';
    
    var state = null;
    var runningActions = {};
    var newActions = [];
    var states = null;

    // Let's make a prototype for actions
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

    var ExtendedAction = function (action, params) {
        Action.call(this, action, params);

        runningActions[this.id] = this;

        return this;
    };

    ExtendedAction.prototype = {
        // Sets a time period for the action to take place over
        over: function (seconds) {
            // This conflicts with at, so whatever happens last
            // will take precedence
            if (this.speed !== undefined) {
                delete this.speed;
            }
            this.time = seconds;
            return this;
        },

        // Switch to a new state on the completion of this action
        then: function (newState) {
            this.newState = newState;
            return this;
        },

        // Sets a speed for the action to be performed at
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

    function rgb2hsv () {
        var rr, gg, bb,
            r = arguments[0] / 255,
            g = arguments[1] / 255,
            b = arguments[2] / 255,
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
     *  Gets the pixel value at x and y, where x and y are in [-1,1].
     */
    function getPixel(x, y) {
        var pixelX = ((x + 1.0) * 0.5 * (global.sensors.camera.width - 1))|0,
            pixelY = ((1.0 - y) * 0.5 * (global.sensors.camera.height - 1))|0,
            index = (pixelY * global.sensors.camera.width + pixelX) * 4,
            r = global.sensors.camera.data[index],
            g = global.sensors.camera.data[index + 1],
            b = global.sensors.camera.data[index + 2],
            a = global.sensors.camera.data[index + 3],
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
     *  Turns towards the named object
     */
    function turnTowards(objName) {
        var action = new ExtendedAction('turnTowards', {
            objName: objName
        });

        return action;
    }

    /**
     *  Set the walking speed. Can be negative to walk backwards.
     */
    function setSpeed(speed) {
        return new Action('setSpeed', {
            speed: speed
        });
    }

    /**
     *  Set the turning speed in radians per second. Can be negative to turn right.
     */
    function startTurningLeft(speed) {
        return new Action('turn', {
            speed: speed
        });
    }

    /**
     *  Set the turning speed in radians per second. Can be negative to turn left.
     */
    function startTurningRight(speed) {
        return new Action('turn', {
            speed: -speed
        });
    }

    /**
     *  Turns left by the given number of degrees.
     */
    function turnLeft(degrees) {
        return new ExtendedAction('turnLeft', {
            degrees: degrees
        });
    }

    /**
     *  Turns right by the given number of degrees.
     */
    function turnRight(degrees) {
        return new ExtendedAction('turnRight', {
            degrees: degrees
        });
    }

    /**
     *  Go forward by the given distance.
     */
    function goForward(distance) {
        return new ExtendedAction('goForward', {
            distance: distance
        });
    }

    /**
     *  Go backwards by the given distance.
     */
    function goBackward(distance) {
        return new ExtendedAction('turnLeft', {
            distance: distance
        });
    }

    /**
     *  Tries to point the right arm at the given object.
     */
    function pointRightArmAt(objName) {
        return new ExtendedAction('pointArmAt', {
            arm: 'right',
            objName: objName
        });
    }

    /**
     *  Tries to point the left arm at the given object.
     */
    function pointLeftArmAt(objName) {
        return new ExtendedAction('pointArmAt', {
            arm: 'left',
            objName: objName
        });
    }

    /**
     *  Tries to point the left arm in the direction of the given vector.
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
     *  Tries to point the right arm in the direction of the given vector.
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
     *  Changes the expression.
     */
    function changeExpression(expression) {
        return new ExtendedAction('changeExpression', {
            expression: expression
        });
    }
    
    /**
     *  Grabs the object that is intersecting with the hand
     *  with the right arm
     */
    function grabWithRightArm() {
        return new Action('grab', {
            arm: 'right'
        });
    }
    
    /**
     *  Grabs the object that is intersecting with the hand
     *  with the left arm
     */
    function grabWithLeftArm() {
        return new Action('grab', {
            arm: 'left'
        });
    }

    /**
     *  Sets the state.
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

    Object.defineProperty(global, 'getPixel', {value: getPixel});
    Object.defineProperty(global, 'turnTowards', {value: turnTowards});
    Object.defineProperty(global, 'setSpeed', {value: setSpeed});
    Object.defineProperty(global, 'startTurningLeft', {value: startTurningLeft});
    Object.defineProperty(global, 'startTurningRight', {value: startTurningRight});
    Object.defineProperty(global, 'turnLeft', {value: turnLeft});
    Object.defineProperty(global, 'turnRight', {value: turnRight});
    Object.defineProperty(global, 'goForward', {value: goForward});
    Object.defineProperty(global, 'goBackward', {value: goBackward});
    Object.defineProperty(global, 'pointRightArmAt', {value: pointRightArmAt});
    Object.defineProperty(global, 'pointLeftArmAt', {value: pointLeftArmAt});
    Object.defineProperty(global, 'pointRightArm', {value: pointRightArm});
    Object.defineProperty(global, 'pointLeftArm', {value: pointLeftArm});
    Object.defineProperty(global, 'changeExpression', {value: changeExpression});
    Object.defineProperty(global, 'grabWithLeftArm', {value: grabWithLeftArm});
    Object.defineProperty(global, 'grabWithRightArm', {value: grabWithRightArm});
    Object.defineProperty(global, 'next', {value: next});
    Object.defineProperty(global, 'state',
        {
            get: function () { return state; }
        });
    Object.defineProperty(global, 'dequeue', {
        value: newActions.shift.bind(newActions)
    });
    Object.defineProperty(global, 'loadScript', {
        value: loadScript
    });
    Object.defineProperty(global, 'getAction', {
        value: function (actionId) {
            return runningActions[actionId];
        }
    });
    Object.defineProperty(global, 'completeAction', {
        value: function (actionId) {
            delete runningActions[actionId];
        }
    });
    Object.defineProperty(global, 'run', {
        get: function () { return states[state]; }
    });
    Object.defineProperty(global, 'newActions', {
        get: function () { return newActions; }
    });
    Object.defineProperty(global, 'clearNewActions', {
        value: function () {
            while (newActions.length > 0) {
                newActions.pop();
            }
        }
    });
    Object.defineProperty(global, 'log', {
        value: function (error) {
            postMessage({error: error});
        }
    });
    
    global.addEventListener('message', function (oEvent) {
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
            global.sensors = oEvent.data.sensors;
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
}(self));
