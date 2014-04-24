/* jslint worker: true, evil: true */
/* global esprima, loadScript, getAction, next, completeAction, run,
          newActions, clearNewActions */

'use strict';

importScripts('../../lib/esprima.js');

var sensors = null;

(function (global) {
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
        var that = new Action(action, params);

        runningActions[that.id] = that;

        return that;
    };

    // Sets a time period for the action to take place over
    ExtendedAction.prototype.over = function (seconds) {
        this.time = seconds;
        return this;
    };

    // Switch to a new state on the completion of this action
    ExtendedAction.prototype.then = function (newState) {
        this.newState = newState;
        return this;
    };

    // Switch to a new state on the completion of this action
    ExtendedAction.prototype.at = function (newState) {
        this.newState = newState;
        return this;
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
        var pixelX = (x * sensors.camera.width + 0.5)|0,
            pixelY = (y * sensors.camera.height + 0.5)|0,
            index = pixelY * sensors.camera.width + pixelX,
            r = sensors.camera.data[index],
            g = sensors.camera.data[index + 1],
            b = sensors.camera.data[index + 2],
            a = sensors.camera.data[index + 3],
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
     *  Sets the state.
     */
    function next(newState) {
        state = newState;
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
                    // If this is the first function, then it's the starting
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
    Object.defineProperty(global, 'turnLeft', {value: turnLeft});
    Object.defineProperty(global, 'turnRight', {value: turnRight});
    Object.defineProperty(global, 'goForward', {value: goForward});
    Object.defineProperty(global, 'goBackward', {value: goBackward});
    Object.defineProperty(global, 'pointRightArmAt', {value: pointRightArmAt});
    Object.defineProperty(global, 'pointLeftArmAt', {value: pointLeftArmAt});
    Object.defineProperty(global, 'pointRightArm', {value: pointRightArm});
    Object.defineProperty(global, 'pointLeftArm', {value: pointLeftArm});
    Object.defineProperty(global, 'changeExpression', {value: changeExpression});
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
}(self));

self.addEventListener('message', function (oEvent) {
    var reply = {};

    // If this is our first message
    if (oEvent.data.start) {
        // Load up the script as a state machine
        try {
            loadScript(oEvent.data.script);
            postMessage('worker loaded script');
        } catch (e) {
            // If there's an error in their script, send them a message
            reply.error = e.message;
            postMessage(reply);
        }
    } else if (oEvent.data.sensors) {
        // Make sensor data available
        sensors = oEvent.sensors;
        // Check if any of the current actions have completed
        if (oEvent.actions.completed) {
            oEvent.actions.completed.forEach(function (actionId) {
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
            clearNewActions();
            // Send the message back
            postMessage(reply);
        } catch (e) {
            // If there was an error running their stuff, tell them
            reply.error = e.message;
            postMessage(reply);
        }
    }
});
