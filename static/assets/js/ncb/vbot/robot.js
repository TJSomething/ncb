/* jslint browser: true */
/* global THREE, _:false, console, VBOT: true */

(function (global) {
    'use strict';

    global.VBOT = global.VBOT || {};
    var VBOT = global.VBOT;
    
    // So, we need to make some function generators so that we can have more
    // code reuse between robots
    function animateWalking(lleg, rleg, maxLegAngle) {
        return (function() {
            var walkState = 0,
                lrleg = rleg.children[0],
                llleg = lleg.children[0];

            return function (ds, dtheta) {
                // Don't let undefined through
                ds = ds || 0;
                dtheta = dtheta || 0;

                // Change our walk state
                walkState += ds;

                rleg.rotation.x =
                    maxLegAngle * Math.sin(walkState / maxLegAngle);
                lleg.rotation.x =
                    -maxLegAngle * Math.sin(walkState / maxLegAngle);
                lrleg.rotation.x = maxLegAngle -
                    maxLegAngle * Math.cos(walkState / maxLegAngle);
                llleg.rotation.x = maxLegAngle +
                    maxLegAngle * Math.cos(walkState / maxLegAngle);
            };
        }());
    }
    
    function grab(larm, rarm) {
        return function (target, arm) {
                var worldPosition = target.positionWorld(),
                    llarm = larm.children[0], // lower left arm
                    lrarm = rarm.children[0], // lower right arm
                    leftElbow = llarm.positionWorld(),
                    rightElbow = lrarm.positionWorld(),
                    distToLeftElbow = worldPosition.distanceTo(leftElbow),
                    distToRightElbow = worldPosition.distanceTo(rightElbow),
                    oldParentMatrix = target.parent.matrixWorld,
                    reverseParentTransform = new THREE.Matrix4(),
                    isArmSet = arm !== undefined,
                    isLeft = isArmSet && (arm[0] === 'l' || arm[0] === 'L'),
                    isLeftCloser = distToLeftElbow < distToRightElbow;

                // Attach the target to the nearest arm
                if ((isArmSet && isLeft) ||
                    (!isArmSet && isLeftCloser)) {
                    llarm.add(target);
                } else {
                    lrarm.add(target);
                }

                // Position the target properly
                reverseParentTransform.getInverse(
                    target.parent.matrixWorld);
                reverseParentTransform. // Remove the new parent's transform
                    multiply(oldParentMatrix). // Add the old parent's
                    multiply(target.matrix). // Add the original transform
                    decompose(target.position, // And set
                        target.rotation,
                        target.scale);

                // Note that we should do collision detection for this
                VBOT.physics.changeObjectState(target, 'held');
            };
    }
    
    function release(heldObjects) {
        return function (arm) {
            var isLeft = (arm[0] === 'l' || arm[0] === 'L'),
                target = isLeft ?
                    heldObjects.left : heldObjects.right,
                oldParentMatrix,
                reverseParentTransform = new THREE.Matrix4();

            if (target) {
                oldParentMatrix = target.parent.matrixWorld;

                this.parent.add(target);
                // Position the target properly
                reverseParentTransform.getInverse(
                    target.parent.matrixWorld);
                reverseParentTransform. // Remove the new parent's transform
                    multiply(oldParentMatrix). // Add the old parent's
                    multiply(target.matrix). // Add the original transform
                    decompose(target.position, // And set
                        target.rotation,
                        target.scale);

                // We need to note that we're holding nothing
                if (isLeft) {
                    heldObjects.left = null;
                } else {
                    heldObjects.right = null;
                }
            }

            // Switch to a falling state
            VBOT.physics.changeObjectState(target, 'falling');
        };
    }
    
    function instantPointArm(obj) {
        /**
         * Points an arm at a target vector, oriented with an up vector
         *
         * @param arm string which arm to move
         * @param targetVector THREE.Vector3 where the arm should point
         * @param upVector THREE.Vector3 the direction the top of the arm
         * @param isGlobal boolean whether targetVector is local or global
         */
        return function (arm, targetVector, isGlobal, upVector) {
            obj.rotateArmTowards(arm, targetVector, 1, false, isGlobal, upVector);
        };
    }
    
    function rotateArmTowards(larm, rarm) {
        /**
         * Rotates an arm toward pointing at a location
         *
         * @param arm string which arm to move
         * @param targetVector THREE.Vector3 where the arm should point
         * @param amount float the amount to move the arm, in either radians or 
         *                     percent of motion remaining
         * @param inRadians boolean whether the amount is in radians
         * @param isGlobal boolean whether targetVector is local or global
         * @param upVector THREE.Vector3 the direction the top of the arm, optional
         * @return boolean if we're done rotating the arm toward the given vector
         */
        return function (arm, targetVector, amount, inRadians, isGlobal, upVector) {
            var isLeft = arm[0] === 'l' || arm[0] === 'L',
                armObj = isLeft ? larm : rarm,
                localMatrix = new THREE.Matrix4(),
                shoulderLoc = armObj.positionWorld(),
                shoulderDisplacement = shoulderLoc.clone().sub(obj.positionWorld()),
                direction = new THREE.Vector3(0, -1, 0),
                thirdBasis = new THREE.Vector3(),
                rotMatrix,
                angleDiff,
                s;

            // Don't want to mangle the formal parameters
            targetVector = targetVector.clone();

            localMatrix.getInverse(obj.matrixWorld);

            // isGlobal defaults to false
            if (isGlobal === undefined) {
                isGlobal = false;
            }

            // I've found that you get the most intuitive motion if you
            // have up be up and out for the arm
            if (upVector === undefined) {
                upVector = new THREE.Vector3(isLeft ? -1 : 1,
                    isLeft ? 1 : -1, 0);
            } else {
                upVector = upVector.clone();
            }

            // Convert the targetVector into local space, if needed
            if (isGlobal) {
                targetVector.
                    sub(shoulderDisplacement).
                    applyMatrix4(localMatrix);
            }
            targetVector.
                multiplyScalar(-1);

            targetVector.normalize();
            upVector.normalize();

            // For a rotation we need three bases
            thirdBasis.crossVectors(targetVector, upVector).
                normalize();

            // Also, force the up vector to be perpendicular to the target
            // vector
            upVector.crossVectors(targetVector, thirdBasis).
                normalize();

            // Make a rotation matrix
            var t = targetVector,
                u = upVector,
                v = thirdBasis;
            rotMatrix = new THREE.Matrix4(
                u.x, t.x, v.x, 0,
                u.y, t.y, v.y, 0,
                u.z, t.z, v.z, 0,
                0,   0,    0,   1
            );

            // Make a quaternion to rotate to
            var q1 = armObj.quaternion,
                q2 = new THREE.Quaternion();
            q2.setFromRotationMatrix(rotMatrix);

            // If the amount to rotate is in radians, we need to convert to
            // amount of arc to rotate by in order use SLERP
            if (inRadians) {
                s = q1.w * q2.w +
                    q1.x * q2.x +
                    q1.y * q2.y +
                    q1.z * q2.z;
                angleDiff = 2 * Math.acos(Math.abs(s));
                amount = amount/angleDiff;
            }

            // Prevent overshoot
            if (amount > 1) {
                amount = 1;
            }

            // Rotate the stuff
            armObj.quaternion.slerp(q2, amount);

            // If it's done moving, then let's say that
            if (amount === 1) {
                return true;
            } else {
                return false;
            }
        };
    }
    
    function pickUpAtHand(larm, rarm, forearmLength) {
        return function (arm) {
            var intersectingObj,
                i;

            function checkHandCollision(arm, target) {
                var isRight = arm[0] === 'r' || arm[0] === 'R',
                    armObj = isRight ? rarm : larm,
                    handLocation = new THREE.Vector3(0, -forearmLength, 0).
                        applyMatrix4(armObj.children[0].matrixWorld),
                    targetBox = new THREE.Box3().setFromObject(target);

                    return targetBox.containsPoint(handLocation);
            }

            // Find an object that's intesecting with the hand
            for (i = 0; i < obj.app.portables.length; i += 1) {
                if (checkHandCollision(arm, obj.app.portables[i])) {
                    intersectingObj = obj.app.portables[i];
                    break;
                }
            }

            if (intersectingObj !== undefined) {
                obj.grab(intersectingObj, arm);

                if (arm[0] === 'l' || arm[0] === 'L') {
                    heldObjects.left = intersectingObj;
                } else {
                    heldObjects.right = intersectingObj;
                }

                return true;
            } else {
                return false;
            }
        };
    }


    VBOT.Robot = (function () {
        function loadSteve(obj) {
            var loader = new THREE.JSONLoader(true);
            loader.load('assets/json/steve.js', function (geometry, materials) {
                var i, camHeight, near,
                    steve = new THREE.SkinnedMesh(geometry,
                        new THREE.MeshFaceMaterial(materials)),
                    rleg = steve.getObjectByName('uleg.R', true),
                    lleg = steve.getObjectByName('uleg.L', true),
                    rarm = steve.getObjectByName('uarm.R', true),
                    larm = steve.getObjectByName('uarm.L', true),
                    forearmLength = 0.518,
                    heldObjects = {right: null, left: null},
                    scale = 2/3;

                // For some weird reason, Steve is the wrong size.
                steve.scale.set(scale, scale, scale);

                obj.add(steve);
                obj.app.physics.addObject(obj,
                    {
                        type: 'dynamic',
                        state: 'controlled'
                    });

                for (i = 0; i < materials.length; i += 1) {
                    // For bones
                    materials[i].skinning = true;
                    // For that pixelated Minecraft effect
                    materials[i].map.magFilter = THREE.NearestFilter;
                    // Fix transparency rendering
                    materials[i].alphaTest = 0.5;
                    // Fixes fringes on hat texture, in theory
                    materials[i].map.premultiplyAlpha = true;
                }

                // Place a camera in the robot
                // Find the bounding box
                obj.bounds = (new THREE.Box3()).setFromObject(steve);
                // The camera should be at eye level
                camHeight = 0.75 * obj.bounds.max.y;
                // The near clipping plane should be a little further out
                //  than the front
                near = 1.01 * obj.bounds.max.z;
                // Make our camera
                obj.camera = new THREE.PerspectiveCamera(45, 4 / 3, near, 1000);
                obj.camera.position.set(0, camHeight, 0);
                obj.camera.rotateY(Math.PI); // Otherwise, it'll point backward
                obj.add(obj.camera);

                // Add walking
                obj.animateWalking = animateWalking(lleg, rleg, Math.PI/9);

                // This anchors an object to the nearest arm
                obj.grab = grab(larm, rarm);

                // This removes the target from the arm
                obj.release = release(heldObjects);

                obj.instantPointArm = instantPointArm(obj);

                obj.rotateArmTowards = rotateArmTowards(larm, rarm);
                
                // Picks up the object that's in the hand, if there is one
                obj.pickUpAtHand = pickUpAtHand(larm, rarm, forearmLength);
                
                Object.defineProperties(obj, {
                       leftHandObject: {
                               get: function () {
                                       return heldObjects.left;
                               }
                       },
                       rightHandObject: {
                               get: function () {
                                       return heldObjects.right;
                               }
                       }
                });

                obj.app.fire('robot-ready');
            });
        }
        
        function loadCarl(obj) {
            var loader = new THREE.JSONLoader(true);
            loader.load('assets/json/carl.js', function (geometry, materials) {
                var i, camHeight, near,
                    carl = new THREE.SkinnedMesh(geometry,
                        new THREE.MeshFaceMaterial(materials)),
                    scale = 0.01,
                    rleg = carl.getObjectByName('Carl:RightUpLeg', true),
                    lleg = carl.getObjectByName('Carl:LeftUpLeg', true),
                    rarm = carl.getObjectByName('Carl:RightArm', true),
                    larm = carl.getObjectByName('Carl:LeftArm', true),
                    lhand = carl.getObjectByName('Carl:LeftHand', true),
                    forearmLength = lhand.position.length(),
                    heldObjects = {right: null, left: null};

                // Why is it in centimeters?
                carl.scale.set(scale, scale, scale);

                obj.add(carl);
                obj.app.physics.addObject(obj,
                    {
                        type: 'dynamic',
                        state: 'controlled'
                    });

                for (i = 0; i < materials.length; i += 1) {
                    // For bones
                    materials[i].skinning = true;
                }
                
                // Move the arms down
                larm.rotateZ(-Math.PI/2);
                rarm.rotateZ(Math.PI/2);

                // Place a camera in the robot
                // Find the bounding box
                obj.bounds = (new THREE.Box3()).setFromObject(carl);
                // The camera should be at eye level
                camHeight = 0.95 * obj.bounds.max.y;
                // The near clipping plane should be a little further out
                //  than the front
                near = 1.01 * obj.bounds.max.z;
                // Make our camera
                obj.camera = new THREE.PerspectiveCamera(45, 4 / 3, near, 1000);
                obj.camera.position.set(0, camHeight, 0);
                obj.camera.rotateY(Math.PI); // Otherwise, it'll point backward
                obj.add(obj.camera);

                // Add walking
                obj.animateWalking = animateWalking(lleg, rleg, Math.PI/9);

                // This anchors an object to the nearest arm
                obj.grab = grab(larm, rarm);

                // This removes the target from the arm
                obj.release = release(heldObjects);

                obj.instantPointArm = instantPointArm(obj);

                obj.rotateArmTowards = rotateArmTowards(larm, rarm);
                
                // Picks up the object that's in the hand, if there is one
                obj.pickUpAtHand = pickUpAtHand(larm, rarm, forearmLength);
                
                Object.defineProperties(obj, {
                       leftHandObject: {
                               get: function () {
                                       return heldObjects.left;
                               }
                       },
                       rightHandObject: {
                               get: function () {
                                       return heldObjects.right;
                               }
                       }
                });

                obj.app.fire('robot-ready');
            });
        }

        function loadMouse(obj) {
            var loader = new THREE.ObjectLoader();

            loader.load('assets/json/mouse.js', function (robotObj) {
                var camHeight, near;

                robotObj.scale.set(0.2, 0.2, 0.2);
                obj.add(robotObj);
                obj.app.physics.addObject(obj,
                    {
                        isStatic: false,
                        state: 'controlled'
                    });

                // Place a camera in the robot
                // Find the bounding box
                obj.bounds = (new THREE.Box3()).setFromObject(robotObj);
                // The camera should be 3/4 up the robot
                camHeight = 0.75 * obj.bounds.max.y;
                // The near clipping plane should be a little further out
                //  than the front
                near = 1.05 * obj.bounds.max.z;
                // Make our camera
                obj.camera = new THREE.PerspectiveCamera(45, 4 / 3, near, 1000);
                obj.camera.position.set(0, camHeight, 0);
                obj.camera.rotateY(Math.PI); // Otherwise, it'll point backward
                obj.add(obj.camera);

                // Animated walking is a no-op
                obj.animateWalking = function () {};

                obj.app.fire('robot-ready');
            });
        }

        var modelLoaders = {
            mouse: loadMouse,
            steve: loadSteve,
            carl: loadCarl
        };


        return function (obj, robotType, app) {
            var that = this;

            // Allow usage of new
            if (!(this instanceof VBOT.Robot)) {
                return new VBOT.Robot(obj, robotType);
            }

            // Clear the dummy geometry used to stand in for
            //  the robot
            obj.geometry = new THREE.Geometry();

            // We're going to make a new object that isn't subject to the scaling
            //  in the scene and won't kill accidental children
            that = new THREE.Object3D();
            that.position.copy(obj.position);
            that.quaternion.copy(obj.quaternion);
            that.name = 'Robot Parent';
            that.app = app;
            that.app.scene.add(that);

            // Load the robot model asynchronously
            modelLoaders[robotType](that);

            // Methods for robots
            that.instantForward = function (displacement) {
                var model = new THREE.Vector3(0, 0, displacement),
                    world = model.applyQuaternion(this.quaternion);

                this.animateWalking(displacement);
                this.position.add(world);
            };

            that.instantBackward = function(displacement) {
                this.instantForward(-displacement);
            };

            that.instantRight = function (displacement) {
                this.animateWalking(0, displacement);
                this.rotateY(displacement);
            };

            that.instantLeft = function (displacement) {
                that.instantRight(-displacement);
            };

            // This finds exactly how the robot would move if
            //  it were going forward and turning at the same
            //  time.
            that.simultaneousTurnMove = function (ds, dtheta) {
                var phi, turningRadius, dist;

                if (Math.abs(dtheta) > 1e-6) {
                    phi = (Math.PI - dtheta) / 2;
                    turningRadius = ds / dtheta;
                    dist = turningRadius * Math.sin(dtheta) / Math.sin(phi);

                    this.instantRight(Math.PI/2 - phi);
                    this.instantForward(dist);
                    this.instantRight(dtheta - Math.PI/2 + phi);
                } else {
                    // We need to handle the case where there is no turning
                    this.instantForward(ds);
                }
                // Make sure everyone knows that we've moved
                this.updateMatrix();
            };

            that.move = function (dt) {
                var ds = this.speed * dt,
                    dtheta = this.angularVelocity * dt;

                // Move
                this.simultaneousTurnMove(ds, dtheta);
            };
            that.app.on('logic-tick', that.move, that);

            document.addEventListener( 'keydown', function (event) {
                if (event.altKey) {
                    return;
                }

                switch (event.keyCode) {
                    case 83: // S
                        that.speed = -1.7;
                        break;
                    case 87: // W
                        that.speed = 1.7;
                        break;
                    case 65: // A
                        that.angularVelocity = 1;
                        break;
                    case 68: // D
                        that.angularVelocity = -1;
                        break;
                }
            }, false );
            document.addEventListener( 'keyup', function (event) {
                switch (event.keyCode) {
                    case 83: // S
                        that.speed = 0;
                        break;
                    case 87: // W
                        that.speed = 0;
                        break;
                    case 65: // A
                        that.angularVelocity = 0;
                        break;
                    case 68: // D
                        that.angularVelocity = 0;
                        break;
                }
            }, false );

            // Properties
            that.speed = 0;
            that.angularVelocity = 0;

            return that;
        };
    }());
})(this);