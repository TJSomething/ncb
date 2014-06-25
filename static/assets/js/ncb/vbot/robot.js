/* jslint browser: true */
/* global THREE, _:false, console, VBOT: true */

(function (global) {
    'use strict';

    global.VBOT = global.VBOT || {};
    var VBOT = global.VBOT;

    VBOT.Robot = (function () {
        function loadSteve(obj) {
            var loader = new THREE.JSONLoader(true);
            loader.load('assets/json/steve.js', function (geometry, materials) {
                var i, camHeight, near,
                    steve = new THREE.SkinnedMesh(geometry,
                        new THREE.MeshFaceMaterial(materials)),
                    findBoneIndex = (function () {
                        var boneNames = steve.
                                bones.
                                map(function (bone) { return bone.name; });
                        return Array.prototype.indexOf.bind(boneNames);
                    }()),
                    rleg = steve.bones[findBoneIndex('uleg.R')],
                    lleg = steve.bones[findBoneIndex('uleg.L')],
                    rarm = steve.bones[findBoneIndex('uarm.R')],
                    larm = steve.bones[findBoneIndex('uarm.L')],
                    leftMotionRemaining = 0,
                    rightMotionRemaining = 0,
                    leftPointTarget,
                    rightPointTarget,
                    forearmLength = 0.518,
                    armLength = forearmLength - rarm.children[0].position.y,
                    leftHandObject,
                    rightHandObject,
                    scale = 2/3,
                    lArmAngularVelocity = 0,
                    rArmAngularVelocity = 0;

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
                obj.animateWalking = (function() {
                    var walkState = 0,
                        maxLegAngle = Math.PI/9,
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

                // This anchors an object to the nearest arm
                obj.grab = function (target, arm) {
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
                    obj.app.physics.changeObjectState(target, 'held');
                };

                // This removes the target from the arm
                obj.release = function (arm) {
                    var isLeft = (arm[0] === 'l' || arm[0] === 'L'),
                        target = isLeft ?
                            leftHandObject : rightHandObject,
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
                            leftHandObject = undefined;
                        } else {
                            rightHandObject = undefined;
                        }
                    }

                    // Switch to a falling state
                    obj.app.physics.changeObjectState(target, 'falling');
                };

                obj.setArmAngle = function (arm, horiz, vert, roll) {
                    var armObj;

                    // We're setting a specific arm
                    if (arm[0] === 'l' || arm[0] === 'L') {
                        armObj = larm;
                    } else {
                        armObj = rarm;
                    }

                    armObj.rotation.x = vert;
                    // We need to clamp motion to force motion to be
                    //  relatively realistic
                    /*armObj.rotation.y = Math.max(-Math.PI/2,
                        Math.min(Math.PI, roll));
                    armObj.rotation.z = Math.max(-Math.PI/4,
                            Math.min(3*Math.PI/4, horiz));*/
                };

                obj.getArmAngle = function (arm) {
                    var armObj;

                    // We're setting a specific arm
                    if (arm[0] === 'l' || arm[0] === 'L') {
                        armObj = larm;
                    } else {
                        armObj = rarm;
                    }

                    return {
                        flexion: armObj.rotation.x,
                        adduction: armObj.rotation.z,
                        rotation: armObj.rotation.y
                    };
                };

                obj.changeArmAngle = function (arm, horiz, vert, roll) {
                    var armObj;

                    // We're setting a specific arm
                    if (arm[0] === 'l' || arm[0] === 'L') {
                        armObj = larm;
                    } else {
                        armObj = rarm;
                    }

                    armObj.rotation.x = armObj.rotation.x + vert;
                    // We need to clamp motion to force motion to be
                    //  relatively realistic
                    /*armObj.rotation.y =
                        Math.max(-Math.PI/2,
                            Math.min(Math.PI,
                                armObj.rotation.y + roll));
                    armObj.rotation.z =
                        Math.max(-Math.PI/4,
                            Math.min(3*Math.PI/4,
                                armObj.rotation.z + horiz));*/
                };

                obj.flexShoulder = function (arm, amount) {
                    this.changeArmAngle(arm, 0, -amount, 0);
                };

                obj.adductShoulder = function (arm, amount) {
                    this.changeArmAngle(arm, amount, 0, 0);
                };

                obj.rotateShoulder = function (arm, amount) {
                    this.changeArmAngle(arm, 0, 0, -amount);
                };

                obj.pointArm = function (arm, target, duration, callback) {
                    var targetBox = new THREE.Box3().setFromObject(target),
                        targetCenter = new THREE.Vector3();

                    // Find the center of the objects bounding box
                    targetCenter.addVectors(targetBox.min, targetBox.max);
                    targetCenter.multiplyScalar(0.5);

                    if (arm[0] === 'l' || arm[0] === 'L') {
                        leftMotionRemaining = duration;
                        leftPointTarget = targetCenter;
                    } else {
                        rightMotionRemaining = duration;
                        rightPointTarget = targetCenter;
                    }
                };

                /**
                 * Points an arm at a target vector, oriented with an up vector
                 *
                 * @param arm string which arm to move
                 * @param targetVector THREE.Vector3 where the arm should point
                 * @param upVector THREE.Vector3 the direction the top of the arm
                 * @param isGlobal boolean whether targetVector is local or global
                 */
                obj.instantPointArm = function (arm, targetVector, isGlobal, upVector) {
                    obj.rotateArmTowards(arm, targetVector, 1, false, isGlobal, upVector);
                };

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
                obj.rotateArmTowards = function (arm, targetVector, amount, inRadians, isGlobal, upVector) {
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

                // Move the arms for animation purposes
                function moveArms(dt) {
                    function moveArm(arm, target, motionRemaining) {
                        var localTarget,
                            localMatrix = new THREE.Matrix4(),
                            armDirection = new THREE.Vector3(0, -1, 0),
                            axis = new THREE.Vector3(),
                            angle,
                            motionAmount;

                        // Calculate the axis and angle to rotate to
                        localMatrix.getInverse(arm.matrixWorld);
                        localTarget = target.clone().
                            applyMatrix4(localMatrix).
                            normalize();
                        armDirection.normalize();
                        axis.crossVectors(armDirection, localTarget).
                            normalize();
                        // Order on the dot product matters, because
                        //  armDirection is a Vector4
                        angle = Math.acos(localTarget.dot(armDirection));

                        // Calculate how much we're going to move toward our
                        //  target angle
                        if (dt > motionRemaining) {
                            // If we get there within the frame,
                            //  then say we move all the way
                            motionAmount = 1;
                            motionRemaining = 0;
                        } else {
                            motionAmount = dt/motionRemaining;
                            motionRemaining -= dt;
                        }

                        // Actually rotate
                        arm.rotateOnAxis(axis, motionAmount * angle);
                        arm.updateMatrixWorld();

                        return motionRemaining;
                    }

                    if (leftMotionRemaining > 0) {
                        leftMotionRemaining =
                            moveArm(larm,
                                leftPointTarget,
                                leftMotionRemaining);
                        if (leftMotionRemaining === 0) {
                            obj.app.fire('left-arm-done');
                        }
                    } else {
                        obj.flexShoulder('left', lArmAngularVelocity * dt);
                    }

                    if (rightMotionRemaining > 0) {
                        rightMotionRemaining =
                            moveArm(rarm,
                                rightPointTarget,
                                rightMotionRemaining);
                        if (rightMotionRemaining === 0) {
                            obj.app.fire('right-arm-done');
                        }
                    } else {
                        obj.flexShoulder('right', rArmAngularVelocity * dt);
                    }
                }
                obj.app.on('logic-tick', function (dt) { moveArms(dt); });

                function checkHandCollision(arm, target) {
                    var isRight = arm[0] === 'r' || arm[0] === 'R',
                        armObj = isRight ? rarm : larm,
                        handLocation = new THREE.Vector3(0, -forearmLength, 0).
                            applyMatrix4(armObj.children[0].matrixWorld),
                        targetBox = new THREE.Box3().setFromObject(target);

                        return targetBox.containsPoint(handLocation);
                }

                obj.pickUp = function (target) {
                    var llarm = larm.children[0], // lower left arm
                        lrarm = rarm.children[0], // lower right arm
                        llarmLocation = llarm.positionWorld(),
                        lrarmLocation = lrarm.positionWorld(),
                        targetLocation = target.positionWorld(),
                        llarmDist = targetLocation.distanceTo(llarmLocation),
                        lrarmDist = targetLocation.distanceTo(lrarmLocation),
                        isLeftCloser = llarmDist < lrarmDist,
                        arm;

                    // Which arm is used is based on distance and
                    //  hand fullness
                    if (( isLeftCloser  && !leftHandObject) ||
                        (!isLeftCloser &&   rightHandObject)) {
                        arm = 'left';
                    } else if ((!isLeftCloser && !rightHandObject) ||
                               ( isLeftCloser &&  leftHandObject)) {
                        arm = 'right';
                    }


                    // If we have a free hand, point it at the target and,
                    //  when it's pointed, try to grab the target
                    if (arm) {
                        this.pointArm(arm, target, 1);

                        obj.app.on(arm + '-arm-done', finishPickingUp);
                    }

                    // Check if we can reach the object
                    function finishPickingUp () {
                        if (checkHandCollision(arm, target)) {
                            obj.grab(target, arm);

                            if (isLeftCloser) {
                                leftHandObject = target;
                            } else {
                                rightHandObject = target;
                            }
                        }

                        obj.app.remove(arm + '-arm-done', finishPickingUp);
                    }
                };

                obj.pickUpNearest = function () {
                    var larmPos = larm.positionWorld(),
                        rarmPos = rarm.positionWorld(),
                        i,
                        target,
                        targetBox,
                        closestTarget,
                        closestDist = Infinity,
                        targetDist;

                    // Find the portable object that's closest to a shoulder
                    for (i = 0; i < obj.app.portables.length; i += 1) {
                        target = obj.app.portables[i];
                        targetBox = new THREE.Box3().setFromObject(target);
                        targetDist = Math.min(
                            targetBox.distanceToPoint(larmPos),
                            targetBox.distanceToPoint(rarmPos));
                        if (targetDist < closestDist) {
                            closestTarget = target;
                            closestDist = targetDist;
                        }
                    }

                    // If the object is in arm's length, pick it up
                    if (closestDist < armLength * scale) {
                        this.pickUp(closestTarget);

                        return true;
                    } else {
                        return false;
                    }
                };

                // Picks up the object that's in the hand, if there is one
                obj.pickUpAtHand = function (arm) {
                    var intersectingObj,
                        i;

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
                            leftHandObject = intersectingObj;
                        } else {
                            rightHandObject = intersectingObj;
                        }

                        return true;
                    } else {
                        return false;
                    }
                };

                Object.defineProperties(obj, {
                       leftHandObject: {
                               get: function () {
                                       return leftHandObject;
                               }
                       },
                       rightHandObject: {
                               get: function () {
                                       return rightHandObject;
                               }
                       }
                });

                document.addEventListener( 'keydown', function (event) {
                    switch (event.keyCode) {
                        case 82: // R
                            lArmAngularVelocity = +1;
                            break;
                        case 70: // F
                            lArmAngularVelocity = -1;
                            break;
                        case 84: // T
                            rArmAngularVelocity = +1;
                            break;
                        case 71: // G
                            rArmAngularVelocity = -1;
                            break;
                    }
                }, false );

                document.addEventListener( 'keyup', function (event) {
                    switch (event.keyCode) {
                        case 69: // E
                            if (leftHandObject) {
                                obj.release('left');
                            } else if (rightHandObject) {
                                obj.release('right');
                            } else {
                                obj.pickUpNearest();
                            }
                            break;
                        case 82: // R
                            lArmAngularVelocity = 0;
                            break;
                        case 70: // F
                            lArmAngularVelocity = 0;
                            break;
                        case 84: // T
                            rArmAngularVelocity = 0;
                            break;
                        case 71: // G
                            rArmAngularVelocity = 0;
                            break;
                    }
                }, false );

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
            steve: loadSteve
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
            that.rotation = obj.rotation;
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