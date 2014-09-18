/* jslint browser: true */
/* global THREE, _:false, console */

define(['three', 'vbot/physics', 'vbot/motion'],
function (THREE, physics, motion) {
    'use strict';

    /** @exports vbot/Robot */

    /**
     * The methods needed for using arms. Requires several properties:
     *
     * @property {THREE.Bone} larm the left humerus
     * @property {THREE.Bone} rarm the right humerus
     * @property {Object} heldObject the two held objects
     * @property {?THREE.Object3D} heldObject.left
     * @property {?THREE.Object3D} heldObject.right
     * @property {Object} defaultArmUpVectors the up vectors used for rotating
     *                                        arms
     * @property {THREE.Vector3} defaultArmUpVectors.left
     * @property {THREE.Vector3} defaultArmUpVectors.right
     * @property {Array.<Number>} armMatrixOrder the order of the columns in the
     *   arm rotation matrix; 0 is the target vector, 1 is the up vector, and 2
     *   is perpendicular to both of those
     * @property {function} calculateHandLocation a function that, given a hand
     * as a string, left or right, will give the global coordinates of that hand
     *
     * @mixin
     * @memberof Robot~
     */
    var hasArms = {
        /**
         * Attaches an object to the given arm.
         *
         * @instance
         * @param  {THREE.Object3D} target the object to attach to the arm
         * @param  {string=} arm           the arm to attach to; defaults to the
         *                                 nearest arm to the given object
         */
        grab: function (target, arm) {
            var worldPosition = target.positionWorld(),
                llarm = this.larm.children[0], // lower left arm
                lrarm = this.rarm.children[0], // lower right arm
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

            // Note that we grabbed the object
            if (isLeft) {
                this.heldObjects.left = target;
            } else {
                this.heldObjects.right = target;
            }
            // Note that we should do collision detection for this
            physics.changeObjectState(target, 'held');
        },
        /**
         * Detaches the held object from the given arm.
         *
         * @instance
         * @param  {string} arm the arm, either "left" or "right"
         */
        release: function (arm) {
            var isLeft = (arm[0] === 'l' || arm[0] === 'L'),
                target = isLeft ?
                    this.heldObjects.left : this.heldObjects.right,
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
                    this.heldObjects.left = null;
                } else {
                    this.heldObjects.right = null;
                }
            }

            // Switch to a falling state
            physics.changeObjectState(target, 'falling');
        },
        /**
         * Points an arm at a target vector, oriented with an up vector
         *
         * @instance
         * @param {string} arm which arm to move
         * @param {THREE.Vector3} targetVector where the arm should point
         * @param {THREE.Vector3=} upVector the direction the top of the arm
         * @param {boolean=} isGlobal whether targetVector is local (default) or
         *                            global
         */
        instantPointArm: function (arm, targetVector, isGlobal, upVector) {
            obj.rotateArmTowards(arm, targetVector, 1, false, isGlobal,
                                 upVector);
        },
        /**
         * Rotates an arm toward pointing at a location
         *
         * @instance
         * @param {arm} string                 which arm to move
         * @param {THREE.Vector3} targetVector where the arm should point
         * @param {Number} amount              the amount to move the arm, in
         *                                     either radians or percent of
         *                                     motion remaining
         * @param {boolean} inRadians          whether the amount is in radians;
         *                                     if not then it's assumed that
         *                                     it's a fraction of the arc
         * @param {boolean=} isGlobal          whether targetVector is local
         *                                     (default) or global
         * @param {THREE.Vector3=} upVector    the direction the top of the arm,
         *                                     optional
         * @return {boolean}                   if we're done rotating the arm
         *                                     toward the given vector
         */
        rotateArmTowards: function (arm, targetVector, amount, inRadians,
                                    isGlobal, upVector) {
            var isLeft = arm[0] === 'l' || arm[0] === 'L',
                armObj = isLeft ? this.larm : this.rarm,
                localMatrix = new THREE.Matrix4(),
                shoulderLoc = armObj.positionWorld(),
                shoulderDisplacement = shoulderLoc.clone().sub(this.positionWorld()),
                thirdBasis = new THREE.Vector3(),
                rotMatrix,
                angleDiff,
                s;

            // Don't want to mangle the formal parameters
            targetVector = targetVector.clone();

            localMatrix.getInverse(this.matrixWorld);

            // isGlobal defaults to false
            if (isGlobal === undefined) {
                isGlobal = false;
            }

            // I've found that you get the most intuitive motion if you
            // have up be up and out for the arm
            if (upVector === undefined) {
                upVector = isLeft ? this.defaultArmUpVectors.left.clone() :
                                    this.defaultArmUpVectors.right.clone();
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
            var b = [targetVector,
                     upVector,
                     thirdBasis];
            // Reorder depending on model
            b = [b[this.armMatrixOrder[0]],
                 b[this.armMatrixOrder[1]],
                 b[this.armMatrixOrder[2]]];
            rotMatrix = new THREE.Matrix4(
                b[0].x, b[1].x, b[2].x, 0,
                b[0].y, b[1].y, b[2].y, 0,
                b[0].z, b[1].z, b[2].z, 0,
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
        },
        /**
         * A function that picks up the object that's intersecting with the
         * given hand
         *
         * @instance
         * @param  {string} arm the side that the hand is on ('left' or 'right')
         * @return {boolean}    whether an object was picked up
         */
       pickUpAtHand: function (arm) {
            var intersectingObj,
                i;

            /**
             * Checks if an object is intersecting with the given hand
             *
             * @param  {string} arm            the side that the hand is on
             *                                 ('left' or 'right')
             * @param  {THREE.Object3D} target the object we're checking for
             *                                 intersection
             * @return {boolean}               whether the target is
             *                                 intersecting with the given hand
             */
            function checkHandCollision(arm, target) {
                var isRight = arm[0] === 'r' || arm[0] === 'R',
                    armObj = isRight ? this.rarm : this.larm,
                    handLocation = this.calculateHandLocation(armObj),
                    targetBox = new THREE.Box3().setFromObject(target);

                    return targetBox.containsPoint(handLocation);
            }

            // Find an object that's intesecting with the hand
            for (i = 0; i < this.app.portables.length; i += 1) {
                if (checkHandCollision(arm, this.app.portables[i])) {
                    intersectingObj = this.app.portables[i];
                    break;
                }
            }

            if (intersectingObj !== undefined) {
                this.grab(intersectingObj, arm);

                return true;
            } else {
                return false;
            }
        },
        
        /**
         * Moves the hand toward a target position.
         *
         * TODO: parameterize or derive:
         *  - palm position
         *  - joint limits
         *  - distance threshold
         *  - model scale
         *
         * @instance
         * @param {string} arm the name of the arm to move
         * @param {Number} planningStepSize the distance to move
         *                                  the target configuration
         *                                  while hill climbing;
         *                                  units are in cm
         * @param {Number} motionStepSize the angle to move the
         *                                joints; in radians
         * @param {THREE.Vector3} targetWristPos the location to
         *                          move the wrist toward in
         *                          world space
         * @param {THREE.Vector3} targetPalmPos the location to
         *                          move the palm toward in
         *                          world space
         * @param {THREE.Vector3} targetCenterPos the location
         *                          at the center of the object
         *                          we're reaching for
         * @return {string} the status of moving the arm, which
         *                  can be one of three values: success,
         *                  failure, and moving
         */
        stepHandTowardsTarget: function (arm,
                                         planningStepSize,
                                         motionStepSize,
                                         targetWristPos,
                                         targetPalmPos,
                                         targetCenterPos) {
            var isLeft = (arm[0] === 'l' || arm[0] === 'L'),
                sideName = isLeft ? 'left' : 'right',
                shoulder = isLeft ? this.larm : this.rarm,
                shoulderMat = shoulder.matrix,
                elbow = shoulder.children[0],
                wrist = elbow.children[0],
                elbowMat = elbow.matrix,
                wristMat = wrist.matrix,
                handMat = wrist.matrixWorld,
                upperArmLength = elbow.matrix.elements[12],
                lowerArmLength = wrist.matrix.elements[12],
                torsoMat = new THREE.Matrix4().getInverse(shoulder.parent.matrixWorld),
                localTargetWristPos = targetWristPos.clone().applyMatrix4(torsoMat),
                localTargetPalmPos = targetPalmPos.clone().applyMatrix4(torsoMat),
                localTargetCenterPos = targetCenterPos.clone().applyMatrix4(torsoMat),
                localPalmPos = new THREE.Vector3(0, -3, 0),
                i, j, lastVal,
                newConfig,
                lastDist, currentDist;
                
            var lowerLimit = [-0.5, -0.4, -0.4, -0.75, 0, -0.4, -0.25];
            var upperLimit = [0.5, 0.4, 0.4, 0, 0.5, 0.4, 0.25];
            
            /**
             * Clamps the values of this config in place.
             */
            function clampConfig(config) {
                for (i = 0; i < 7; i += 1) {
                    config[i] = THREE.Math.clamp(config[i],
                                    lowerLimit[i]*Math.PI,
                                    upperLimit[i]*Math.PI);
                }
                
                return config;
            }
            
            function randomConfig() {
                var config = new Array(7);
                for (i = 0; i < 7; i += 1) {
                    config[i] = Math.PI * (
                                    Math.random() *
                                    (upperLimit[i] - lowerLimit[i]) +
                                    lowerLimit[i]);
                }
                
                return config;
            }
            
            function distanceToGoal(config) {
                return motion.handDist(shoulderMat,
                                       upperArmLength,
                                       lowerArmLength,
                                       localPalmPos,
                                       localTargetWristPos,
                                       localTargetPalmPos,
                                       config);
            }
            
            function stepTowardsGoal(config) {
                return motion.stepConfig(shoulderMat,
                                         config,
                                         planningStepSize,
                                         upperArmLength,
                                         lowerArmLength,
                                         localPalmPos,
                                         localTargetWristPos,
                                         localTargetPalmPos)
            }
              
            // Make sure our configuration and targeting data is initialized
            if (this.planningData === undefined) {
                this.planningData = {};
            }
            if (this.planningData[sideName] === undefined) {
                this.planningData[sideName] = {
                    config: motion.calcConfig(shoulderMat, elbowMat, wristMat)
                };
            }
            var plan = this.planningData[sideName];
            // Compensate for rounding errors in the position of the robot
            if (plan.targetCenterPos !== undefined &&
                plan.targetCenterPos.distanceTo(localTargetCenterPos) < 0.0001) {
                    plan.targetCenterPos = localTargetCenterPos;
                }
            // If the target has changed or has never been set, update the target
            // and reset the best config and distance for the best config
            if (plan.targetWristPos === undefined ||
                plan.targetPalmPos === undefined ||
                plan.targetCenterPos === undefined ||
                !plan.targetCenterPos.equals(localTargetCenterPos)) {
                plan.targetWristPos = localTargetWristPos;
                plan.targetPalmPos = localTargetPalmPos;
                plan.targetCenterPos = localTargetCenterPos;
                plan.bestConfig = plan.config.slice(0);
                plan.bestDist = distanceToGoal(plan.config);
            }
            
            // Spend 10 ms searching for a better target configuration
            var startTime = Date.now();
            while (Date.now() - startTime < 10) {
                // Inspired by Weghe, et al.'s JT-RRT algorithm
                if (Math.random() < 0.5) {
                    newConfig = randomConfig();
                } else {
                    newConfig = stepTowardsGoal(plan.bestConfig);
                }
                // Make sure the new config is valid
                clampConfig(newConfig);
                // If the new configuration's workspace distance is closer
                // than the previous best found, then keep that
                currentDist = distanceToGoal(newConfig);
                if (currentDist < plan.bestDist) {
                    plan.bestConfig = newConfig.slice(0);
                    plan.bestDist = currentDist;
                }
            }
            
            // Move toward the best configuration
            var configDist = 0;
            var configDiff = new Array(7);
            for (i = 0; i < 7; i += 1) {
                configDiff[i] = plan.bestConfig[i] - plan.config[i]
                configDist += Math.pow(configDiff[i], 2);
            }
            configDist = Math.sqrt(configDist);
            // Calculate the percentage of the distance to move.
            // If a step would take us past our target, then just
            // take us to the target
            var amountToMove = configDist > motionStepSize ?
                motionStepSize/configDist :
                1;
            for (i = 0; i < 7; i += 1) {    
                plan.config[i] += configDiff[i] * amountToMove;
            }
            clampConfig(plan.config);
            
            var newMatrices = motion.configToMatrices(shoulderMat,
                                                      upperArmLength,
                                                      lowerArmLength,
                                                      plan.config);
            
            shoulder.matrixAutoUpdate = false;
            elbow.matrixAutoUpdate = false;
            wrist.matrixAutoUpdate = false;
            shoulderMat.copy(newMatrices.shoulder);
            elbowMat.copy(newMatrices.elbow);
            wristMat.copy(newMatrices.wrist);
            
            // If we've reached our target configuration and
            // the hand is close enough to the target, then
            // we're done
            if (configDist === 0) {
                if (plan.bestDist < 20) {
                    return 'success';
                } else {
                    // If we're not close enough, the we've failed
                    return 'failure';
                }
            } else {
                // If we're not there yet, we're still moving
                return 'moving';
            }
        }
    }

    /**
     * Adds the hasArms mixin to the given robot.
     *
     * @memberof Robot~
     * @param  {Robot} obj
     */
    function addArms(obj) {
        Object.keys(hasArms).forEach(function (methodName) {
            obj[methodName] = hasArms[methodName];
        });
    }


    var Robot = (function () {
        /**
         * Loads the Steve model, puts it as a child under the given object,
         * and loads the appropriate methods.
         *
         * @class
         * @memberof Robot~
         * @augments {Robot}
         * @augments {Robot~hasArms}
         * @param  {THREE.Object3D} obj the object in the scene that represents
         *                              where the robot should be placed

         */
        function Steve(obj) {
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
                    scale = 2/3,
                    bounds;

                // For some weird reason, Steve is the wrong size.
                steve.scale.set(scale, scale, scale);

                obj.add(steve);
                physics.addObject(obj,
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
                bounds = (new THREE.Box3()).setFromObject(steve);
                // The camera should be at eye level
                camHeight = 0.75 * bounds.max.y;
                // The near clipping plane should be a little further out
                //  than the front
                near = 1.01 * bounds.max.z;
                // Make our camera
                obj.camera = new THREE.PerspectiveCamera(45, 4 / 3, near, 1000);
                obj.camera.position.set(0, camHeight, 0);
                obj.camera.rotateY(Math.PI); // Otherwise, it'll point backward
                obj.add(obj.camera);

                /**
                 * Makes a function to animate Steve walking.
                 *
                 * @param  {THREE.Bone} lleg    Left femur
                 * @param  {THREE.Bone} rleg    Right femur
                 * @param  {Number} maxLegAngle the maximum angle of a leg
                 * @return {Function}           a function that actually
                 *                              animates
                 */
                obj.animateWalking = function(lleg, rleg, maxLegAngle) {
                    var walkState = 0,
                        lrleg = rleg.children[0],
                        llleg = lleg.children[0];

                    /**
                     * Animate Steve's walking.
                     *
                     * @memberof Robot~Steve#
                     * @param  {Number} ds     the distance moved
                     * @param  {Number} dtheta the amount rotated
                     */
                    function animateWalking(ds, dtheta) {
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
                    return animateWalking;
                }(lleg, rleg, Math.PI/9);

                /**
                 * Calculates the hand location of Steve as one forearm length
                 * below the given humerus' child
                 *
                 * @memberof Robot~Steve#
                 * @param  {THREE.Bone} armObj a humerus
                 * @return {THREE.Vector3}     the hand location
                 */
                function calculateHandLocation(armObj) {
                    return new THREE.Vector3(0, -forearmLength, 0).
                        applyMatrix4(armObj.children[0].matrixWorld)
                }

                // Set up the properties needed to have arms
                obj.larm = larm;
                obj.rarm = rarm;
                obj.heldObjects = heldObjects;
                obj.defaultArmUpVectors = {
                    left: new THREE.Vector3(1, -1, 0),
                    right: new THREE.Vector3(1, -1, 0)
                }
                obj.armMatrixOrder = [0, 1, 2];
                obj.calculateHandLocation = calculateHandLocation;
                // Add all the arm movement methods
                addArms(obj);

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

        /**
         * Loads the Carl model as the robot
         *
         * @class
         * @memberof Robot~
         * @augments {Robot}
         * @augments {Robot~hasArms}
         * @param  {THREE.Object3D} obj the object in the scene that represents
         *                              where the robot should be placed
         */
        function Carl(obj) {
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
                physics.addObject(obj,
                    {
                        type: 'dynamic',
                        state: 'controlled'
                    });

                for (i = 0; i < materials.length; i += 1) {
                    // For bones
                    materials[i].skinning = true;
                }

                // Move the arms down
                //larm.rotateZ(-Math.PI/2);
                //rarm.rotateZ(Math.PI/2);

                // Place a camera in the robot
                // Find the bounding box
                var bounds = (new THREE.Box3()).setFromObject(carl);
                // The camera should be at eye level
                camHeight = 0.95 * bounds.max.y;
                // The near clipping plane should be a little further out
                //  than the front
                near = 1.01 * bounds.max.z;
                // Make our camera
                obj.camera = new THREE.PerspectiveCamera(45, 4 / 3, near, 1000);
                obj.camera.position.set(0, camHeight, 0);
                obj.camera.rotateY(Math.PI); // Otherwise, it'll point backward
                obj.add(obj.camera);

                /**
                 * Generate a function for Carl walking.
                 *
                 * @param  {THREE.Bone} rleg    right femur
                 * @param  {THREE.Bone} lleg    left femur
                 * @param  {Number} maxLegAngle how much to swing the femurs
                 *                              away from vertical, in radians
                 * @return {Function}           the actual animation function
                 */
                obj.animateWalking = (function (rleg, lleg, maxLegAngle) {
                    var walkState = 0,
                        lrleg = rleg.children[0],
                        llleg = lleg.children[0],
                        lankle = llleg.children[0],
                        rankle = lrleg.children[0],
                        legAngle = 0,
                        lastDs = 0;

                    /**
                     * Animate Carl walking.
                     *
                     * @param  {Number} ds     the distance moved
                     * @param  {Number} dtheta the angle rotated

                     */
                    return function (ds, dtheta) {
                        // Don't let undefined through
                        ds = ds || 0;
                        dtheta = dtheta || 0;

                        // If we just started walking, slowly increase our stride
                        if (Math.abs(ds) > 0) {
                            // Remember the current walking rate as a approximation
                            // of speed, which we use to figure out deceleration
                            // speed
                            lastDs = Math.abs(ds);

                            if (legAngle + lastDs < maxLegAngle) {
                                legAngle += lastDs;
                            } else {
                                legAngle = maxLegAngle;
                            }
                        } else if (Math.abs(dtheta) === 0) {
                            if (legAngle - lastDs > 0) {
                                legAngle -= lastDs;
                            } else {
                                legAngle = 0;
                                walkState = 0;
                            }
                        }

                        // Change our walk state
                        walkState += ds;

                        var stride = legAngle/maxLegAngle;
                        rleg.rotation.x =
                            stride * maxLegAngle * Math.sin(walkState / maxLegAngle);
                        lleg.rotation.x =
                            stride * -maxLegAngle * Math.sin(walkState / maxLegAngle);
                        lrleg.rotation.x = stride * (maxLegAngle -
                            maxLegAngle * Math.cos(walkState / maxLegAngle));
                        llleg.rotation.x = stride * (maxLegAngle +
                            maxLegAngle * Math.cos(walkState / maxLegAngle));
                        rankle.rotation.x = stride * (-maxLegAngle/2 +
                            maxLegAngle/2 * Math.cos(walkState / maxLegAngle));
                        lankle.rotation.x = stride * (-maxLegAngle/2 -
                            maxLegAngle/2 * Math.cos(walkState / maxLegAngle));
                    };
                }(rleg, lleg, Math.PI/9));

                /**
                 * Calculates the location of the hand to see if it's picking
                 * something up.
                 *
                 * @memberof Robot~Carl#
                 * @param  {THREE.Bone} armObj the humerus above the wanted hand
                 * @return {THREE.Vector3} the location of the hand
                 */
                function calculateHandLocation(armObj) {
                    return armObj.children[0].children[0].children[0].positionWorld();
                }
                obj.calculateHandLocation = calculateHandLocation;

                // Set up the properties needed to have arms
                obj.larm = larm;
                obj.rarm = rarm;
                obj.heldObjects = heldObjects;
                obj.defaultArmUpVectors = {
                    left: new THREE.Vector3(1, 0, 0),
                    right: new THREE.Vector3(-1, 0, 0)
                }
                obj.armMatrixOrder = [0, 2, 1];
                obj.calculateHandLocation = calculateHandLocation;
                // Add all the arm movement methods
                addArms(obj);

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

        /**
         * Loads the computer mouse model as the robot.
         *
         * @class
         * @memberof Robot~
         * @augments {Robot}
         * @param  {THREE.Object3D} obj the object in the scene that represents
         *                              where the robot should be placed
         */
        function Mouse(obj) {
            var loader = new THREE.ObjectLoader();

            loader.load('assets/json/mouse.js', function (robotObj) {
                var camHeight, near;

                robotObj.scale.set(0.2, 0.2, 0.2);
                obj.add(robotObj);
                physics.addObject(obj,
                    {
                        isStatic: false,
                        state: 'controlled'
                    });

                // Place a camera in the robot
                // Find the bounding box
                var bounds = (new THREE.Box3()).setFromObject(robotObj);
                // The camera should be 3/4 up the robot
                camHeight = 0.75 * bounds.max.y;
                // The near clipping plane should be a little further out
                //  than the front
                near = 1.05 * bounds.max.z;
                // Make our camera
                obj.camera = new THREE.PerspectiveCamera(45, 4 / 3, near, 1000);
                obj.camera.position.set(0, camHeight, 0);
                obj.camera.rotateY(Math.PI); // Otherwise, it'll point backward
                obj.add(obj.camera);

                /**
                 * A stand-in for animating the mouse, as the mouse is not
                 * animated.
                 *

                 */
                obj.animateWalking = function () {};

                obj.app.fire('robot-ready');
            });
        }

        var modelLoaders = {
            mouse: Mouse,
            steve: Steve,
            carl: Carl
        };

        /**
         * Loads the given robot underneath the given model.
         *
         * @class Robot
         * @augments {THREE.Object3D}
         * @property {Object} app the application
         * @property {Number} speed the current walking speed in m/s
         * @property {Number} angularVelocity the current rotation speed in rad/s
         * @property {THREE.PerspectiveCamera} camera the robot's "eyes"
         *
         * @param  {THREE.Object3D} obj An object used to set the initial
         *                              position of the robot
         * @param  {string} robotType   Which robot to load (mouse, steve, or
         *                              carl)
         * @param {Object} app          the virtual robot application state
         *                              object
         */
        function Robot(obj, robotType, app) {
            // Clear the dummy geometry used to stand in for
            //  the robot
            obj.geometry = new THREE.Geometry();

            // We're going to make a new object that isn't subject to the scaling
            //  in the scene and won't kill accidental children
            var robot = new THREE.Object3D();
            robot.position.copy(obj.position);
            robot.quaternion.copy(obj.quaternion);
            robot.name = 'Robot Parent';
            // We need a reference to the VBOT module to avoid a circular
            // dependency
            robot.app = app;
            // Put the robot into the scene
            app.scene.add(robot);
            // Odometer!
            robot.odometer = 0;

            // Load the robot model asynchronously
            modelLoaders[robotType](robot);

            /**
             * Move forward instantly
             *
             * @memberof Robot#
             * @param  {Number} displacement the displacement forward in
             *                               meters
             */
            function instantForward(displacement) {
                var model = new THREE.Vector3(0, 0, displacement),
                    world = model.applyQuaternion(this.quaternion);


                this.animateWalking(displacement);
                this.position.add(world);
                this.odometer += displacement;
            }
            robot.instantForward = instantForward;

            /**
             * Move backwards instantly
             *
             * @memberof Robot#
             * @param  {Number} displacement the displacement backwards in
             *                               meters
             */
             function instantBackward(displacement) {
                this.instantForward(-displacement);
            };
            robot.instantBackward = instantBackward;

            /**
             * Turns the robot right instantly
             *
             * @memberof Robot#
             * @param  {Number} displacement the number of radians to turn right
             */
            function instantRight(displacement) {
                this.animateWalking(0, displacement);
                this.rotateY(displacement);
            };
            robot.instantRight = instantRight;

            /**
             * Turns the robot left instantly.
             *
             * @memberof Robot#
             * @param {Number} displacement the number of radians to turn left
             */
            function instantLeft(displacement) {
                this.instantRight(-displacement);
            };
            robot.instantLeft = instantLeft;

            /**
             * This goes forward and turns simultaneously, teleporting to robot
             * to the location it would reach if it had been travelling forward
             * and turning simultaneously.
             *
             * @memberof Robot#
             * @param  {Number} ds     the number of meters to travel forward
             * @param  {Number} dtheta the number of radians to turn right by
             */
            function simultaneousTurnMove(ds, dtheta) {
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
            robot.simultaneousTurnMove = simultaneousTurnMove;


            /**
             * Moves the robot in accordance with the current walking and
             * turning speed.
             *
             * @param  {Number} dt the time step to advance by
             */
            function move(dt) {
                var ds = this.speed * dt,
                    dtheta = this.angularVelocity * dt;

                // Move
                this.simultaneousTurnMove(ds, dtheta);
            };
            robot.move = move;

            robot.app.on('logic-tick', robot.move, robot);
            
            // Properties
            robot.speed = 0;
            robot.angularVelocity = 0;

            return robot;
        }
        Robot.prototype = THREE.Object3D.prototype;

        return Robot;
    }());

    return Robot;
});
