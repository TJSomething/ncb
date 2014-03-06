/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console */

'use strict';

/////////////////////////////// Constants ////////////////////////////
var BOTSIM = {
    ASPECT: 16 / 9,
    FPS: 60,
    CHARACTER: 'mouse'
};

//////////////// Convenience functions and objects ////////////////////
// Yay, incompatibility! Probably should put this in its own file
window.requestAnimationFrame = window.requestAnimationFrame ||
                               window.mozRequestAnimationFrame ||
                               window.webkitRequestAnimationFrame ||
                               window.msRequestAnimationFrame;

// A nice implementation of the observer pattern from
//  Javascript patterns by Stoyan Stefanov
var publisher = {
    subscribers: {
        any: []
    },
    on: function (type, fn, context) {
        type = type || 'any';
        fn = typeof fn === 'function' ? fn : context[fn];
        
        if (typeof this.subscribers[type] === 'undefined') {
            this.subscribers[type] = [];
        }
        this.subscribers[type].push({fn: fn, context: context || this});
    },
    remove: function (type, fn, context) {
        this.visitSubscribers('unsubscribe', type, fn, context);
    },
    fire: function (type, publication) {
        this.visitSubscribers('publish', type, publication);
    },
    visitSubscribers: function (action, type, arg, context) {
        var pubtype = type || 'any',
            subscribers = this.subscribers[pubtype],
            i,
            max = subscribers ? subscribers.length : 0;
            
        for (i = 0; i < max; i += 1) {
            if (action === 'publish') {
                subscribers[i].fn.call(subscribers[i].context, arg);
            } else {
                if (subscribers[i].fn === arg && subscribers[i].context === context) {
                    subscribers.splice(i, 1);
                }
            }
        }
    }
};


function makePublisher(o) {
    var i;
    for (i in publisher) {
        if (publisher.hasOwnProperty(i) && typeof publisher[i] === 'function') {
            o[i] = publisher[i];
        }
    }
    o.subscribers = {any: []};
}

makePublisher(BOTSIM);

// Let's deal with the fact that we can't normally attach objects
//  to bones
THREE.Bone.prototype.update = (function () {
    var update = THREE.Bone.prototype.update;
    return function (parentSkinMatrix, forceUpdate) {
        update.call(this, parentSkinMatrix, forceUpdate);
        this.updateMatrixWorld( true );
    };
}());

THREE.Object3D.prototype.update = function() {};

// World position is really nice to have
THREE.Object3D.prototype.positionWorld = function () {
    return new THREE.Vector3().applyMatrix4(this.matrixWorld);
};

// Sometimes, we want the center more than the position
THREE.Object3D.prototype.centerWorld = function () {
    var targetBox = new THREE.Box3().setFromObject(this),
        center = new THREE.Vector3();

    // Find the center of the objects bounding box
    center.addVectors(targetBox.min, targetBox.max);
    center.multiplyScalar(0.5);

    return center;
};

/////////////////////////// Game Logic ///////////////////////////
// Methods that the robot needs
function Robot(obj, robotType) {
    var that = this;

    // Allow usage of new
    if (!(this instanceof Robot)) {
        return new Robot(obj, robotType);
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
    BOTSIM.scene.add(that);

    // Load the robot model asynchronously
    Robot.loadModel(that, robotType);

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
    BOTSIM.on('tick', that.move, that);
    
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
}

// This loads the robot model asynchronously
Robot.loadModel = function (obj, robotType) {
    Robot.modelLoaders[robotType](obj);
};

Robot.modelLoaders = {
    mouse:
        function (obj) {
            var loader = new THREE.ObjectLoader();

            loader.load('/assets/json/mouse.js', function (robotObj) {
                var camHeight, near;

                robotObj.scale.set(0.2, 0.2, 0.2);
                obj.add(robotObj);
                BOTSIM.physics.addObject(obj,
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

                BOTSIM.fire('robot-ready');
            });
        },
    steve:
        function (obj) {
            var loader = new THREE.JSONLoader(true);
            loader.load('/assets/json/steve.js', function (geometry, materials) {
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
                    scale = 2/3;

                // For some weird reason, Steve is the wrong size.
                steve.scale.set(scale, scale, scale);

                obj.add(steve);
                BOTSIM.physics.addObject(obj,
                    {
                        isStatic: false,
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
                obj.grab = function (target) {
                    var worldPosition = target.positionWorld(),
                        llarm = larm.children[0], // lower left arm
                        lrarm = rarm.children[0], // lower right arm
                        leftElbow = llarm.positionWorld(),
                        rightElbow = lrarm.positionWorld(),
                        distToLeftElbow = worldPosition.distanceTo(leftElbow),
                        distToRightElbow = worldPosition.distanceTo(rightElbow),
                        oldParentMatrix = target.parent.matrixWorld,
                        reverseParentTransform = new THREE.Matrix4();

                    // Attach the target to the nearest arm
                    if (distToLeftElbow < distToRightElbow) {
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
                    BOTSIM.physics.changeObjectState(target, 'held');
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
                    BOTSIM.physics.changeObjectState(target, 'falling');
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
                    armObj.rotation.y = Math.max(-Math.PI/2,
                        Math.min(Math.PI, roll));
                    armObj.rotation.z = Math.max(-Math.PI/4,
                            Math.min(3*Math.PI/4, horiz));
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
                    armObj.rotation.y = Math.max(-Math.PI/2,
                        Math.min(Math.PI, roll));
                    armObj.rotation.z = Math.max(-Math.PI/4,
                            Math.min(3*Math.PI/4, horiz));
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
                    armObj.rotation.y =
                        Math.max(-Math.PI/2,
                            Math.min(Math.PI,
                                armObj.rotation.y + roll));
                    armObj.rotation.z =
                        Math.max(-Math.PI/4,
                            Math.min(3*Math.PI/4,
                                armObj.rotation.z + horiz));
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

                obj.pointArm = function (arm, target, duration) {
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
                            BOTSIM.fire('left-arm-done');
                        }
                    }
                    if (rightMotionRemaining > 0) {
                        rightMotionRemaining =
                            moveArm(rarm,
                                rightPointTarget,
                                rightMotionRemaining);
                        if (rightMotionRemaining === 0) {
                            BOTSIM.fire('right-arm-done');
                        }
                    }
                }
                BOTSIM.on('tick', function (dt) { moveArms(dt); });

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

                        BOTSIM.on(arm + '-arm-done', finishPickingUp);
                    }

                    // Check if we can reach the object
                    function finishPickingUp () {
                        if (checkHandCollision(arm, target)) {
                            obj.grab(target);

                            if (isLeftCloser) {
                                leftHandObject = target;
                            } else {
                                rightHandObject = target;
                            }
                        }

                        BOTSIM.remove(arm + '-arm-done', finishPickingUp);
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
                    for (i = 0; i < BOTSIM.portables.length; i += 1) {
                        target = BOTSIM.portables[i];
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
                    }
                }, false );

                BOTSIM.fire('robot-ready');
            });
        }
};


BOTSIM.initViewport = function () {
    var width, height, left, top;

    this.container = document.getElementById('botsim-body');
    width = this.container.offsetWidth;
    height = width / this.ASPECT;
    left = this.container.offsetLeft;
    top = this.container.offsetTop;

    this.renderer = new THREE.WebGLRenderer({
        antialias: true,
        preserveDrawingBuffer: true
    });

    this.renderer.setSize(width, height);

    this.container.appendChild(this.renderer.domElement);
    
    // FPS indicators
    this.renderStats = new Stats();
    this.renderStats.domElement.style.position = 'absolute';
    this.renderStats.domElement.style.top = top + 'px';
    this.renderStats.domElement.style.left = left + 'px';
    this.renderStats.setMode(1);
    this.container.appendChild(this.renderStats.domElement);
    
    this.logicStats = new Stats();
    this.logicStats.domElement.style.position = 'absolute';
    this.logicStats.domElement.style.top = top + 50 + 'px';
    this.logicStats.domElement.style.left = left + 'px';
    this.logicStats.setMode(1);
    this.container.appendChild(this.logicStats.domElement);
};

BOTSIM.loadScene = function (files) {
    var tasksLeft = 0,
        i,
        imageLibrary = {},
        sceneSource,
        sceneBuffer,
        app = this,
        fileType = 'dae';

    // Loads each file
    function loadFile(file) {
        var name = file.name,
            type = file.type,
            reader = new FileReader();

        if (type.slice(0, 5) === 'image') {
            tasksLeft += 1;

            reader.onload = function () {
                imageLibrary[name] = reader.result;
                taskDone();
            };

            reader.readAsDataURL(file);
        } else if (name.slice(-3).toLowerCase() === 'dae') {
            tasksLeft += 1;

            fileType = 'dae';

            reader.onload = function () {
                sceneSource = reader.result;
                taskDone();
            };

            reader.readAsText(file);
        } else if (name.slice(-3).toLowerCase() === 'kmz') {
            tasksLeft += 1;

            fileType = 'kmz';

            reader.onload = function () {
                sceneBuffer = reader.result;
                taskDone();
            };

            reader.readAsArrayBuffer(file);
        }
    }

    // Assembles and shows the scene, if it's ready
    function taskDone() {
        var imageName,
            loader = new THREE.ColladaLoader(),
            xmlParser = new DOMParser(),
            xml;

        // If we don't do this, the axes will be wrong
        loader.options.convertUpAxis = true;

        tasksLeft -= 1;

        // Make sure it's ready
        if (tasksLeft <= 0) {
            if (fileType === 'dae' && sceneSource) {
                // Place the image data into the scene source
                for (imageName in imageLibrary) {
                    if (imageLibrary.hasOwnProperty(imageName)) {
                        sceneSource =
                            sceneSource.replace(imageName,
                                imageLibrary[imageName]);
                    }
                }

                // Now that the textures have been inlined with
                //  the scene, we can load it.
                xml = xmlParser.parseFromString(sceneSource,
                    'application/xml');
                
                loader.parse(xml, function (obj) {
                    app.scene = new THREE.Scene();
                    app.scene.add(obj.scene);

                    app.fire('scene-loaded');
                });
            } else if (fileType === 'kmz' && sceneBuffer) {
                loader = new THREE.KMZLoader(loader);

                loader.parse(sceneBuffer, function (obj) {
                    app.scene = new THREE.Scene();
                    app.scene.add(obj.scene);

                    app.fire('scene-loaded');
                });
            } else {
                window.alert('No level found. Try loading again.');
            }
        }
    }

    tasksLeft += 1;
    // Load all the files
    for (i = 0; i < files.length; i += 1) {
        loadFile(files[i]);
    }
    taskDone();
};

// This grabs and modifies objects in the scene that are important so that
//  the simulation can use them.
BOTSIM.readyScene = function () {
    var app = this,
        // This is so we can do a few asynchronous tasks
        tasksLeft = 0,
        camera,
        robot,
        light;

    this.initViewport();

    function taskDone() {
        tasksLeft -= 1;

        if (tasksLeft === 0) {
            app.fire('scene-ready');
        }
    }

    function initCamera(obj) {
        if (obj.hasOwnProperty('aspect')) {
            // We don't care what you say your aspect ratio is,
            //  we've already sized the viewport.
            obj.aspect = app.ASPECT;
            app.camera = obj;

            // I have no idea why this has to be done, but
            //  it has something to do with the Collada
            //  axis fixing.
            app.camera.rotateX(-Math.PI/2);

            app.controls = new THREE.OrbitControls(app.camera);
        }
    }

    function initRobot(obj) {
        if (obj.name === 'Robot' && !app.robot) {
            tasksLeft += 1;

            app.on('robot-ready', taskDone);

            // Set up our robot
            app.robot = new Robot(obj, BOTSIM.CHARACTER);
        }
    }

    app.portables = [];
    function initPortable(obj) {
        // If the word portable is in the name of the object,
        //  add it to the portable list
        if (/portable/i.test(obj.name)) {
            app.portables.push(obj);
            app.physics.addObject(obj, {isStatic: false});
        }
    }

    function addCollidable(obj) {
        var isThisPortable = false,
            ancestor = obj;
        if (obj.hasOwnProperty('geometry') &&
            obj.name !== 'Robot') {

            // Only add collidable mesh if none of the parents are portable
            while (ancestor) {
                if (/portable/i.test(ancestor.name) ||
                    ancestor.name === 'Robot') {
                    isThisPortable = true;
                }
                ancestor = ancestor.parent;
            }

            if (!isThisPortable) {
                app.physics.addObject(obj);
            }
        }
    }

    function fixTextures(obj) {
        function resizeNPOTImage(image) {
            var canvas, ctx;
            function isPowerOfTwo(x) {
                return (x & (x - 1)) === 0;
            }
             
            function nextHighestPowerOfTwo(x) {
                --x;
                for (var i = 1; i < 32; i <<= 1) {
                    x = x | x >> i;
                }
                return x + 1;
            }
            if (!isPowerOfTwo(image.width) || !isPowerOfTwo(image.height)) {
                // Scale up the texture to the next highest power of two dimensions.
                canvas = document.createElement('canvas');
                canvas.width = nextHighestPowerOfTwo(image.width);
                canvas.height = nextHighestPowerOfTwo(image.height);
                ctx = canvas.getContext('2d');
                ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
                image = canvas;
            }
            return image;
        }

        function fixMaterial(material) {
            ['lightMap', 'specularMap', 'envMap', 'map'].forEach(
                function (mapType) {
                    if (material[mapType]) {
                        material[mapType].image =
                            resizeNPOTImage(material[mapType].image);
                    }
                });
        }

        if (obj.material) {
            if (obj.material.materials) {
                obj.material.materials.forEach(fixMaterial);
            } else {
                fixMaterial(obj.material);
            }
        }
    }

    function initObject(obj) {
        initCamera(obj);
        initRobot(obj);
        initPortable(obj);
        addCollidable(obj);
        fixTextures(obj);
    }

    tasksLeft += 1;
    // So that we have reasonable matrices for the children
    app.scene.updateMatrixWorld();
    app.scene.traverse(initObject);

    // If we didn't add a camera or a robot, then place them in
    //  reasonable locations
    if (!app.robot) {
        robot = new THREE.Object3D();
        robot.position.set(0, 0, 0);
        app.scene.add(robot);

        initRobot(robot);
    }

    if (!app.camera) {
        camera = new THREE.PerspectiveCamera( 45, app.ASPECT, 0.1, 1000 );
        camera.position.set(5, 5, 5);
        camera.lookAt(app.robot.position);
        // This counters the hack used for Collada cameras
        camera.rotateX(Math.PI/2);
        app.scene.add(app.camera);

        initCamera(camera);
    }

    // TODO: Only add lights if there are none
    // Add some lights
    app.scene.add(new THREE.AmbientLight( 0x222222 ));
    light = new THREE.DirectionalLight(0xffffff, 0.7);
    light.position.set(1, 5, 1);
    app.scene.add(light);

    taskDone();
};

BOTSIM.startLoop = function () {
    var that = this,
        width = this.container.offsetWidth,
        height = this.container.offsetHeight,
        dpr = window.devicePixelRatio || 1,
        renderer = this.renderer,
        gl = renderer.getContext(),
        views = [
            {
                left: 0,
                bottom: 0,
                width: width * dpr,
                height: height * dpr,
                camera: this.camera
            },
            {
                left: 0,
                bottom: 0,
                width: Math.floor(width / 4) * dpr,
                height: Math.floor(height / 4) * dpr,
                camera: this.robot.camera,
                postRender: (function () {
                    var arraySize, i, intView,
                        rowView1, rowView2, tempRow, rows, cols;
                    
                    return function () {
                        if (!arraySize) {
                            rows = this.height;
                            cols = this.width;

                            arraySize = rows * cols * 4;

                            BOTSIM.cameraData = new Uint8Array(arraySize);

                            intView = new Uint32Array(BOTSIM.cameraData.buffer);

                            tempRow = new Uint32Array(cols);
                        }

                        gl.readPixels(this.left, this.bottom,
                            this.width, this.height, gl.RGBA,
                            gl.UNSIGNED_BYTE, BOTSIM.cameraData);

                        // Flip the image data
                        for (i = 0; i < rows / 2; i += 1) {
                            rowView1 = intView.subarray(i * cols,
                                (i + 1) *cols);
                            rowView2 = intView.subarray((rows - i - 1) * cols,
                                                        (rows - i) * cols);
                            tempRow.set(rowView1);
                            rowView1.set(rowView2);
                            rowView2.set(tempRow);
                        }
                    };
                }())
            }
        ];

    // Animate
    function run() {
        var i, left, bottom, width, height;
        
       that.renderStats.begin();

        for (i = 0; i < views.length; i += 1) {
            left = views[i].left;
            bottom = views[i].bottom;
            width = views[i].width;
            height = views[i].height;

            renderer.setViewport(left, bottom, width, height);
            renderer.setScissor(left, bottom, width, height);
            renderer.enableScissorTest(true);

            renderer.render(that.scene, views[i].camera);

            if (views[i].postRender) {
                views[i].postRender();
            }
        }

        window.requestAnimationFrame(run);
        
        that.renderStats.end();
        
        that.fire('render');
    }
    run();

    // A physics and stuff loop
    (function () {
        var time = Date.now()/1000,
            accumulator = 0,
            tickLength = 1/that.FPS;

        window.setInterval(
            function () {
                accumulator += Date.now()/1000 - time;
                time = Date.now()/1000;

                if (accumulator > tickLength) {
                    that.logicStats.begin();
                    that.fire('tick', accumulator - (accumulator % tickLength));
                    accumulator %= tickLength;
                    that.logicStats.end();
                }

            }, tickLength);
    }());
};

BOTSIM.physics = (function () {
    var staticObjects = [],
        dynamicObjects = [],
        // This is used to quickly address objects
        dynamicObjectIndices = {},
        staticObjectIndices = {},
        EPSILON = 0.000001;

    function initDynamicObject(obj) {
        var bWorld = (new THREE.Box3()).setFromObject(obj.geometry),
            Vec = THREE.Vector3,
            invMatrixWorld = (new THREE.Matrix4()).
                getInverse(obj.geometry.matrixWorld),
            // In local coordinates
            b = bWorld.clone().applyMatrix4(invMatrixWorld);

        // We're really going to be doing to do collisions with the
        //  bounding box
        obj.physics.collisionVertices =
           [new Vec( b.min.x, b.min.y, b.min.z ),
            new Vec( b.min.x, b.min.y, b.max.z ),
            new Vec( b.min.x, b.max.y, b.min.z ),
            new Vec( b.min.x, b.max.y, b.max.z ),
            new Vec( b.max.x, b.min.y, b.min.z ),
            new Vec( b.max.x, b.min.y, b.max.z ),
            new Vec( b.max.x, b.max.y, b.min.z ),
            new Vec( b.max.x, b.max.y, b.max.z )];

        obj.physics.boundingSphere =
            b.getBoundingSphere();

        Object.defineProperty(obj.physics, 'boundingSphereWorld',
            {
                get: function() {
                    return obj.physics.boundingSphere.clone().
                        applyMatrix4(obj.geometry.matrixWorld);
                }
            });

        Object.defineProperty(obj.physics, 'orientedBoundingBox',
            {
                get: (function () {
                        var c = obj.physics.boundingSphere.center,
                            // Half extents
                            e =  bWorld.max.clone().
                                sub(bWorld.min).
                                divideScalar(2),
                            mat = obj.geometry.matrixWorld;

                        return function () {
                            return {
                                // Put center into global coordinates
                                c: c.clone().applyMatrix4(mat),
                                // Decompose matrix into bases
                                u: (function () {
                                    var x = new Vec(mat.elements[0],
                                                    mat.elements[1],
                                                    mat.elements[2]),
                                        y = new Vec(mat.elements[4],
                                                    mat.elements[5],
                                                    mat.elements[6]),
                                        z = new Vec(mat.elements[8],
                                                    mat.elements[9],
                                                    mat.elements[10]);

                                    return [x.normalize(),
                                            y.normalize(),
                                            z.normalize()];
                                }()),
                                e: e
                            };
                        };
                    }())
            });

        obj.physics.orientedBox = (function () {
            var c = obj.physics.boundingSphere.center,
                // Half extents
                e =  bWorld.max.clone().sub(bWorld.min).divideScalar(2),
                mat = obj.geometry.matrixWorld;

            return function () {
                return {
                    // Put center into global coordinates
                    c: c.clone().applyMatrix4(mat),
                    // Decompose matrix into bases
                    u: (function () {
                        var x = new Vec(mat.elements[0],
                                        mat.elements[1],
                                        mat.elements[2]),
                            y = new Vec(mat.elements[4],
                                        mat.elements[5],
                                        mat.elements[6]),
                            z = new Vec(mat.elements[8],
                                        mat.elements[9],
                                        mat.elements[10]);

                        return [x.normalize(),
                                y.normalize(),
                                z.normalize()];
                    }()),
                    e: e
                };
            };
        }());

        dynamicObjects.push(obj);
        dynamicObjectIndices[obj.geometry.id] = dynamicObjects.length - 1;
    }

    function initStaticObject(obj) {
        obj.physics.faces = getGlobalFaces(obj);
        obj.physics.boundingSphereWorld =
            new THREE.Sphere().setFromPoints(
                obj.geometry.geometry.vertices);
        obj.physics.boundingSphereWorld.applyMatrix4(
            obj.geometry.matrixWorld);
        staticObjects.push(obj);
        staticObjectIndices[obj.geometry.id] = staticObjects.length - 1;
    }

    function getGlobalFaces(obj) {
        var vertices = obj.geometry.geometry.vertices,
            matrixWorld = obj.geometry.matrixWorld,
            // Transform our vertices into global coordinates
            globalVertices;

        if (vertices) {
            globalVertices = vertices.map(function (vertex) {
                return vertex.clone().applyMatrix4(matrixWorld);
            });
            return obj.geometry.geometry.faces.map(function (face) {
                return [globalVertices[face.a],
                        globalVertices[face.b],
                        globalVertices[face.c]];
            });
        } else {
            return [];
        }
    }

    // Adds the object to the physics engine
    function addObject(obj, physicalProperties) {
        var physicsObj = {
            geometry: obj,
            physics: (function () {
                var p = physicalProperties || {};

                return {
                    position: obj.positionWorld(),
                    lastGoodPosition: obj.positionWorld(),
                    quaternion: obj.quaternion,
                    lastGoodQuaternion: obj.quaternion,
                    velocity: p.velocity || new THREE.Vector3(0, 0, 0),
                    state: p.state || 'rest',
                    // The object should be static if this flag is not passed,
                    //  or to the passed value
                    isStatic: p.isStatic === undefined || p.isStatic
                };
            }())
        };

        if (physicsObj.physics.isStatic) {
            initStaticObject(physicsObj);
        } else {
            initDynamicObject(physicsObj);
        }
    }

    function updateObject(dt, obj, ignoreCollisions) {
        var lastPosition,
            position,
            velocity,
            quaternion,
            lastQuaternion;

        // Update position and velocity
        lastPosition = obj.physics.position;
        position = obj.geometry.positionWorld();
        velocity = position.
            clone().
            sub(lastPosition).
            divideScalar(dt);
        lastQuaternion = obj.physics.quaternion;
        quaternion = obj.geometry.quaternion;

        obj.physics.position = position;
        obj.physics.quaternion = quaternion;
        obj.physics.velocity = velocity;

        // Only note good positions if there are no collisions
        if (obj.geometry.collisions &&
            obj.geometry.collisions.length === 0 &&
            !ignoreCollisions) {
            obj.physics.lastGoodQuaternion = lastQuaternion;
            obj.physics.lastGoodPosition = lastPosition;
        }
    }

    function updateObjects(dt) {
        var i;

        // Update position and velocity
        for (i = 0; i < dynamicObjects.length; i += 1) {
            updateObject(dt, dynamicObjects[i]);
        }
    }

    function testOBBOBB(a, b) {
        var ra = 0,
            rb = 0,
            R = new THREE.Matrix3(),
            AbsR = new THREE.Matrix3(),
            i, j,
            t = new THREE.Vector3(),
            // Putting these into arrays for for-loops
            tArr,
            ae = [a.e.x, a.e.y, a.e.z],
            be = [b.e.x, b.e.y, b.e.z];

        // Compute rotation matrix expressing b in a's coordinate frame
        for (i = 0; i < 3; i += 1) {
            for (j = 0; j < 3; j += 1) {
                R.elements[3 * j + i] = a.u[i].dot(b.u[j]);
            }
        }

        // Compute translation vector t
        t.subVectors(b.c, a.c);
        // Bring translation into a's coordinate frame
        t = new THREE.Vector3(
            t.dot(a.u[0]),
            t.dot(a.u[1]),
            t.dot(a.u[2]));
        tArr = [t.x, t.y, t.z];

        // Compute common subexpressions. Add in an epsilon term to
        // counteract arthimetic errors when two edges are parallel and
        // their cross product is (near) null
        for (i = 0; i < 3; i += 1) {
            for (j = 0; j < 3; j += 1) {
                AbsR.elements[3 * j + i] =
                    Math.abs(R.elements[3 * j + i]) + EPSILON;
            }
        }

        // Test axes L = A0, L = A1, L = A2
        for (i = 0; i < 3; i += 1) {
            ra = ae[i];
            rb = b.e.x * AbsR.elements[i] +
                 b.e.y * AbsR.elements[3 + i] +
                 b.e.z * AbsR.elements[6 + i];
            if (Math.abs(tArr[i]) > ra + rb) {
                return false;
            }
        }

        // Test axes L = B0, L = B1, L = B2
        for (i = 0; i < 3; i += 1) {
            ra = a.e.x * AbsR.elements[3 * i] +
                 a.e.y * AbsR.elements[3 * i + 1] +
                 a.e.z * AbsR.elements[3 * i + 2];
            rb = be[i];
            if (Math.abs(t.x * R.elements[3 * i] +
                         t.y * R.elements[3 * i + 1] +
                         t.z * R.elements[3 * i + 2]) > ra + rb) {
                return false;
            }
        }
        

        // Test axis L = A0 x B0
        ra = ae[1] * AbsR.elements[3 * 0 + 2] +
             ae[2] * AbsR.elements[3 * 0 + 1];
        rb = be[1] * AbsR.elements[3 * 2 + 0] +
             be[2] * AbsR.elements[3 * 1 + 0];
        if (Math.abs(t[2] * R.elements[3 * 0 + 1] -
                     t[1] * R.elements[3 * 0 + 2]) > ra + rb) {
            return false;
        }

        // Test axis L = A0 x B1
        ra = ae[1] * AbsR.elements[3 * 1 + 2] +
             ae[2] * AbsR.elements[3 * 1 + 1];
        rb = be[0] * AbsR.elements[3 * 2 + 0] +
             be[2] * AbsR.elements[3 * 0 + 0];
        if (Math.abs(t[2] * R.elements[3 * 1 + 1] -
                     t[1] * R.elements[3 * 1 + 2]) > ra + rb) {
            return false;
        }

        // Test axis L = A0 x B2
        ra = ae[1] * AbsR.elements[3 * 2 + 2] +
             ae[2] * AbsR.elements[3 * 2 + 1];
        rb = be[0] * AbsR.elements[3 * 1 + 0] +
             be[1] * AbsR.elements[3 * 0 + 0];
        if (Math.abs(t[2] * R.elements[3 * 2 + 1] -
                     t[1] * R.elements[3 * 2 + 2]) > ra + rb) {
            return false;
        }

        // Test axis L = A1 x B0
        ra = ae[0] * AbsR.elements[3 * 0 + 2] +
             ae[2] * AbsR.elements[3 * 0 + 0];
        rb = be[1] * AbsR.elements[3 * 2 + 1] +
             be[2] * AbsR.elements[3 * 1 + 1];
        if (Math.abs(t[0] * R.elements[3 * 0 + 2] -
                     t[2] * R.elements[3 * 0 + 0]) > ra + rb) {
            return false;
        }

        // Test axis L = A1 x B1
        ra = ae[0] * AbsR.elements[3 * 1 + 2] +
             ae[2] * AbsR.elements[3 * 1 + 0];
        rb = be[0] * AbsR.elements[3 * 2 + 1] +
             be[2] * AbsR.elements[3 * 0 + 1];
        if (Math.abs(t[0] * R.elements[3 * 1 + 2] +
                     t[2] * R.elements[3 * 1 + 0]) > ra + rb) {
            return false;
        }
        // Test axis L = A1 x B2
        ra = ae[0] * AbsR.elements[3 * 2 + 2] +
             ae[2] * AbsR.elements[3 * 2 + 0];
        rb = be[0] * AbsR.elements[3 * 1 + 1] +
             be[1] * AbsR.elements[3 * 0 + 1];
        if (Math.abs(t[0] * R.elements[3 * 2 + 2] +
                     t[2] * R.elements[3 * 2 + 0]) > ra + rb) {
            return false;
        }
        // Test axis L = A2 x B0
        ra = ae[0] * AbsR.elements[3 * 0 + 1] +
             ae[1] * AbsR.elements[3 * 0 + 0];
        rb = be[1] * AbsR.elements[3 * 2 + 2] +
             be[2] * AbsR.elements[3 * 1 + 2];
        if (Math.abs(t[1] * R.elements[3 * 0 + 0] +
                     t[0] * R.elements[3 * 0 + 1]) > ra + rb) {
            return false;
        }
        // Test axis L = A2 x B1
        ra = ae[0] * AbsR.elements[3 * 1 + 1] +
             ae[1] * AbsR.elements[3 * 1 + 0];
        rb = be[0] * AbsR.elements[3 * 2 + 2] +
             be[2] * AbsR.elements[3 * 0 + 2];
        if (Math.abs(t[1] * R.elements[3 * 1 + 0] +
                     t[0] * R.elements[3 * 1 + 1]) > ra + rb) {
            return false;
        }
        // Test axis L = A2 x B2
        ra = ae[0] * AbsR.elements[3 * 2 + 1] +
             ae[1] * AbsR.elements[3 * 2 + 0];
        rb = be[0] * AbsR.elements[3 * 1 + 2] +
             be[1] * AbsR.elements[3 * 0 + 2];
        if (Math.abs(t[1] * R.elements[3 * 2 + 0] +
                     t[0] * R.elements[3 * 2 + 1]) > ra + rb) {
            return false;
        }
        // Since no separating axis is found, the OBBs must be intersecting
        return true;
    }

    function testOBBTri(obb, tri) {
        var i,
            v = [new THREE.Vector3(),
                 new THREE.Vector3(),
                 new THREE.Vector3()],
            e = [new THREE.Vector3(),
                 new THREE.Vector3(),
                 new THREE.Vector3()],
            fex,
            fey,
            fez,
            // Variables for axis tests
            p0, p1, p2, min, max, rad,
            vmax = new THREE.Vector3(),
            vmin = new THREE.Vector3(),
            normal = new THREE.Vector3();

        function planeBoxOverlap(normal, vert, maxbox) {
            var q, v;

            for (q = 0; q < 3; q += 1) {
                v = vert.getComponent(q);
                if (normal.getComponent(q) > 0) {
                    vmin.setComponent(q, -maxbox.getComponent(q) - v);
                    vmax.setComponent(q,  maxbox.getComponent(q) - v);
                } else {
                    vmin.setComponent(q,  maxbox.getComponent(q) - v);
                    vmax.setComponent(q, -maxbox.getComponent(q) - v);
                }
            }

            if (normal.dot(vmin) > 0) {
                return false;
            }
            if (normal.dot(vmax) >= 0) {
                return true;
            }

            return false;
        }

        // Axis tests
        function axisTestX01(a, b, fa, fb) {
            p0 = a * v[0].y - b * v[0].z;
            p2 = a * v[2].y - b * v[2].z;
            if (p0 < p2) {
                min = p0;
                max = p2;
            } else {
                min = p2;
                max = p0;
            }
            rad = fa * obb.e.y + fb * obb.e.z;
            if (min > rad || max < -rad) {
                return false;
            }
            return true;
        }

        function axisTestX2(a, b, fa, fb) {
            p0 = a * v[0].y - b * v[0].z;
            p1 = a * v[1].y - b * v[1].z;
            if (p0 < p1) {
                min = p0;
                max = p1;
            } else {
                min = p1;
                max = p0;
            }
            rad = fa * obb.e.y + fb * obb.e.z;
            if (min > rad || max < -rad) {
                return false;
            }
            return true;
        }

        function axisTestY02(a, b, fa, fb) {
            p0 = -a * v[0].x + b * v[0].z;
            p2 = -a * v[2].x + b * v[2].z;
            if (p0 < p2) {
                min = p0;
                max = p2;
            } else {
                min = p2;
                max = p0;
            }
            rad = fa * obb.e.x + fb * obb.e.z;
            if (min > rad || max < -rad) {
                return false;
            }
            return true;
        }

        function axisTestY1(a, b, fa, fb) {
            p0 = -a * v[0].x + b * v[0].z;
            p1 = -a * v[1].x + b * v[1].z;
            if (p0 < p1) {
                min = p0;
                max = p1;
            } else {
                min = p1;
                max = p0;
            }
            rad = fa * obb.e.x + fb * obb.e.z;
            if (min > rad || max < -rad) {
                return false;
            }
            return true;
        }

        function axisTestZ12(a, b, fa, fb) {
            p1 = a * v[1].x - b * v[1].y;
            p2 = a * v[2].x - b * v[2].y;
            if (p2 < p1) {
                min = p2;
                max = p1;
            } else {
                min = p1;
                max = p2;
            }
            rad = fa * obb.e.x + fb * obb.e.y;
            if (min > rad || max < -rad) {
                return false;
            }
            return true;
        }

        function axisTestZ0(a, b, fa, fb) {
            p0 = a * v[0].x - b * v[0].y;
            p1 = a * v[1].x - b * v[1].y;
            if (p0 < p1) {
                min = p0;
                max = p1;
            } else {
                min = p1;
                max = p0;
            }
            rad = fa * obb.e.x + fb * obb.e.y;
            if (min > rad || max < -rad) {
                return false;
            }
            return true;
        }

        function findMinMax(x0, x1, x2) {
            min = max = x0;
            if (x1 < min) {
                min = x1;
            } else if (x1 > max) {
                max = x1;
            }
            if (x2 < min) {
                min = x2;
            } else if (x2 > max) {
                max = x2;
            }
        }

        // Calculate the rotated triangle, which reduces this to an
        //  AABB-triangle test
        for (i = 0; i < 3; i += 1) {
            v[i].subVectors(tri[i], obb.c);
            v[i].set(v[i].dot(obb.u[0]),
                     v[i].dot(obb.u[1]),
                     v[i].dot(obb.u[2]));
        }

        // Compute edges
        for (i = 0; i < 3; i += 1) {
            e[i].subVectors(v[(i + 1) % 3], v[i]);
        }

        fex = Math.abs(e[0].x);
        fey = Math.abs(e[0].y);
        fez = Math.abs(e[0].z);
        if (!axisTestX01(e[0].z, e[0].y, fez, fey) ||
            !axisTestY02(e[0].z, e[0].x, fez, fex) ||
            !axisTestZ12(e[0].y, e[0].x, fey, fex)) {
            return false;
        }

        fex = Math.abs(e[1].x);
        fey = Math.abs(e[1].y);
        fez = Math.abs(e[1].z);
        if (!axisTestX01(e[1].z, e[1].y, fez, fey) ||
            !axisTestY02(e[1].z, e[1].x, fez, fex) ||
            !axisTestZ0(e[1].y, e[1].x, fey, fex)) {
            return false;
        }

        fex = Math.abs(e[2].x);
        fey = Math.abs(e[2].y);
        fez = Math.abs(e[2].z);
        if (!axisTestX2(e[2].z, e[2].y, fez, fey) ||
            !axisTestY1(e[2].z, e[2].x, fez, fex) ||
            !axisTestZ12(e[2].y, e[2].x, fey, fex)) {
            return false;
        }

        // Test for the AABB of the triangle
        findMinMax(v[0].x, v[1].x, v[2].x);
        if (min > obb.e.x ||
            max < -obb.e.x) {
            return false;
        }

        findMinMax(v[0].y, v[1].y, v[2].y);
        if (min > obb.e.y ||
            max < -obb.e.y) {
            return false;
        }

        findMinMax(v[0].z, v[1].z, v[2].z);
        if (min > obb.e.z ||
            max < -obb.e.z) {
            return false;
        }

        // Test if the box penetrates the triangle's plane
        normal.crossVectors(e[0], e[1]);
        if (!planeBoxOverlap(normal, v[0], obb.e)) {
            return false;
        }

        return {
            contactNormal: normal.normalize(),
            penetration: normal.dot(vmin)
        };
    }

    // Takes a dynamic physics object and checks if it's colliding
    //  with another dynamic physics object
    function detectDynamicCollision(obj1, obj2) {
        // Prevent self testing
        if (obj1 !== obj2) {
            // Oriented bounding box check
            if (testOBBOBB(obj1.physics.orientedBoundingBox,
                        obj2.physics.orientedBoundingBox)) {
                return true;
            }
        }

        return false;
    }

    // Detects a collision between a dynamic object and
    // a static object
    function detectSingleStaticCollision (obj, obj2) {
        var obb = obj.physics.orientedBoundingBox,
            sphere = obj.physics.boundingSphereWorld,
            result;
        obj2.physics.faces.some(function (face) {
            // Broad phase test with spheres
            if (sphere.intersectsSphere(
                    obj2.physics.boundingSphereWorld)) {
                result = testOBBTri(obb, face);
                result.otherObject = obj2;
                return result;
            }

            return false;
        });

        return result;
    }

    // Takes a dynamic physics object and finds if it's colliding with
    //  any static objects
    function detectStaticCollision(obj) {
        var obb = obj.physics.orientedBoundingBox,
            sphere = obj.physics.boundingSphereWorld,
            result = false;
        // Check if any face of any object intersects with
        //  this object's bounding box
        staticObjects.some(function (obj2) {
            return obj2.physics.faces.some(function (face) {
                // Broad phase test with spheres
                if (sphere.intersectsSphere(
                        obj2.physics.boundingSphereWorld)) {
                    result = testOBBTri(obb, face);
                    result.otherObject = obj2.geometry;
                    return result;
                }

                return false;
            });
        });

        return result;
    }

    function detectCollisions() {
        var i, j, collision;

        // Clear previous collisions
        for (i = 0; i < dynamicObjects.length; i += 1) {
            dynamicObjects[i].geometry.collisions = [];
        }

        // For each dynamic object
        for (i = 0; i < dynamicObjects.length; i += 1) {
            // Don't test objects at rest
            if (dynamicObjects[i].physics.state !== 'rest') {
                // Test against static objects
                collision = detectStaticCollision(dynamicObjects[i]);
                if (collision) {
                    collision.isEnvironment = true;
                    dynamicObjects[i].geometry.collisions.push(collision);
                }
            }

            // Test against dynamic objects
            for (j = i + 1; j < dynamicObjects.length; j += 1) {
                if (detectDynamicCollision(dynamicObjects[i],
                                           dynamicObjects[j])) {
                    dynamicObjects[i].geometry.collisions.push({
                        isEnvironment: false,
                        otherObject: dynamicObjects[j].geometry
                    });
                    dynamicObjects[j].geometry.collisions.push({
                        isEnvironment: false,
                        otherObject: dynamicObjects[i].geometry
                    });
                }
            }
        }
    }

    function changeObjectState(obj, newState) {
        getPhysicsObject(obj, false).physics.state = newState;
    }

    function getPhysicsObject(obj, isStatic) {
        if (!isStatic &&
            dynamicObjectIndices.hasOwnProperty(obj.id)) {
            return dynamicObjects[dynamicObjectIndices[obj.id]];
        }
        if (isStatic &&
            staticObjectIndices.hasOwnProperty(obj.id)) {
            return staticObjects[staticObjectIndices[obj.id]];
        }
    }

    function resolveCollision(dt, obj) {
        var worldToLocal = new THREE.Matrix4(),
            position,
            quaternion,
            lastPosition,
            lastQuaternion,
            isStatic,
            otherObject,
            ancestor;

        function stepObject(obj, t) {
            obj.geometry.position =
                lastPosition.clone().lerp(position, t);
            obj.geometry.quaternion =
                lastQuaternion.clone().slerp(quaternion, t);
            obj.geometry.updateMatrixWorld();
            updateObject(dt, obj, true);
        }

        function bisectCollision(movingObj, collidingObj1, collidingObj2,
                isStatic) {
            // t is the fraction of the step we're at
            var t = 1,
                tStep = 0.5,
                isColliding = true;
            while (dt * tStep > 0.000001) {
                // Step the object forward or backward in time
                if (isColliding) {
                    t -= tStep;
                } else {
                    t += tStep;
                }
                stepObject(movingObj, t);
                // Check if it's still colliding
                if (isStatic) {
                    isColliding = detectSingleStaticCollision(collidingObj1,
                        collidingObj2);
                } else {
                    isColliding = detectDynamicCollision(collidingObj1,
                        collidingObj2);
                }
                // Cut tStep in two
                tStep /= 2;
            }
            // If it's still colliding, step it back to a guaranteed
            //  good spot
            if (isColliding) {
                t -= tStep * 2;
                stepObject(movingObj, t);
            }
        }

        // If there is a collision
        if (obj.geometry.collisions.length > 0) {
            // Initialize convienience variables, but only if we have
            //  a collision
            worldToLocal.getInverse(obj.geometry.parent.matrixWorld);
            position = obj.physics.position.clone().
                    applyMatrix4(worldToLocal);
            quaternion = obj.physics.quaternion.clone();
            lastPosition =
                obj.physics.lastGoodPosition.clone().
                    applyMatrix4(worldToLocal);
            lastQuaternion =
                obj.physics.lastGoodQuaternion.clone();
            obj.geometry.collisions.forEach(function (collision) {
                isStatic = collision.isEnvironment;
                otherObject = getPhysicsObject(
                    collision.otherObject,
                    isStatic);
                switch (obj.physics.state) {
                    // If it's controlled, then revert to a previous
                    // state with no collision
                    case 'controlled':
                        // Ignore collisions with held objects
                        if (otherObject.state !== 'held') {
                            bisectCollision(obj, obj, otherObject, isStatic);
                        }
                        break;
                    case 'held':
                        /* We're just not going to bother with this for now.
                           I'm not sure it's worth it.
                        // If the object has moved, then revert the object
                        if (!lastPosition.equals(position) ||
                            !lastQuaternion.equals(quaternion)) {
                            bisectCollision(obj, obj, otherObject, isStatic);
                        } else {
                            // Otherwise, the parent moved the object, so
                            // find the object holding this one and back that
                            // up
                            ancestor = obj.geometry;
                            while (!getPhysicsObject(ancestor) ||
                                getPhysicsObject(ancestor).physics.state ===
                                'controlled') {
                                ancestor = ancestor.parent;
                            }
                            ancestor = getPhysicsObject(ancestor);
                            bisectCollision(ancestor, obj, otherObject, isStatic);
                        }
                        */
                        break;
                }
            });
        }
    }

    function resolveCollisions(dt) {
        var i;
        for (i = 0; i < dynamicObjects.length; i += 1) {
            resolveCollision(dt, dynamicObjects[i]);
        }
    }

    return {
            addObject: addObject,
            detectCollisions: detectCollisions,
            updateObjects: updateObjects,
            changeObjectState: changeObjectState,
            resolveCollisions: resolveCollisions
        };
}());

BOTSIM.on('scene-loaded', 'readyScene', BOTSIM);

BOTSIM.on('scene-ready', 'startLoop', BOTSIM);

BOTSIM.on('tick', function (dt) {
    this.controls.update(60 * dt);
}, BOTSIM);

BOTSIM.on('tick', function (dt) {
    BOTSIM.physics.updateObjects(dt);
    BOTSIM.physics.detectCollisions();
    BOTSIM.physics.resolveCollisions(dt);
});

BOTSIM.on('render', (function () {
    var canvas = document.createElement('canvas'),
        ctx = canvas.getContext('2d'),
        dpr = window.devicePixelRatio || 1,
        container = document.getElementById('botsim-body'),
        imageData;

    // This might be needed for a demo, but probably not
    //container.appendChild(canvas);

    return function () {
        if (!imageData) {
            canvas.height = Math.floor(container.offsetHeight / 4) * dpr;
            canvas.width = Math.floor(container.offsetWidth / 4) * dpr;
            imageData = ctx.createImageData(canvas.width, canvas.height);
        }

        imageData.data.set(BOTSIM.cameraData);
        ctx.putImageData(imageData, 0, 0);
        ctx.scale(1, -1);
    };
}()), BOTSIM);

///////////////////// GUI callbacks //////////////////////////

$('input[name=botsim-character]').on('change', function () {
    BOTSIM.CHARACTER = $(this).val();
});

$('#botsim-file').on('change', function () {
    BOTSIM.loadScene(this.files);
});

BOTSIM.on('scene-loaded', function () {
    $('div #botsim-options').hide();
});
