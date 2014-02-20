/* jslint browser: true */
/* global THREE, $:false */

'use strict';

/////////////////////////////// Constants ////////////////////////////
var BOTSIM = BOTSIM || {
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

        // Check for collisions
        if (this.detectCollisions()) {
            // Resolve the collision
            this.simultaneousTurnMove(-ds, -dtheta);
        }
    };
    BOTSIM.on('tick', that.move, that);


    that.detectCollisions = function () {
        var idx,
            originPoint = this.position.clone(),
            localVertex,
            globalVertex,
            directionVector,
            ray,
            collisionResults;

        // Lazy initialization!
        that.collisionVertices = that.collisionVertices || (function () {
            var b = that.bounds,
                Vec = THREE.Vector3;
            // We're really going to be doing to do collisions with the
            //  bounding box of the robot
            return [new Vec( b.min.x, b.min.y, b.min.z ),
                    new Vec( b.min.x, b.min.y, b.max.z ),
                    new Vec( b.min.x, b.max.y, b.min.z ),
                    new Vec( b.min.x, b.max.y, b.max.z ),
                    new Vec( b.max.x, b.min.y, b.min.z ),
                    new Vec( b.max.x, b.min.y, b.max.z ),
                    new Vec( b.max.x, b.max.y, b.min.z ),
                    new Vec( b.max.x, b.max.y, b.max.z )];
        }());

        for (idx = 0; idx < that.collisionVertices.length; idx += 1) {
            localVertex = that.collisionVertices[idx].clone();
            globalVertex = localVertex.applyMatrix4(this.matrix);
            directionVector = globalVertex.sub(this.position);
            ray = new THREE.Raycaster(originPoint, directionVector.clone().normalize());
            collisionResults = ray.intersectObjects(BOTSIM.collidableMeshList);

            if (collisionResults.length > 0 &&
                collisionResults[0].distance < directionVector.length()) {
                return true;
            }
        }

        return false;
    };

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
            case 83: // W
                that.speed = 0;
                break;
            case 87: // A
                that.speed = 0;
                break;
            case 65: // S
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
                    rightPointTarget;

                // For some weird reason, Steve is the wrong size.
                steve.scale.set(0.66667, 0.66667, 0.66667);

                obj.add(steve);

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
                        maxLegAngle = Math.PI/9;

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
                    };
                }());

                // This anchors an object to the nearest arm
                obj.grab = function (target) {
                    var worldPosition = new THREE.Vector3().
                            applyMatrix4(target.matrixWorld),
                        llarm = larm.children[0], // lower left arm
                        lrarm = rarm.children[0], // lower right arm
                        leftElbow = new THREE.Vector3().
                            applyMatrix4(llarm.matrixWorld),
                        rightElbow = new THREE.Vector3().
                            applyMatrix4(lrarm.matrixWorld),
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
                };

                // This anchors an object to the nearest arm
                obj.release = function (target) {
                    var oldParentMatrix = target.parent.matrixWorld,
                        reverseParentTransform = new THREE.Matrix4();

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

                // TODO: Make this DRYer
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
                        //armDirection.copy(arm.children[0].position);
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

                        return motionRemaining;
                    }

                    if (leftMotionRemaining > 0) {
                        leftMotionRemaining =
                            moveArm(larm,
                                leftPointTarget,
                                leftMotionRemaining);
                    }
                    if (rightMotionRemaining > 0) {
                        rightMotionRemaining =
                            moveArm(rarm,
                                rightPointTarget,
                                rightMotionRemaining);
                    }
                }
                BOTSIM.on('tick', function () { moveArms(1/BOTSIM.FPS); });

                BOTSIM.fire('robot-ready');
            });
        }
};


BOTSIM.initViewport = function () {
    var width, height;

    this.container = document.getElementById('botsim-body');
    width = this.container.offsetWidth;
    height = width / this.ASPECT;

    this.renderer = new THREE.WebGLRenderer({
        antialias: true,
        preserveDrawingBuffer: true
    });

    this.renderer.setSize(width, height);

    this.container.appendChild(this.renderer.domElement);
};

BOTSIM.loadScene = function (files) {
    var tasksLeft = 0,
        i,
        imageLibrary = {},
        sceneSource,
        app = this;

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

            reader.onload = function () {
                sceneSource = reader.result;
                taskDone();
            };

            reader.readAsText(file);
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
            if (sceneSource) {
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

// This grabs objects in the scene that are important so that
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
        }
    }

    app.collidableMeshList = [];
    function addCollidable(obj) {
        if (obj.hasOwnProperty('geometry') &&
            obj.name !== 'Robot') {
            app.collidableMeshList.push(obj);
        }
    }

    function initObject(obj) {
        initCamera(obj);
        initRobot(obj);
        initPortable(obj);
        addCollidable(obj);
    }

    tasksLeft += 1;
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
                    var arraySize, i, j, intView,
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
                views[i].postRender()
;            }
        }

        window.requestAnimationFrame(run);
        that.fire('render');
    }
    run();

    // A physics and stuff loop
    window.setInterval(this.fire.bind(this, 'tick', 1/this.FPS), 1000/this.FPS);
};

BOTSIM.on('scene-loaded', 'readyScene', BOTSIM);

BOTSIM.on('scene-ready', 'startLoop', BOTSIM);

BOTSIM.on('tick', function () {
    this.controls.update(1);
}, BOTSIM);

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
