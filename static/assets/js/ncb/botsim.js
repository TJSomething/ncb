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

/////////////////////////// Game Logic ///////////////////////////
// Methods that the robot needs
function Robot(obj, robotType) {
    // Allow usage of new
    if (!(this instanceof Robot)) {
        return new Robot(obj, robotType);
    }

    // Clear the dummy geometry used to stand in for
    //  the robot
    obj.geometry = new THREE.Geometry();

    // Load the robot model asynchronously
    Robot.loadModel(obj, robotType);

    // Methods for robots
    obj.instantForward = function (displacement) {
        var model = new THREE.Vector3(0, 0, displacement),
            world = model.applyQuaternion(this.quaternion);

        this.animateWalking(displacement);
        this.position.add(world);
    };

    obj.instantBackward = function(displacement) {
        this.instantForward(-displacement);
    };

    obj.instantRight = function (displacement) {
        this.animateWalking(0, displacement);
        this.rotateY(-displacement);
    };

    obj.instantLeft = obj.rotateY;

    // This finds exactly how the robot would move if
    //  it were going forward and turning at the same
    //  time.
    obj.simultaneousTurnMove = function (ds, dtheta) {
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

    obj.move = function (dt) {
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
    BOTSIM.on('tick', obj.move, obj);


    obj.detectCollisions = function () {
        var idx,
            originPoint = this.position.clone(),
            localVertex,
            globalVertex,
            directionVector,
            ray,
            collisionResults;

        // Lazy initialization!
        obj.collisionVertices = obj.collisionVertices || (function () {
            var b = obj.bounds,
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

        for (idx = 0; idx < obj.collisionVertices.length; idx += 1) {
            localVertex = obj.collisionVertices[idx].clone();
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
    BOTSIM.on('tick', obj.detectCollisions, obj);

    document.addEventListener( 'keydown', function (event) {
        if (event.altKey) {
            return;
        }

        switch (event.keyCode) {
            case 98: // NUM 2
                obj.speed = -1;
                break;
            case 104: // NUM 8
                obj.speed = 1;
                break;
            case 102: // NUM 6
                obj.angularVelocity = -1;
                break;
            case 100: // NUM 4
                obj.angularVelocity = 1;
                break;
        }
    }, false );
    document.addEventListener( 'keyup', function (event) {
        switch (event.keyCode) {
            case 98: // NUM 2
                obj.speed = 0;
                break;
            case 104: // NUM 8
                obj.speed = 0;
                break;
            case 102: // NUM 6
                obj.angularVelocity = 0;
                break;
            case 100: // NUM 4
                obj.angularVelocity = 0;
                break;
        }
    }, false );

    // Properties
    obj.speed = 0;
    obj.angularVelocity = 0;

    return obj;
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
            console.log('steve');
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
                    lleg = steve.bones[findBoneIndex('uleg.L')];


                obj.add(steve);

                for (i = 0; i < materials.length; i += 1) {
                    materials[i].skinning = true;
                }

                // Place a camera in the robot
                // Find the bounding box
                obj.bounds = (new THREE.Box3()).setFromObject(steve);
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
        antialias: true
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
        }
    }

    this.initViewport();

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
        tasksLeft = 0;

    function taskDone() {
        tasksLeft -= 1;

        if (tasksLeft <= 0) {
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

            app.controls = new THREE.FlyControls(app.camera);
            app.controls.dragToLook = true;
            app.controls.movementSpeed *= 0.25;
        }
    }

    function initRobot(obj) {
        if (obj.name === 'Robot') {
            tasksLeft += 1;

            BOTSIM.on('robot-ready', taskDone);

            // Set up our robot
            app.robot = new Robot(obj, BOTSIM.CHARACTER);
        }
    }

    app.collidableMeshList = [];
    function addCollidable(obj) {
        if (obj.hasOwnProperty('geometry') &&
            obj.name !== 'Robot') {
            app.collidableMeshList.push(obj);
        }
    }

    function traverse(obj) {
        var i;

        // Try to initialize the current object
        initCamera(obj);
        initRobot(obj);
        addCollidable(obj);

        for (i = 0; i < obj.children.length; i += 1) {
            traverse(obj.children[i]);
        }
    }

    tasksLeft += 1;
    traverse(app.scene);

    taskDone();
};

BOTSIM.run = function () {
    var that = this,
        width = this.container.offsetWidth,
        height = this.container.offsetHeight,
        dpr = window.devicePixelRatio || 1,
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
                camera: this.robot.camera
            }
        ],
        renderer = this.renderer;


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
        }

        window.requestAnimationFrame(run);
        that.fire('render');
    }
    run();

    // A physics and stuff loop
    window.setInterval(this.fire.bind(this, 'tick', 1/this.FPS), 1000/this.FPS);
};

BOTSIM.on('scene-loaded', 'readyScene', BOTSIM);

BOTSIM.on('scene-ready', 'run', BOTSIM);

BOTSIM.on('tick', function () {
    this.controls.update(1);
}, BOTSIM);

///////////////////// GUI callbacks //////////////////////////

$('input[name=botsim-character]').on('change', function () {
    BOTSIM.CHARACTER = $(this).val();
});

$('#botsim-file').on('change', function () {
    $('div #botsim-options').hide();
    BOTSIM.loadScene(this.files);
});