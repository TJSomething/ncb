/* jslint browser: true */
/* global THREE */

'use strict';

/////////////////////////////// Constants ////////////////////////////
var BOTSIM = BOTSIM || {
    
    ASPECT: 16 / 9,
    FPS: 60
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
function Robot(obj) {
    var loader = new THREE.ObjectLoader();

    // Allow usage of new
    if (!(this instanceof Robot)) {
        return new Robot(obj);
    }
    // Clear the dummy geometry used to stand in for
    //  the robot
    obj.geometry = new THREE.Geometry();

    // Make the robot model a child of the dummy object
    loader.load('/assets/json/mouse.js', function (robotObj) {
        var bounds, camHeight, near;

        robotObj.scale.set(0.2, 0.2, 0.2);
        obj.add(robotObj);

        // Place a camera in the robot
        // Find the bounding box
        bounds = (new THREE.Box3()).setFromObject(robotObj);
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

        BOTSIM.fire('robot-ready');
    });

    // Methods for robots
    obj.instantForward = function (displacement) {
        var model = new THREE.Vector3(0, 0, displacement),
            world = model.applyQuaternion(this.quaternion);

        this.position.add(world);
    };

    obj.instantBackward = function(displacement) {
        this.instantForward(-displacement);
    };

    obj.instantRight = function (displacement) {
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

            this.rotateY(Math.PI/2 - phi);
            this.instantForward(dist);
            this.rotateY(dtheta - Math.PI/2 + phi);
        } else {
            // We need to handle the case where there is no turning
            this.instantForward(ds);
        }
    };

    obj.move = function (dt) {
        var ds = this.speed * dt,
            dtheta = this.angularVelocity * dt;

        this.simultaneousTurnMove(ds, dtheta);
    };
    BOTSIM.on('tick', obj.move, obj);

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

BOTSIM.initScene = function () {
    var that = this;

    this.scene = new THREE.Scene();

    this.camera = new THREE.PerspectiveCamera(45,
            this.ASPECT, 1, 1000);
    this.camera.position.set(0, 0, 3);
    this.scene.add(this.camera);

    this.lights = {};
    this.lights.directed = new THREE.DirectionalLight(0xffffff, 1.0);
    this.lights.directed.position.set(0, 1, 1);
    this.scene.add(this.lights.directed);

    this.lights.ambient = new THREE.AmbientLight(0x202020);
    this.scene.add(this.lights.ambient);

    this.loader = new THREE.ObjectLoader();

    this.loader.load('/assets/json/mouse.js', function (obj) {
        that.robot = obj;
        that.robot.scale.set(0.2, 0.2, 0.2);
        that.scene.add(that.robot);
        that.fire('scene-loaded');
    });
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

        // Everytime we start a task that blocks loading,
        //  we add a task left
        tasksLeft += 1;

        if (type.slice(0, 5) === 'image') {
            tasksLeft += 1;

            reader.onload = function () {
                imageLibrary[name] = reader.result;

                taskDone();
            };

            reader.readAsDataURL(file);
        } else if (type === 'model/vnd.collada+xml') {
            tasksLeft += 1;

            reader.onload = function () {
                sceneSource = reader.result;
                taskDone();
            };

            reader.readAsText(file);
        }

        taskDone();
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

    // Load all the files
    for (i = 0; i < files.length; i += 1) {
        loadFile(files[i]);
    }
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
            app.robot = new Robot(obj);
        }
    }

    function traverse(obj) {
        var i;

        // Try to initialize the current object
        initCamera(obj);
        initRobot(obj);

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

////////////// Initializing the simulation ///////////////////
// Initialize the view
BOTSIM.initViewport();

// Initialize the scene
//app.initScene();

