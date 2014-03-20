/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console, makePublisher */

'use strict';

var BOTSIM = BOTSIM ||  {};

BOTSIM.ASPECT = 16 / 9;
BOTSIM.FPS = 60;

makePublisher(BOTSIM);

BOTSIM.initViewport = function () {
    var width, height, left, top;

    this.container = document.createElement('div');
    document.getElementById('botsim-body').appendChild(this.container);
    this.container.style.position = 'relative';

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
    this.renderStats.domElement.style.top = '0px';
    this.renderStats.domElement.style.left = '0px';
    this.renderStats.setMode(1);
    this.container.appendChild(this.renderStats.domElement);
    
    this.logicStats = new Stats();
    this.logicStats.domElement.style.position = 'absolute';
    this.logicStats.domElement.style.top = '50px';
    this.logicStats.domElement.style.left = '0px';
    this.logicStats.setMode(1);
    this.container.appendChild(this.logicStats.domElement);
};

BOTSIM.loadScene = function (files, character) {
    var tasksLeft = 0,
        i,
        imageLibrary = {},
        sceneSource,
        sceneBuffer,
        app = this,
        fileType = 'dae';

    function loadURL(url) {
        var xhr = new XMLHttpRequest();

        xhr.open('GET', url, true);
        xhr.responseType = 'blob';
        tasksLeft += 1;

        xhr.addEventListener('load', function () {
            if (xhr.status === 200) {
                // Throw on the name, which we use for extension recognition
                xhr.response.name = url;

                loadFile(xhr.response);
                taskDone();
            }
        }, false);

        xhr.send();
    }

    // Loads each file
    function loadFile(file) {
        var name = file.name,
            type = file.type,
            reader = new FileReader();

        // If we got a URL, load that separately
        if (typeof file === 'string') {
            loadURL(file);
            return;
        }

        if (type.slice(0, 5) === 'image') {
            tasksLeft += 1;

            reader.onload = function (evt) {
                imageLibrary[name] = evt.target.result;
                taskDone();
            };

            reader.readAsDataURL(file);
        } else if (name.slice(-3).toLowerCase() === 'dae') {
            tasksLeft += 1;

            fileType = 'dae';

            reader.onload = function (evt) {
                sceneSource = evt.target.result;
                taskDone();
            };

            reader.readAsText(file);
        } else if (name.slice(-3).toLowerCase() === 'kmz') {
            tasksLeft += 1;

            fileType = 'kmz';

            reader.onload = function (evt) {
                sceneBuffer = evt.target.result;
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

                    app.fire('scene-loaded', character);
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
BOTSIM.readyScene = function (character) {
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
            app.robot = new BOTSIM.Robot(obj, character, app);
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
            tickLength = 1/that.FPS,
            dt = 0;

        window.setInterval(
            function () {
                accumulator += Date.now()/1000 - time;
                time = Date.now()/1000;

                if (accumulator > tickLength) {
                    dt = accumulator - (accumulator % tickLength);
                    // If we have an especially long frame
                    if (dt > 0.1) {
                        // We're going to pretend that it was short
                        dt = tickLength;
                        // And reset the accumulator
                        accumulator = 0;
                    } else {
                        accumulator -= dt;
                    }
                    that.logicStats.begin();
                    that.fire('logic-tick', dt);
                    that.fire('physics-tick', dt);
                    that.logicStats.end();
                }

            }, tickLength);
    }());
};

BOTSIM.on('scene-loaded', 'readyScene', BOTSIM);

BOTSIM.on('scene-ready', 'startLoop', BOTSIM);

BOTSIM.on('logic-tick', function (dt) {
    this.controls.update(60 * dt);
}, BOTSIM);

BOTSIM.on('physics-tick', function (dt) {
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