/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console */

define(['three', 'stats', 'vbot/utils', 'vbot/robot', 'vbot/physics',
        'vbot/controller', 'KMZLoader'],
function (THREE, Stats, utils, Robot, physics, controller, KMZLoader) {
    'use strict';

    /**
     * Holds the methods and state for the virtual robot component.
     *
     * @exports vbot/app
     * @mixes publisher
     * @property {Number} FPS the target frame rate
     * @property {Object} size the size of the viewport
     * @property {Boolean} running whether the virtual robot is paused
     * @property {THREE.Scene} scene the scene
     * @property {HTMLElement} container the div that the rendering occurs in
     * @property {THREE.WebGLRenderer} renderer the renderer
     * @property {Stats} renderStats displays statistics on render speed
     * @property {Stats} logicStats displays statistics on logic speed
     * @property {Array.<THREE.Object3D>} portables a list of the objects that
     *                                              can be picked up.
     * @property {THREE.Object3D} robot the robot
     * @property {THREE.PerspectiveCamera} camera the global camera
     * @property {THREE.OrbitControls} controls camera controls
     * @property {Uint8Array} cameraData raw data from the camera inside the
     *                                   robot
     * @property {number} cameraWidth the width of the robot camera
     * @property {number} cameraHeight the height of the robot camera
     */
    var app = {};

    app.FPS = 60;
    app.size = calcSize();
    app.running = true;

    utils.makePublisher(app);

    /**
     * Calculates the size and aspect ratio of the viewport.
     *
     * @memberof module:vbot/app~
     * @return {{width: Number,
     *           height: Number,
     *           aspect: Number}} an object with keys width, height, and aspect,
     *                            which are the width in pixels, the height in
     *                            pixels, and the aspect ratio, respectively
     */
    function calcSize() {
        var width = window.innerWidth,
            height = window.innerHeight - $('div.navbar').outerHeight(),
            aspect = width / height;

        return {
            width: width,
            height: height,
            aspect: aspect
        };
    }
    window.addEventListener('resize', function () { app.size = calcSize(); }, false);

    /**
     * Handles the display of the progress bar.
     *
     * @memberof module:vbot/app~
     * @param  {string} currentTask   the current task to display below the
     *                                progress bar
     * @param  {number} subtasksLeft  the number of tasks left
     * @param  {number} subtasksTotal the total number of tasks

     */
    function showProgress(currentTask, subtasksLeft, subtasksTotal) {
        var bar = document.getElementById('vbot-progress-bar'),
            message = document.getElementById('vbot-progress-message'),
            box = document.getElementById('vbot-progress-box'),
            tasksDone = subtasksTotal - subtasksLeft;

        if (bar) {
            setTimeout(function () {
                box.style.display = 'block';
                bar.value = 100 * tasksDone / subtasksTotal;
                message.innerHTML = currentTask;
            }, 0);

            // If we're done, then hide the box after a second
            if (subtasksLeft === 0) {
                setTimeout(function () {
                    box.style.display = 'none';
                }, 1000);
            }
        }
    }
    /**
     * Resizes the viewport to fill the screen under the toolbar.
     *
     * @memberof module:vbot/app~
     */
    function resizeViewportContainer() {
        if (app.hasOwnProperty('container')) {
            var style = app.container.style,
                toolbarHeight = $('div.navbar').outerHeight();

            style.position = 'absolute';
            style.width = app.size.width + 'px';
            style.height = app.size.height + 'px';
            style.left = '0px';
            style.top = toolbarHeight + 'px';
        }
    }
    window.addEventListener('resize', resizeViewportContainer, false);

    /**
     * Initializes the viewport.
     * @memberof module:vbot/app~
     */
    function initViewport() {
        var width, height, left,
            totalTasks = 6,
            tasksLeft = totalTasks,
            progressBox;

        function taskDone() {
            tasksLeft -= 1;
            showProgress('Initializing viewport', tasksLeft, totalTasks);
        }

        // Put the view pane in the right spot
        app.container = document.createElement('div');
        document.getElementById('vbot-body').appendChild(app.container);
        resizeViewportContainer();

        taskDone();

        width = app.container.offsetWidth;
        left = app.container.offsetLeft;

        // Stick the progress box in the center of the viewport
        progressBox = document.createElement('div');
        progressBox.id = 'vbot-progress-box';
        progressBox.innerHTML =
            '<div id="vbot-progress-padding">' +
                '<progress id="vbot-progress-bar" ' +
                          'max="100"></progress>' +
            '</div>' +
            '<div id="vbot-progress-message"></div>';
        app.container.appendChild(progressBox);
        // Center it
        progressBox.style.left = width/2 - progressBox.offsetWidth/2 + 'px';
        progressBox.style.top = height/2 - progressBox.offsetHeight/2 + 'px';

        taskDone();

        app.renderer = new THREE.WebGLRenderer({
            antialias: true,
            preserveDrawingBuffer: true
        });

        app.renderer.setSize(width, height);
        app.renderer.setClearColor('#87CEEB', 1);

        app.container.appendChild(app.renderer.domElement);

        taskDone();

        // FPS indicators
        app.renderStats = new Stats();
        app.renderStats.domElement.style.position = 'absolute';
        app.renderStats.domElement.style.top = '0px';
        app.renderStats.domElement.style.left = '0px';
        app.renderStats.setMode(1);
        app.container.appendChild(app.renderStats.domElement);

        taskDone();

        app.logicStats = new Stats();
        app.logicStats.domElement.style.position = 'absolute';
        app.logicStats.domElement.style.top = '50px';
        app.logicStats.domElement.style.left = '0px';
        app.logicStats.setMode(1);
        app.container.appendChild(app.logicStats.domElement);

        taskDone();
    }
    /**
     * Loads the scene file and the robot.
     *
     * @memberof module:vbot/app
     * @param  {Array.<(string|File)>} files an array of the files and URLs to
     *                                       load for the scene
     * @param  {string} character            the character to load
     */
    app.loadScene = function (files, character) {
        var tasksLeft = 0,
            tasksTotal = 0,
            i,
            imageLibrary = {},
            sceneSource,
            sceneBuffer,
            fileType = 'dae';

        /**
         * Increment the number of tasks by the number given and update the
         * progress bar for the scene loader.
         *
         * @param  {number} diff the number of tasks to increment by

         */
        function task(diff) {
            tasksLeft += diff;
            if (diff > 0) {
                tasksTotal += diff;
            }
            showProgress('Loading scene', tasksLeft, tasksTotal);
        }

        /**
         * Load a given URL for the scene.
         *
         * @param  {string} url the URL

         */
        function loadURL(url) {
            var xhr = new XMLHttpRequest();

            xhr.open('GET', url, true);
            xhr.responseType = 'blob';
            task(1);

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

        /**
         * Given either a file or URL, load that for the scene loader.
         *
         * @param  {(string|File)} file the URL or file

         */
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
                task(1);

                reader.onload = function (evt) {
                    imageLibrary[name] = evt.target.result;
                    taskDone();
                };

                reader.readAsDataURL(file);
            } else if (name.slice(-3).toLowerCase() === 'dae') {
                task(1);

                fileType = 'dae';

                reader.onload = function (evt) {
                    sceneSource = evt.target.result;
                    taskDone();
                };

                reader.readAsText(file);
            } else if (name.slice(-3).toLowerCase() === 'kmz') {
                task(1);

                fileType = 'kmz';

                reader.onload = function (evt) {
                    sceneBuffer = evt.target.result;
                    taskDone();
                };

                reader.readAsArrayBuffer(file);
            }
        }

        /**
         * Marks one task as completed and, if there are no tasks left,
         * assemble and show the scene.
         *

         */
        function taskDone() {
            var imageName,
                loader = new THREE.ColladaLoader(),
                xmlParser = new DOMParser(),
                xml;

            // If we don't do this, the axes will be wrong
            loader.options.convertUpAxis = true;

            task(-1);

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
                    loader = new KMZLoader(loader);

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

        task(1);
        // Load all the files
        for (i = 0; i < files.length; i += 1) {
            loadFile(files[i]);
        }
        taskDone();
    };

    /**
     * Adds auxiliary information necessary for the simulation based on the
     * objects in the scene.
     *
     * @memberof module:vbot/app~
     * @param  {string} character the name of the character
     */
    function readyScene(character) {
        // This is so we can do a few asynchronous tasks
        var tasksLeft = 0,
            maxTasks = 0,
            camera,
            robot,
            light,
            taskQueue = [];

        initViewport();

        /**
         * For auxiliary scene data creation, adds a task to a task queue and
         * updates the progress bar.
         *
         * @param {function=} task a function that performs a task
         */
        function addTask(task) {
            tasksLeft += 1;
            maxTasks += 1;

            if (task !== undefined) {
                taskQueue.push(function () {
                    task();
                    taskDone();
                });
            }

            showProgress('Prepping objects', tasksLeft, maxTasks);
        }

        /**
         * For auxilary scene data creation, decrements the number of tasks left
         * and, if there are none left, then it adds the finishing touches and
         * starts the simulation.
         *

         */
        function taskDone() {
            tasksLeft -= 1;

            showProgress('Prepping objects', tasksLeft, maxTasks);

            if (tasksLeft === 0) {
                // If we didn't add a camera or a robot, then place them in
                //  reasonable locations
                if (!app.robot) {
                    robot = new THREE.Object3D();
                    robot.position.set(0, 0, 0);
                    app.scene.add(robot);

                    initRobot(robot);
                }

                if (!app.camera) {
                    camera = new THREE.PerspectiveCamera( 45, app.size.aspect, 0.1, 1000 );
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

                app.fire('scene-ready');
            }
        }

        /**
         * Runs all of the tasks asynchronously.
         *

         */
        function runTasks() {
            taskQueue.shift().call();
            if (taskQueue.length > 0) {
                setTimeout(runTasks, 0);
            }
        }

        /**
         * Initializes the camera for the scene.
         *
         * @param  {THREE.Camera} obj

         */
        function initCamera(obj) {
            if (obj.hasOwnProperty('aspect')) {
                // Make sure that the camera's aspect ratio matches the window shape
                obj.aspect = app.size.aspect;
                app.camera = obj;

                // I have no idea why this has to be done, but
                //  it has something to do with the Collada
                //  axis fixing.
                app.camera.rotateX(-Math.PI/2);

                app.controls = new THREE.OrbitControls(app.camera);
            }
        }

        /**
         * Loads the robot and adds it to the scene.
         *
         * @param  {THREE.Object3D} obj the robot's parent object

         */
        function initRobot(obj) {
            if (obj.name === 'Robot' && !app.robot) {
                addTask();

                app.on('robot-ready', taskDone);

                // Set up our robot
                app.robot = new Robot(obj, character, app);
            }
        }

        app.portables = [];
        /**
         * Initializes the auxilary data for an object that can be moved.
         *
         * @param  {THREE.Object3D} obj the movable object

         */
        function initPortable(obj) {
            // If the word portable is in the name of the object,
            //  add it to the portable list
            if (/portable/i.test(obj.name)) {
                app.portables.push(obj);
                physics.addObject(obj, {type: 'dynamic'});
            }
        }

        /**
         * Sets up the auxiliary data to make a static object collidable.
         *
         * @param {THREE.Object3D} obj the collidable object
         */
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
                    physics.addObject(obj);
                }
            }
        }

        /**
         * Fixes textures so that non-power-of-two sized textures renders by
         * resizing them to powers of two.
         *
         * @param  {THREE.Object3D} obj an object to have it's textures resized

         */
        function fixTextures(obj) {
            /**
             * Resizes an image to the next highest power of two in each
             * dimension.
             *
             * @param  {HTMLImageElement} image the original image
             * @return {HTMLImageElement}       the resized image
             */
            function resizeNPOTImage(image) {
                var canvas, ctx;
                /**
                 * Checks if a number is a power of two.
                 *
                 * @param  {number} x
                 * @return {Boolean} if x is a power of two
                 */
                function isPowerOfTwo(x) {
                    return (x & (x - 1)) === 0;
                }

                /**
                 * Finds the next highest power of two from a number, if that
                 * number is not already a power of two.
                 *
                 * @param  {Number} x
                 * @return {Number} the next highest power of two after x
                 */
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

            /**
             * Fix all non-power-of-two textures for a material.
             *
             * @param  {THREE.Material} material

             */
            function fixMaterial(material) {
                ['lightMap', 'specularMap', 'envMap', 'map'].forEach(
                    function (mapType) {
                        if (material[mapType]) {
                            // If the texture isn't loaded yet, then we'll need to resize it later
                            material[mapType].image.addEventListener('load', (function () {
                                var materialMap = material[mapType];
                                return function () {
                                        materialMap.image =
                                            resizeNPOTImage(materialMap.image);
                                };
                            }()));
                            // If it has been loaded already, then we'll resize it
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

        /**
         * Queues an object to have auxiliary information added.
         *
         * @param  {THREE.Object3D} obj

         */
        function initObject(obj) {
            addTask(function () {
                initCamera(obj);
                initRobot(obj);
                initPortable(obj);
                addCollidable(obj);
                fixTextures(obj);
            });
        }

        // So that we have reasonable matrices for the children
        app.scene.updateMatrixWorld();

        addTask(app.scene.traverse.bind(app.scene, initObject));

        setTimeout(runTasks, 0);
    }
    /**
     * Starts the event loop.
     *
     * @memberof module:vbot/app~
     */
    function startLoop() {
        var width = app.container.offsetWidth,
            height = app.container.offsetHeight,
            dpr = window.devicePixelRatio || 1,
            renderer = app.renderer,
            gl = renderer.getContext(),
            views = [
                {
                    left: 0,
                    bottom: 0,
                    width: width * dpr,
                    height: height * dpr,
                    camera: app.camera
                },
                {
                    left: 0,
                    bottom: 0,
                    width: 320,
                    height: 240,
                    camera: app.robot.camera,
                    postRender: (function () {
                        var arraySize, i, intView,
                            rowView1, rowView2, tempRow, rows, cols;

                        return function () {
                            if (!arraySize) {
                                rows = this.height;
                                cols = this.width;

                                arraySize = rows * cols * 4;

                                app.cameraData = new Uint8Array(arraySize);
                                app.cameraWidth = this.width;
                                app.cameraHeight = this.height;

                                intView = new Uint32Array(app.cameraData.buffer);

                                tempRow = new Uint32Array(cols);
                            }

                            gl.readPixels(this.left, this.bottom,
                                this.width, this.height, gl.RGBA,
                                gl.UNSIGNED_BYTE, app.cameraData);

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

        function resizeRenderer () {
            width = app.size.width;
            height = app.size.height;
            views[0].width = width * dpr;
            views[0].height = height * dpr;

            renderer.setSize(width, height);

            app.camera.aspect = width / height;
            app.camera.updateProjectionMatrix();
        }
        window.addEventListener('resize', resizeRenderer, false);

        // Animate
        function run() {
            var i, left, bottom, width, height;

            app.renderStats.begin();

            for (i = 0; i < views.length; i += 1) {
                left = views[i].left;
                bottom = views[i].bottom;
                width = views[i].width;
                height = views[i].height;

                renderer.setViewport(left, bottom, width, height);
                renderer.setScissor(left, bottom, width, height);
                renderer.enableScissorTest(true);

                renderer.render(app.scene, views[i].camera);

                if (views[i].postRender) {
                    views[i].postRender();
                }
            }

            utils.requestAnimationFrame(run);

            app.renderStats.end();

            app.fire('render');
        }
        run();

        // A physics and stuff loop
        (function () {
            var time = Date.now()/1000,
                accumulator = 0,
                tickLength = 1/app.FPS,
                dt = 0;

            window.setInterval(
                function () {
                    if (app.running) {
                        accumulator += Date.now()/1000 - time;
                        time = Date.now()/1000;

                        if (accumulator > tickLength) {
                            dt = accumulator - (accumulator % tickLength);
                            // If we have an especially long frame
                            if (dt > 0.1) {
                                // We're going to pretend that it was short
                                dt = 0.1;
                                // And reset the accumulator
                                accumulator = 0;
                            } else {
                                accumulator -= dt;
                            }
                            app.logicStats.begin();
                            app.fire('logic-tick', dt);
                            app.fire('physics-tick', dt);
                            app.logicStats.end();
                        }
                    }
                }, tickLength);
        }());
    }
    /**
     * Pauses the simulation.
     *
     * @memberof module:vbot/app
     * @param  {Boolean} pauseState if it's to be paused
     */
    app.pause = function (pauseState) {
        if (app.hasOwnProperty('controls')) {
            app.controls.enabled = !pauseState;
        }
        app.running = !pauseState;
    };

    /**
     * Runs a test of the controller.
     *
     * @memberof module:vbot/app
     */
    app.testController = function () {
        controller.test(app);
    };

    /**
     * Toggles visibility of collision volumes.
     *
     * @memberof module:vbot/app
     */
    app.toggleCollisionVolumes = function () {
        physics.toggleCollisionVolumes(app.scene);
    };

    app.on('scene-loaded', readyScene);

    app.on('scene-ready', startLoop);

    app.on('logic-tick', function (dt) {
        this.controls.update(60 * dt);
    }, app);

    app.on('physics-tick', function (dt) {
        physics.updateObjects(dt);
        physics.detectCollisions();
        physics.resolveCollisions(dt);
        controller.step(dt);
    });

    app.on('render', (function () {
        var canvas = document.createElement('canvas'),
            ctx = canvas.getContext('2d'),
            imageData;

        // This might be needed for a demo, but probably not
        //container.appendChild(canvas);

        return function () {
            if (!imageData) {
                canvas.height = app.cameraHeight;
                canvas.width = app.cameraWidth;
                imageData = ctx.createImageData(canvas.width, canvas.height);
            }

            imageData.data.set(app.cameraData);
            ctx.putImageData(imageData, 0, 0);
            ctx.scale(1, -1);
        };
    }()), app);

    return app;
});
