/* jslint browser: true */
/* global THREE */

'use strict';

var BOTSIM = BOTSIM || {
    // Constants:
    ASPECT: 16 / 9,
    FPS: 60
};


// Convienience functions and classes
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

// The actual game logic
BOTSIM.initViewport = function () {
    var width, height;

    this.container = document.getElementById('botsim-body');
    width = this.container.offsetWidth;
    height = width / this.ASPECT;

    this.renderer = new THREE.WebGLRenderer();

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
        that.mouse = obj;
        that.mouse.scale.set(0.2, 0.2, 0.2);
        that.scene.add(that.mouse);
        that.fire('scene-loaded');
    });
};

BOTSIM.run = function () {
    var that = this;

    // Animate
    function run() {
        that.renderer.render(that.scene, that.camera);

        window.requestAnimationFrame(run);
        that.fire('render');
    }
    run();

    // A physics and stuff loop
    window.setInterval(this.fire.bind(this, 'tick'), 1000/this.FPS);
};

// When the scene's loaded, run the animation and the physics
BOTSIM.on('scene-loaded', 'run', BOTSIM);

BOTSIM.on('tick', function () {
    this.mouse.rotation.y += 0.01;
}, BOTSIM);

(function (app) {
    // Initialize the view
    app.initViewport();

    // Initialize the scene
    app.initScene();
}(BOTSIM));