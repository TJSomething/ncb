/* jslint browser: true */
/* global THREE */

'use strict';

// Yay, incompatibility! Probably should put this in its own file
window.requestAnimationFrame = window.requestAnimationFrame ||
                               window.mozRequestAnimationFrame ||
                               window.webkitRequestAnimationFrame ||
                               window.msRequestAnimationFrame;

var BOTSIM = BOTSIM || {};

(function (app) {
    var width, height;

    app.ASPECT = 16 / 9;

    // Initialize the view
    app.container = document.getElementById('botsim-body');
    width = app.container.offsetWidth;
    height = width / app.ASPECT;

    app.renderer = new THREE.WebGLRenderer();

    app.renderer.setSize(width,
        height);
    app.container.appendChild(app.renderer.domElement);

    app.camera = new THREE.PerspectiveCamera(45,
            app.ASPECT, 1, 1000);
    app.camera.position.set(0, 0, 3);

    // Initialize the scene
    app.scene = new THREE.Scene();

    app.scene.add(app.camera);

    app.lights = {};
    app.lights.directed = new THREE.DirectionalLight(0xffffff, 1.0);
    app.lights.directed.position.set(0, 1, 1);
    app.scene.add(app.lights.directed);

    app.lights.ambient = new THREE.AmbientLight(0x202020);
    app.scene.add(app.lights.ambient);

    app.loader = new THREE.ObjectLoader();

    app.loader.load('/assets/json/mouse.js', function (obj) {
        app.mouse = obj;
        app.mouse.scale.set(0.2, 0.2, 0.2);
        app.scene.add(app.mouse);
    });

    // Animate
    function run() {
        app.renderer.render(app.scene, app.camera);
        if (app.mouse) {
            app.mouse.rotation.y -= 0.01;
        }

        window.requestAnimationFrame(run);
    }
    run();
}(BOTSIM));