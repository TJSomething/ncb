/* jslint browser: true */
/* global THREE */

"use strict";

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
    app.container = document.getElementById("botsim-body");
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

    app.cube = new THREE.Mesh(
        new THREE.CubeGeometry(1, 1, 1),
        new THREE.MeshPhongMaterial());

    app.scene.add(app.cube);

    // Animate
    function run() {
        app.renderer.render(app.scene, app.camera);
        app.cube.rotation.y -= 0.01;
        app.cube.rotation.z -= 0.01;

        window.requestAnimationFrame(run);
    }
    run();
}(BOTSIM));