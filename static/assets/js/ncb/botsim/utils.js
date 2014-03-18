/* jslint browser: true */
/* global THREE: false */

'use strict';

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

/* exported makePublisher */
function makePublisher(o) {
    var i;
    for (i in publisher) {
        if (publisher.hasOwnProperty(i) && typeof publisher[i] === 'function') {
            o[i] = publisher[i];
        }
    }
    o.subscribers = {any: []};
}

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