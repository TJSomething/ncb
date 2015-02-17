// We're loading all of our three.js dependencies and then adding a few of
// our own convenience methods.
var THREE = require('three.js/three');

require('imports?THREE=three.js/three!threex-controls/controls/OrbitControls');
require('imports?THREE=three.js/three!threex-colladaloader/ColladaLoader');

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

module.exports = THREE;
