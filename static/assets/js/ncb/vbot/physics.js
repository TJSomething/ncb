/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console, VBOT: true */

define(['three', 'numeric', 'underscore'],
function (THREE, numeric, _) {
    'use strict';

    /** @exports vbot/physics */

    var physics = (function () {
        var staticObjects = [],
            dynamicObjects = [],
            // This is used to quickly address objects
            dynamicObjectIndices = {},
            staticObjectIndices = {},
            EPSILON = 0.000001,
            staticCollisionResolution = 0.1,
            collisionVolumeObjects = new THREE.Object3D(),
            groundElevation = Infinity;

        /**
         * Creates an OBB
         *
         * @memberof module:vbot/physics~
         * @class
         * @param {THREE.Vector3} center      the center
         * @param {THREE.Vector3[]} bases     an array of the bases, which determine the
         *                                    rotation of the OBB
         * @param {number[]} halfExtents      an array of the half-lengths of each dimension
         *                                    of the OBB
         * @param {THREE.Object3D} parent     the parent of this OBB
         * @return {module:vbot/physics~OrientedBoundingBox} an oriented bounding box
         */
        function OrientedBoundingBox(center, bases, halfExtents, parent) {
            var obb = {};
            var rotMatrix =
                new THREE.Matrix4(bases[0].x, bases[1].x, bases[2].x, 0,
                                  bases[0].y, bases[1].y, bases[2].y, 0,
                                  bases[0].z, bases[1].z, bases[2].z, 0,
                                           0,          0,          0, 1);
            var c,
                u = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
            var parentMemo = new THREE.Matrix4();
            var mat = new THREE.Matrix4();
            var len;
            var pMat = parent.matrixWorld.elements;
            // Calculate the length of each basis in scaling the extents into world space
            var basisLengths = [Math.sqrt(pMat[0]*pMat[0] + pMat[1]*pMat[1] + pMat[2]*pMat[2]),
                                Math.sqrt(pMat[4]*pMat[4] + pMat[5]*pMat[5] + pMat[6]*pMat[6]),
                                Math.sqrt(pMat[8]*pMat[8] + pMat[9]*pMat[9] + pMat[10]*pMat[10])];
            var scaledExtents = new THREE.Vector3(
                                    halfExtents[0]*basisLengths[0],
                                    halfExtents[1]*basisLengths[1],
                                    halfExtents[2]*basisLengths[2]);


            /**
             * Checks if two 4x4 matrices are equal
             * @param  {THREE.Matrix4} mat1 the first matrix
             * @param  {THREE.Matrix4} mat2 the second matrix
             * @return {boolean}            whether they're equal
             */
            function matrix4Equals(mat1, mat2) {
                for (var i = 0; i < 16; i += 1) {
                    if (mat1.elements[i] !== mat2.elements[i]) {
                        return false;
                    }
                }
                return true;
            }

            /**
             * Updates this OBB's matrix based on its parent
             *
             * @memberof module:vbot/physics~OrientedBoundingBox#
             */
            function update() {
                if (!matrix4Equals(parent.matrixWorld, parentMemo)) {
                    c = center.clone().applyMatrix4(parent.matrixWorld);

                    mat.multiplyMatrices(parent.matrixWorld, rotMatrix);
                    u[0].x = mat.elements[0];
                    u[0].y = mat.elements[1];
                    u[0].z = mat.elements[2];
                    u[1].x = mat.elements[4];
                    u[1].y = mat.elements[5];
                    u[1].z = mat.elements[6];
                    u[2].x = mat.elements[8];
                    u[2].y = mat.elements[9];
                    u[2].z = mat.elements[10];

                    len = Math.sqrt(u[0].x*u[0].x + u[0].y*u[0].y + u[0].z*u[0].z);
                    for (var i = 0; i < 3; i += 1) {
                        u[i].x /= len;
                        u[i].y /= len;
                        u[i].z /= len;
                    }

                    parentMemo.copy(parent.matrixWorld);
                }
            }

            /**
             * Makes a copy of this object without the ability to recompute
             * the properties based on the parent of this OBB. This speeds up
             * collision detection.
             *
             * @memberof module:vbot/physics~OrientedBoundingBox#
             * @return {module:vbot/physics~OrientedBoundingBox} a dumb copy of this
             *                                            OBB
             */
            function clone() {
                update();
                return {
                        c: this.c,
                        e: this.e,
                        u: this.u,
                        update: function () {},
                        clone: function () { return this; }
                    };
            }

            /**
             * the center of the OBB
             *
             * @memberof module:vbot/physics~OrientedBoundingBox#
             * @var {THREE.Vector3} c
             */
            Object.defineProperty(obb, 'c',
                {
                    get: function() {
                             update();
                             return c;
                         }
                });
            /**
             * the half-extents of the OBB
             *
             * @memberof module:vbot/physics~OrientedBoundingBox#
             * @var {THREE.Vector3} e
             */
            Object.defineProperty(obb, 'e',
                {
                    get: function() {
                             return scaledExtents;
                         }
                });
            /**
             * the bases of the OBB
             *
             * @memberof module:vbot/physics~OrientedBoundingBox#
             * @var {Array.<THREE.Vector3>} u
             */
            Object.defineProperty(obb, 'u',
                {
                    get: function() {
                             update();
                             return u;
                         }
                });
            Object.defineProperty(obb, 'update',
                {
                    value: update
                });
            Object.defineProperty(obb, 'clone',
                {
                    value: clone
                });

            return obb;
        }

        /**
         * Calculates a matrix that transforms a unit cube centered at (0,0,0)
         * to the given OBB
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox} obb the OBB to create the matrix
         *                                        with
         * @return {THREE.Matrix4}                the matrix to get the OBB
         */
        function makeOBBMatrix(obb) {
            // scale
            var sc = obb.e.toArray();
            // bases
            var E = [[obb.u[0].x, obb.u[1].x, obb.u[2].x],
                     [obb.u[0].y, obb.u[1].y, obb.u[2].y],
                     [obb.u[0].z, obb.u[1].z, obb.u[2].z]];
            // center
            var p = obb.c;

            var mat = new THREE.Matrix4(sc[0] * E[0][0], sc[1] * E[0][1], sc[2] * E[0][2], p.x,
                                        sc[0] * E[1][0], sc[1] * E[1][1], sc[2] * E[1][2], p.y,
                                        sc[0] * E[2][0], sc[1] * E[2][1], sc[2] * E[2][2], p.z,
                                        0,               0,               0,               1);

            return mat;
        }

        /**
         * Creates a three.js object that corresponds to an OrientedBoundingBox
         *
         * @class
         * @memberof module:vbot/physics~
         * @param {module:vbot/physics~OrientedBoundingBox} obb the OBB to be shown with the
         *                                       object
         * @return {THREE.Mesh}                  the three.js object indicating
         *                                       the OBB
         */
        function OBBHelper(obb) {
            var boundingBox = new THREE.Mesh(
                    new THREE.BoxGeometry(2, 2, 2),
                    new THREE.MeshBasicMaterial({
                        color: 0x888888,
                        wireframe: true
                    })),
                originalUpdate = boundingBox.updateMatrixWorld.bind(boundingBox);

            function updateOBB() {
                boundingBox.matrix.copy(makeOBBMatrix(obb));
            }

            // Make it so that updating the matrixWorld updates the OBB
            boundingBox.updateMatrixWorld = function(force) {
                updateOBB();
                originalUpdate(force);
                boundingBox.matrixWorldNeedsUpdate = true;
            }

            // We're not going to use position, quaternion, and scale
            boundingBox.matrixAutoUpdate = false;

            updateOBB();
            originalUpdate(true);

            return boundingBox;
        }

        /**
         * Calculates the vertices of an OBB
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox} obb the OBB we're calculating the
         *                                   vertices of
         * @return {THREE.Vector3[]}         an array of the OBB's vertices
         */
        function calcOBBVertices(obb) {
            var mat = makeOBBMatrix(obb),
                Vec = function (x,y,z) { return new THREE.Vector3(x,y,z); },
                startVertices = [Vec(-0.5, -0.5, -0.5),
                                 Vec(-0.5, -0.5, +0.5),
                                 Vec(-0.5, +0.5, -0.5),
                                 Vec(-0.5, +0.5, +0.5),
                                 Vec(+0.5, -0.5, -0.5),
                                 Vec(+0.5, -0.5, +0.5),
                                 Vec(+0.5, +0.5, -0.5),
                                 Vec(+0.5, +0.5, +0.5)],
                transformedVerts = startVertices.map(function(vert) {
                    return vert.applyMatrix4(mat);
                });

            return transformedVerts;
        }

        /**
         * Calculates the vertices of multiple OBBs
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox[]} obbs the OBBs to calculate the
         *                                      vertices of
         * @return {THREE.Vector3[]}            an array of the vertices of all
         *                                      the OBBs
         */
        function calcOBBsVertices(obbs) {
            var vertsPerBox = obbs.map(calcOBBVertices),
                combinedVerts = [].concat.apply([], vertsPerBox);

            return combinedVerts;
        }

        /**
         * Calculates the bounding sphere of multiple OBBs
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox[]} obbs
         * @return {THREE.Sphere}               the bounding sphere of the given
         *                                      OBBs
         */
        function calcBoundingSphereFromOBBs(obbs) {
            var verts = calcOBBsVertices(obbs),
                sphere = new THREE.Sphere().setFromPoints(verts);

            return sphere;
        }

        /**
         * Holds the physics attributes of an object.
         *
         * @typedef {Object} module:vbot/physics~PhysicsObject
         * @property {THREE.Sphere} boundingSphere the bounding sphere of the object
         * @property {Array.<module:vbot/physics~OrientedBoundingBox>} obbs all of the OBBs
         *                                                      for the object
         * @property {Number} verticalVelocity the vertical velocity in m/s
         */
        /**
         * Holds the combined physical and the renderable attributes of an
         * object in the environment.
         *
         * @typedef {Object} module:vbot/physics~CombinedPhysicsObject
         *
         * @property {module:vbot/physics~PhysicsObject} physics the attributes of the
         *                                                object used by the
         *                                                physics engine
         * @property {THREE.Object3D} geometry the publicly available aspects
         *                                     of the object, used by the
         *                                     graphics engine
         */

        /**
         * Initializes a CombinedPhysicsObject as a dynamic object.
         *
         * This adds properties like:
         *
         * - oriented bounding boxes
         * - a bounding sphere
         * - vertical velocity
         *
         * as well as adding the CombinedPhysicsObject to the dynamic object
         * list.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         */
        function initDynamicObject(obj) {
            var bWorld = (new THREE.Box3()).setFromObject(obj.geometry),
                Vec = THREE.Vector3,
                invMatrixWorld = (new THREE.Matrix4()).
                    getInverse(obj.geometry.matrixWorld),
                // In local coordinates
                b = bWorld.clone().applyMatrix4(invMatrixWorld);

            if (obj.physics.state === "controlled") {
                obj.physics.obbs = buildRobotOBBs(obj.geometry);
            } else {
                obj.physics.obbs =
                    [function () {
                        var boundingSphere = b.getBoundingSphere(),
                            c = boundingSphere.center,
                            tmpC = new Vec(),
                            // Half extents
                            e = b.max.clone().
                                sub(b.min).
                                divideScalar(2),
                            u = [new Vec(1, 0, 0),
                                 new Vec(0, 1, 0),
                                 new Vec(0, 0, 1)],
                            mat = obj.geometry.matrixWorld,
                            len = 0;

                        return new OrientedBoundingBox(c, u, e.toArray(), obj.geometry);
                    }()];
            }
            obj.physics.obbs.forEach(function (obb) {
                collisionVolumeObjects.add(OBBHelper(obb));
            });

            obj.physics.boundingSphere = calcBoundingSphereFromOBBs(obj.physics.obbs);

            obj.physics.updateBoundingSphere = function () {
                obj.physics.boundingSphere = calcBoundingSphereFromOBBs(obj.physics.obbs);
            }

            obj.physics.verticalVelocity = obj.physics.verticalVelocity || 0;

            dynamicObjects.push(obj);
            dynamicObjectIndices[obj.geometry.id] = dynamicObjects.length - 1;
        }

        /**
         * Initializes a CombinedPhysicsObject as a static object.
         *
         * This adds axis-aligned bounding boxes (implemented as
         * OrientedBoundingBoxes) and bounding spheres. This also adds the
         * CombinedPhysicsObject to the static object list and updates the
         * ground elevation to be below this object.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         */
        function initStaticObject(obj) {
            var mat = obj.geometry.matrixWorld,
                Vec = THREE.Vector3,
                aabbWorld = (new THREE.Box3()).setFromObject(obj.geometry);

            obj.physics.faces = getGlobalFaces(obj);

            obj.physics.obbs = buildObjectOBBs(obj, staticCollisionResolution);

            obj.physics.boundingSphere = calcBoundingSphereFromOBBs(obj.physics.obbs);

            // Static objects don't move, so they don't need updating
            obj.physics.updateBoundingSphere = function () {};

            staticObjects.push(obj);
            staticObjectIndices[obj.geometry.id] = staticObjects.length - 1;

            // Static objects don't move
            obj.physics.verticalVelocity = 0;

            updateGround(obj);
        }

        /**
         * Updates the ground elevation to make sure it's below the given
         * object.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         */
        function updateGround(obj) {
            var objAABB = new THREE.Box3().setFromObject(obj.geometry);

            if (groundElevation > objAABB.max.y) {
                groundElevation = objAABB.max.y;
            }
        }

        /**
         * Finds the faces of an object in world space.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj the object to find the faces of
         * @return {Array.<Array.<THREE.Vector3>>}         an array of arrays of vertices,
         *                                     where each inner array is a face
         */
        function getGlobalFaces(obj) {
            var vertices,
                matrixWorld = obj.geometry.matrixWorld,
                // Transform our vertices into global coordinates
                globalVertices;

            if (obj.geometry.hasOwnProperty('geometry')) {
                vertices = obj.geometry.geometry.vertices;
                globalVertices = vertices.map(function (vertex) {
                    return vertex.clone().applyMatrix4(matrixWorld);
                });
                return obj.geometry.geometry.faces.map(function (face) {
                    return [globalVertices[face.a],
                            globalVertices[face.b],
                            globalVertices[face.c]];
                });
            } else {
                return [];
            }
        }

        /**
         * Adds a three.js object to the physics engine.
         *
         * Available physicalProperties:
         * - state: the state of the object, which can be:
         *   + rest (default)
         *   + falling
         *   + controlled
         *   + held
         * - type: the type of object, which can be:
         *   + static (default)
         *   + dynamic
         *
         * @memberof module:vbot/physics
         * @param {THREE.Object3D} obj
         * @param {object} physicalProperties An object containing any
         *                                    PhysicsObject properties that
         *                                    should be set.
         */
        function addObject(obj, physicalProperties) {
            var physicsObj = {
                geometry: obj,
                physics: (function () {
                    var p = physicalProperties || {};

                    return {
                        position: obj.positionWorld(),
                        quaternion: obj.quaternion,
                        state: p.state || 'rest',
                        // The object should be static if this flag is not passed,
                        //  or to the passed value
                        type: p.type === undefined ? 'static' : p.type
                    };
                }())
            };

            if (physicsObj.physics.type === 'static') {
                initStaticObject(physicsObj);
            } else if (physicsObj.physics.type === 'dynamic') {
                initDynamicObject(physicsObj);
            } else {
                throw physicsObj.physics.type + ' is not an object type';
            }
        }


        /**
         * Updates the PhysicsObject of a CombinedPhysicsObject from the
         * THREE.Object3D. The properties updated are the position, rotation
         * quaternion, and the bounding sphere.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj the object to update
         */
        function updateObjectLocation(obj) {
            var position,
                quaternion;

            obj.geometry.updateMatrixWorld();

            position = obj.geometry.positionWorld();
            quaternion = obj.geometry.quaternion;

            obj.physics.position = position;
            obj.physics.quaternion = quaternion;

            obj.physics.updateBoundingSphere();
        }

        /**
         * If the object is falling, this steps the falling physics forward.
         *
         * @memberof module:vbot/physics~
         * @param  {number} dt                 the amount of time to step
         *                                     forward
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj the object to move
         */
        function updateObject(dt, obj) {
            // Update position and velocity
            if (obj.physics.state === 'controlled' ||
                obj.physics.state === 'falling') {
                obj.physics.verticalVelocity -= 9.8 * dt;
                obj.geometry.position.y += dt * obj.physics.verticalVelocity;
            }

            updateObjectLocation(obj);
        }

        /**
         * Steps the falling physics forward on all dynamic objects.
         *
         * @memberof module:vbot/physics
         * @param  {number} dt the amount of time to step forward
         */
        function updateObjects(dt) {
            var i;

            // Update position and velocity
            for (i = 0; i < dynamicObjects.length; i += 1) {
                updateObject(dt, dynamicObjects[i]);
            }
        }

        /**
         * Performs a dot product of two vectors.
         *
         * @memberof module:vbot/physics~
         * @param  {THREE.Vector3} v
         * @param  {THREE.Vector3} u
         * @return {number}
         */
        function dot(v, u) {
            return u.x * v.x + u.y * v.y + u.z * v.z;
        }

        /**
         * Represents a collision.
         *
         * @typedef module:vbot/physics~Collision
         * @property {Number} penetration the penetration of the collision
         * @property {THREE.Vector3} contactNormal a vector of length 1 on the
         *                                         collision axis
         * @property {THREE.Object3D?} otherObject the other object in the
         *                                         collision
         * @property {string} type the type of the other object, either
         *                         'static', 'dynamic', or 'ground'.
         *                         note that the ground type has no other
         *                         object, as the ground is not a real object
         */

        /**
         * Finds if two OBB's are colliding.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox} a
         * @param  {module:vbot/physics~OrientedBoundingBox} b
         * @return {module:vbot/physics~Collision?} if the boxes are collding, then the
         *                             collision is returned. If they're not,
         *                             then false is returned.
         */
        function testOBBOBB(a, b) {
            // Put all of the OBB properties in nice locations
            var ac = a.c, bc = b.c,
                ae = a.e.toArray(), be = b.e.toArray(),
                au = a.u, bu = b.u,
                aeLength = Math.sqrt(ae[0]*ae[0] + ae[1]*ae[1] + ae[2]*ae[2]),
                beLength = Math.sqrt(be[0]*be[0] + be[1]*be[1] + be[2]*be[2]),
                ra = 0,
                rb = 0,
                R = {elements: new Float32Array(9)},
                AbsR = {elements: new Float32Array(9)},
                i, j,
                t = new THREE.Vector3(),
                // Putting these into arrays for for-loops
                tArr,
                axisDist,
                penetration = Infinity,
                normal = new THREE.Vector3();

            // Before anything else, test bounding spheres
            // Compute translation vector t
            t.subVectors(bc, ac);
            // If the distance between the centers of the boxes
            // is greater than the sum of the lengths of the extents,
            // then the boxes can't intersect
            if (t.length() > aeLength + beLength) {
                return null;
            }

            // Compute rotation matrix expressing b in a's coordinate frame
            for (i = 0; i < 3; i += 1) {
                for (j = 0; j < 3; j += 1) {
                    R.elements[3 * j + i] = dot(au[i], bu[j]);
                }
            }

            // Bring translation into a's coordinate frame
            t = new THREE.Vector3(
                dot(t, au[0]),
                dot(t, au[1]),
                dot(t, au[2]));
            tArr = [t.x, t.y, t.z];

            // Compute common subexpressions. Add in an epsilon term to
            // counteract arthimetic errors when two edges are parallel and
            // their cross product is (near) null
            for (i = 0; i < 3; i += 1) {
                for (j = 0; j < 3; j += 1) {
                    AbsR.elements[3 * j + i] =
                        Math.abs(R.elements[3 * j + i]) + EPSILON;
                }
            }

            // Test axes L = A0, L = A1, L = A2
            for (i = 0; i < 3; i += 1) {
                ra = ae[i];
                rb = b.e.x * AbsR.elements[i] +
                     b.e.y * AbsR.elements[3 + i] +
                     b.e.z * AbsR.elements[6 + i];
                axisDist = Math.abs(tArr[i]) - (ra + rb);
                if (axisDist > 0) {
                    return null;
                } else if (-axisDist < penetration) {
                    penetration = -axisDist;
                    normal.set(0, 0, 0);
                    normal.setComponent(i, -Math.sign(tArr[i]));
                }
            }

            // Test axes L = B0, L = B1, L = B2
            for (i = 0; i < 3; i += 1) {
                ra = a.e.x * AbsR.elements[3 * i] +
                     a.e.y * AbsR.elements[3 * i + 1] +
                     a.e.z * AbsR.elements[3 * i + 2];
                rb = be[i];
                axisDist = Math.abs(t.x * R.elements[3 * i] +
                                    t.y * R.elements[3 * i + 1] +
                                    t.z * R.elements[3 * i + 2]) -
                           (ra + rb);
                if (axisDist > 0) {
                    return null;
                } else if (-axisDist < penetration) {
                    penetration = -axisDist;
                    normal.set(R.elements[3 * i],
                               R.elements[3 * i + 1],
                               R.elements[3 * i + 2]);
                    normal.multiplyScalar(-Math.sign(dot(normal, t)));
                }
            }


            // Test axis L = A0 x B0
            ra = ae[1] * AbsR.elements[3 * 0 + 2] +
                 ae[2] * AbsR.elements[3 * 0 + 1];
            rb = be[1] * AbsR.elements[3 * 2 + 0] +
                 be[2] * AbsR.elements[3 * 1 + 0];
            axisDist = Math.abs(t[2] * R.elements[3 * 0 + 1] -
                                t[1] * R.elements[3 * 0 + 2]) -
                       (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(0,
                           -R.elements[3 * 0 + 2],
                           R.elements[3 * 0 + 1]);
                normal.multiplyScalar(-Math.sign(dot(normal,t)));
            }

            // Test axis L = A0 x B1
            ra = ae[1] * AbsR.elements[3 * 1 + 2] +
                 ae[2] * AbsR.elements[3 * 1 + 1];
            rb = be[0] * AbsR.elements[3 * 2 + 0] +
                 be[2] * AbsR.elements[3 * 0 + 0];
            axisDist = Math.abs(t[2] * R.elements[3 * 1 + 1] -
                                t[1] * R.elements[3 * 1 + 2]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(0,
                           -R.elements[3 * 1 + 2],
                           R.elements[3 * 1 + 1]);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A0 x B2
            ra = ae[1] * AbsR.elements[3 * 2 + 2] +
                 ae[2] * AbsR.elements[3 * 2 + 1];
            rb = be[0] * AbsR.elements[3 * 1 + 0] +
                 be[1] * AbsR.elements[3 * 0 + 0];
            axisDist = Math.abs(t[2] * R.elements[3 * 2 + 1] -
                                t[1] * R.elements[3 * 2 + 2]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(0,
                           -R.elements[3 * 2 + 2],
                           R.elements[3 * 2 + 1]);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A1 x B0
            ra = ae[0] * AbsR.elements[3 * 0 + 2] +
                 ae[2] * AbsR.elements[3 * 0 + 0];
            rb = be[1] * AbsR.elements[3 * 2 + 1] +
                 be[2] * AbsR.elements[3 * 1 + 1];
            axisDist = Math.abs(t[0] * R.elements[3 * 0 + 2] -
                                t[2] * R.elements[3 * 0 + 0]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(-R.elements[3 * 0 + 2],
                           0,
                           R.elements[3 * 0 + 0]);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A1 x B1
            ra = ae[0] * AbsR.elements[3 * 1 + 2] +
                 ae[2] * AbsR.elements[3 * 1 + 0];
            rb = be[0] * AbsR.elements[3 * 2 + 1] +
                 be[2] * AbsR.elements[3 * 0 + 1];
            axisDist = Math.abs(t[0] * R.elements[3 * 1 + 2] +
                                t[2] * R.elements[3 * 1 + 0]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(R.elements[3 * 1 + 2],
                           0,
                           R.elements[3 * 1 + 0]);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A1 x B2
            ra = ae[0] * AbsR.elements[3 * 2 + 2] +
                 ae[2] * AbsR.elements[3 * 2 + 0];
            rb = be[0] * AbsR.elements[3 * 1 + 1] +
                 be[1] * AbsR.elements[3 * 0 + 1];
            axisDist = Math.abs(t[0] * R.elements[3 * 2 + 2] +
                                t[2] * R.elements[3 * 2 + 0]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(R.elements[3 * 2 + 2],
                           0,
                           R.elements[3 * 2 + 0]);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A2 x B0
            ra = ae[0] * AbsR.elements[3 * 0 + 1] +
                 ae[1] * AbsR.elements[3 * 0 + 0];
            rb = be[1] * AbsR.elements[3 * 2 + 2] +
                 be[2] * AbsR.elements[3 * 1 + 2];
            axisDist = Math.abs(t[1] * R.elements[3 * 0 + 0] +
                                t[0] * R.elements[3 * 0 + 1]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(R.elements[3 * 0 + 1],
                           R.elements[3 * 0 + 0],
                           0);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A2 x B1
            ra = ae[0] * AbsR.elements[3 * 1 + 1] +
                 ae[1] * AbsR.elements[3 * 1 + 0];
            rb = be[0] * AbsR.elements[3 * 2 + 2] +
                 be[2] * AbsR.elements[3 * 0 + 2];
            axisDist = Math.abs(t[1] * R.elements[3 * 1 + 0] +
                                t[0] * R.elements[3 * 1 + 1]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(R.elements[3 * 1 + 1],
                           R.elements[3 * 1 + 0],
                           0);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Test axis L = A2 x B2
            ra = ae[0] * AbsR.elements[3 * 2 + 1] +
                 ae[1] * AbsR.elements[3 * 2 + 0];
            rb = be[0] * AbsR.elements[3 * 1 + 2] +
                 be[1] * AbsR.elements[3 * 0 + 2];
            axisDist = Math.abs(t[1] * R.elements[3 * 2 + 0] +
                                t[0] * R.elements[3 * 2 + 1]) - (ra + rb);
            if (axisDist > 0) {
                return null;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(R.elements[3 * 2 + 1],
                           R.elements[3 * 2 + 0],
                           0);
                normal.multiplyScalar(-Math.sign(dot(normal, t)));
            }

            // Rotate our normal into the first object's rotation
            for (i = 0; i < 3; i += 1) {
                for (j = 0; j < 3; j += 1) {
                    R.elements[3 * i + j] = au[i].getComponent(j);
                }
            }
            normal.applyMatrix3(R);

            // Since no separating axis is found, the OBBs must be intersecting
            return {
                penetration: penetration,
                contactNormal: normal
            };
        }

        /**
         * Tests for OBB triangle intersection. Not useful for collision
         * resolution.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox} obb
         * @param  {THREE.Vector3[]} tri    the triangle to test against,
         *                                  as an array of vectors in
         *                                  counterclockwise order around
         *                                  the triangle
         * @return {module:vbot/physics~Collision} the collision if they're colliding,
         *                                  or null otherwise
         */
        function testOBBTri(obb, tri) {
            var i,
                v = [new THREE.Vector3(),
                     new THREE.Vector3(),
                     new THREE.Vector3()],
                e = [new THREE.Vector3(),
                     new THREE.Vector3(),
                     new THREE.Vector3()],
                fex,
                fey,
                fez,
                // Variables for axis tests
                p0, p1, p2, min, max, rad,
                vmax = new THREE.Vector3(),
                vmin = new THREE.Vector3(),
                normal = new THREE.Vector3(),
                penetration = Infinity,
                contactNormal = new THREE.Vector3();

            /**
             * Checks if centered AABB penetrates a plane
             *
             * @param  {THREE.Vector3} normal the normal of the plane
             * @param  {THREE.Vector3} vert   a vertex on the plane
             * @param  {THREE.Vector3} maxbox the extent of the AABB
             * @return {boolean}              whether the plane was penetrated
             */
            function planeBoxOverlap(normal, vert, maxbox) {
                var q, v;

                for (q = 0; q < 3; q += 1) {
                    v = vert.getComponent(q);
                    if (normal.getComponent(q) > 0) {
                        vmin.setComponent(q, -maxbox.getComponent(q) - v);
                        vmax.setComponent(q, maxbox.getComponent(q) - v);
                    } else {
                        vmin.setComponent(q, maxbox.getComponent(q) - v);
                        vmax.setComponent(q, -maxbox.getComponent(q) - v);
                    }
                }

                if (normal.dot(vmin) > 0) {
                    return false;
                }
                if (normal.dot(vmax) >= 0) {
                    if (-normal.clone().normalize().dot(vmin) < penetration) {
                        contactNormal = normal.clone().normalize();
                        penetration = -contactNormal.dot(vmin);
                    }
                    return true;
                }

                return false;
            }

            // Axis tests
            function axisTestX01(a, b, fa, fb) {
                p0 = a * v[0].y - b * v[0].z;
                p2 = a * v[2].y - b * v[2].z;
                if (p0 < p2) {
                    min = p0;
                    max = p2;
                } else {
                    min = p2;
                    max = p0;
                }
                rad = fa * obb.e.y + fb * obb.e.z;
                if (min > rad || max < -rad) {
                    return false;
                }
                return true;
            }

            function axisTestX2(a, b, fa, fb) {
                p0 = a * v[0].y - b * v[0].z;
                p1 = a * v[1].y - b * v[1].z;
                if (p0 < p1) {
                    min = p0;
                    max = p1;
                } else {
                    min = p1;
                    max = p0;
                }
                rad = fa * obb.e.y + fb * obb.e.z;
                if (min > rad || max < -rad) {
                    return false;
                }
                return true;
            }

            function axisTestY02(a, b, fa, fb) {
                p0 = -a * v[0].x + b * v[0].z;
                p2 = -a * v[2].x + b * v[2].z;
                if (p0 < p2) {
                    min = p0;
                    max = p2;
                } else {
                    min = p2;
                    max = p0;
                }
                rad = fa * obb.e.x + fb * obb.e.z;
                if (min > rad || max < -rad) {
                    return false;
                }
                return true;
            }

            function axisTestY1(a, b, fa, fb) {
                p0 = -a * v[0].x + b * v[0].z;
                p1 = -a * v[1].x + b * v[1].z;
                if (p0 < p1) {
                    min = p0;
                    max = p1;
                } else {
                    min = p1;
                    max = p0;
                }
                rad = fa * obb.e.x + fb * obb.e.z;
                if (min > rad || max < -rad) {
                    return false;
                }
                return true;
            }

            function axisTestZ12(a, b, fa, fb) {
                p1 = a * v[1].x - b * v[1].y;
                p2 = a * v[2].x - b * v[2].y;
                if (p2 < p1) {
                    min = p2;
                    max = p1;
                } else {
                    min = p1;
                    max = p2;
                }
                rad = fa * obb.e.x + fb * obb.e.y;
                if (min > rad || max < -rad) {
                    return false;
                }
                return true;
            }

            function axisTestZ0(a, b, fa, fb) {
                p0 = a * v[0].x - b * v[0].y;
                p1 = a * v[1].x - b * v[1].y;
                if (p0 < p1) {
                    min = p0;
                    max = p1;
                } else {
                    min = p1;
                    max = p0;
                }
                rad = fa * obb.e.x + fb * obb.e.y;
                if (min > rad || max < -rad) {
                    return false;
                }
                return true;
            }

            /**
             * Sets max and min to the maximum and minimum of x0, x1, and x2.
             *
             * @param  {number} x0
             * @param  {number} x1
             * @param  {number} x2

             */
            function findMinMax(x0, x1, x2) {
                min = max = x0;
                if (x1 < min) {
                    min = x1;
                } else if (x1 > max) {
                    max = x1;
                }
                if (x2 < min) {
                    min = x2;
                } else if (x2 > max) {
                    max = x2;
                }
            }

            // Calculate the rotated triangle, which reduces this to an
            // AABB-triangle test
            for (i = 0; i < 3; i += 1) {
                v[i].subVectors(tri[i], obb.c);
                v[i].set(v[i].dot(obb.u[0]),
                         v[i].dot(obb.u[1]),
                         v[i].dot(obb.u[2]));
            }

            // Compute edges
            for (i = 0; i < 3; i += 1) {
                e[i].subVectors(v[(i + 1) % 3], v[i]);
            }

            fex = Math.abs(e[0].x);
            fey = Math.abs(e[0].y);
            fez = Math.abs(e[0].z);
            if (!axisTestX01(e[0].z, e[0].y, fez, fey) ||
                !axisTestY02(e[0].z, e[0].x, fez, fex) ||
                !axisTestZ12(e[0].y, e[0].x, fey, fex)) {
                return null;
            }

            fex = Math.abs(e[1].x);
            fey = Math.abs(e[1].y);
            fez = Math.abs(e[1].z);
            if (!axisTestX01(e[1].z, e[1].y, fez, fey) ||
                !axisTestY02(e[1].z, e[1].x, fez, fex) ||
                !axisTestZ0(e[1].y, e[1].x, fey, fex)) {
                return null;
            }

            fex = Math.abs(e[2].x);
            fey = Math.abs(e[2].y);
            fez = Math.abs(e[2].z);
            if (!axisTestX2(e[2].z, e[2].y, fez, fey) ||
                !axisTestY1(e[2].z, e[2].x, fez, fex) ||
                !axisTestZ12(e[2].y, e[2].x, fey, fex)) {
                return null;
            }

            // Test for the AABB of the triangle
            findMinMax(v[0].x, v[1].x, v[2].x);
            if (min > obb.e.x ||
                max < -obb.e.x) {
                return null;
            }

            findMinMax(v[0].y, v[1].y, v[2].y);
            if (min > obb.e.y ||
                max < -obb.e.y) {
                return null;
            }

            findMinMax(v[0].z, v[1].z, v[2].z);
            if (min > obb.e.z ||
                max < -obb.e.z) {
                return null;
            }

            // Test if the box penetrates the triangle's plane
            normal.crossVectors(e[0], e[1]);
            if (!planeBoxOverlap(normal, v[0], obb.e)) {
                return null;
            }

            return {
                contactNormal: contactNormal,
                penetration: penetration
            };
        }

        /**
         * Tests if an OBB is penetrating the ground
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~OrientedBoundingBox} obb
         * @param  {number} groundElevation   height of the ground
         * @return {(module:vbot/physics~Collision|boolean)}   the collision if they're colliding
         *                                    or false otherwise
         */
        function testOBBGround(obb, groundElevation) {
            var signX, signY, signZ,
                Y,
                minY = Infinity,
                penetration;

            for (signX = -1; signX <= 1; signX += 2) {
                for (signY = -1; signY <= 1; signY += 2) {
                    for (signZ = -1; signZ <= 1; signZ += 2) {
                        Y = obb.c.y + signX * obb.e.x * obb.u[0].y +
                                      signY * obb.e.y * obb.u[1].y +
                                      signZ * obb.e.z * obb.u[2].y;
                        if (minY > Y) {
                            minY = Y;
                        }
                    }
                }
            }

            penetration = groundElevation - minY;
            if (penetration > 0) {
                return {
                    penetration: penetration,
                    contactNormal: new THREE.Vector3(0, 1, 0)
                };
            } else {
                return null;
            }
        }

        /**
         * Converts a THREE.Box3 into an OrientedBoundingBox.
         *
         * Note a THREE.Box3 is an axis-aligned bounding box.
         *
         * @memberof module:vbot/physics~
         * @param  {THREE.Box3} box
         * @return {module:vbot/physics~OrientedBoundingBox}
         */
        function convertBox3ToOBB(box) {
            return {
                c: box.center(),
                e: box.size().divideScalar(2),
                u: [new THREE.Vector3(1, 0, 0),
                    new THREE.Vector3(0, 1, 0),
                    new THREE.Vector3(0, 0, 1)],
                clone: function () { return this; }
            };
        }

        /**
         * Makes an array of OBBs that, together, bound a CombinedPhysicsObject.
         *
         * While they are implemented as oriented bounding boxes, the boxes
         * are actually axis-aligned.
         *
         * This is done by finding the AABB of the object and subdividing the
         * box on each axis iteratively, largest-axis first, until every
         * dimension of every box is less than scale. This tree is then merged
         * upward if there are adjacent boxes that, after merging would result
         * in a box. This reduces the number of boxes, saving time during later
         * collision detection.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         * @param  {number} scale how tightly the boxes will bound the
         *                        object
         * @return {Array.<module:vbot/physics~OrientedBoundingBox>}
         */
        function buildObjectOBBs(obj, scale) {
            // Calculate bounding box
            var aabb = (new THREE.Box3()).setFromObject(obj.geometry),
                // Actual bounds
                xmax = aabb.max.x,
                xmin = aabb.min.x,
                ymax = aabb.max.y,
                ymin = aabb.min.y,
                zmax = aabb.max.z,
                zmin = aabb.min.z,
                boxes = [],
                xScale = xmax - xmin,
                yScale = ymax - ymin,
                zScale = zmax - zmin;

            /**
             * Find all the faces that intersect the box.
             *
             * @param  {THREE.Box3} box
             * @param  {Array.<Array.<THREE.Vector3>>} faces
             * @return {Array.<Array.<THREE.Vector3>>} all the faces that intersect the box
             */
            function findIntersectingFaces(box, faces) {
                return faces.filter(function (face) {
                    return testOBBTri(convertBox3ToOBB(box), face);
                });
            }

            /**
             * Find all the faces that are entirely in the box.
             *
             * @param  {THREE.Box3} box
             * @param  {Array.<Array.<THREE.Vector3>>} faces
             * @return {Array.<Array.<THREE.Vector3>>} the faces that are entirely in the
             *                             box
             */
            function findContainedFaces(box, faces) {
                return faces.filter(function (face) {
                    return box.containsPoint(face[0]) &&
                           box.containsPoint(face[1]) &&
                           box.containsPoint(face[2]);
                });
            }

            /**
             * Combine all the combinable boxes along the given axis.
             *
             * Combinable boxes are those
             * that are bordering each other on the border between the "left"
             * side and the "right" side that have the same dimensions
             * perpendicular to the axis. If any boxes could not be combined,
             * those are included too.
             *
             * These boxes are combinable:
             *
             *     +--------------+-------------+
             *     |              |             |
             *     |     Left     |    Right    |
             *     |              |             |
             *     +--------------+-------------+
             *
             *                    |
             *                    v
             *
             *     +----------------------------+
             *     |                            |
             *     |          Combined          |
             *     |                            |
             *     +----------------------------+
             *
             * These are not:
             *
             *     +--------------+-------------+
             *     |              |             |
             *     |     Left     |             |
             *     |              |    Right    |
             *     +--------------+             |
             *                    |             |
             *                    +-------------+
             *
             * @param  {THREE.Box3[]} leftBoxes
             * @param  {THREE.Box3[]} rightBoxes
             * @param  {number} axis  the index of the axis
             * @return {THREE.Box3[]} the combined boxes
             */
            function mergeBoxes(leftBoxes, rightBoxes, axis) {
                // We're going to combine the boxes across the given axis
                // using a hashmap
                var boxMap = Object.create(null),
                    boxes = [];

                function addToMap(box) {
                    var nonAxisDimensions = [],
                        i;

                    for (i = 0; i < 3; i += 1) {
                        if (i !== axis) {
                            nonAxisDimensions.push(box.min.getComponent(i));
                            nonAxisDimensions.push(box.max.getComponent(i));
                        }
                    }

                    // Convert to use as key
                    nonAxisDimensions = nonAxisDimensions.toString();

                    // Create entry if none exists
                    if (!boxMap[nonAxisDimensions]) {
                        boxMap[nonAxisDimensions] = [box];
                    } else {
                        // Otherwise, just add to the list of boxes with the
                        // same dimensions
                        boxMap[nonAxisDimensions].push(box);
                    }
                }

                /**
                 * Finds the box that contains all of the given boxes.
                 *
                 * @param  {THREE.Box3[]} boxes
                 * @return {THREE.Box3}
                 */
                function unionBoxes(boxes) {
                    return boxes.reduce(
                        function (box1, box2) {
                            return box1.union(box2);
                        });
                }

                // Add them to the map
                leftBoxes.forEach(addToMap);
                rightBoxes.forEach(addToMap);

                // Then, merge mergable boxes, if possible
                for (var boxDimensions in boxMap) {
                    if (boxMap[boxDimensions].length === 1) {
                        boxes.push(boxMap[boxDimensions][0]);
                    } else {
                        boxes.push(unionBoxes(boxMap[boxDimensions]));
                    }
                }

                return boxes;
            }

            /**
             * Fits a box to the given faces and splits it in half along the
             * longest axis, recursively, until the resulting boxes' dimensions
             * are all less than the scale.
             *
             * @param  {THREE.Box3} box          The starting bounding box
             * @param  {Array.<Array.<THREE.Vector3>>} faces
             * @return {THREE.Box3[]}            All of the boxes that bound
             *                                   the given faces.
             */
            function splitBox(box, faces) {
                var extent,
                    faceBox,
                    splitAxis,
                    splitAxisLength = -Infinity,
                    currentAxis,
                    midpoint,
                    leftBox,
                    leftFaces,
                    rightBox,
                    rightFaces,
                    leftBoxes,
                    rightBoxes;

                // Call it a day if there's nothing in the box
                if (faces.length === 0) {
                    return [];
                }

                // Shrink the box to fit the faces better
                faceBox = (new THREE.Box3()).setFromPoints(_.flatten(faces));
                box = box.clone().intersect(faceBox);

                // Calculate the extend of the box
                extent = box.max.clone().sub(box.min);

                // Find the largest axis to use as the splitting axis
                for (currentAxis = 0; currentAxis < 3; currentAxis += 1) {
                    if (extent.getComponent(currentAxis) > splitAxisLength &&
                        extent.getComponent(currentAxis) > scale) {
                        splitAxis = currentAxis;
                        splitAxisLength = extent.getComponent(currentAxis);
                    }
                }

                // If every dimension is smaller than the scale, save the
                // box and terminate
                if (splitAxis === undefined ||
                    (extent.x === 0 || extent.y === 0 || extent.z === 0)) {
                    return [box];
                }

                // Find the spatial median
                midpoint = box.center().getComponent(splitAxis);

                // Split the box at the midpoint
                leftBox = new THREE.Box3(box.min.clone(),
                    box.max.clone());
                leftBox.max.setComponent(splitAxis, midpoint);
                rightBox = new THREE.Box3(box.min.clone(),
                    box.max.clone());
                rightBox.min.setComponent(splitAxis, midpoint);

                // Find the faces for each box
                leftFaces = findIntersectingFaces(leftBox, faces);
                rightFaces = findIntersectingFaces(rightBox, faces);

                // Recurse into each box
                leftBoxes = splitBox(leftBox, leftFaces);
                rightBoxes = splitBox(rightBox, rightFaces);

                return mergeBoxes(leftBoxes, rightBoxes, splitAxis);
            }

            // Let's only try subdividing if there's thickness. This test
            //  also excludes the ground nicely
            if (xScale !== 0 &&
                yScale !== 0 &&
                zScale !== 0) {
                // Build all candidate boxes recursively
                boxes = splitBox(aabb, obj.physics.faces);
            } else {
                boxes = [aabb];
            }


            // Help me with visualization
            boxes.forEach(function (box) {
                var geom = new THREE.BoxGeometry(1, 1, 1),
                    mesh = new THREE.Mesh(geom,
                                          new THREE.MeshBasicMaterial({
                                              color: 0x888888,
                                              wireframe: true
                                          }));

                mesh.position = box.center();
                mesh.scale.set(box.max.x - box.min.x,
                               box.max.y - box.min.y,
                               box.max.z - box.min.z);

                collisionVolumeObjects.add(mesh);
            });

            // We'll convert those to OBBs
            var result = boxes.map(convertBox3ToOBB);
            return result;
        }

        /**
         * Builds an array of OBBs for a skinned robot.
         *
         * Each OBB corresponds to a bone in the robot, fitting around all
         * of the vertices that are primarily controlled by that bone (i.e. out
         * of all the bones that move a given vertex, the one that has the
         * most weight is said to "control" a vertex). These OBBs automatically
         * update when the bones move.
         *
         * It is expected that we pass in the parent of the actual robot object,
         * which consists of a single THREE.SkinnedMesh.
         *
         * @memberof module:vbot/physics~
         * @param  {THREE.Object3D} robot     the robot object
         * @return {module:vbot/physics~OrientedBoundingBox[]} the OBBs of the bones
         */
        function buildRobotOBBs(robot) {
           var s = robot.children[0].geometry;
           var boneMap = {};
           var i, j;
           var boxes = [];
           var vertexWeights = {};

           // Find which vertices belong to which bone
           for (i = 0; i < s.skinWeights.length; i += 1) {
               var idxs = s.skinIndices[i].toArray(),
                   weights = s.skinWeights[i].toArray(),
                   maxWeight = -Infinity,
                   maxIndex = undefined,
                   bone;

               for (j = 0; j < 4; j += 1) {
                   if (maxWeight !== undefined &&
                       maxWeight < weights[j]) {
                       maxWeight = weights[j];
                       maxIndex = idxs[j];
                   }
               }

               if (maxIndex !== undefined &&
                   weights[1] !== undefined) {
                   if (!boneMap.hasOwnProperty(maxIndex)) {
                       boneMap[maxIndex] = [];
                   }

                   boneMap[maxIndex].push(i);
                   vertexWeights[i] = /*findFaces(s.faces, i).
                       map(findFaceArea.bind(null, s.vertices)).
                       reduce(function (x, y) { return x+y; })*/ 1;
               }
           }

           /**
            * Project given points onto the given axis.
            *
            * @param  {THREE.Vector3} basis    the axis to project onto
            * @param  {THREE.Vector3[]} points the points to project
            * @return {number[]}               the projections of the points
            */
           function projectPoints(basis, points) {
               return points.map(function (pt) { return numeric.dot(basis, pt); });
           }

           Object.keys(boneMap).forEach(function (boneIndex) {
               var bone = robot.children[0].skeleton.bones[boneIndex],
                   vertices = boneMap[boneIndex].map(function (vertIndex) {
                           return (s.vertices[vertIndex].clone()).toArray();
                       }),
                   weights = boneMap[boneIndex].map(function (vertIndex) {
                           return vertexWeights[vertIndex];
                       }),
                   totalWeight = weights.reduce(function (x, y) {
                           return x + y;
                       }),
                   weightedVertices = _.zip(vertices, weights).
                       map(function (params) {
                           var vert = params[0],
                               weight = params[1];
                           return numeric.mul(vert, weight);
                       }),
                   centroid = numeric.div(weightedVertices.reduce(function (x,y) {
                           return numeric.add(x,y);
                       }), totalWeight),
                   covariance = weightedVertices.map(function (x) {
                           var diff = [numeric.sub(x, centroid)];
                           return numeric.dot(numeric.transpose(diff), diff);
                       }).reduce(function (cov, x) {
                           return numeric.add(cov, x);
                       });

               if (vertices.length > 2) {
                   var eigen = numeric.eig(covariance);

                   var E = eigen.E.x;
                   var sc = [0, 0, 0];
                   var center = [0, 0, 0];
                   var boxBases = [];

                   for (var i = 0; i < 3; i += 1) {
                       var basis = [E[0][i], E[1][i], E[2][i]];
                       var projectedPoints = projectPoints(basis, vertices);
                       var max = Math.max.apply(Math, projectedPoints);
                       var min = Math.min.apply(Math, projectedPoints);
                       var middle = (max + min) / 2;
                       boxBases.push(new THREE.Vector3(E[0][i], E[1][i], E[2][i]));
                       center = numeric.add(center, numeric.dot(basis, middle));
                       sc[i] = (max - min)/2;
                   }

                   var p = bone.worldToLocal(
                       robot.children[0].localToWorld(
                           new THREE.Vector3().fromArray(center)));

                   var obb = new OrientedBoundingBox(p, boxBases, sc, bone);

                   boxes.push(obb);
               }
           });

           return boxes;
        }

        /**
         * Detects a collision between two CombinedPhysicsObjects.
         *
         * The first object is assumed to be a dynamic object.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj1
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj2
         * @return {module:vbot/physics~Collision?}            The collision if
         *                                                     they're
         *                                                     colliding
         */
        function detectSingleCollision(obj1, obj2) {
            var collision, i, j,
                bestCollision = {
                    penetration: -Infinity
                },
                obbs1 = obj1.physics.obbs,
                obbs2 = obj2.physics.obbs,
                obb1, obb2,
                sphere1 = obj1.physics.boundingSphere,
                sphere2 = obj2.physics.boundingSphere;

            // Prevent self testing
            if (obj1 !== obj2) {
                // Check bounding spheres first
                if (!sphere1.intersectsSphere(sphere2)) {
                    return null;
                }

                for (i = 0; i < obbs1.length; i += 1) {
                    obb1 = obbs1[i].clone();
                    for (j = 0; j < obbs2.length; j += 1) {
                        obb2 = obbs2[j].clone();
                        collision = testOBBOBB(obb1, obb2);
                        if (collision) {
                            if (collision.penetration !== Infinity) {
                                collision.otherObject = obj2.geometry;
                                collision.type = obj2.physics.type;

                                if (collision.penetration > bestCollision.penetration) {
                                    bestCollision = collision;
                                }
                            }
                        }
                    }
                }

                if (bestCollision.penetration > 0) {
                    return bestCollision;
                }
            }

            return null;
        }

        /**
         * Takes a dynamic physics object and finds if it's colliding with
         * any static objects.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         * @return {module:vbot/physics~Collision[]} An array of collisions
         */
        function detectStaticCollisions(obj) {
            var collision,
                // Check if any face of any object intersects with
                //  this object's bounding box
                totalCollisions =
                    staticObjects.reduce(function (collisions, obj2) {
                        collision = detectSingleCollision(obj, obj2);
                        if (collision !== null) {
                            collisions.push(collision);
                        }

                        return collisions;
                    }, []);

            if (totalCollisions.length > 0) {
                return totalCollisions;
            } else {
                return [];
            }
        }

        /**
         * Detects if a CombinedPhysicsObject is colliding with the ground.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         * @return {module:vbot/physics~Collision?} the collision if there is one
         */
        function detectGroundCollision(obj) {
            var i, collision, bestCollision = {
                penetration: 0
            };
            for (i = 0; i < obj.physics.obbs.length; i += 1) {
                collision = testOBBGround(obj.physics.obbs[i], groundElevation);
                if (collision) {
                    if (collision.penetration > bestCollision.penetration) {
                        bestCollision = collision;
                        bestCollision.type = 'ground';
                    }
                }
            }
            if (bestCollision.penetration > 0) {
                return bestCollision;
            } else {
                return null;
            }
        }

        /**
         * If this object is held, then find the holding object, which
         * should be the robot. I'm not sure why this exists. If the object
         * is not held, then return undefined.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj       the held object
         * @return {(module:vbot/physics~CombinedPhysicsObject|undefined)} the holding object, if
         *                                                there is one
         */
        function getHoldingObject(obj) {
            var ancestor;

            if (obj.physics.state === 'held') {
                ancestor = obj.geometry;
                while (!(getPhysicsObject(ancestor, 'dynamic') &&
                    getPhysicsObject(ancestor, 'dynamic').physics.state === 'controlled')) {
                    ancestor = ancestor.parent;
                }
                ancestor = getPhysicsObject(ancestor, 'dynamic');
                return ancestor;
            }
        }

        /**
         * Finds all of the collisions and attaches them to the geometries of
         * of the colliding CombinedPhysicsObjects.
         *
         * @memberof module:vbot/physics
         */
        function detectCollisions() {
            var i, j, collision, collisions;

            // Clear previous collisions
            for (i = 0; i < dynamicObjects.length; i += 1) {
                dynamicObjects[i].geometry.collisions = [];
            }

            // For each dynamic object
            for (i = 0; i < dynamicObjects.length; i += 1) {
                // Don't test objects at rest
                if (dynamicObjects[i].physics.state !== 'rest') {
                    // Test against static objects
                    collisions = detectStaticCollisions(dynamicObjects[i]);
                    if (collisions.length > 0) {
                        dynamicObjects[i].geometry.collisions =
                            dynamicObjects[i].geometry.collisions.concat(collisions);
                    }
                }

                // Test against dynamic objects
                for (j = 0; j < dynamicObjects.length; j += 1) {
                    collision = detectSingleCollision(dynamicObjects[i],
                                               dynamicObjects[j]);
                    if (collision !== null) {
                        dynamicObjects[i].geometry.collisions.push(collision);
                    }
                }

                // Test against the ground
                collision = detectGroundCollision(dynamicObjects[i]);
                if (collision !== null) {
                    dynamicObjects[i].geometry.collisions.push(collision);
                }
            }
        }

        /**
         * Change a dynamic object's state. This function is mostly for use
         * by parts of the program that are outside of the physics engine.
         *
         * Note that valid states are:
         * - rest
         * - falling
         * - controlled
         * - held
         *
         * @memberof module:vbot/physics
         * @param  {THREE.Object3D} obj the object whose state is being changed
         * @param  {string} newState    the new state
         */
        function changeObjectState(obj, newState) {
            getPhysicsObject(obj, 'dynamic').physics.state = newState;
        }

        /**
         * Get the CombinedPhysicsObject of a three.js object. Requires the
         * type of the object.
         *
         * Available types are:
         * - dynamic
         * - static
         * - ground (returns undefined, as the ground is not a real object)
         *
         * @memberof module:vbot/physics~
         * @param  {THREE.Object3D} obj
         * @param  {string} type
         * @return {(module:vbot/physics~CombinedPhysicsObject|undefined)}
         */
        function getPhysicsObject(obj, type) {
            if (type === 'dynamic' &&
                dynamicObjectIndices.hasOwnProperty(obj.id)) {
                return dynamicObjects[dynamicObjectIndices[obj.id]];
            }
            if (type === 'static' &&
                staticObjectIndices.hasOwnProperty(obj.id)) {
                return staticObjects[staticObjectIndices[obj.id]];
            }
            if (type === 'ground') {
                return undefined;
            }
            if (type === undefined) {
                throw type + ' is not an object type';
            }
            return undefined;
        }

        /**
         * Detects a collision between two given objects.
         *
         * @memberof module:vbot/physics~
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj1
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj2
         * @param  {string} type
         * @return {module:vbot/physics~Collision?} a collision if there is one
         */
        function detectCollision(obj1, obj2, type) {
            if (type === 'static' || type === 'dynamic') {
                return detectSingleCollision(obj1, obj2);
            } else if (type === 'ground') {
                return detectGroundCollision(obj1);
            } else {
                throw type + ' is not an object type';
            }
        }

        /**
         * If this object has any collisions attached, it resolves them.
         *
         * @memberof module:vbot/physics~
         * @param  {number} dt the amount of time that's passed. Unused.
         * @param  {module:vbot/physics~CombinedPhysicsObject} obj
         */
        function resolveCollision(dt, obj) {
            /**
             * Resolves collisions between the robot and another object.
             *
             * @param  {module:vbot/physics~CombinedPhysicsObject} obj         the robot
             * @param  {module:vbot/physics~CombinedPhysicsObject} otherObject
             * @param  {string} type                       the other object's
             *                                             type

             */
            function resolveControlled(obj, otherObject, type) {
                var newCollision = detectCollision(obj, otherObject, type),
                    displacement;

                // Ignore collisions with held objects
                if (newCollision &&
                    (newCollision.type === 'ground' ||
                     otherObject.physics.state !== 'held')) {
                    // If it's a horizontal collision, check if the object can be climbed
                    if (newCollision.contactNormal.y === 0) {
                        obj.geometry.position.y += staticCollisionResolution;
                        updateObjectLocation(obj);
                        // Run collision detection again
                        newCollision = detectCollision(obj, otherObject, type);
                        // If it can't be climbed
                        if (newCollision) {
                            // Undo the climbing
                            obj.geometry.position.y -= staticCollisionResolution;
                        } else {
                            return;
                        }
                    }
                    // Add the displacement needed to resolve the collision
                    displacement = newCollision.contactNormal.clone().
                            multiplyScalar(newCollision.penetration);
                    obj.geometry.position.add(displacement);

                    // If we get pushed up, stop falling
                    if (displacement.y > 0) {
                        obj.physics.verticalVelocity = 0;
                    }
                    updateObjectLocation(obj);
                }
            }

            /**
             * Resolves collisions between held objects and other objects.
             *
             * @param  {module:vbot/physics~CombinedPhysicsObject} obj         the held object
             * @param  {module:vbot/physics~CombinedPhysicsObject} otherObject the colliding object
             * @param  {string} type                       the type of the
             *                                             colliding object

             */
            function resolveHeld(obj, otherObject, type) {
                // If the object is held, then we'll displace the
                // holding object, unless the other object is
                // the holding object
                var holder = getHoldingObject(obj),
                    shoulder = obj.geometry.parent.parent,
                    newCollision,
                    displacement,
                    worldToBody = new THREE.Matrix4(),
                    armVector,
                    armLength,
                    displacementAlongArm,
                    displacementPerpendicularToArm,
                    angularDisplacement,
                    yDisplacement,
                    temp;

                if (otherObject !== holder) {
                    newCollision = detectCollision(obj, otherObject, type);
                    if (newCollision) {
                        displacement = newCollision.contactNormal.clone().
                                multiplyScalar(newCollision.penetration);
                        // All calculation should be done in body space
                        worldToBody.extractRotation(shoulder.parent.matrixWorld);
                        worldToBody.getInverse(worldToBody);

                        armVector = shoulder.positionWorld().clone().
                            sub(obj.geometry.positionWorld());

                        armVector.applyMatrix4(worldToBody);

                        armLength = armVector.length();

                        // Normalize the arm vector
                        armVector.divideScalar(armLength);

                        // We're only going to move the arm with vertical
                        // displacement, so that moving the arm doesn't lift
                        // the robot
                        yDisplacement = new THREE.Vector3(0, displacement.y,
                            0);
                        displacement.y = 0;

                        displacementAlongArm = armVector.clone().
                            multiplyScalar(-yDisplacement.dot(armVector));

                        // All the displacement that's not along the arm
                        //  is perpendicular
                        displacementPerpendicularToArm = yDisplacement.clone().
                            sub(displacementAlongArm);

                        angularDisplacement = displacementPerpendicularToArm.
                            clone().
                            cross(armVector).
                            multiplyScalar(armLength);

                        // For some godawful reason, x and z get switched and I
                        // don't know why, but you get some weird behavior if you don't
                        // switch them
                        temp = angularDisplacement.x;
                        angularDisplacement.x = angularDisplacement.z;
                        angularDisplacement.z = temp;

                        shoulder.rotateOnAxis(
                            angularDisplacement.clone().normalize(),
                            angularDisplacement.length());

                        holder.geometry.position.add(displacement);

                        // If we get pushed up, neutralize speed
                        if (displacement.y > 0) {
                            holder.physics.verticalVelocity = 0;
                        }
                        updateObjectLocation(holder);
                    }
                }
            }

            /**
             * Resolve collisions for a falling object.
             *
             * @param  {module:vbot/physics~CombinedPhysicsObject} obj         the falling
             *                                                  object
             * @param  {module:vbot/physics~CombinedPhysicsObject} otherObject the colliding
             *                                                  object
             * @param  {string} type                            the type of
             *                                                  the other object
             */
            function resolveFalling(obj, otherObject, type) {
                var newCollision = detectCollision(obj, otherObject, type),
                    displacement;

                if (newCollision) {
                    displacement = newCollision.contactNormal.clone().
                            multiplyScalar(newCollision.penetration);
                    obj.geometry.position.add(displacement);
                    updateObjectLocation(obj);
                    // If we hit the ground, then stop falling
                    if (displacement.y > 0 &&
                        (newCollision.type === 'static' ||
                         newCollision.type === 'ground')) {
                        obj.physics.state = 'rest';
                    }
                }
            }

            // If there is a collision
            if (obj.geometry.collisions.length > 0) {
                obj.geometry.collisions.sort(function (a, b) {
                    return b.penetration - a.penetration;
                });
                obj.geometry.collisions.forEach(function (collision) {
                    var type = collision.type,
                        otherObject = getPhysicsObject(
                            collision.otherObject,
                            type);

                    updateObjectLocation(obj);

                    switch (obj.physics.state) {
                        case 'controlled':
                            resolveControlled(obj, otherObject, type);
                            break;
                        case 'held':
                            resolveHeld(obj, otherObject, type);
                            break;
                        case 'falling':
                            resolveFalling(obj, otherObject, type);
                            break;
                    }
                });
            }
        }

        /**
         * Resolve all the collisions.
         *
         * @memberof module:vbot/physics
         * @param  {number} dt the step length in seconds (unused)
         */
        function resolveCollisions(dt) {
            var i;
            for (i = 0; i < dynamicObjects.length; i += 1) {
                resolveCollision(dt, dynamicObjects[i]);
            }
        }

        /**
         * Toggles visibility on all of the collision volumes, which show up
         * as wireframes.
         *
         * @memberof module:vbot/physics
         * @param {THREE.Scene} scene the rendered scene to add or remove the
         *                            collision volumes from
         */
        function toggleCollisionVolumes(scene) {
            if (collisionVolumeObjects.parent) {
                scene.remove(collisionVolumeObjects);
            } else {
                scene.add(collisionVolumeObjects);
            }
        }

        /**
         * Sets the resolution used to determine the minimum size of
         * the bounding boxes on static objects. See the scale parameter of
         * buildObjectOBBs for more detail.
         *
         * @memberof module:vbot/physics
         * @param {number} size
         */
        function setCollisionVolumeResolution(size) {
            staticCollisionResolution = size;
        }

        return {
                addObject: addObject,
                detectCollisions: detectCollisions,
                updateObjects: updateObjects,
                changeObjectState: changeObjectState,
                resolveCollisions: resolveCollisions,
                //toggleCollisionVolumes: toggleCollisionVolumes,
                setCollisionVolumeResolution: setCollisionVolumeResolution
            };
    }());

    return physics;
});
