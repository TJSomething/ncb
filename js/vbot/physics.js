/* jslint browser: true */

var THREE = require('./three');
var numeric = require('numeric');
var collision = require('./collision');
var boxIntersect = require('box-intersect');
var _ = require('underscore');

module.exports = (function () {
    'use strict';

    var physicsObjects = [],
        // This is used to quickly address objects
        objectIndices = {},
        staticCollisionResolution = 0.1,
        collisionVolumeObjects = new THREE.Object3D(),
        groundElevation = Infinity,
        robot;

    /**
     * Holds the physics attributes of an object.
     *
     * @typedef {Object} module:vbot/physics~PhysicsObject
     * @property {THREE.Sphere} boundingSphere the bounding sphere of the object
     * @property {Array.<module:vbot/collision.OBB>} obbs all of the OBBs
     *                                                      for the object
     * @property {Number} verticalVelocity the vertical velocity in m/s
     * @property {string} type the type of the object (static, dynamic, or ground)
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
     * Finds the robot's arm bones
     *
     * @param {THREE.Object3D} robot the robot parent
     * @returns {Array.<THREE.Bone>} the arm bones
     */
    function findArmBones(robot) {
        // Find the arm bones
        var armBones = [];
        // If there are arms, then there are arm bones
        if (robot.larm !== undefined) {
            [robot.larm, robot.rarm].forEach(function (arm) {
                arm.traverse(function (bone) {
                    armBones.push(bone);
                });
            });
        }

        return armBones;
    }

    /**
     * Updates the sweep lists for a physics object.
     */
    function updateSweepList(physicsObj) {
        var i;

        physicsObj.aabbs = physicsObj.obbs.map(function (obb) {
            var box = new THREE.Box3().setFromPoints(obb.calcVertices());
            return [box.min.x, box.min.y, box.min.z,
                    box.max.x, box.max.y, box.max.z];
        });

        /*
            physicsObj.sweepLists = [];
            for (i = 0; i < 3; i += 1) {
                physicsObj.sweepLists.push({
                    start: _.range(physicsObj.obbs.length),
                    end: _.range(physicsObj.obb.length)
                });
            }

            physicsObj.obbs.forEach(function (obb) {
                obb.aabb = new THREE.Box3();
            });
        }

        // Update the axis-aligned bounds for each OBB
        physicsObj.obbs.forEach(function (obb) {
            obb.aabb.fromPoints(obb.calcVertices());
        });

        // Update the sweep lists
        for (i = 0; i < 3; i += 1) {
            physicsObj.sweepLists[i].start.sort(function (a, b) {
                return physicsObj.obbs[a].aabb.min.getComponent(i) <
                       physicsObj.obbs[b].aabb.min.getComponent(i); 
            });
            physicsObj.sweepLists[i].end.sort(function (a, b) {
                return physicsObj.obbs[a].aabb.max.getComponent(i) <
                       physicsObj.obbs[b].aabb.max.getComponent(i); 
            });
        }
        */
    }

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
            // Build a broad OBB around the torso and legs
            obj.geometry.broadOBB = obj.physics.obbs[0];
            // Make a set of the arms bones. We're going to
            // handle arm collisions differently from body collisions
            obj.geometry.armBones = findArmBones(obj.geometry).reduce(function (boneSet, bone) {
                boneSet[bone.name] = true;
                return boneSet;
            }, {});
            robot = obj;
        } else {
            obj.physics.obbs =
                [function () {
                    var boundingSphere = b.getBoundingSphere(),
                        c = boundingSphere.center,
                        // Half extents
                        e = b.max.clone().
                            sub(b.min).
                            divideScalar(2),
                        u = [new Vec(1, 0, 0),
                             new Vec(0, 1, 0),
                             new Vec(0, 0, 1)];
                    return new collision.OBB(c, u, e.toArray(), obj.geometry);
                }()];
        }
        obj.physics.obbs.forEach(function (obb) {
            collisionVolumeObjects.add(collision.OBBHelper(obb));
        });
        var scene = obj.geometry.parent;
        while (scene.parent) {
            scene = scene.parent;
        }

        obj.physics.boundingSphere = collision.calcBoundingSphereFromOBBs(obj.physics.obbs);
        updateSweepList(obj.physics);

        obj.physics.updateBoundingVolumes = function () {
            obj.physics.boundingSphere = collision.calcBoundingSphereFromOBBs(obj.physics.obbs);
            updateSweepList(obj.physics);
        };

        obj.physics.verticalVelocity = obj.physics.verticalVelocity || 0;

        physicsObjects.push(obj);
        objectIndices[obj.geometry.id] = physicsObjects.length - 1;
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
        obj.physics.faces = getGlobalFaces(obj);

        obj.physics.obbs = buildObjectOBBs(obj, staticCollisionResolution);

        obj.physics.boundingSphere = collision.calcBoundingSphereFromOBBs(obj.physics.obbs);
        updateSweepList(obj.physics);

        // Static objects don't move, so they don't need updating
        obj.physics.updateBoundingVolumes = function () {};

        physicsObjects.push(obj);
        objectIndices[obj.geometry.id] = physicsObjects.length - 1;

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

        if (groundElevation > objAABB.min.y) {
            groundElevation = objAABB.min.y;
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

        obj.physics.updateBoundingVolumes();
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
        if (obj.physics.type === 'dynamic' &&
            obj.physics.state === 'controlled' ||
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
        for (i = 0; i < physicsObjects.length; i += 1) {
            updateObject(dt, physicsObjects[i]);
        }
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
     * @return {Array.<module:vbot/collision.OBB>}
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

        function addBoxToVisuals(box) {
            collisionVolumeObjects.add(collision.OBBHelper(box));
        }

        function voxelize(box, faces) {
            // Find dimensions of a voxel
            var extent = box.max.clone().sub(box.min);
            // For the voxel size
            var vSize = new THREE.Vector3(extent.x, extent.y, extent.z);
            var currentDim;
            for (var currentAxis = 0; currentAxis < 3; currentAxis += 1) {
                currentDim = vSize.getComponent(currentAxis);
                while (currentDim > staticCollisionResolution) {
                    currentDim /= 2;
                }
                vSize.setComponent(currentAxis, currentDim);
            }

            var uvStep = [new THREE.Vector3(), new THREE.Vector3()];
            var uv = [new THREE.Vector3(), new THREE.Vector3()];
            // Indices for the grid on the face
            var u, v;
            var uVec = new THREE.Vector3(),
                vVec = new THREE.Vector3();
            var maxUvIndices = [0,0];
            var i, j;
            var totalIndex;
            var voxelIndices = new THREE.Vector3();
            var shape = extent.clone().divide(vSize);
            var boxesByIndex = new Uint8Array(shape.x*shape.y*shape.z);
            var minIndices = new THREE.Vector3();

            /* The idea behind this algorithm is that we divide the face
               into smaller triangles, such that the subface vertices are
               close enough that one is in every voxel that penetrates
               the face. This is inspired by Fei et al.'s paper on 
               Point-Tesselated Voxelization. However, while they
               use the triangle centroids to generate voxels, I use the vertices.
               Furthermore, instead of scaling all of the sides of every
               triangle, I only scale two of the sides. */
            for (i = 0; i < faces.length; i += 1) {
                // Calculate two of the sides of the face
                uvStep[0].copy(faces[i][1])
                    .sub(faces[i][0]);
                uvStep[1].copy(faces[i][2])
                    .sub(faces[i][0]);
                // Create vectors that we can walk across the face with
                // steps that are smaller than a voxel.
                for (j = 0; j < 2; j += 1) {
                    maxUvIndices[j] = 1;
                    while (Math.abs(uvStep[j].x) > vSize.x ||
                           Math.abs(uvStep[j].y) > vSize.y ||
                           Math.abs(uvStep[j].z) > vSize.z) {
                        uvStep[j].divideScalar(2);
                        maxUvIndices[j] *= 2;
                    }
                }

                // Walk over a grid on the triangle
                uVec.copy(faces[i][0]);
                for (u = 0; u < maxUvIndices[0]; u += 1) {
                    vVec.set(0,0,0);
                    for (v = 0; u + v < maxUvIndices[1]; v += 1) {
                        voxelIndices.addVectors(uVec, vVec)
                            .sub(box.min)
                            .divide(vSize);

                        // Compensate for floating point errors
                        if (Math.abs(voxelIndices.x - shape.x) < 0.00001) {
                            voxelIndices.x = shape.x - 1;
                        }
                        if (Math.abs(voxelIndices.y - shape.y) < 0.00001) {
                            voxelIndices.y = shape.y - 1;
                        }
                        if (Math.abs(voxelIndices.z - shape.z) < 0.00001) {
                            voxelIndices.z = shape.z - 1;
                        }

                        voxelIndices.floor();

                        if (voxelIndices.x < shape.x &&
                            voxelIndices.y < shape.y &&
                            voxelIndices.z < shape.z) {
                            totalIndex = voxelIndices.x * shape.y * shape.z +
                                         voxelIndices.y * shape.z +
                                         voxelIndices.z;
                            boxesByIndex[totalIndex] = 1;
                        }
                        vVec.add(uvStep[1]);
                    }
                    uVec.add(uvStep[0]);
                }
            }
            
            // Now, we can make the boxes
            var boxes = [];
            var tempBox = new THREE.Box3();
            var boxLength;

            var start = Date.now();
            for (i = 0; i < boxesByIndex.length; i += 1) {
                if (boxesByIndex[i] === 1) {
                    tempBox.min.set(Math.floor(i/shape.y/shape.z),
                                    Math.floor(i/shape.z) % shape.y,
                                    i % shape.z)
                        .multiply(vSize)
                        .add(box.min);

                    // Combine boxes in the same row
                    while (boxesByIndex[i+1] === 1 &&
                           (i+1) % shape.z !== 0 ) {
                        i += 1;
                    }

                    tempBox.max.set(Math.floor(i/shape.y/shape.z) + 1,
                                    Math.floor(i/shape.z) % shape.y + 1,
                                    i % shape.z + 1)
                        .multiply(vSize)
                        .add(box.min);

                    boxes.push(collision.OBB.fromBox3(tempBox));
                }
            }
            return boxes;
        }

        // Let's only try subdividing if there's thickness. This test
        //  also excludes the ground nicely
        if (xScale !== 0 &&
            yScale !== 0 &&
            zScale !== 0) {
            // Build all candidate boxes recursively
            boxes = voxelize(aabb, obj.physics.faces);
        } else {
            boxes = [collision.OBB.fromBox3(aabb)];
        }

        boxes.forEach(addBoxToVisuals);

        return boxes;
    }

    /**
     * Figures out which vertices are attached to which bone.
     *
     * It is expected that we pass in the parent of the actual robot object,
     * which consists of a single THREE.SkinnedMesh.
     *
     * @memberof module:vbot/physics~
     * @param {THREE.Object3D} robot the robot parent
     * @returns {object} a map from bone names to the vertices of that bone
     */
    function getRobotBoneVertices(robot) {
        var s = robot.children[0].geometry;
        var boneMap = {};
        var i, j;
        var vertexWeights = {};
        var bonePointMap = {};

        // Find which vertices belong to which bone
        for (i = 0; i < s.skinWeights.length; i += 1) {
            var idxs = s.skinIndices[i].toArray(),
                weights = s.skinWeights[i].toArray(),
                maxWeight = -Infinity,
                maxIndex;

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
            }
        }


        Object.keys(boneMap).forEach(function (boneIndex) {
            var bone = robot.children[0].skeleton.bones[boneIndex];

            bonePointMap[bone.name] =
                boneMap[boneIndex].map(function (vertIndex) {
                    return robot.children[0].localToWorld(s.vertices[vertIndex].clone());
                });
        });

        return bonePointMap;
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
     * @return {module:vbot/collision.OBB[]} the OBBs of the bones
     */
    function buildRobotOBBs(robot) {
        var bonePointMap = getRobotBoneVertices(robot),
            armBones = findArmBones(robot),
            armBoneNameSet = armBones.reduce(function (boneSet, bone) {
                boneSet[bone.name] = true;
                return boneSet;
            }, {}),
            armOBBs = armBones.map(function (bone) {
                if (bonePointMap.hasOwnProperty(bone.name)) {
                    return collision.OBB.fromPoints(bonePointMap[bone.name], bone);
                } else {
                    return null;
                }
            }),
            bodyVertices = Object.keys(bonePointMap).reduce(function (vertices, bone) {
                if (!armBoneNameSet.hasOwnProperty(bone)) {
                    vertices.push.apply(vertices, bonePointMap[bone]);
                }
                return vertices;
            }, []),
            bodyOBB = collision.OBB.fromBox3(new THREE.Box3().setFromPoints(bodyVertices), robot),
            boxes = [bodyOBB];

        armOBBs.forEach(function (obb) {
            if (obb) {
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
     * @return {module:vbot/collision~Collision?}            The collision if
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
            pairs,
            sphere1 = obj1.physics.boundingSphere,
            sphere2 = obj2.physics.boundingSphere;

        // Prevent self testing
        if (obj1 !== obj2) {
            // Check bounding spheres first
            if (!sphere1.intersectsSphere(sphere2)) {
                return null;
            }
            // Find all possibly interesecting pairs of boxes
            pairs = boxIntersect(obj1.physics.aabbs, obj2.physics.aabbs);

            // Test those pairs
            for (i = 0; i < pairs.length; i += 1) {
                obb1 = obbs1[pairs[i][0]].clone();
                obb2 = obbs2[pairs[i][1]].clone();

                collision = obb1.testOBB(obb2);
                if (collision) {
                    if (collision.penetration !== Infinity) {
                        collision.otherObject = obj2.geometry;
                        collision.type = obj2.physics.type;
                        // The robot bones are not the immediate children
                        // of the robot, so we add a body part if we have that
                        if (obb1.name &&
                            obb1.name !== obj1.name) {
                            collision.bodyPart = obb1.name;
                        } else if (obb2.name &&
                                   obb2.name !== obj2.name) {
                            collision.bodyPart = obb2.name;
                        }

                        if (collision.penetration > bestCollision.penetration) {
                            bestCollision = collision;
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
     * any objects.
     *
     * @memberof module:vbot/physics~
     * @param  {module:vbot/physics~CombinedPhysicsObject} obj
     * @return {module:vbot/collision~Collision[]} An array of collisions
     */
    function detectObjectCollisions(obj) {
        var collision,
            // Check if any face of any object intersects with
            //  this object's bounding box
            totalCollisions =
                physicsObjects.reduce(function (collisions, obj2) {
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
     * @return {module:vbot/collision~Collision?} the collision if there is one
     */
    function detectGroundCollision(obj) {
        var i, collision, bestCollision = {
            penetration: 0
        };
        for (i = 0; i < obj.physics.obbs.length; i += 1) {
            collision = obj.physics.obbs[i].testGround(groundElevation);
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
     * Finds all of the collisions and attaches them to the geometries of
     * of the colliding CombinedPhysicsObjects.
     *
     * @memberof module:vbot/physics
     */
    function detectCollisions() {
        var i, collision, collisions;

        // Clear previous collisions
        for (i = 0; i < physicsObjects.length; i += 1) {
            physicsObjects[i].geometry.collisions = [];
        }

        // For each dynamic object
        for (i = 0; i < physicsObjects.length; i += 1) {
            // Don't test objects at rest
            if (physicsObjects[i].physics.type === 'dynamic' &&
                physicsObjects[i].physics.state !== 'rest') {
                // Test against other bjects
                collisions = detectObjectCollisions(physicsObjects[i]);
                physicsObjects[i].geometry.collisions =
                    physicsObjects[i].geometry.collisions.concat(collisions);
                // Test against the ground
                collision = detectGroundCollision(physicsObjects[i]);
                if (collision !== null) {
                    physicsObjects[i].geometry.collisions.push(collision);
                }
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
        var physObj = getPhysicsObject(obj);
        if (physObj.physics.type === 'dynamic') {
            getPhysicsObject(obj).physics.state = newState;
        }
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
     * @return {(module:vbot/physics~CombinedPhysicsObject|undefined)}
     */
    function getPhysicsObject(obj) {
        if (obj !== undefined &&
            objectIndices.hasOwnProperty(obj.id)) {
            return physicsObjects[objectIndices[obj.id]];
        }
        return undefined;
    }

    /**
     * Detects a collision between two given objects.
     *
     * @memberof module:vbot/physics~
     * @param  {module:vbot/physics~CombinedPhysicsObject} obj1
     * @param  {module:vbot/physics~CombinedPhysicsObject} obj2
     * @return {module:vbot/collision~Collision?} a collision if there is one
     */
    function detectCollision(obj1, obj2) {
        if (obj2 === undefined) {
            return detectGroundCollision(obj1);
        } else if (obj2.physics.type === 'static' ||
                   obj2.physics.type === 'dynamic') {
            return detectSingleCollision(obj1, obj2);
        } else {
            throw obj2.physics.type + ' is not an object type';
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
         */
        function resolveControlled(obj, otherObject) {
            var newCollision = detectCollision(obj, otherObject),
                displacement;

            // Ignore collisions with held objects
            if (newCollision &&
                (newCollision.type === 'ground' ||
                 otherObject.physics.state !== 'held')) {
                var horizDist = Math.sqrt(newCollision.contactNormal.x * newCollision.contactNormal.x +
                                          newCollision.contactNormal.z * newCollision.contactNormal.z);

                // If it's a horizontal collision with non-ground, check if the object can bestCollision
                // climbed
                if (Math.abs(newCollision.contactNormal.y) < horizDist &&
                    !obj.geometry.armBones.hasOwnProperty(newCollision.bodyPart)) {
                    obj.geometry.position.y += staticCollisionResolution;
                    updateObjectLocation(obj);
                    // Run collision detection again
                    newCollision = detectCollision(obj, otherObject);
                    // If it can't be climbed
                    if (newCollision) {
                        // Undo the climbing
                        obj.geometry.position.y -= staticCollisionResolution;
                    } else {
                        return;
                    }
                }

                displacement = newCollision.contactNormal.clone().
                        multiplyScalar(newCollision.penetration);

                // Add the displacement needed to resolve the collision
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
         */
        function resolveHeld(obj, otherObject) {
            // If the object is held, then we'll displace the
            // holding object, unless the other object is
            // the holding object
            var holder = robot,
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
                newCollision = detectCollision(obj, otherObject);
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
         */
        function resolveFalling(obj, otherObject) {
            var newCollision = detectCollision(obj, otherObject),
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
        if (obj.physics.type === 'dynamic' &&
            obj.geometry.collisions.length > 0) {
                obj.geometry.collisions.sort(function (a, b) {
                    return b.penetration - a.penetration;
                });
                obj.geometry.collisions.forEach(function (collision) {
                    var otherObject = getPhysicsObject(
                            collision.otherObject);

                    updateObjectLocation(obj);

                    switch (obj.physics.state) {
                        case 'controlled':
                            resolveControlled(obj, otherObject);
                            break;
                        case 'held':
                            resolveHeld(obj, otherObject);
                            break;
                        case 'falling':
                            resolveFalling(obj, otherObject);
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
        for (i = 0; i < physicsObjects.length; i += 1) {
            resolveCollision(dt, physicsObjects[i]);
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

    /** @exports vbot/physics */
    var physics = {
            addObject: addObject,
            detectCollisions: detectCollisions,
            updateObjects: updateObjects,
            changeObjectState: changeObjectState,
            resolveCollisions: resolveCollisions,
            toggleCollisionVolumes: toggleCollisionVolumes,
            setCollisionVolumeResolution: setCollisionVolumeResolution
        };

    return physics;
}());
