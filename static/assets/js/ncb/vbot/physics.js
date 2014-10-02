/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console, VBOT: true */

define(['three', 'numeric', 'underscore', 'vbot/collision'],
function (THREE, numeric, _, collision) {
    'use strict';

    /** @exports vbot/physics */

    return (function () {
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
            obj.physics.obbs.forEach(function (obb) {
                scene.add(
                    new collision.OBBHelper(obb));
            });

            obj.physics.boundingSphere = collision.calcBoundingSphereFromOBBs(obj.physics.obbs);

            obj.physics.updateBoundingSphere = function () {
                obj.physics.boundingSphere = collision.calcBoundingSphereFromOBBs(obj.physics.obbs);
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

            // Static objects don't move, so they don't need updating
            obj.physics.updateBoundingSphere = function () {};

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

            /**
             * Find all the faces that intersect the box.
             *
             * @param  {THREE.Box3} box
             * @param  {Array.<Array.<THREE.Vector3>>} faces
             * @return {Array.<Array.<THREE.Vector3>>} all the faces that intersect the box
             */
            function findIntersectingFaces(box, faces) {
                return faces.filter(function (face) {
                    return collision.OBB.fromBox3(box).testTri(face);
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
            return boxes.map(collision.OBB.fromBox3);
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
                    maxIndex = undefined;

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

        return {
                addObject: addObject,
                detectCollisions: detectCollisions,
                updateObjects: updateObjects,
                changeObjectState: changeObjectState,
                resolveCollisions: resolveCollisions,
                toggleCollisionVolumes: toggleCollisionVolumes,
                setCollisionVolumeResolution: setCollisionVolumeResolution
            };
    }());
});
