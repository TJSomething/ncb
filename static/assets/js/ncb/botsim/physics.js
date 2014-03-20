/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console, BOTSIM: true */

'use strict';

var BOTSIM = BOTSIM || {};

BOTSIM.physics = (function () {
    var staticObjects = [],
        dynamicObjects = [],
        // This is used to quickly address objects
        dynamicObjectIndices = {},
        staticObjectIndices = {},
        EPSILON = 0.000001;

    function initDynamicObject(obj) {
        var bWorld = (new THREE.Box3()).setFromObject(obj.geometry),
            Vec = THREE.Vector3,
            invMatrixWorld = (new THREE.Matrix4()).
                getInverse(obj.geometry.matrixWorld),
            // In local coordinates
            b = bWorld.clone().applyMatrix4(invMatrixWorld);

        // We're really going to be doing to do collisions with the
        //  bounding box
        obj.physics.collisionVertices =
           [new Vec( b.min.x, b.min.y, b.min.z ),
            new Vec( b.min.x, b.min.y, b.max.z ),
            new Vec( b.min.x, b.max.y, b.min.z ),
            new Vec( b.min.x, b.max.y, b.max.z ),
            new Vec( b.max.x, b.min.y, b.min.z ),
            new Vec( b.max.x, b.min.y, b.max.z ),
            new Vec( b.max.x, b.max.y, b.min.z ),
            new Vec( b.max.x, b.max.y, b.max.z )];

        obj.physics.boundingSphere =
            b.getBoundingSphere();

        Object.defineProperty(obj.physics, 'boundingSphereWorld',
            {
                get: function() {
                    return obj.physics.boundingSphere.clone().
                        applyMatrix4(obj.geometry.matrixWorld);
                }
            });

        Object.defineProperty(obj.physics, 'orientedBoundingBox',
            {
                get: (function () {
                        var c = obj.physics.boundingSphere.center,
                            // Half extents
                            e = bWorld.max.clone().
                                sub(bWorld.min).
                                divideScalar(2),
                            mat = obj.geometry.matrixWorld;

                        return function () {
                            return {
                                // Put center into global coordinates
                                c: c.clone().applyMatrix4(mat),
                                // Decompose matrix into bases
                                u: (function () {
                                    var x = new Vec(mat.elements[0],
                                                    mat.elements[1],
                                                    mat.elements[2]),
                                        y = new Vec(mat.elements[4],
                                                    mat.elements[5],
                                                    mat.elements[6]),
                                        z = new Vec(mat.elements[8],
                                                    mat.elements[9],
                                                    mat.elements[10]);

                                    return [x.normalize(),
                                            y.normalize(),
                                            z.normalize()];
                                }()),
                                e: e
                            };
                        };
                    }())
            });
        
        obj.physics.verticalVelocity = obj.physics.verticalVelocity || 0;

        dynamicObjects.push(obj);
        dynamicObjectIndices[obj.geometry.id] = dynamicObjects.length - 1;
    }

    function initStaticObject(obj) {
        var mat = obj.geometry.matrixWorld,
            Vec = THREE.Vector3,
            aabbWorld = (new THREE.Box3()).setFromObject(obj.geometry);

        obj.physics.faces = getGlobalFaces(obj);
        obj.physics.boundingSphereWorld =
            new THREE.Sphere().setFromPoints(
                obj.geometry.geometry.vertices);
        obj.physics.boundingSphereWorld.applyMatrix4(mat);

        obj.physics.orientedBoundingBox = {
            c: obj.physics.boundingSphereWorld.center,
            u: (function () {
                    var x = new Vec(mat.elements[0],
                                    mat.elements[1],
                                    mat.elements[2]),
                        y = new Vec(mat.elements[4],
                                    mat.elements[5],
                                    mat.elements[6]),
                        z = new Vec(mat.elements[8],
                                    mat.elements[9],
                                    mat.elements[10]);

                    return [x.normalize(),
                            y.normalize(),
                            z.normalize()];
                }()),
            e: aabbWorld.max.clone().
                         sub(aabbWorld.min).
                         divideScalar(2)
        };

        staticObjects.push(obj);
        staticObjectIndices[obj.geometry.id] = staticObjects.length - 1;
    }

    function getGlobalFaces(obj) {
        var vertices = obj.geometry.geometry.vertices,
            matrixWorld = obj.geometry.matrixWorld,
            // Transform our vertices into global coordinates
            globalVertices;

        if (vertices) {
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

    // Adds the object to the physics engine
    function addObject(obj, physicalProperties) {
        var physicsObj = {
            geometry: obj,
            physics: (function () {
                var p = physicalProperties || {};

                return {
                    position: obj.positionWorld(),
                    lastGoodPosition: obj.positionWorld(),
                    quaternion: obj.quaternion,
                    lastGoodQuaternion: obj.quaternion,
                    velocity: p.velocity || new THREE.Vector3(0, 0, 0),
                    state: p.state || 'rest',
                    // The object should be static if this flag is not passed,
                    //  or to the passed value
                    isStatic: p.isStatic === undefined || p.isStatic
                };
            }())
        };

        if (physicsObj.physics.isStatic) {
            initStaticObject(physicsObj);
        } else {
            initDynamicObject(physicsObj);
        }
    }

    function updateObject(dt, obj) {
        var position,
            quaternion;

        // Update position and velocity
        if (obj.physics.state === 'controlled' ||
            obj.physics.state === 'falling') {
            obj.physics.verticalVelocity -= 9.8 * dt;
            obj.geometry.position.y += dt * obj.physics.verticalVelocity;
        }
        obj.geometry.updateMatrixWorld();

        position = obj.geometry.positionWorld();
        quaternion = obj.geometry.quaternion;

        obj.physics.position = position;
        obj.physics.quaternion = quaternion;
    }

    function updateObjects(dt) {
        var i;

        // Update position and velocity
        for (i = 0; i < dynamicObjects.length; i += 1) {
            updateObject(dt, dynamicObjects[i]);
        }
    }

    function testOBBOBB(a, b) {
        var ra = 0,
            rb = 0,
            R = new THREE.Matrix3(),
            AbsR = new THREE.Matrix3(),
            i, j,
            t = new THREE.Vector3(),
            // Putting these into arrays for for-loops
            tArr,
            ae = [a.e.x, a.e.y, a.e.z],
            be = [b.e.x, b.e.y, b.e.z],
            axisDist,
            penetration = Infinity,
            normal = new THREE.Vector3();

        // Compute rotation matrix expressing b in a's coordinate frame
        for (i = 0; i < 3; i += 1) {
            for (j = 0; j < 3; j += 1) {
                R.elements[3 * j + i] = a.u[i].dot(b.u[j]);
            }
        }

        // Compute translation vector t
        t.subVectors(b.c, a.c);
        // Bring translation into a's coordinate frame
        t = new THREE.Vector3(
            t.dot(a.u[0]),
            t.dot(a.u[1]),
            t.dot(a.u[2]));
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
                return false;
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
                return false;
            } else if (-axisDist < penetration) {
                penetration = -axisDist;
                normal.set(R.elements[3 * i],
                           R.elements[3 * i + 1],
                           R.elements[3 * i + 2]);
                normal.multiplyScalar(-Math.sign(normal.dot(t)));
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
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(0,
                       -R.elements[3 * 0 + 2],
                       R.elements[3 * 0 + 1]);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A0 x B1
        ra = ae[1] * AbsR.elements[3 * 1 + 2] +
             ae[2] * AbsR.elements[3 * 1 + 1];
        rb = be[0] * AbsR.elements[3 * 2 + 0] +
             be[2] * AbsR.elements[3 * 0 + 0];
        axisDist = Math.abs(t[2] * R.elements[3 * 1 + 1] -
                            t[1] * R.elements[3 * 1 + 2]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(0,
                       -R.elements[3 * 1 + 2],
                       R.elements[3 * 1 + 1]);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A0 x B2
        ra = ae[1] * AbsR.elements[3 * 2 + 2] +
             ae[2] * AbsR.elements[3 * 2 + 1];
        rb = be[0] * AbsR.elements[3 * 1 + 0] +
             be[1] * AbsR.elements[3 * 0 + 0];
        axisDist = Math.abs(t[2] * R.elements[3 * 2 + 1] -
                            t[1] * R.elements[3 * 2 + 2]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(0,
                       -R.elements[3 * 2 + 2],
                       R.elements[3 * 2 + 1]);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A1 x B0
        ra = ae[0] * AbsR.elements[3 * 0 + 2] +
             ae[2] * AbsR.elements[3 * 0 + 0];
        rb = be[1] * AbsR.elements[3 * 2 + 1] +
             be[2] * AbsR.elements[3 * 1 + 1];
        axisDist = Math.abs(t[0] * R.elements[3 * 0 + 2] -
                            t[2] * R.elements[3 * 0 + 0]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(-R.elements[3 * 0 + 2],
                       0,
                       R.elements[3 * 0 + 0]);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A1 x B1
        ra = ae[0] * AbsR.elements[3 * 1 + 2] +
             ae[2] * AbsR.elements[3 * 1 + 0];
        rb = be[0] * AbsR.elements[3 * 2 + 1] +
             be[2] * AbsR.elements[3 * 0 + 1];
        axisDist = Math.abs(t[0] * R.elements[3 * 1 + 2] +
                            t[2] * R.elements[3 * 1 + 0]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(R.elements[3 * 1 + 2],
                       0,
                       R.elements[3 * 1 + 0]);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A1 x B2
        ra = ae[0] * AbsR.elements[3 * 2 + 2] +
             ae[2] * AbsR.elements[3 * 2 + 0];
        rb = be[0] * AbsR.elements[3 * 1 + 1] +
             be[1] * AbsR.elements[3 * 0 + 1];
        axisDist = Math.abs(t[0] * R.elements[3 * 2 + 2] +
                            t[2] * R.elements[3 * 2 + 0]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(R.elements[3 * 2 + 2],
                       0,
                       R.elements[3 * 2 + 0]);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A2 x B0
        ra = ae[0] * AbsR.elements[3 * 0 + 1] +
             ae[1] * AbsR.elements[3 * 0 + 0];
        rb = be[1] * AbsR.elements[3 * 2 + 2] +
             be[2] * AbsR.elements[3 * 1 + 2];
        axisDist = Math.abs(t[1] * R.elements[3 * 0 + 0] +
                            t[0] * R.elements[3 * 0 + 1]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(R.elements[3 * 0 + 1],
                       R.elements[3 * 0 + 0],
                       0);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A2 x B1
        ra = ae[0] * AbsR.elements[3 * 1 + 1] +
             ae[1] * AbsR.elements[3 * 1 + 0];
        rb = be[0] * AbsR.elements[3 * 2 + 2] +
             be[2] * AbsR.elements[3 * 0 + 2];
        axisDist = Math.abs(t[1] * R.elements[3 * 1 + 0] +
                            t[0] * R.elements[3 * 1 + 1]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(R.elements[3 * 1 + 1],
                       R.elements[3 * 1 + 0],
                       0);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Test axis L = A2 x B2
        ra = ae[0] * AbsR.elements[3 * 2 + 1] +
             ae[1] * AbsR.elements[3 * 2 + 0];
        rb = be[0] * AbsR.elements[3 * 1 + 2] +
             be[1] * AbsR.elements[3 * 0 + 2];
        axisDist = Math.abs(t[1] * R.elements[3 * 2 + 0] +
                            t[0] * R.elements[3 * 2 + 1]) - (ra + rb);
        if (axisDist > 0) {
            return false;
        } else if (-axisDist < penetration) {
            penetration = -axisDist;
            normal.set(R.elements[3 * 2 + 1],
                       R.elements[3 * 2 + 0],
                       0);
            normal.multiplyScalar(-Math.sign(normal.dot(t)));
        }

        // Rotate our normal into the first object's rotation
        for (i = 0; i < 3; i += 1) {
            for (j = 0; j < 3; j += 1) {
                R.elements[3 * i + j] = a.u[i].getComponent(j);
            }
        }
        normal.applyMatrix3(R);

        // Since no separating axis is found, the OBBs must be intersecting
        return {
            penetration: penetration,
            contactNormal: normal
        };
    }

    // Takes a dynamic physics object and checks if it's colliding
    //  with another dynamic physics object
    function detectDynamicCollision(obj1, obj2) {
        var collision;
        // Prevent self testing
        if (obj1 !== obj2) {
            // Oriented bounding box check
            collision = testOBBOBB(obj1.physics.orientedBoundingBox,
                    obj2.physics.orientedBoundingBox);
            if (collision) {
                collision.otherObject = obj2.geometry;
                collision.isEnvironment = false;
                return collision;
            }
        }

        return false;
    }

    // Detects a collision between a dynamic object and
    // a static object
    function detectSingleStaticCollision (obj1, obj2) {
        var collision;
        // Prevent self testing
        if (obj1 !== obj2) {
            // Oriented bounding box check
            collision = testOBBOBB(obj1.physics.orientedBoundingBox,
                    obj2.physics.orientedBoundingBox);
            if (collision) {
                collision.otherObject = obj2.geometry;
                collision.isEnvironment = true;
                return collision;
            }
        }

        return false;
    }

    // Takes a dynamic physics object and finds if it's colliding with
    //  any static objects
    function detectStaticCollision(obj) {
        var collision,
            // Check if any face of any object intersects with
            //  this object's bounding box
            totalCollisions =
                staticObjects.reduce(function (collisions, obj2) {
                    collision = detectSingleStaticCollision(obj, obj2);
                    if (collision) {
                        collisions.push(collision);
                    }

                    return collisions;
                }, []);

        if (totalCollisions.length > 0) {
            return totalCollisions;
        } else {
            return false;
        }
    }

    function getHoldingObject(obj) {
        var ancestor;

        if (obj.physics.state === 'held') {
            ancestor = obj.geometry;
            while (!(getPhysicsObject(ancestor) &&
                getPhysicsObject(ancestor).physics.state === 'controlled')) {
                ancestor = ancestor.parent;
            }
            ancestor = getPhysicsObject(ancestor);
            return ancestor;
        }
    }

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
                collisions = detectStaticCollision(dynamicObjects[i]);
                if (collisions) {
                    dynamicObjects[i].geometry.collisions =
                        dynamicObjects[i].geometry.collisions.concat(collisions);
                }
            }

            // Test against dynamic objects
            for (j = 0; j < dynamicObjects.length; j += 1) {
                collision = detectDynamicCollision(dynamicObjects[i],
                                           dynamicObjects[j]);
                if (collision) {
                    dynamicObjects[i].geometry.collisions.push(collision);
                }
            }
        }

        // If the current position has no collisions, note the current
        //  position
        for (i = 0; i < dynamicObjects.length; i += 1) {
            if (dynamicObjects[i].geometry.collisions.length === 0) {
                dynamicObjects[i].physics.lastGoodPosition =
                    dynamicObjects[i].geometry.position.clone();
                dynamicObjects[i].physics.lastGoodQuaternion =
                    dynamicObjects[i].geometry.quaternion.clone();
            }
        }
    }

    function changeObjectState(obj, newState) {
        getPhysicsObject(obj, false).physics.state = newState;
    }

    function getPhysicsObject(obj, isStatic) {
        if (!isStatic &&
            dynamicObjectIndices.hasOwnProperty(obj.id)) {
            return dynamicObjects[dynamicObjectIndices[obj.id]];
        }
        if (isStatic &&
            staticObjectIndices.hasOwnProperty(obj.id)) {
            return staticObjects[staticObjectIndices[obj.id]];
        }
    }

    function detectCollision(obj1, obj2, isStatic) {
        if (isStatic) {
            return detectSingleStaticCollision(obj1, obj2);
        } else {
            return detectDynamicCollision(obj1, obj2);
        }
    }

    function resolveCollision(dt, obj) {
        function resolveControlled(obj, otherObject, isStatic) {
            var newCollision = detectCollision(obj, otherObject, isStatic),
                displacement;

            // Ignore collisions with held objects
            if (newCollision &&
                otherObject.physics.state !== 'held') {
                displacement = newCollision.contactNormal.clone().
                        multiplyScalar(newCollision.penetration);
                obj.geometry.position.add(displacement);
                // If we get pushed up, neutralize speed
                if (displacement.y > 0) {
                    obj.physics.verticalVelocity = 0;
                }
                obj.geometry.updateMatrixWorld();
            }
        }

        function resolveHeld(obj, otherObject, isStatic) {
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
                yDisplacement;

            if (otherObject !== holder) {
                newCollision = detectCollision(obj, otherObject, isStatic);
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
                        multiplyScalar(yDisplacement.dot(armVector));

                    // All the displacement that's not along the arm
                    //  is perpendicular
                    displacementPerpendicularToArm = yDisplacement.clone().
                        sub(displacementAlongArm);

                    angularDisplacement = displacementPerpendicularToArm.
                        clone().
                        cross(armVector).
                        multiplyScalar(armLength);

                    shoulder.rotateOnAxis(
                        angularDisplacement.clone().normalize(),
                        angularDisplacement.length());

                    holder.geometry.position.add(displacement);

                    // If we get pushed up, neutralize speed
                    if (displacement.y > 0) {
                        holder.physics.verticalVelocity = 0;
                    }
                    holder.geometry.updateMatrixWorld();
                }
            }
        }

        function resolveFalling(obj, otherObject, isStatic) {
            var newCollision = detectCollision(obj, otherObject, isStatic),
                displacement;

            if (newCollision) {
                displacement = newCollision.contactNormal.clone().
                        multiplyScalar(newCollision.penetration);
                obj.geometry.position.add(displacement);
                obj.geometry.updateMatrixWorld();
                // If we hit the ground, then stop falling
                if (displacement.y > 0 &&
                    newCollision.isEnvironment) {
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
                var isStatic = collision.isEnvironment,
                    otherObject = getPhysicsObject(
                        collision.otherObject,
                        isStatic);

                obj.geometry.updateMatrixWorld();

                switch (obj.physics.state) {
                    case 'controlled':
                        resolveControlled(obj, otherObject, isStatic);
                        break;
                    case 'held':
                        resolveHeld(obj, otherObject, isStatic);
                        break;
                    case 'falling':
                        resolveFalling(obj, otherObject, isStatic);
                        break;
                }
            });
        }
    }

    function resolveCollisions(dt) {
        var i;
        for (i = 0; i < dynamicObjects.length; i += 1) {
            resolveCollision(dt, dynamicObjects[i]);
        }
    }

    return {
            addObject: addObject,
            detectCollisions: detectCollisions,
            updateObjects: updateObjects,
            changeObjectState: changeObjectState,
            resolveCollisions: resolveCollisions
        };
}());
