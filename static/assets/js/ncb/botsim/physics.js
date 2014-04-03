/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console, BOTSIM: true */

'use strict';

var BOTSIM = BOTSIM || {};

BOTSIM.physics = (function () {
    var staticObjects = [],
        staticOBBs = [],
        dynamicObjects = [],
        // This is used to quickly address objects
        dynamicObjectIndices = {},
        staticObjectIndices = {},
        EPSILON = 0.000001,
        staticCollisionResolution = 1,
        collisionVolumeObjects = new THREE.Object3D();

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

        obj.physics.obbs = buildObjectOBBs(obj, staticCollisionResolution);

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

    /* An OBB triangle intersection test. Not useful for collision resolution.
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
            return false;
        }

        fex = Math.abs(e[1].x);
        fey = Math.abs(e[1].y);
        fez = Math.abs(e[1].z);
        if (!axisTestX01(e[1].z, e[1].y, fez, fey) ||
            !axisTestY02(e[1].z, e[1].x, fez, fex) ||
            !axisTestZ0(e[1].y, e[1].x, fey, fex)) {
            return false;
        }

        fex = Math.abs(e[2].x);
        fey = Math.abs(e[2].y);
        fez = Math.abs(e[2].z);
        if (!axisTestX2(e[2].z, e[2].y, fez, fey) ||
            !axisTestY1(e[2].z, e[2].x, fez, fex) ||
            !axisTestZ12(e[2].y, e[2].x, fey, fex)) {
            return false;
        }

        // Test for the AABB of the triangle
        findMinMax(v[0].x, v[1].x, v[2].x);
        if (min > obb.e.x ||
            max < -obb.e.x) {
            return false;
        }

        findMinMax(v[0].y, v[1].y, v[2].y);
        if (min > obb.e.y ||
            max < -obb.e.y) {
            return false;
        }

        findMinMax(v[0].z, v[1].z, v[2].z);
        if (min > obb.e.z ||
            max < -obb.e.z) {
            return false;
        }

        // Test if the box penetrates the triangle's plane
        normal.crossVectors(e[0], e[1]);
        if (!planeBoxOverlap(normal, v[0], obb.e)) {
            return false;
        }

        return {
            contactNormal: contactNormal,
            penetration: penetration
        };
    }

    function convertBox3ToOBB(box) {
        return {
            c: box.center(),
            e: box.size().divideScalar(2),
            u: [new THREE.Vector3(1, 0, 0),
                new THREE.Vector3(0, 1, 0),
                new THREE.Vector3(0, 0, 1)]
        };
    }

    /** Makes an array of OBBs, given a geometry object and a scale

        scale specifies stuff
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

        function findIntersectingFaces(box, faces) {
            return faces.filter(function (face) {
                return testOBBTri(convertBox3ToOBB(box), face);
            });
        }

        function median(ary) {
            var mid;
            if (ary.length === 0) {
                return null;
            }
            ary.sort(function (a,b){return a - b;});
            mid = Math.floor(ary.length / 2);
            if ((ary.length % 2) == 1) { // length is odd
                return ary[mid];
            } else {
                return (ary[mid - 1] + ary[mid]) / 2;
            }
        }

        function splitBox(box, faces) {
            var extent = box.max.clone().sub(box.min),
                splitAxis,
                splitAxisLength = Infinity,
                currentAxis,
                points = [],
                maxPoint,
                minPoint,
                midpoint,
                leftBox,
                leftFaces,
                rightBox,
                rightFaces;

            // Find the largest axis to use as the splitting axis
            for (currentAxis = 0; currentAxis < 3; currentAxis += 1) {
                if (extent.getComponent(currentAxis) < splitAxisLength &&
                    extent.getComponent(currentAxis) > scale) {
                    splitAxis = currentAxis;
                    splitAxisLength = extent.getComponent(currentAxis);
                }
            }

            // If every dimension is smaller than the scale, save the
            // box and terminate
            if (splitAxis === undefined) {
                boxes.push(box);
                return;
            }

            // Note the maximum and minimum points on the splitting axis
            maxPoint = box.max.getComponent(splitAxis);
            minPoint = box.min.getComponent(splitAxis);
            points.push(minPoint);
            points.push(maxPoint);

            // Project the points of all faces onto the splitting axis
            faces.forEach(function (face) {
                var i, point;

                for (i = 0; i < 3; i += 1) {
                    point = face[i].getComponent(splitAxis);
                    if (point < maxPoint &&
                        point > minPoint) {
                        points.push(point);
                    }
                }
            });
            
            // Find the median point
            midpoint = (maxPoint + minPoint)/2;

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
            splitBox(leftBox, leftFaces);
            splitBox(rightBox, rightFaces);
        }

        // Let's only try subdividing if there's thickness. This test
        //  also excludes the ground nicely
        if (xScale !== 0 &&
            yScale !== 0 &&
            zScale !== 0) {
            // Build all candidate boxes recursively
            splitBox(aabb, obj.physics.faces);
        } else {
            boxes.push(aabb);
        }

        
        // Help me with visualization
        boxes.forEach(function (box) {
            var geom = new THREE.CubeGeometry(1.01, 1.01, 1.01),
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
        return boxes.map(convertBox3ToOBB);
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
        var collision, bestCollision = {
                penetration: -Infinity
            };
        // Prevent self testing
        if (obj1 !== obj2) {
            obj2.physics.obbs.forEach(function (obb) {
                // Oriented bounding box check
                collision = testOBBOBB(obj1.physics.orientedBoundingBox,
                        obb);
                if (collision) {
                    collision.otherObject = obj2.geometry;
                    collision.isEnvironment = true;
                    if (collision.penetration > bestCollision.penetration) {
                        bestCollision = collision;
                    }
                }
            });

            if (bestCollision.penetration > 0) {
                return bestCollision;
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
                        multiplyScalar(-yDisplacement.dot(armVector));

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

    function toggleCollisionVolumes() {
        if (collisionVolumeObjects.parent) {
            BOTSIM.scene.remove(collisionVolumeObjects);
        } else {
            BOTSIM.scene.add(collisionVolumeObjects);
        }
    }

    return {
            addObject: addObject,
            detectCollisions: detectCollisions,
            updateObjects: updateObjects,
            changeObjectState: changeObjectState,
            resolveCollisions: resolveCollisions,
            toggleCollisionVolumes: toggleCollisionVolumes
        };
}());
