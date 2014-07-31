/* jslint browser: true */
/* global THREE, $:false, _:false, Stats, console, VBOT: true */

(function (global) {
    'use strict';

    global.VBOT = global.VBOT || {};
    var VBOT = global.VBOT;

    VBOT.physics = (function () {
        var staticObjects = [],
            dynamicObjects = [],
            // This is used to quickly address objects
            dynamicObjectIndices = {},
            staticObjectIndices = {},
            EPSILON = 0.000001,
            staticCollisionResolution = 0.1,
            collisionVolumeObjects = new THREE.Object3D(),
            groundElevation = Infinity;
            
        /** Creates an OBB
         *
         * @param center THREE.Vector3 the center
         * @param bases [THREE.Vector3] an array of the bases, which determine the
         *                              rotation of the OBB
         * @param halfExtents [number] an array of the half-lengths of each dimension
         *                             of the OBB
         * @param parent THREE.Object3D the parent of this OBB
         */
        function makeOBB(center, bases, halfExtents, parent) {
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


            function matrix4Equals(mat1, mat2) {
                for (var i = 0; i < 16; i += 1) {
                    if (mat1.elements[i] !== mat2.elements[i]) {
                        return false;
                    }
                }
                return true;
            }

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

            Object.defineProperty(obb, 'c',
                {
                    get: function() {
                             update();
                             return c;
                         }
                });
            Object.defineProperty(obb, 'e',
                {
                    get: function() {
                             return scaledExtents;
                         }
                });
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

            return obb;
        }

        function OBBHelper(obb) {
            var boundingBox = new THREE.Mesh(
                    new THREE.BoxGeometry(1, 1, 1),
                    new THREE.MeshBasicMaterial({
                        color: 0x888888,
                        wireframe: true
                    })),
                originalUpdate = boundingBox.updateMatrixWorld.bind(boundingBox);

            function updateOBB() {
                // scale
                var sc = obb.e;
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

                boundingBox.matrix.copy(mat);
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

        function initDynamicObject(obj) {
            var bWorld = (new THREE.Box3()).setFromObject(obj.geometry),
                Vec = THREE.Vector3,
                invMatrixWorld = (new THREE.Matrix4()).
                    getInverse(obj.geometry.matrixWorld),
                // In local coordinates
                b = bWorld.clone().applyMatrix4(invMatrixWorld);
                
            var boundingSphere =
                b.getBoundingSphere();

            var calcBoundingSphereWorld = function() {
                        return obj.physics.boundingSphere.clone().
                            applyMatrix4(obj.geometry.matrixWorld);
                    };
                    
            obj.physics.obbs =
                [function () {
                    var c = boundingSphere.center,
                        tmpC = new Vec(),
                        // Half extents
                        e = bWorld.max.clone().
                            sub(bWorld.min).
                            divideScalar(2),
                        u = [new Vec(1, 0, 0),
                             new Vec(0, 1, 0),
                             new Vec(0, 0, 1)],
                        mat = obj.geometry.matrixWorld,
                        len = 0;

                    return makeOBB(c, u, e.toArray(), obj.geometry);
                }()];
                
            obj.physics.verticalVelocity = obj.physics.verticalVelocity || 0;

            dynamicObjects.push(obj);
            dynamicObjectIndices[obj.geometry.id] = dynamicObjects.length - 1;
        }

        function initStaticObject(obj) {
            var mat = obj.geometry.matrixWorld,
                Vec = THREE.Vector3,
                aabbWorld = (new THREE.Box3()).setFromObject(obj.geometry);

            obj.physics.faces = getGlobalFaces(obj);

            obj.physics.obbs = buildObjectOBBs(obj, staticCollisionResolution);

            staticObjects.push(obj);
            staticObjectIndices[obj.geometry.id] = staticObjects.length - 1;

            updateGround(obj);
        }

        function updateGround(obj) {
            var objAABB = new THREE.Box3().setFromObject(obj.geometry);

            if (groundElevation > objAABB.max.y) {
                groundElevation = objAABB.max.y;
            }
        }

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

        function dot(v, u) {
            return u.x * v.x + u.y * v.y + u.z * v.z;
        }

        function testOBBOBB(a, b) {
            // Compute the OBBs at this time
            a = {c: a.c, u: a.u, e: a.e};
            b = {c: b.c, u: b.u, e: b.e};
            
            var ra = 0,
                rb = 0,
                R = {elements: new Float32Array(9)},
                AbsR = {elements: new Float32Array(9)},
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
                    R.elements[3 * j + i] = dot(a.u[i], b.u[j]);
                }
            }

            // Compute translation vector t
            t.subVectors(b.c, a.c);
            // Bring translation into a's coordinate frame
            t = new THREE.Vector3(
                dot(t, a.u[0]),
                dot(t, a.u[1]),
                dot(t, a.u[2]));
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
            }
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

            // Finds all the faces that are in the box completely
            function findContainedFaces(box, faces) {
                return faces.filter(function (face) {
                    return box.containsPoint(face[0]) &&
                           box.containsPoint(face[1]) &&
                           box.containsPoint(face[2]);
                });
            }

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
        
        function detectSingleCollision(obj1, obj2) {
            var collision, i, j,
                bestCollision = {
                    penetration: -Infinity
                },
                obbs1 = obj1.physics.obbs,
                obbs2 = obj2.physics.obbs;
            
            // Prevent self testing
            if (obj1 !== obj2) {
                for (i = 0; i < obbs1.length; i += 1) {
                    for (j = 0; j < obbs2.length; j += 1) {
                        collision = testOBBOBB(obbs1[i], obbs2[j]);
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
                        collision = detectSingleCollision(obj, obj2);
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

        function detectGroundCollision(obj) {
            var collision = testOBBGround(obj.physics.obbs[0], groundElevation);
            if (collision) {
                collision.type = 'ground';
            }
            return collision;
        }

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
                    collision = detectSingleCollision(dynamicObjects[i],
                                               dynamicObjects[j]);
                    if (collision) {
                        dynamicObjects[i].geometry.collisions.push(collision);
                    }
                }

                // Test against the ground
                collision = detectGroundCollision(dynamicObjects[i]);
                if (collision) {
                    dynamicObjects[i].geometry.collisions.push(collision);
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
            getPhysicsObject(obj, 'dynamic').physics.state = newState;
        }

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

        function detectCollision(obj1, obj2, type) {
            if (type === 'static' || type === 'dynamic') {
                return detectSingleCollision(obj1, obj2);
            } else if (type === 'ground') {
                return detectGroundCollision(obj1);
            } else {
                throw type + ' is not an object type';
            }
        }

        function resolveCollision(dt, obj) {
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
                        obj.geometry.updateMatrixWorld();
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
                    obj.geometry.updateMatrixWorld();
                }
            }

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
                        holder.geometry.updateMatrixWorld();
                    }
                }
            }

            function resolveFalling(obj, otherObject, type) {
                var newCollision = detectCollision(obj, otherObject, type),
                    displacement;

                if (newCollision) {
                    displacement = newCollision.contactNormal.clone().
                            multiplyScalar(newCollision.penetration);
                    obj.geometry.position.add(displacement);
                    obj.geometry.updateMatrixWorld();
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

                    obj.geometry.updateMatrixWorld();

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

        function resolveCollisions(dt) {
            var i;
            for (i = 0; i < dynamicObjects.length; i += 1) {
                resolveCollision(dt, dynamicObjects[i]);
            }
        }

        function toggleCollisionVolumes() {
            if (collisionVolumeObjects.parent) {
                VBOT.scene.remove(collisionVolumeObjects);
            } else {
                VBOT.scene.add(collisionVolumeObjects);
            }
        }

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
})(this);