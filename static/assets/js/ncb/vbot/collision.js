define(['three', 'numeric', 'underscore'],
function (THREE, numeric, _) {
    'use strict';
    var EPSILON = 0.000001;

    /** @exports vbot/collision */
    var collision = {};

    /**
     * Creates an OBB
     *
     * @class
     * @param {THREE.Vector3} center      the center
     * @param {THREE.Vector3[]} bases     an array of the bases, which determine the
     *                                    rotation of the OBB
     * @param {number[]} halfExtents      an array of the half-lengths of each dimension
     *                                    of the OBB
     * @param {THREE.Object3D} parent     the parent of this OBB
     * @return {OBB} an oriented bounding box
     */
    collision.OBB = function (center, bases, halfExtents, parent) {
        var obb = Object.create(collision.OBB.prototype);
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
         * @instance
         * @memberof OBB
         */
        function update() {
            if (!matrix4Equals(parent.matrixWorld, parentMemo) ||
                c === undefined) {
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
         * @instance collision.OBB
         * @memberof OBB
         * @return {OBB} a dumb copy of this
         *                                            OBB
         */
        function clone() {
            update();
            return Object.create(collision.OBB.prototype, {
                c: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: this.c
                },
                e: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: this.e
                },
                u: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: this.u
                },
                name: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: this.name
                },
                clone: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: function () { return this; }
                },
                update: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: function () {}
                }
            });
        }

        /**
         * the center of the OBB
         *
         * @instance
         * @memberof OBB
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
         * @instance
         * @memberof OBB
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
         * @instance
         * @memberof OBB
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
        /**
         * the name of the OBB's parent
         *
         * @instance
         * @memberof OBB
         * @var {string} name
         */
        Object.defineProperty(obb, 'name',
            {
                value: parent.name
            }
        );

        return obb;
    };

    collision.OBB.prototype = {
        /**
         * Calculates a matrix that transforms a unit cube centered at (0,0,0)
         * to the given OBB
         *
         * @return {THREE.Matrix4}                the matrix to get the OBB
         */
        makeMatrix: function () {
            var obb = this;
            // scale
            var sc = obb.e.toArray();
            // bases
            var E = [[obb.u[0].x, obb.u[1].x, obb.u[2].x],
                     [obb.u[0].y, obb.u[1].y, obb.u[2].y],
                     [obb.u[0].z, obb.u[1].z, obb.u[2].z]];
            // center
            var p = obb.c;

            return new THREE.Matrix4(sc[0] * E[0][0], sc[1] * E[0][1], sc[2] * E[0][2], p.x,
                                        sc[0] * E[1][0], sc[1] * E[1][1], sc[2] * E[1][2], p.y,
                                        sc[0] * E[2][0], sc[1] * E[2][1], sc[2] * E[2][2], p.z,
                                        0,               0,               0,               1);
        },
        /**
         * Calculates the vertices of an OBB
         *
         * @return {THREE.Vector3[]}         an array of the OBB's vertices
         */
        calcVertices: function () {
            var obb = this,
                mat = obb.makeMatrix(),
                Vec = function (x,y,z) { return new THREE.Vector3(x,y,z); },
                startVertices = [Vec(-1, -1, -1),
                    Vec(-1, -1, +1),
                    Vec(-1, +1, -1),
                    Vec(-1, +1, +1),
                    Vec(+1, -1, -1),
                    Vec(+1, -1, +1),
                    Vec(+1, +1, -1),
                    Vec(+1, +1, +1)];
            return startVertices.map(function(vert) {
                return vert.applyMatrix4(mat);
            });
        },
        /**
         * Finds if two OBB's are colliding.
         *
         * @param  {OBB} b
         * @return {module:vbot/collision~Collision?} if the boxes are collding, then the
         *                             collision is returned. If they're not,
         *                             then false is returned.
         */
        testOBB: function (b) {
            // Put all of the OBB properties in nice locations
            var a = this,
                ac = a.c, bc = b.c,
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
        },
        /**
         * Tests for OBB triangle intersection. Not useful for collision
         * resolution.
         *
         * @param  {THREE.Vector3[]} tri    the triangle to test against,
         *                                  as an array of vectors in
         *                                  counterclockwise order around
         *                                  the triangle
         * @return {module:vbot/collision~Collision} the collision if they're colliding,
         *                                  or null otherwise
         */
        testTri: function (tri) {
            var obb = this,
                i,
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
                return !(min > rad || max < -rad);

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
                return !(min > rad || max < -rad);

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
                return !(min > rad || max < -rad);

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
                return !(min > rad || max < -rad);

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
                return !(min > rad || max < -rad);

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
                return !(min > rad || max < -rad);

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
        },
        /**
         * Tests if an OBB is penetrating the ground
         *
         * @param  {number} groundElevation   height of the ground
         * @return {(module:vbot/collision~Collision|boolean)}   the collision if they're colliding
         *                                    or false otherwise
         */
        testGround: function (groundElevation) {
            var obb = this,
                signX, signY, signZ,
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

    };

    /**
     * Converts a THREE.Box3 into an OBB.
     *
     * Note a THREE.Box3 is an axis-aligned bounding box.
     *
     * @memberof module:vbot/physics~
     * @param  {THREE.Box3} box
     * @return {OBB}
     */
    collision.OBB.fromBox3 =
        function (box, parent) {
            if (parent instanceof THREE.Object3D) {
                return new collision.OBB(box.center(),
                    [new THREE.Vector3(1, 0, 0),
                     new THREE.Vector3(0, 1, 0),
                     new THREE.Vector3(0, 0, 1)],
                    box.size().divideScalar(2).toArray(),
                    parent);
            }
            
            return Object.create(collision.OBB.prototype, {
                c: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: box.center()
                },
                e: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: box.size().divideScalar(2)
                },
                u: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: [new THREE.Vector3(1, 0, 0),
                            new THREE.Vector3(0, 1, 0),
                            new THREE.Vector3(0, 0, 1)]
                },
                clone: {
                    writable: false,
                    configurable: false,
                    enumerable: false,
                    value: function () { return this; }
                }
            });
        };

    /**
     *  Fits an OBB to a bunch of bytes
     *
     * @param {Array.<THREE.Vector3>} vertices global vertices for the object
     * @param {THREE.Object3D} parent the parent object for this OBB
     * @returns {OBB?} the OBB if we have more than two vertices
     */
    collision.OBB.fromPoints = function (vertices, parent) {
        function projectPoints(basis, points) {
            return points.map(function (pt) {
                return numeric.dot(basis, pt);
            });
        }
        
        // Convert the vertices from THREE.Vector3s to arrays
        vertices = vertices.map(function (vertex) {
            return parent.worldToLocal(vertex.clone()).toArray();
        });
        var totalWeight = vertices.length,
            centroid = numeric.div(vertices.reduce(function (x,y) {
                    return numeric.add(x,y);
                }), totalWeight),
            covariance = vertices.map(function (x) {
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
            var det = numeric.det(E);

            for (var i = 0; i < 3; i += 1) {
                var basis = [E[0][i], E[1][i], E[2][i]];
                var projectedPoints = projectPoints(basis, vertices);
                var max = Math.max.apply(Math, projectedPoints);
                var min = Math.min.apply(Math, projectedPoints);
                var middle = (max + min) / 2;
                boxBases.push(new THREE.Vector3(E[0][i], E[1][i], E[2][i]));
                var basisSign = Math.sign(
                    _.max(basis, Math.abs)
                );
                center = numeric.add(center, numeric.mul(basis, middle));
                
                sc[i] = (max - min) / 2;
                
            }

            var p = new THREE.Vector3().fromArray(center);

            return new collision.OBB(p, boxBases, sc, parent);
        } else {
            return null;
        }
    };



    /**
     * Creates a three.js object that corresponds to an OBB
     *
     * @class
     * @param {OBB} obb the OBB to be shown with the
     *                                       object
     * @return {THREE.Mesh}                  the three.js object indicating
     *                                       the OBB
     */
    collision.OBBHelper = function (obb) {
        var boundingBox = new THREE.Mesh(
                new THREE.BoxGeometry(2, 2, 2),
                new THREE.MeshBasicMaterial({
                    color: 0x888888,
                    wireframe: true
                })),
            originalUpdate = boundingBox.updateMatrixWorld.bind(boundingBox);

        function updateOBB() {
            boundingBox.matrix.copy(obb.makeMatrix());
        }

        // Make it so that updating the matrixWorld updates the OBB
        boundingBox.updateMatrixWorld = function(force) {
            updateOBB();
            originalUpdate(force);
            boundingBox.matrixWorldNeedsUpdate = true;
        };

        // We're not going to use position, quaternion, and scale
        boundingBox.matrixAutoUpdate = false;

        updateOBB();
        originalUpdate(true);

        return boundingBox;
    };


    /**
     * Calculates the vertices of multiple OBBs
     * @param  {OBB[]} obbs the OBBs to calculate the
     *                                      vertices of
     * @return {THREE.Vector3[]}            an array of the vertices of all
     *                                      the OBBs
     */
    collision.calcOBBsVertices = function (obbs) {
        var vertsPerBox = obbs.map(
            function (obb) {
                return obb.calcVertices();
            }
        );
        return [].concat.apply([], vertsPerBox);
    };

    /**
     * Calculates the bounding sphere of multiple OBBs
     * @param  {OBB[]} obbs
     * @return {THREE.Sphere}               the bounding sphere of the given
     *                                      OBBs
     */
    collision.calcBoundingSphereFromOBBs = function (obbs) {
        var verts = collision.calcOBBsVertices(obbs);
        return new THREE.Sphere().setFromPoints(verts);
    };

    /**
     * Represents a collision.
     *
     * @typedef module:vbot/collision~Collision
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
     * Performs a dot product of two vectors.
     *
     * @param  {THREE.Vector3} v
     * @param  {THREE.Vector3} u
     * @return {number}
     */
    function dot(v, u) {
        return u.x * v.x + u.y * v.y + u.z * v.z;
    }

    return collision;
});
