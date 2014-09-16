#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <markdowncell>

# First, we're going to import some modules, add a few utility functions, and specify some symbols and variables we need.

# <codecell>

# Import all sorts of things
from sympy.interactive.printing import init_printing
init_printing(use_unicode=False, wrap_line=False, no_global=True)
from sympy.matrices import *
from sympy import var, sin, cos, symbol, MatrixSymbol, Matrix, eye, Symbol, simplify, cse, solve, jscode, Function, acos, asin, sqrt, Wild
from functools import reduce
from jinja2 import Template
import re

# <codecell>

# Replace references to matrix elements with JS references
def subs_matrix_elements(expr_str, matrix_name):
    for row in range(4):
        for col in range(4):
            expr_str = expr_str.replace("%s_%d,%d" % (matrix_name, row, col),
                                        "%s.elements[%d]" % (matrix_name,
                                                             4*col + row,))
    return expr_str

# Given an expression, find the parameters that will be needed
def extract_parameters(expr):
    atoms = expr.atoms(Symbol)
    matrices = set([])
    scalars = set([])
    for atom in atoms:
        matrix_name = re.findall("([a-zA-Z]+)_\d,\d", atom.name)
        if len(matrix_name) > 0:
            matrices.add(matrix_name[0])
        else:
            scalars.add(atom.name)
    
    # Put in a nice order
    matrices = list(matrices)
    matrices.sort()
    scalars = list(scalars)
    scalars.sort()
    return matrices+scalars

# Dummy matrix for solving
def make_dummy_matrix(name):
    elements = []
    for i in range(4):
        for j in range(4):
            elements.append(Symbol("%s_%d,%d" % (name, i, j)))

    return Matrix(4,4, elements)

# Formats a 4x4 Sympy matrix as a Javascript matrix
def format_js_mat(mat):
    js_exprs = [jsify_expr(x) for x in mat]
    return "new THREE.Matrix4(%s)" % (", ".join(js_exprs),)

# Makes massive redundant expression easier to read for debugging
def format_big_expr(expr):
    terms, vec = cse(expr)
    return (Matrix(terms), vec)

def jsify_expr(expr):
    clamp = Function("clamp")
    bottom_clamp = Function("bottom")
    x = Wild("x")
    # Prevent NaNs on inverse trig functions
    expr = expr.replace(asin, lambda x: asin(clamp(x, -1, 1)))
    expr = expr.replace(acos, lambda x: acos(clamp(x, -1, 1)))
    # Prevent NaNs on sqrts
    expr = expr.replace(sqrt(x + 1),
                        sqrt(bottom_clamp(x + 1, 0)))
    js_expr = jscode(expr)
    # Convert all matrix references for compatibility with
    # three.js
    atoms = expr.atoms(Symbol)
    matrices = set([])
    for atom in atoms:
        matrix_name = re.findall("([a-zA-Z]+)_\d,\d", atom.name)
        if len(matrix_name) > 0:
            matrices.add(matrix_name[0])
    for matrix in matrices:
        js_expr = subs_matrix_elements(js_expr, matrix)
    # Convert the clamp function for three.js
    js_expr = js_expr.replace("clamp", "THREE.Math.clamp")
    js_expr = js_expr.replace("bottom", "THREE.Math.clampBottom")
    return js_expr

# Utilities to format JS
def jsify_terms(terms):
    lines = []
    for var_name, term in terms:
        term_js = jsify_expr(term)
        lines.append("var %s = %s;" % (jscode(var_name), term_js))
    return "\n".join(lines)

def jsify_list(list_expr):
    return "[%s]" % ", ".join([jsify_expr(x) for x in list_expr])

# <codecell>

# Variable setup
for i in range(1,8):
    var("theta_" + str(i))

for i in range(1,8):
    var("s_%d" % i)
    var("c_%d" % i)

# Shoulder location
var("shoulder_x")
var("shoulder_y")
var("shoulder_z")
# Arm bone length
var("l_4")
var("l_5")

# <markdowncell>

# We're making assumptions about the model geometry here. We assume that the x-axis is used to
# displace the elbow from the shoulder and the wrist from the elbow. We also assume that, in
# the elbow's coordinate system the arm is bending in the XZ-plane and the wrist twists in
# the YZ-plane.

# <codecell>

R = [Matrix([[1, 0, 0, shoulder_x],
             [0, 1, 0, shoulder_y],
             [0, 0, 1, shoulder_z],
             [0, 0, 0, 1]])]
R.append(Matrix([[   1,    0,    0, 0],
                 [   0,  c_1, -s_1, 0],
                 [   0,  s_1,  c_1, 0],
                 [   0,    0,    0, 1]]))
R.append(Matrix([[ c_2, -s_2,    0, 0],
                 [ s_2,  c_2,    0, 0],
                 [   0,    0,    1, 0],
                 [   0,    0,    0, 1]]))
R.append(Matrix([[ c_3,    0, -s_3, 0],
                 [   0,    1,    0, 0],
                 [ s_3,    0,  c_3, 0],
                 [  0,     0,    0, 1]]))
R.append(Matrix([[ c_4,    0, -s_4, l_4],
                 [   0,    1,    0, 0],
                 [ s_4,    0,  c_4, 0],
                 [   0,    0,    0, 1]]))
R.append(Matrix([[   1,    0,    0, 0],
                 [   0,  c_5, -s_5, 0],
                 [   0,  s_5,  c_5, 0],
                 [   0,    0,    0, 1]]))
R.append(Matrix([[ c_6, -s_6,    0, l_5],
                 [ s_6,  c_6,    0, 0],
                 [   0,    0,    1, 0],
                 [   0,    0,    0, 1]]))
R.append(Matrix([[ c_7,    0, -s_7, 0],
                 [   0,    1,    0, 0],
                 [ s_7,    0,  c_7, 0],
                 [   0,    0,    0, 1]]))

z_0 = Matrix([0,0,1,0])
o_0 = Matrix([0,0,0,1])

# These are to represent a black box matrix for the joints
# to use in a solver
dummy_shoulder_mat = make_dummy_matrix("shoulder")
dummy_elbow_mat = make_dummy_matrix("elbow")
dummy_wrist_mat = make_dummy_matrix("wrist")
generic_mat4 = make_dummy_matrix("M")

# These are to specify a new matrix for the joints
shoulder_mat = R[0] * R[1] * R[2] * R[3]
elbow_mat = R[4] * R[5]
wrist_mat = R[6] * R[7]

def rotation_matrix(i):
    R_0_i = eye(4)
    for i in range(0, i+1):
        R_0_i = R_0_i * R[i]
    return R_0_i

R_0 = [rotation_matrix(i) for i in range(8)]
o_7 = R_0[7] * o_0

# External variables
# The current hand world matrix
hand = make_dummy_matrix("hand")
# The current torso world matrix
torso = make_dummy_matrix("torso")
# The hand displacement for this step
delta_s = Symbol("delta_s")
# rotational displacement
delta_r = Symbol("delta_r")

# <markdowncell>

# We're going to work on the configuration, but, before we can work with the configuration, we need to calculate a sane initial configuration from the model matrices.

# <codecell>

from sympy.solvers import solve
from sympy import Symbol

# Replace the placeholder trig function variables with actual trig functions
def subs_trig_exprs(expr):
    result = expr
    for i in range(1,8):
        result = result.subs("s_%d" % (i,), "sin(theta_%d)" % (i,))
        result = result.subs("c_%d" % (i,), "cos(theta_%d)" % (i,))
    return result

trig_shoulder = subs_trig_exprs(shoulder_mat)
trig_elbow = subs_trig_exprs(elbow_mat)
trig_wrist = subs_trig_exprs(wrist_mat)

shoulder_solns = solve(dummy_shoulder_mat - trig_shoulder, theta_1, theta_2, theta_3)
elbow_solns = solve(dummy_elbow_mat - trig_elbow, theta_4, theta_5)
wrist_solns = solve(dummy_wrist_mat - trig_wrist, theta_6, theta_7)

def find_shortest_soln(solns):
    shortest_sol_len = float("inf")
    shortest_sol = None
    for sol in solns:
        if len(str(sol)) < shortest_sol_len:
            shortest_sol_len = len(str(sol))
            shortest_sol = sol
    return shortest_sol

inverse_config = reduce(lambda x, y: x+y, [list(find_shortest_soln(solns)) for solns in [shoulder_solns, elbow_solns, wrist_solns]])

config_terms, config_list = cse(inverse_config)
config_terms_js = jsify_terms(config_terms)
config_list_js = ", ".join([jsify_expr(x) for x in config_list])
config_list_js = "[%s]" %config_list_js

# <markdowncell>

# Maybe I'm calculating the Jacobian wrong? Sympy has a Jacobian function. I also just realized that those angles have nothing to do with the actual Euler angle of the hand. Since the most important component of the hand orientation is the direction the palm of the hand faces, we will take a local palm vector, which corresponds to a point on the palm, and rotate it into world space. This is nice because all the units will match, allowing us to use Euclidean distance.

# <codecell>

local_palm_vector = Matrix([Symbol("palm_x"), Symbol("palm_y"), Symbol("palm_z"), 1])
global_palm_vector = R_0[7] * local_palm_vector

target2 = Matrix([Symbol("target_%s" % s) for s in ["x", "y", "z", "palm_x", "palm_y", "palm_z"]])

hand_pose2 = Matrix([R_0[7][0,3],
                     R_0[7][1,3],
                     R_0[7][2,3]]).col_join(global_palm_vector[:3,0])

hand_pose2 = hand_pose2.subs([("shoulder_%s" % axis, dummy_shoulder_mat[row, 3])
                                  for row, axis in enumerate(["x","y","z"])])
hand_pose2 = subs_trig_exprs(hand_pose2)
J2 = hand_pose2.jacobian(
        [Symbol("theta_%d" % i) for i in range(1,8)])
hand_dist = (Matrix(hand_pose2)-target2).norm()
delta_pose2 = delta_s * (Matrix(hand_pose2)-target2)
delta_config2 = J2.T * delta_pose2
new_config = Matrix([Symbol("theta_%d" % i) for i in range(1,8)]) + delta_config2

# <codecell>

step_config_terms, step_config_list = cse(new_config, optimizations='basic')
step_config_terms_js = jsify_terms(step_config_terms)
step_config_list_js = jsify_list(step_config_list[0])
extract_parameters(new_config)

# <codecell>

# Joining all the matrices is a hack to get around a problem in cse
config_mats = trig_shoulder.col_join(trig_elbow).col_join(trig_wrist)
config_mats = config_mats.subs([("shoulder_%s" % axis, dummy_shoulder_mat[row, 3])                                      for row, axis in enumerate(["x","y","z"])])
config_mat_terms, config_mat_clean = cse(config_mats, optimizations='basic')
config_mat_terms_js = jsify_terms(config_mat_terms)
config_shoulder_mat_js = format_js_mat(config_mat_clean[0][0:4,:])
config_elbow_mat_js = format_js_mat(config_mat_clean[0][4:8,:])
config_wrist_mat_js = format_js_mat(config_mat_clean[0][8:12,:])

# <codecell>

hand_dist_terms, hand_dist_clean = cse(hand_dist, optimizations='basic')
hand_dist_terms_js = jsify_terms(hand_dist_terms)
hand_dist_js = jsify_expr(hand_dist_clean[0])

# <codecell>

func_template2 = Template("""

// This file is automatically generated. Do not commit or edit. Instead, edit
// build_motion.py

define([],
function () {
    'use strict';

    /** @exports vbot/motion */

    /**
     * Calculates the distance of the hand from it's target in the workspace.
     * 
     * @memberof module:vbot/motion
     * @param {THREE.Matrix4} shoulder the local shoulder matrix
     * @param {Number} upperArmLength
     * @param {Number} lowerArmLength
     * @param {THREE.Vector3} localPalmPos the location of the center of
     *                                     the palm in wrist space
     * @param {THREE.Vector3} targetWristPos the target position of the
     *                                       wrist in torso space
     * @param {THREE.Vector3} targetPalmPos the target position of the
     *                                      center of the palm in torso
     *                                      space
     * @param {Number[]} config the configuration of the joints
     * @returns {Number} the distance of config from the target
     */
    function handDist(shoulder,
                      upperArmLength,
                      lowerArmLength,
                      localPalmPos,
                      targetWristPos,
                      targetPalmPos,
                      config) {
        var theta_1 = config[0];
        var theta_2 = config[1];
        var theta_3 = config[2];
        var theta_4 = config[3];
        var theta_5 = config[4];
        var theta_6 = config[5];
        var theta_7 = config[6];
        var target_x = targetWristPos.x;
        var target_y = targetWristPos.y;
        var target_z = targetWristPos.z;
        var target_palm_x = targetPalmPos.x;
        var target_palm_y = targetPalmPos.y;
        var target_palm_z = targetPalmPos.z;
        var palm_x = localPalmPos.x;
        var palm_y = localPalmPos.y;
        var palm_z = localPalmPos.z;
        var l_4 = upperArmLength;
        var l_5 = lowerArmLength;
        {{hand_dist_terms|indent|indent}}
        return {{hand_dist}};
    }
    
    /**
     * Calculates the configuration parameters from matrices.
     * 
     * @param {THREE.Matrix4} shoulder the local shoulder matrix
     * @param {THREE.Matrix4} elbow the local elbow matrix
     * @param {THREE.Matrix4} wrist the local wrist matrix
     * @returns {Number[]} the configuration those matrices indicate
     */
    function calcConfig(shoulder, elbow, wrist) {
        {{config_terms|indent|indent}}
        return {{config_list}};
    }
    
    /**
     * Calculates the next step of motion in configuration space.
     * 
     * @memberof module:vbot/motion
     * @param {THREE.Matrix4} shoulder the local shoulder matrix
     * @param {Number[]} config the configuration of the joints
     * @param {Number} delta_s the distance to step the hand
     * @param {Number} upperArmLength
     * @param {Number} lowerArmLength
     * @param {THREE.Vector3} localPalmPos the location of the center of
     *                                     the palm in wrist space
     * @param {THREE.Vector3} targetWristPos the target position of the
     *                                       wrist in torso space
     * @param {THREE.Vector3} targetPalmPos the target position of the
     *                                      center of the palm in torso
     *                                      space
     * @returns {Number[]} the new configuration
     */
    function stepConfig(shoulder,
                        config,
                        delta_s,
                        upperArmLength,
                        lowerArmLength,
                        localPalmPos,
                        targetWristPos,
                        targetPalmPos) {
        var l_4 = upperArmLength;
        var l_5 = lowerArmLength;
        var theta_1 = config[0];
        var theta_2 = config[1];
        var theta_3 = config[2];
        var theta_4 = config[3];
        var theta_5 = config[4];
        var theta_6 = config[5];
        var theta_7 = config[6];
        var pose_rx = theta_1 + theta_5;
        var pose_ry = theta_3 + theta_4 + theta_7;
        var pose_rz = theta_2 + theta_6;
        var target_x = targetWristPos.x;
        var target_y = targetWristPos.y;
        var target_z = targetWristPos.z;
        var target_palm_x = targetPalmPos.x;
        var target_palm_y = targetPalmPos.y;
        var target_palm_z = targetPalmPos.z;
        var palm_x = localPalmPos.x;
        var palm_y = localPalmPos.y;
        var palm_z = localPalmPos.z;
        {{step_config_terms|indent|indent}}
        return {{step_config_list}};
    }
    
    
    
    /**
     * Calculates the matrices of each joint
     * 
     * @memberof module:vbot/motion
     * @param {THREE.Matrix4} shoulder the local shoulder matrix
     * @param {Number} upperArmLength
     * @param {Number} lowerArmLength
     * @param {Number[]} config the configuration of the joints
     * @returns {Object} the matrices
     */
    function configToMatrices(shoulder,
                              upperArmLength,
                              lowerArmLength,
                              config) {
        var theta_1 = config[0];
        var theta_2 = config[1];
        var theta_3 = config[2];
        var theta_4 = config[3];
        var theta_5 = config[4];
        var theta_6 = config[5];
        var theta_7 = config[6];
        var l_4 = upperArmLength;
        var l_5 = lowerArmLength;
        {{config_mat_terms|indent|indent}}
        return {
            shoulder: {{config_shoulder_mat}},
            elbow: {{config_elbow_mat}},
            wrist: {{config_wrist_mat}}
        };
    }
    
    return {
        configToMatrices: configToMatrices,
        stepConfig: stepConfig,
        calcConfig: calcConfig,
        handDist: handDist
    };
});
""")

print(func_template2.render(config_terms=config_terms_js,
                            config_list=config_list_js,
                            step_config_terms=step_config_terms_js,
                            step_config_list=step_config_list_js,
                            config_mat_terms=config_mat_terms_js,
                            config_shoulder_mat=config_shoulder_mat_js,
                            config_elbow_mat=config_elbow_mat_js,
                            config_wrist_mat=config_wrist_mat_js,
                            hand_dist_terms=hand_dist_terms_js,
                            hand_dist=hand_dist_js))

# <codecell>


