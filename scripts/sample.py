#!/usr/bin/env python
"""
File: ik_controll_commander.py
Description: Control the arm by Inverse kinematics
"""

import kdl_parser_py.urdf
import PyKDL as kdl
import numpy as np
import rospy


if __name__ == "__main__":
    (ok, tree) = kdl_parser_py.urdf.treeFromParam("robot_description")
    if ok is False:
        print("failed")
        exit(-1)
    print(tree)
    chain = tree.getChain("craft_link_base", "adk_camera")
    print(chain)
    ik_solver_1 = kdl.ChainIkSolverVel_pinv(chain)
    num = 5
    q_in = kdl.JntArray(num)
    q_in[0] = 0 * np.pi/180
    q_in[1] = 30 * np.pi/180
    q_in[2] = 30 * np.pi/180
    q_in[3] = 30 * np.pi/180
    q_in[4] = 0 * np.pi/180
    vel = kdl.Vector(0, 0, -0.1)
    rot = kdl.Vector(0, 0, 0)
    v_in = kdl.Twist(vel, rot)
    print("v_in: ", v_in.vel)
    q_out = kdl.JntArray(num)
    ik_solver_1.CartToJnt(q_in=q_in, v_in=v_in, qdot_out=q_out)
    for i in range(num):
        print("q_out: ", q_out[i]*180.0/np.pi)

    # res = kdl.JntArray.Add(q_in, q_out)
    res = kdl.JntArray(num)
    # res = kdl.JntArray.Add(q_in, q_out)
    for i in range(num):
        res[i] = q_in[i] + q_out[i]
        print("res", res[i]*180.0/np.pi)

    fk_solver = kdl.ChainFkSolverPos_recursive(chain)
    ik_solver_2 = kdl.ChainIkSolverPos_NR(chain, fk_solver, ik_solver_1)
    cartpos = kdl.Frame()
    fk_solver.JntToCart(q_in, cartpos)
    print(cartpos)
    print('===')
    cartpos.p.z(cartpos.p.z() - 0.1)
    print(cartpos)
    print('===')
    q_out2 = kdl.JntArray(num)
    ik_solver_2.CartToJnt(q_in, cartpos, q_out2)
    for i in range(num):
        print("q_out2: ", q_out2[i]*180.0/np.pi)


