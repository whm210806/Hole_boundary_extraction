#!/usr/bin/env python

import open3d as o3d
import rospy
import numpy as np
import math
import casadi as ca

def random_orthogonal_matrix():
    Q, _ = np.linalg.qr(np.random.rand(3, 3))  # QR分解生成正交矩阵
    return Q

def rotation_matrices(roll, pitch, yaw):
    # 滚转（Roll）矩阵
    R_x = np.array([[1.0, 0.0, 0.0],
                    [0.0, math.cos(roll), -math.sin(roll)],
                    [0.0, math.sin(roll), math.cos(roll)]], dtype=float)
    
    # 俯仰（Pitch）矩阵
    R_y = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                    [0.0, 1.0, 0.0],
                    [-math.sin(pitch), 0.0, math.cos(pitch)]], dtype=float)
    
    # 偏航（Yaw）矩阵
    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                    [math.sin(yaw), math.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0]], dtype=float)

    # 组合旋转矩阵
    Rot = np.dot(R_z, np.dot(R_y, R_x))
    return Rot

# 读取点云数据

def point_extraction(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return np.asarray(pcd.points)

def multi_start_optimization(solver, num_trials=10):
    best_solution = None
    best_objective = float('inf')
    
    for _ in range(num_trials):
        # 随机初始化
        initial_guess = np.random.uniform(-1.6, 1.6, 6)  # 这里范围根据问题调整
        
        # 求解问题
        # solution = solver(x0=initial_guess)

        lbx = [-ca.pi, -ca.pi, -ca.pi, -ca.inf, -ca.inf, -ca.inf]  # 对变量的下界，第 2 个变量设置下界为 0，第 3 个为 -5
        ubx = [ca.pi, ca.pi, ca.pi, ca.inf, ca.inf, ca.inf] 
        solution = solver(x0=initial_guess, lbx=lbx, ubx=ubx)
        optimal_d = solution['x']
        optimal_obj = solution['f']
        
        # 记录最优解
        if optimal_obj < best_objective:
            best_solution = optimal_d
            best_objective = optimal_obj
    
    return best_solution, best_objective

def param_cal(points_0,points_1,points_2):
    # pcd = o3d.io.read_point_cloud("/home/mike/sick_Ws/transformed_circle_boundary.pcd")

    # # 提取 x, y, z 坐标
    # points = np.asarray(pcd.points)

    # P_target = np.zeros((3,1)) 
    P_target_0 =np.array([[0.], 
                       [0.], 
                       [0.1]])
    
    P_target_1 =np.array([[0.], 
                       [-0.23], 
                       [0.1]])

    P_target_2 =np.array([[-0.5], 
                       [0.275], 
                       [0.1]])
    
    R = ca.MX.sym('R', 6)
    R_x = ca.vertcat(
        ca.horzcat(1, 0, 0),
        ca.horzcat(0, ca.cos(R[0]), -ca.sin(R[0])),
        ca.horzcat(0, ca.sin(R[0]), ca.cos(R[0]))
    )

    # 绕Y轴的旋转矩阵（俯仰）
    R_y = ca.vertcat(
        ca.horzcat(ca.cos(R[1]), 0, ca.sin(R[1])),
        ca.horzcat(0, 1, 0),
        ca.horzcat(-ca.sin(R[1]), 0, ca.cos(R[1]))
    )

    # 绕Z轴的旋转矩阵（偏航）
    R_z = ca.vertcat(
        ca.horzcat(ca.cos(R[2]), -ca.sin(R[2]), 0),
        ca.horzcat(ca.sin(R[2]), ca.cos(R[2]), 0),
        ca.horzcat(0, 0, 1)
    )

    Rot = R_z @ R_y @ R_x

    objective = 0

    for i in range(points_0.shape[0]):
        term = Rot @ points_0[i, :] + R[3:] - P_target_0
        norm_value = ca.norm_2(term)
        objective += (norm_value - 0.085) ** 2
    
    for i in range(points_1.shape[0]):
        term = Rot @ points_1[i, :] + R[3:] - P_target_1
        norm_value = ca.norm_2(term)
        objective += (norm_value - 0.085) ** 2
    for i in range(points_2.shape[0]):
        term = Rot @ points_2[i, :] + R[3:] - P_target_2
        norm_value = ca.norm_2(term)
        objective += (norm_value - 0.085) ** 2
    
    nlp = {'x': ca.vec(R), 'f': objective}#, 'g': ca.vertcat(*constraints)}

    solver = ca.nlpsol('solver', 'ipopt', nlp)

    R_opt, optimized_objective_value = multi_start_optimization(solver)

    # R0 =  np.zeros((6,1))

    # sol = solver(x0=R0)


    # R_opt = np.array(sol['x'])

    # rospy.loginfo(R_opt)

    # optimized_objective_value = sol['f'].full().item()  # .full() 返回一个 NumPy 数组，.item() 获取标量
    print(R_opt)

    print("优化后的目标值:", optimized_objective_value)

    # 在最终 result 计算中添加调试信息
    Rotat = rotation_matrices(R_opt[0], R_opt[1], R_opt[2])
    # R_opt_t = np.array([[0.55], 
    #                    [-0.], 
    #                    [0.1]])
    # Rotat = rotation_matrices(0, 1.57,-1.57)
    result = 0
    for i in range(points_0.shape[0]):
        term = Rotat @ points_0[i, :] + R_opt[3:] - P_target_0
        norm_value = np.linalg.norm(term)
        print(norm_value - 0.085)
        # print(f"点 {i} 的最终计算误差: {norm_value}")
        result += (norm_value - 0.085) ** 2

    print("最终计算结果:", result)


if __name__ == '__main__':
    rospy.init_node("param_calibration_node")
    file_name_0 = "/home/mike/sick_Ws/circle_0.pcd"
    file_name_1 = "/home/mike/sick_Ws/circle_1.pcd"
    file_name_2 = "/home/mike/sick_Ws/circle_2.pcd"
    points_0 = point_extraction(file_path=file_name_0)
    points_1 = point_extraction(file_path=file_name_1)
    points_2 = point_extraction(file_path=file_name_2)
    param_cal(points_0,points_1,points_2)