import math
import numpy as np

# math.sin() und math.cos() verwenden radian als input

def forwards_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
	T01 = transformation_matrix(np.pi / 2, theta1, 0, 0.1625)
	T12 = transformation_matrix(0, theta2, -0.425, 0)
	T23 = transformation_matrix(0, theta3, -0.3992, 0)
	T34 = transformation_matrix(np.pi / 2, theta4, 0, 0.1333)
	T45 = transformation_matrix(-np.pi / 2, theta5, 0, 0.0997)
	T56 = transformation_matrix(0, theta6, 0, 0.0996)
	T = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T01, T12), T23), T34), T45), T56)
	return T


def transformation_matrix(alpha, theta, a, d):
	matrix = [
		[math.cos(theta), -math.cos(alpha) * math.sin(theta), math.sin(alpha) * math.sin(theta), a * math.cos(theta)],
		[math.sin(theta), math.cos(alpha) * math.cos(theta), -math.sin(alpha) * math.cos(theta), a * math.sin(theta)],
		[0, math.sin(alpha), math.cos(alpha), d],
		[0, 0, 0, 1]
	]
	return matrix


T = forwards_kinematics(-0.7381773798123205, -1.4648116097453374, -1.6673837067734418, -1.5806872649043584, 1.5745233581703098, -2.3104228411877177)
print(T)

print(f"position: {T[0][3]}, {T[1][3]}, {T[2][3]}")
