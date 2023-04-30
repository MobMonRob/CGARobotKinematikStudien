import math
import numpy as np
from csv import reader

def multivector_to_string(object, name, array):
	string = object
	string += "," + name
	for x in array:
		string += "," + str(x)
	string += "\n"
	return string

def inverse_kinematics_ur5e(ae_1, ae_2, ae_3, se_1, se_2, se_3, p_x, p_y, p_z):
	P_e = np.zeros(32)
	P_e[1] = p_x # e1
	P_e[2] = p_y # e2
	P_e[3] = p_z # e3
	P_e[4] = (p_x * p_x + p_y * p_y + p_z * p_z) / 2.0 # einf
	P_e[5] = 1.0 # e0
	P_5 = np.zeros(32)
	P_5[1] = p_x - 0.0996 * ae_1 # e1
	P_5[2] = p_y - 0.0996 * ae_2 # e2
	P_5[3] = p_z - 0.0996 * ae_3 # e3
	P_5[4] = ((p_x - 0.0996 * ae_1) * (p_x - 0.0996 * ae_1) + (p_y - 0.0996 * ae_2) * (p_y - 0.0996 * ae_2) + (p_z - 0.0996 * ae_3) * (p_z - 0.0996 * ae_3)) / 2.0 # einf
	P_5[5] = 1.0 # e0
	S_c = np.zeros(32)
	S_c[1] = P_5[1] # e1
	S_c[2] = P_5[2] # e2
	S_c[3] = P_5[3] # e3
	S_c[4] = P_5[4] - 0.008884445 # einf
	S_c[5] = 1.0 # e0
	K_0 = np.zeros(32)
	K_0[4] = (-S_c[4]) # einf
	K_0[5] = 1.0 # e0
	C_5k = np.zeros(32)
	C_5k[8] = S_c[1] * K_0[4] # e1 ^ einf
	C_5k[9] = S_c[1] # e1 ^ e0
	C_5k[11] = S_c[2] * K_0[4] # e2 ^ einf
	C_5k[12] = S_c[2] # e2 ^ e0
	C_5k[13] = S_c[3] * K_0[4] # e3 ^ einf
	C_5k[14] = S_c[3] # e3 ^ e0
	C_5k[15] = S_c[4] + (-K_0[4]) # einf ^ e0
	Q_c = np.zeros(32)
	Q_c[19] = C_5k[8] # e1 ^ (e3 ^ einf)
	Q_c[20] = C_5k[9] # e1 ^ (e3 ^ e0)
	Q_c[21] = C_5k[9] * P_5[3] # e1 ^ (einf ^ e0)
	Q_c[22] = C_5k[11] # e2 ^ (e3 ^ einf)
	Q_c[23] = C_5k[12] # e2 ^ (e3 ^ e0)
	Q_c[24] = C_5k[12] * P_5[3] # e2 ^ (einf ^ e0)
	Q_c[25] = C_5k[14] * P_5[3] + (-C_5k[15]) # e3 ^ (einf ^ e0)
	macro_ExtractFirstPoint_dPP = np.zeros(32)
	macro_ExtractFirstPoint_dPP[6] = (-Q_c[25]) # e1 ^ e2
	macro_ExtractFirstPoint_dPP[8] = (-Q_c[22]) # e1 ^ einf
	macro_ExtractFirstPoint_dPP[10] = (-Q_c[21]) # e2 ^ e3
	macro_ExtractFirstPoint_dPP[12] = (-Q_c[20]) # e2 ^ e0
	P_c = np.zeros(32)
	P_c[1] = math.sqrt(abs(Q_c[19] * Q_c[20] + Q_c[20] * Q_c[19] + Q_c[21] * Q_c[21] + Q_c[22] * Q_c[23] + Q_c[23] * Q_c[22] + Q_c[24] * Q_c[24] + Q_c[25] * Q_c[25])) * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]) + macro_ExtractFirstPoint_dPP[6] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]) # e1
	P_c[2] = math.sqrt(abs(Q_c[19] * Q_c[20] + Q_c[20] * Q_c[19] + Q_c[21] * Q_c[21] + Q_c[22] * Q_c[23] + Q_c[23] * Q_c[22] + Q_c[24] * Q_c[24] + Q_c[25] * Q_c[25])) * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]) + (-(macro_ExtractFirstPoint_dPP[6] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) # e2
	P_c[3] = (-(Q_c[24] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) + (-(macro_ExtractFirstPoint_dPP[10] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) # e3
	P_c[4] = (-(macro_ExtractFirstPoint_dPP[8] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) + (-(Q_c[19] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) # einf
	P_c[5] = (-(Q_c[23] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) + (-(macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) # e0
	P_c[16] = (-(Q_c[24] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) + macro_ExtractFirstPoint_dPP[10] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]) # e1 ^ (e2 ^ e3)
	P_c[17] = (-(macro_ExtractFirstPoint_dPP[8] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) + Q_c[19] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]) # e1 ^ (e2 ^ einf)
	P_c[18] = (-(Q_c[23] * macro_ExtractFirstPoint_dPP[12] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]))) + macro_ExtractFirstPoint_dPP[12] * Q_c[23] / (Q_c[23] * Q_c[23] + macro_ExtractFirstPoint_dPP[12] * macro_ExtractFirstPoint_dPP[12]) # e1 ^ (e2 ^ e0)
	PI_c = np.zeros(32)
	PI_c[29] = P_c[1] # e1 ^ (e3 ^ (einf ^ e0))
	PI_c[30] = P_c[2] # e2 ^ (e3 ^ (einf ^ e0))
	PI_c_parallel = np.zeros(32)
	PI_c_parallel[1] = PI_c[30] # e1
	PI_c_parallel[2] = (-PI_c[29]) # e2
	PI_c_parallel[4] = P_5[1] * PI_c[30] + P_5[2] * (-PI_c[29]) # einf
	PI_56_orthogonal = np.zeros(32)
	PI_56_orthogonal[26] = (-(P_5[4] + (-P_e[4]))) # e1 ^ (e2 ^ (e3 ^ einf))
	PI_56_orthogonal[28] = P_5[3] + (-P_e[3]) # e1 ^ (e2 ^ (einf ^ e0))
	PI_56_orthogonal[29] = (-(P_5[2] + (-P_e[2]))) # e1 ^ (e3 ^ (einf ^ e0))
	PI_56_orthogonal[30] = P_5[1] + (-P_e[1]) # e2 ^ (e3 ^ (einf ^ e0))
	n_56_orthogonal = np.zeros(32)
	n_56_orthogonal[6] = PI_56_orthogonal[28] * math.sqrt(abs((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30])))) / math.sqrt(abs(((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30]))) * ((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30]))))) # e1 ^ e2
	n_56_orthogonal[7] = PI_56_orthogonal[29] * math.sqrt(abs((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30])))) / math.sqrt(abs(((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30]))) * ((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30]))))) # e1 ^ e3
	n_56_orthogonal[10] = PI_56_orthogonal[30] * math.sqrt(abs((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30])))) / math.sqrt(abs(((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30]))) * ((-((-PI_56_orthogonal[28]) * PI_56_orthogonal[28])) + (-((-PI_56_orthogonal[29]) * PI_56_orthogonal[29])) + (-((-PI_56_orthogonal[30]) * PI_56_orthogonal[30]))))) # e2 ^ e3
	PI_c_orthogonal = np.zeros(32)
	PI_c_orthogonal[26] = P_5[1] * n_56_orthogonal[10] + (-(P_5[2] * n_56_orthogonal[7])) + P_5[3] * n_56_orthogonal[6] # e1 ^ (e2 ^ (e3 ^ einf))
	PI_c_orthogonal[28] = (-n_56_orthogonal[6]) # e1 ^ (e2 ^ (einf ^ e0))
	PI_c_orthogonal[29] = (-n_56_orthogonal[7]) # e1 ^ (e3 ^ (einf ^ e0))
	PI_c_orthogonal[30] = (-n_56_orthogonal[10]) # e2 ^ (e3 ^ (einf ^ e0))
	L_45 = np.zeros(32)
	L_45[6] = PI_c_parallel[1] * (-PI_c_orthogonal[29]) + (-(PI_c_parallel[2] * PI_c_orthogonal[30])) # e1 ^ e2
	L_45[7] = PI_c_parallel[1] * PI_c_orthogonal[28] # e1 ^ e3
	L_45[8] = PI_c_parallel[1] * (-PI_c_orthogonal[26]) + (-(PI_c_parallel[4] * PI_c_orthogonal[30])) # e1 ^ einf
	L_45[10] = PI_c_parallel[2] * PI_c_orthogonal[28] # e2 ^ e3
	L_45[11] = PI_c_parallel[2] * (-PI_c_orthogonal[26]) + (-(PI_c_parallel[4] * (-PI_c_orthogonal[29]))) # e2 ^ einf
	L_45[13] = (-(PI_c_parallel[4] * PI_c_orthogonal[28])) # e3 ^ einf
	S_5 = np.zeros(32)
	S_5[1] = P_5[1] # e1
	S_5[2] = P_5[2] # e2
	S_5[3] = P_5[3] # e3
	S_5[4] = P_5[4] - 0.004970044999999999 # einf
	S_5[5] = 1.0 # e0
	Q_4 = np.zeros(32)
	Q_4[16] = (-((-(L_45[6] * (-S_5[3]))) + (-(L_45[7] * S_5[2])) + (-(L_45[10] * (-S_5[1]))))) # e1 ^ (e2 ^ e3)
	Q_4[17] = (-(L_45[6] * S_5[4])) + L_45[8] * S_5[2] + L_45[11] * (-S_5[1]) # e1 ^ (e2 ^ einf)
	Q_4[18] = (-L_45[6]) # e1 ^ (e2 ^ e0)
	Q_4[19] = (-(L_45[7] * S_5[4] + L_45[8] * (-S_5[3]) + (-(L_45[13] * (-S_5[1]))))) # e1 ^ (e3 ^ einf)
	Q_4[20] = (-L_45[7]) # e1 ^ (e3 ^ e0)
	Q_4[21] = (-L_45[8]) # e1 ^ (einf ^ e0)
	Q_4[22] = (-(L_45[10] * S_5[4])) + (-(L_45[11] * (-S_5[3]))) + (-(L_45[13] * S_5[2])) # e2 ^ (e3 ^ einf)
	Q_4[23] = (-L_45[10]) # e2 ^ (e3 ^ e0)
	Q_4[24] = (-L_45[11]) # e2 ^ (einf ^ e0)
	Q_4[25] = (-L_45[13]) # e3 ^ (einf ^ e0)
	macro_ExtractFirstPoint_dPP1 = np.zeros(32)
	macro_ExtractFirstPoint_dPP1[6] = (-Q_4[25]) # e1 ^ e2
	macro_ExtractFirstPoint_dPP1[8] = (-Q_4[22]) # e1 ^ einf
	macro_ExtractFirstPoint_dPP1[10] = (-Q_4[21]) # e2 ^ e3
	macro_ExtractFirstPoint_dPP1[12] = (-Q_4[20]) # e2 ^ e0
	macro_ExtractFirstPoint_dPP1[13] = (-Q_4[17]) # e3 ^ einf
	P_4 = np.zeros(32)
	P_4[1] = math.sqrt(abs((-(Q_4[16] * Q_4[16])) + Q_4[17] * Q_4[18] + Q_4[18] * Q_4[17] + Q_4[19] * Q_4[20] + Q_4[20] * Q_4[19] + Q_4[21] * Q_4[21] + Q_4[22] * Q_4[23] + Q_4[23] * Q_4[22] + Q_4[24] * Q_4[24] + Q_4[25] * Q_4[25])) * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + macro_ExtractFirstPoint_dPP1[6] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + Q_4[24] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(Q_4[23] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) # e1
	P_4[2] = math.sqrt(abs((-(Q_4[16] * Q_4[16])) + Q_4[17] * Q_4[18] + Q_4[18] * Q_4[17] + Q_4[19] * Q_4[20] + Q_4[20] * Q_4[19] + Q_4[21] * Q_4[21] + Q_4[22] * Q_4[23] + Q_4[23] * Q_4[22] + Q_4[24] * Q_4[24] + Q_4[25] * Q_4[25])) * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(macro_ExtractFirstPoint_dPP1[6] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + macro_ExtractFirstPoint_dPP1[10] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(macro_ExtractFirstPoint_dPP1[12] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) # e2
	P_4[3] = math.sqrt(abs((-(Q_4[16] * Q_4[16])) + Q_4[17] * Q_4[18] + Q_4[18] * Q_4[17] + Q_4[19] * Q_4[20] + Q_4[20] * Q_4[19] + Q_4[21] * Q_4[21] + Q_4[22] * Q_4[23] + Q_4[23] * Q_4[22] + Q_4[24] * Q_4[24] + Q_4[25] * Q_4[25])) * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(Q_4[24] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(macro_ExtractFirstPoint_dPP1[10] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(Q_4[18] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) # e3
	P_4[4] = math.sqrt(abs((-(Q_4[16] * Q_4[16])) + Q_4[17] * Q_4[18] + Q_4[18] * Q_4[17] + Q_4[19] * Q_4[20] + Q_4[20] * Q_4[19] + Q_4[21] * Q_4[21] + Q_4[22] * Q_4[23] + Q_4[23] * Q_4[22] + Q_4[24] * Q_4[24] + Q_4[25] * Q_4[25])) * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(macro_ExtractFirstPoint_dPP1[8] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(Q_4[19] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(macro_ExtractFirstPoint_dPP1[13] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(Q_4[16] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) # einf
	P_4[5] = (-(Q_4[23] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + (-(Q_4[18] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) # e0
	P_4[16] = macro_ExtractFirstPoint_dPP1[6] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(Q_4[24] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + macro_ExtractFirstPoint_dPP1[10] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e1 ^ (e2 ^ e3)
	P_4[17] = macro_ExtractFirstPoint_dPP1[6] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(macro_ExtractFirstPoint_dPP1[8] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + Q_4[19] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e1 ^ (e2 ^ einf)
	P_4[18] = (-(Q_4[23] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + macro_ExtractFirstPoint_dPP1[12] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e1 ^ (e2 ^ e0)
	P_4[19] = Q_4[24] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(macro_ExtractFirstPoint_dPP1[8] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + macro_ExtractFirstPoint_dPP1[13] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e1 ^ (e3 ^ einf)
	P_4[20] = (-(Q_4[23] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + Q_4[18] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e1 ^ (e3 ^ e0)
	P_4[21] = (-(Q_4[23] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + Q_4[16] * Q_4[23] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e1 ^ (einf ^ e0)
	P_4[22] = macro_ExtractFirstPoint_dPP1[10] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) + (-(Q_4[19] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + macro_ExtractFirstPoint_dPP1[13] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e2 ^ (e3 ^ einf)
	P_4[23] = (-(macro_ExtractFirstPoint_dPP1[12] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + Q_4[18] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e2 ^ (e3 ^ e0)
	P_4[24] = (-(macro_ExtractFirstPoint_dPP1[12] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + Q_4[16] * macro_ExtractFirstPoint_dPP1[12] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e2 ^ (einf ^ e0)
	P_4[25] = (-(Q_4[18] * Q_4[16] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]))) + Q_4[16] * Q_4[18] / (Q_4[23] * Q_4[23] + macro_ExtractFirstPoint_dPP1[12] * macro_ExtractFirstPoint_dPP1[12] + Q_4[18] * Q_4[18]) # e3 ^ (einf ^ e0)
	S_4 = np.zeros(32)
	S_4[1] = P_4[1] # e1
	S_4[2] = P_4[2] # e2
	S_4[3] = P_4[3] # e3
	S_4[4] = P_4[4] + 0.008884445 # einf
	S_4[5] = P_4[5] # e0
	S_4[16] = P_4[16] # e1 ^ (e2 ^ e3)
	S_4[17] = P_4[17] # e1 ^ (e2 ^ einf)
	S_4[18] = P_4[18] # e1 ^ (e2 ^ e0)
	S_4[19] = P_4[19] # e1 ^ (e3 ^ einf)
	S_4[20] = P_4[20] # e1 ^ (e3 ^ e0)
	S_4[21] = P_4[21] # e1 ^ (einf ^ e0)
	S_4[22] = P_4[22] # e2 ^ (e3 ^ einf)
	S_4[23] = P_4[23] # e2 ^ (e3 ^ e0)
	S_4[24] = P_4[24] # e2 ^ (einf ^ e0)
	S_4[25] = P_4[25] # e3 ^ (einf ^ e0)
	L_34 = np.zeros(32)
	L_34[17] = P_4[1] * (-PI_c[29]) + (-(P_4[2] * PI_c[30])) # e1 ^ (e2 ^ einf)
	L_34[19] = (-(P_4[3] * PI_c[30])) # e1 ^ (e3 ^ einf)
	L_34[21] = P_4[5] * PI_c[30] # e1 ^ (einf ^ e0)
	L_34[22] = (-(P_4[3] * (-PI_c[29]))) # e2 ^ (e3 ^ einf)
	L_34[24] = P_4[5] * (-PI_c[29]) # e2 ^ (einf ^ e0)
	L_34[31] = (-(P_4[20] * (-PI_c[29]) + (-(P_4[23] * PI_c[30])))) # e1 ^ (e2 ^ (e3 ^ (einf ^ e0)))
	Q_3 = np.zeros(32)
	Q_3[16] = (-((-(S_4[2] * L_34[24])) + (-((-S_4[1]) * (-L_34[21]))))) # e1 ^ (e2 ^ e3)
	Q_3[17] = S_4[2] * (-L_34[22]) + (-S_4[1]) * L_34[19] # e1 ^ (e2 ^ einf)
	Q_3[19] = (-(S_4[4] * L_34[24] + (-S_4[3]) * (-L_34[22]) + (-((-S_4[1]) * (-L_34[17]))))) # e1 ^ (e3 ^ einf)
	Q_3[20] = (-S_4[5]) * L_34[24] # e1 ^ (e3 ^ e0)
	Q_3[21] = (-S_4[5]) * (-L_34[22]) # e1 ^ (einf ^ e0)
	Q_3[22] = (-(S_4[4] * (-L_34[21]))) + (-((-S_4[3]) * L_34[19])) + (-(S_4[2] * (-L_34[17]))) # e2 ^ (e3 ^ einf)
	Q_3[23] = (-S_4[5]) * (-L_34[21]) # e2 ^ (e3 ^ e0)
	Q_3[24] = (-S_4[5]) * L_34[19] # e2 ^ (einf ^ e0)
	Q_3[25] = (-S_4[5]) * (-L_34[17]) # e3 ^ (einf ^ e0)
	Q_3[31] = (-((-(S_4[24] * L_34[24])) + S_4[23] * (-L_34[22]) + (-((-S_4[21]) * (-L_34[21]))) + (-S_4[20]) * L_34[19] + S_4[18] * (-L_34[17]))) # e1 ^ (e2 ^ (e3 ^ (einf ^ e0)))
	macro_ExtractSecondPoint_dPP = np.zeros(32)
	macro_ExtractSecondPoint_dPP[6] = (-Q_3[25]) # e1 ^ e2
	macro_ExtractSecondPoint_dPP[8] = (-Q_3[22]) # e1 ^ einf
	macro_ExtractSecondPoint_dPP[10] = (-Q_3[21]) # e2 ^ e3
	macro_ExtractSecondPoint_dPP[12] = (-Q_3[20]) # e2 ^ e0
	macro_ExtractSecondPoint_dPP[13] = (-Q_3[17]) # e3 ^ einf
	P_3 = np.zeros(32)
	P_3[1] = (Q_3[31] - math.sqrt(math.sqrt(abs(((-(Q_3[16] * Q_3[16])) + Q_3[19] * Q_3[20] + Q_3[20] * Q_3[19] + Q_3[21] * Q_3[21] + Q_3[22] * Q_3[23] + Q_3[23] * Q_3[22] + Q_3[24] * Q_3[24] + Q_3[25] * Q_3[25] + (-(Q_3[31] * Q_3[31]))) * ((-(Q_3[16] * Q_3[16])) + Q_3[19] * Q_3[20] + Q_3[20] * Q_3[19] + Q_3[21] * Q_3[21] + Q_3[22] * Q_3[23] + Q_3[23] * Q_3[22] + Q_3[24] * Q_3[24] + Q_3[25] * Q_3[25] + (-(Q_3[31] * Q_3[31]))) + (-((Q_3[25] * Q_3[31] + Q_3[31] * Q_3[25]) * (-(Q_3[25] * Q_3[31] + Q_3[31] * Q_3[25])))) + (-(((-(Q_3[24] * Q_3[31])) + (-(Q_3[31] * Q_3[24]))) * (-((-(Q_3[24] * Q_3[31])) + (-(Q_3[31] * Q_3[24])))))) + (Q_3[22] * Q_3[31] + Q_3[31] * Q_3[22]) * (-((-(Q_3[23] * Q_3[31])) + (-(Q_3[31] * Q_3[23])))) + ((-(Q_3[23] * Q_3[31])) + (-(Q_3[31] * Q_3[23]))) * (-(Q_3[22] * Q_3[31] + Q_3[31] * Q_3[22])) + (-((Q_3[21] * Q_3[31] + Q_3[31] * Q_3[21]) * (-(Q_3[21] * Q_3[31] + Q_3[31] * Q_3[21])))) + ((-(Q_3[19] * Q_3[31])) + (-(Q_3[31] * Q_3[19]))) * (-(Q_3[20] * Q_3[31] + Q_3[31] * Q_3[20])) + (Q_3[20] * Q_3[31] + Q_3[31] * Q_3[20]) * (-((-(Q_3[19] * Q_3[31])) + (-(Q_3[31] * Q_3[19])))) + ((-(Q_3[16] * Q_3[31])) + (-(Q_3[31] * Q_3[16]))) * (-((-(Q_3[16] * Q_3[31])) + (-(Q_3[31] * Q_3[16])))))))) * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + macro_ExtractSecondPoint_dPP[6] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + (-(Q_3[23] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) # e1
	P_3[2] = (Q_3[31] - math.sqrt(math.sqrt(abs(((-(Q_3[16] * Q_3[16])) + Q_3[19] * Q_3[20] + Q_3[20] * Q_3[19] + Q_3[21] * Q_3[21] + Q_3[22] * Q_3[23] + Q_3[23] * Q_3[22] + Q_3[24] * Q_3[24] + Q_3[25] * Q_3[25] + (-(Q_3[31] * Q_3[31]))) * ((-(Q_3[16] * Q_3[16])) + Q_3[19] * Q_3[20] + Q_3[20] * Q_3[19] + Q_3[21] * Q_3[21] + Q_3[22] * Q_3[23] + Q_3[23] * Q_3[22] + Q_3[24] * Q_3[24] + Q_3[25] * Q_3[25] + (-(Q_3[31] * Q_3[31]))) + (-((Q_3[25] * Q_3[31] + Q_3[31] * Q_3[25]) * (-(Q_3[25] * Q_3[31] + Q_3[31] * Q_3[25])))) + (-(((-(Q_3[24] * Q_3[31])) + (-(Q_3[31] * Q_3[24]))) * (-((-(Q_3[24] * Q_3[31])) + (-(Q_3[31] * Q_3[24])))))) + (Q_3[22] * Q_3[31] + Q_3[31] * Q_3[22]) * (-((-(Q_3[23] * Q_3[31])) + (-(Q_3[31] * Q_3[23])))) + ((-(Q_3[23] * Q_3[31])) + (-(Q_3[31] * Q_3[23]))) * (-(Q_3[22] * Q_3[31] + Q_3[31] * Q_3[22])) + (-((Q_3[21] * Q_3[31] + Q_3[31] * Q_3[21]) * (-(Q_3[21] * Q_3[31] + Q_3[31] * Q_3[21])))) + ((-(Q_3[19] * Q_3[31])) + (-(Q_3[31] * Q_3[19]))) * (-(Q_3[20] * Q_3[31] + Q_3[31] * Q_3[20])) + (Q_3[20] * Q_3[31] + Q_3[31] * Q_3[20]) * (-((-(Q_3[19] * Q_3[31])) + (-(Q_3[31] * Q_3[19])))) + ((-(Q_3[16] * Q_3[31])) + (-(Q_3[31] * Q_3[16]))) * (-((-(Q_3[16] * Q_3[31])) + (-(Q_3[31] * Q_3[16])))))))) * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + (-(macro_ExtractSecondPoint_dPP[6] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + (-(macro_ExtractSecondPoint_dPP[12] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) # e2
	P_3[3] = (-(Q_3[24] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + (-(macro_ExtractSecondPoint_dPP[10] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) # e3
	P_3[4] = (Q_3[31] - math.sqrt(math.sqrt(abs(((-(Q_3[16] * Q_3[16])) + Q_3[19] * Q_3[20] + Q_3[20] * Q_3[19] + Q_3[21] * Q_3[21] + Q_3[22] * Q_3[23] + Q_3[23] * Q_3[22] + Q_3[24] * Q_3[24] + Q_3[25] * Q_3[25] + (-(Q_3[31] * Q_3[31]))) * ((-(Q_3[16] * Q_3[16])) + Q_3[19] * Q_3[20] + Q_3[20] * Q_3[19] + Q_3[21] * Q_3[21] + Q_3[22] * Q_3[23] + Q_3[23] * Q_3[22] + Q_3[24] * Q_3[24] + Q_3[25] * Q_3[25] + (-(Q_3[31] * Q_3[31]))) + (-((Q_3[25] * Q_3[31] + Q_3[31] * Q_3[25]) * (-(Q_3[25] * Q_3[31] + Q_3[31] * Q_3[25])))) + (-(((-(Q_3[24] * Q_3[31])) + (-(Q_3[31] * Q_3[24]))) * (-((-(Q_3[24] * Q_3[31])) + (-(Q_3[31] * Q_3[24])))))) + (Q_3[22] * Q_3[31] + Q_3[31] * Q_3[22]) * (-((-(Q_3[23] * Q_3[31])) + (-(Q_3[31] * Q_3[23])))) + ((-(Q_3[23] * Q_3[31])) + (-(Q_3[31] * Q_3[23]))) * (-(Q_3[22] * Q_3[31] + Q_3[31] * Q_3[22])) + (-((Q_3[21] * Q_3[31] + Q_3[31] * Q_3[21]) * (-(Q_3[21] * Q_3[31] + Q_3[31] * Q_3[21])))) + ((-(Q_3[19] * Q_3[31])) + (-(Q_3[31] * Q_3[19]))) * (-(Q_3[20] * Q_3[31] + Q_3[31] * Q_3[20])) + (Q_3[20] * Q_3[31] + Q_3[31] * Q_3[20]) * (-((-(Q_3[19] * Q_3[31])) + (-(Q_3[31] * Q_3[19])))) + ((-(Q_3[16] * Q_3[31])) + (-(Q_3[31] * Q_3[16]))) * (-((-(Q_3[16] * Q_3[31])) + (-(Q_3[31] * Q_3[16])))))))) * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + (-(macro_ExtractSecondPoint_dPP[8] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + (-(Q_3[19] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + (-(Q_3[16] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) # einf
	P_3[5] = (-(Q_3[23] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + (-(macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) # e0
	P_3[16] = (-(Q_3[24] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + macro_ExtractSecondPoint_dPP[10] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e1 ^ (e2 ^ e3)
	P_3[17] = macro_ExtractSecondPoint_dPP[6] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + (-(macro_ExtractSecondPoint_dPP[8] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + Q_3[19] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e1 ^ (e2 ^ einf)
	P_3[18] = (-(Q_3[23] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + macro_ExtractSecondPoint_dPP[12] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e1 ^ (e2 ^ e0)
	P_3[19] = Q_3[24] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + macro_ExtractSecondPoint_dPP[13] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e1 ^ (e3 ^ einf)
	P_3[21] = (-(Q_3[23] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + Q_3[16] * Q_3[23] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e1 ^ (einf ^ e0)
	P_3[22] = macro_ExtractSecondPoint_dPP[10] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) + macro_ExtractSecondPoint_dPP[13] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e2 ^ (e3 ^ einf)
	P_3[24] = (-(macro_ExtractSecondPoint_dPP[12] * Q_3[16] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]))) + Q_3[16] * macro_ExtractSecondPoint_dPP[12] / (Q_3[23] * Q_3[23] + macro_ExtractSecondPoint_dPP[12] * macro_ExtractSecondPoint_dPP[12]) # e2 ^ (einf ^ e0)
	P_1 = np.zeros(32)
	P_1[3] = 0.1625 # e3
	P_1[4] = 0.013203125000000001 # einf
	P_1[5] = 1.0 # e0
	S_1 = np.zeros(32)
	S_1[3] = 0.1625 # e3
	S_1[4] = -0.077109375 # einf
	S_1[5] = 1.0 # e0
	S_3 = np.zeros(32)
	S_3[1] = P_3[1] # e1
	S_3[2] = P_3[2] # e2
	S_3[3] = P_3[3] # e3
	S_3[4] = P_3[4] + 0.07691042 # einf
	S_3[5] = P_3[5] # e0
	S_3[16] = P_3[16] # e1 ^ (e2 ^ e3)
	S_3[17] = P_3[17] # e1 ^ (e2 ^ einf)
	S_3[18] = P_3[18] # e1 ^ (e2 ^ e0)
	S_3[19] = P_3[19] # e1 ^ (e3 ^ einf)
	S_3[21] = P_3[21] # e1 ^ (einf ^ e0)
	S_3[22] = P_3[22] # e2 ^ (e3 ^ einf)
	S_3[24] = P_3[24] # e2 ^ (einf ^ e0)
	C_2 = np.zeros(32)
	C_2[7] = (-(0.1625 * S_3[1])) # e1 ^ e3
	C_2[8] = (-(-0.077109375 * S_3[1])) # e1 ^ einf
	C_2[9] = (-S_3[1]) # e1 ^ e0
	C_2[10] = (-(0.1625 * S_3[2])) # e2 ^ e3
	C_2[11] = (-(-0.077109375 * S_3[2])) # e2 ^ einf
	C_2[12] = (-S_3[2]) # e2 ^ e0
	C_2[13] = 0.1625 * S_3[4] + (-(-0.077109375 * S_3[3])) # e3 ^ einf
	C_2[14] = 0.1625 * S_3[5] + (-S_3[3]) # e3 ^ e0
	C_2[15] = -0.077109375 * S_3[5] + (-S_3[4]) # einf ^ e0
	C_2[26] = 0.1625 * S_3[17] + (-(-0.077109375 * S_3[16])) # e1 ^ (e2 ^ (e3 ^ einf))
	C_2[27] = 0.1625 * S_3[18] + (-S_3[16]) # e1 ^ (e2 ^ (e3 ^ e0))
	C_2[28] = -0.077109375 * S_3[18] + (-S_3[17]) # e1 ^ (e2 ^ (einf ^ e0))
	C_2[29] = (-(0.1625 * S_3[21])) + (-S_3[19]) # e1 ^ (e3 ^ (einf ^ e0))
	C_2[30] = (-(0.1625 * S_3[24])) + (-S_3[22]) # e2 ^ (e3 ^ (einf ^ e0))
	Q_2 = np.zeros(32)
	Q_2[16] = (-(C_2[10] * PI_c[30] + (-C_2[7]) * (-PI_c[29]))) # e1 ^ (e2 ^ e3)
	Q_2[17] = (-C_2[11]) * PI_c[30] + C_2[8] * (-PI_c[29]) # e1 ^ (e2 ^ einf)
	Q_2[18] = (-(C_2[12] * PI_c[30] + (-C_2[9]) * (-PI_c[29]))) # e1 ^ (e2 ^ e0)
	Q_2[19] = (-(C_2[13] * PI_c[30])) # e1 ^ (e3 ^ einf)
	Q_2[20] = (-C_2[14]) * PI_c[30] # e1 ^ (e3 ^ e0)
	Q_2[21] = (-C_2[15]) * PI_c[30] # e1 ^ (einf ^ e0)
	Q_2[22] = (-(C_2[13] * (-PI_c[29]))) # e2 ^ (e3 ^ einf)
	Q_2[23] = (-C_2[14]) * (-PI_c[29]) # e2 ^ (e3 ^ e0)
	Q_2[24] = (-C_2[15]) * (-PI_c[29]) # e2 ^ (einf ^ e0)
	Q_2[31] = (-(C_2[30] * PI_c[30] + (-C_2[29]) * (-PI_c[29]))) # e1 ^ (e2 ^ (e3 ^ (einf ^ e0)))
	macro_ExtractSecondPoint_dPP1 = np.zeros(32)
	macro_ExtractSecondPoint_dPP1[8] = (-Q_2[22]) # e1 ^ einf
	macro_ExtractSecondPoint_dPP1[10] = (-Q_2[21]) # e2 ^ e3
	macro_ExtractSecondPoint_dPP1[12] = (-Q_2[20]) # e2 ^ e0
	macro_ExtractSecondPoint_dPP1[13] = (-Q_2[17]) # e3 ^ einf
	P_2 = np.zeros(32)
	P_2[1] = (Q_2[31] - math.sqrt(math.sqrt(abs(((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) * ((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) + (-(((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24]))) * (-((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24])))))) + (Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22]) * (-((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23])))) + ((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23]))) * (-(Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22])) + (-((Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21]) * (-(Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21])))) + ((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19]))) * (-(Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20])) + (Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20]) * (-((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19])))) + (Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17]) * (-((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18])))) + ((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18]))) * (-(Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17])) + ((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16]))) * (-((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16])))))))) * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + Q_2[24] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + (-(Q_2[23] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) # e1
	P_2[2] = (Q_2[31] - math.sqrt(math.sqrt(abs(((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) * ((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) + (-(((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24]))) * (-((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24])))))) + (Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22]) * (-((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23])))) + ((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23]))) * (-(Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22])) + (-((Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21]) * (-(Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21])))) + ((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19]))) * (-(Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20])) + (Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20]) * (-((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19])))) + (Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17]) * (-((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18])))) + ((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18]))) * (-(Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17])) + ((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16]))) * (-((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16])))))))) * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + macro_ExtractSecondPoint_dPP1[10] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + (-(macro_ExtractSecondPoint_dPP1[12] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) # e2
	P_2[3] = (Q_2[31] - math.sqrt(math.sqrt(abs(((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) * ((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) + (-(((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24]))) * (-((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24])))))) + (Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22]) * (-((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23])))) + ((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23]))) * (-(Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22])) + (-((Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21]) * (-(Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21])))) + ((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19]))) * (-(Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20])) + (Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20]) * (-((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19])))) + (Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17]) * (-((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18])))) + ((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18]))) * (-(Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17])) + ((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16]))) * (-((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16])))))))) * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + (-(Q_2[24] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(macro_ExtractSecondPoint_dPP1[10] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(Q_2[18] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) # e3
	P_2[4] = (Q_2[31] - math.sqrt(math.sqrt(abs(((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) * ((-(Q_2[16] * Q_2[16])) + Q_2[17] * Q_2[18] + Q_2[18] * Q_2[17] + Q_2[19] * Q_2[20] + Q_2[20] * Q_2[19] + Q_2[21] * Q_2[21] + Q_2[22] * Q_2[23] + Q_2[23] * Q_2[22] + Q_2[24] * Q_2[24] + (-(Q_2[31] * Q_2[31]))) + (-(((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24]))) * (-((-(Q_2[24] * Q_2[31])) + (-(Q_2[31] * Q_2[24])))))) + (Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22]) * (-((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23])))) + ((-(Q_2[23] * Q_2[31])) + (-(Q_2[31] * Q_2[23]))) * (-(Q_2[22] * Q_2[31] + Q_2[31] * Q_2[22])) + (-((Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21]) * (-(Q_2[21] * Q_2[31] + Q_2[31] * Q_2[21])))) + ((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19]))) * (-(Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20])) + (Q_2[20] * Q_2[31] + Q_2[31] * Q_2[20]) * (-((-(Q_2[19] * Q_2[31])) + (-(Q_2[31] * Q_2[19])))) + (Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17]) * (-((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18])))) + ((-(Q_2[18] * Q_2[31])) + (-(Q_2[31] * Q_2[18]))) * (-(Q_2[17] * Q_2[31] + Q_2[31] * Q_2[17])) + ((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16]))) * (-((-(Q_2[16] * Q_2[31])) + (-(Q_2[31] * Q_2[16])))))))) * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + (-(macro_ExtractSecondPoint_dPP1[8] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(Q_2[19] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(macro_ExtractSecondPoint_dPP1[13] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(Q_2[16] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) # einf
	P_2[5] = (-(Q_2[23] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + (-(Q_2[18] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) # e0
	P_2[16] = (-(Q_2[24] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + macro_ExtractSecondPoint_dPP1[10] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e1 ^ (e2 ^ e3)
	P_2[17] = (-(macro_ExtractSecondPoint_dPP1[8] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + Q_2[19] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e1 ^ (e2 ^ einf)
	P_2[18] = (-(Q_2[23] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + macro_ExtractSecondPoint_dPP1[12] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e1 ^ (e2 ^ e0)
	P_2[19] = Q_2[24] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + (-(macro_ExtractSecondPoint_dPP1[8] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + macro_ExtractSecondPoint_dPP1[13] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e1 ^ (e3 ^ einf)
	P_2[20] = (-(Q_2[23] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + Q_2[18] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e1 ^ (e3 ^ e0)
	P_2[21] = (-(Q_2[23] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + Q_2[16] * Q_2[23] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e1 ^ (einf ^ e0)
	P_2[22] = macro_ExtractSecondPoint_dPP1[10] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) + (-(Q_2[19] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + macro_ExtractSecondPoint_dPP1[13] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e2 ^ (e3 ^ einf)
	P_2[23] = (-(macro_ExtractSecondPoint_dPP1[12] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + Q_2[18] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e2 ^ (e3 ^ e0)
	P_2[24] = (-(macro_ExtractSecondPoint_dPP1[12] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + Q_2[16] * macro_ExtractSecondPoint_dPP1[12] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e2 ^ (einf ^ e0)
	P_2[25] = (-(Q_2[18] * Q_2[16] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]))) + Q_2[16] * Q_2[18] / (Q_2[23] * Q_2[23] + macro_ExtractSecondPoint_dPP1[12] * macro_ExtractSecondPoint_dPP1[12] + Q_2[18] * Q_2[18]) # e3 ^ (einf ^ e0)
	L_01 = np.zeros(32)
	L_01[25] = 1.0 # e3 ^ (einf ^ e0)
	L_12 = np.zeros(32)
	L_12[19] = (-(0.1625 * P_2[1])) # e1 ^ (e3 ^ einf)
	L_12[21] = P_2[1] # e1 ^ (einf ^ e0)
	L_12[22] = (-(0.1625 * P_2[2])) # e2 ^ (e3 ^ einf)
	L_12[24] = P_2[2] # e2 ^ (einf ^ e0)
	L_12[25] = (-(0.1625 * P_2[5] + (-P_2[3]))) # e3 ^ (einf ^ e0)
	L_12[31] = (-(0.1625 * P_2[18] + (-P_2[16]))) # e1 ^ (e2 ^ (e3 ^ (einf ^ e0)))
	L_23 = np.zeros(32)
	L_23[17] = P_2[1] * P_3[2] + (-(P_2[2] * P_3[1])) # e1 ^ (e2 ^ einf)
	L_23[19] = P_2[1] * P_3[3] + (-(P_2[3] * P_3[1])) # e1 ^ (e3 ^ einf)
	L_23[21] = (-(P_2[1] * P_3[5] + (-(P_2[5] * P_3[1])))) # e1 ^ (einf ^ e0)
	L_23[22] = P_2[2] * P_3[3] + (-(P_2[3] * P_3[2])) # e2 ^ (e3 ^ einf)
	L_23[24] = (-(P_2[2] * P_3[5] + (-(P_2[5] * P_3[2])))) # e2 ^ (einf ^ e0)
	L_23[25] = (-(P_2[3] * P_3[5] + (-(P_2[5] * P_3[3])))) # e3 ^ (einf ^ e0)
	L_23[31] = (-(P_2[3] * P_3[18] + (-(P_2[5] * P_3[16])) + P_2[16] * P_3[5] + (-(P_2[18] * P_3[3])) + P_2[20] * P_3[2] + (-(P_2[23] * P_3[1])))) # e1 ^ (e2 ^ (e3 ^ (einf ^ e0)))
	P_0 = np.zeros(32)
	P_0[5] = 1.0 # e0
	
	multivectors = open("multivectors.csv", "w")

	multivectors.write(multivector_to_string("point", "P_0", P_0))
	multivectors.write(multivector_to_string("point", "P_1", P_1))
	multivectors.write(multivector_to_string("point", "P_2", P_2))
	multivectors.write(multivector_to_string("point", "P_3", P_3))
	multivectors.write(multivector_to_string("point", "P_4", P_4))
	multivectors.write(multivector_to_string("point", "P_5", P_5))
	multivectors.write(multivector_to_string("point", "P_e", P_e))
	multivectors.write(multivector_to_string("point", "P_c", P_c))

	multivectors.write(multivector_to_string("sphereIPNS", "S_c", S_c))
	multivectors.write(multivector_to_string("sphereIPNS", "K_0", K_0))
	multivectors.write(multivector_to_string("circleIPNS", "C_5k", C_5k))
	multivectors.write(multivector_to_string("ppIPNS", "Q_c", Q_c))

	multivectors.write(multivector_to_string("planeOPNS", "PI_c", PI_c))
	multivectors.write(multivector_to_string("planeIPNS", "PI_c_parallel", PI_c_parallel))
	multivectors.write(multivector_to_string("planeOPNS", "PI_56_orthogonal", PI_56_orthogonal))
	multivectors.write(multivector_to_string("planeOPNS", "PI_c_orthogonal", PI_c_orthogonal))

	multivectors.write(multivector_to_string("lineIPNS", "L_45", L_45))

	multivectors.write(multivector_to_string("sphereIPNS", "S_5", S_5))
	multivectors.write(multivector_to_string("ppIPNS", "Q_4", Q_4))

	multivectors.write(multivector_to_string("sphereIPNS", "S_4", S_4))
	multivectors.write(multivector_to_string("lineOPNS", "L_34", L_34))
	multivectors.write(multivector_to_string("ppIPNS", "Q_3", Q_3))

	multivectors.write(multivector_to_string("sphereIPNS", "S_1", S_1))
	multivectors.write(multivector_to_string("sphereIPNS", "S_3", S_3))
	multivectors.write(multivector_to_string("circleIPNS", "C_2", C_2))
	multivectors.write(multivector_to_string("ppIPNS", "Q_2", Q_2))

	multivectors.write(multivector_to_string("lineOPNS", "L_01", L_01))
	multivectors.write(multivector_to_string("lineOPNS", "L_12", L_12))
	multivectors.write(multivector_to_string("lineOPNS", "L_23", L_23))

	multivectors.write(multivector_to_string("normalvector", "n_c", n_c))

def calculate_endeffector_rotation_matrix(alpha, beta, gamma):
	matrix = [
		[math.cos(alpha) * math.cos(beta), math.cos(alpha) * math.sin(beta) * math.sin(gamma) - math.sin(alpha) * math.cos(gamma), math.cos(alpha) * math.sin(beta) * math.cos(gamma) + math.sin(alpha) * math.sin(gamma)],
		[math.sin(alpha) * math.cos(beta), math.sin(alpha) * math.sin(beta) * math.sin(gamma) + math.cos(alpha) * math.cos(gamma), math.sin(alpha) * math.sin(beta) * math.cos(gamma) - math.cos(alpha) * math.sin(gamma)],
		[-math.sin(beta), math.cos(beta) * math.sin(gamma), math.cos(beta) * math.cos(gamma)]
	]

	normal_vector = [matrix[0][0], matrix[1][0], matrix[2][0]]
	slide_vector = [matrix[0][1], matrix[1][1], matrix[2][1]]
	approach_vector = [matrix[0][2], matrix[1][2], matrix[2][2]]

	return normal_vector, slide_vector, approach_vector

def calculate_angles(x, y):
	theta1 = np.arctan2(y[0], x[0])
	theta2 = np.arctan2(y[1], x[1] - (np.pi / 2))
	theta3 = np.arctan2(y[2], x[2])
	theta4 = np.arctan2(y[3], x[3] - (np.pi / 2))
	theta5 = np.arctan2(y[4], x[4])
	theta6 = np.arctan2(y[5], x[5])
	return theta1, theta2, theta3, theta4, theta5, theta6

with open('poses.txt', 'r') as read_obj:
	
	csv_reader = reader(read_obj)

	f = open("angles.csv", "w")

	for row in csv_reader:
		x = np.zeros(6)
		y = np.zeros(6)

		normal_vector, slide_vector, approach_vector = calculate_endeffector_rotation_matrix(float(row[0]), float(row[1]), float(row[2]))

		inverse_kinematics_ur5e(approach_vector[0], approach_vector[1], -approach_vector[2], slide_vector[0], slide_vector[1], slide_vector[2], float(row[3]), float(row[4]), float(row[5]))
