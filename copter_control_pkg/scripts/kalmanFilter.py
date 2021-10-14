#coding=utf-8

import numpy as np
from math import sin, cos, tan
import time 
import matplotlib.pyplot as plt

class KalmanFilterComplex():
    def __init__(self):
        self.initPatam()

    def initPatam(self):    
        H1 = np.eye(3)
        O = np.zeros((3,12))
        H = []
        covar = [0.5, 0.5, 0.5] # диагональные элементы ковариационной матрицы измерений
        q_diag = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]# диагональные элементы ковариационной матрицы шумов
        R = np.eye(3)

        for i in range(H1.shape[0]):
            H.append(H1[i].tolist()+O[i].tolist())

        for i in range(len(covar)):
            R[i,i] = covar[i]

        self.H = np.array(H)
        self.R = np.array(R)
        self.Q = np.eye(6)
        for i in range(self.Q.shape[0]):
            self.Q[i][i] = q_diag[i]


        self.a = 6378245
        self.b = 6356863
        self.e_2 = 0.0066934216
        self.u = 7.2922115*10**(-5)

        self.xErr = [0]*15 # вектор состояния модели 1х15 
        self.P = np.eye(15)

        self.dt = 0.01
        self.f = 55.773037 # широта начала движения

    def get_r1(self, f):
        return self.a*(1-self.e_2)*(1-self.e_2*sin(f)**2)**(-3/2)

    def get_r2(self, f):
        return self.a*(1-self.e_2*sin(f)**2)**(-1/2)

    def C_mat(self, p, r, y):
        matrix = np.array([[cos(p)*cos(y), -cos(r)*cos(y)*sin(p) + sin(r)*sin(y), sin(r)*cos(y)*sin(p) + cos(r)*sin(y)],
                        [sin(p),         cos(r)*cos(p),                        -sin(r)*cos(p)                      ],
                        [-cos(p)*sin(y), cos(r)*sin(y)*sin(p) + sin(r)*cos(y), -sin(r)*sin(y)*sin(p)+ cos(r)*cos(y)]])

        return matrix

    def G_mat(self, r, p, y):
        G = []

        c = self.C_mat(r, p, y)

        for i in range(3):
            G.append([0]*6)
        
        for i in range(3):
            G.append(c[i].tolist() + [0]*3)

        for i in range(3):
            G.append([0]*3 + c[i].tolist())

        for i in range(6):
            G.append([0]*6)
        
        G = np.array(G)

        return G

    def get_dynamic_matrix(self, delVh, delVn, delVe, f, r, p, y):
        ed = np.eye(3).tolist()
        zer = np.zeros((3,3)).tolist()

        dt = self.dt
        u = self.u

        r1 = self.get_r1(f)
        r2 = self.get_r2(f)

        Fins1 = np.eye(3)*dt
        Fins1 = Fins1.tolist()

        Fins2 = np.array([[1-(delVh*dt)/(r1),                  -(delVn*dt)/r1,  (-(delVe*tan(f))/r2 - 2*u*sin(f))*dt],
                        [-(delVn*tan(f))/r2*dt,               1                   ,  (delVe/r2 - 2*u*cos(f))*dt          ],
                        [((delVe*tan(f))/r2 + 2*u*sin(f))*dt, 2*u*cos(f)*dt       ,   1 - (delVn/r2 - delVh/r1)*dt]]).tolist()

        Fins3 = self.C_mat(p, r, y)*dt
        Fins4 = self.C_mat(p, r, y)*dt

        Fins3 = Fins3.tolist()
        Fins4 = Fins4.tolist()

        F = []
        for i in range(3):
            F.append(ed[i]+Fins1[i]+zer[i]+zer[i]+zer[i])
        for i in range(3):
            F.append(zer[i]+Fins2[i]+zer[i]+Fins3[i]+zer[i])
        for i in range(3):
            F.append(zer[i]+zer[i]+ed[i]+zer[i]+Fins4[i])
        for i in range(3):
            F.append(zer[i]+zer[i]+zer[i]+zer[i]+zer[i])
        for i in range(3):
            F.append(zer[i]+zer[i]+zer[i]+zer[i]+zer[i])
        
        F = np.array(F)
        return F
    
    def kalmanUpdate(self, measure, rpy):
        self.f = self.f + (self.xErr[0])/self.get_r1(self.f)
        F = self.get_dynamic_matrix(self.xErr[4], self.xErr[3], self.xErr[5], self.f, *rpy)

        xErr_pred = F.dot(self.xErr)

        F_P = F.dot(self.P)
        F_trans = np.transpose(F)
        G = self.G_mat(*rpy)
        G_trans = G.transpose()
        G_Q = G.dot(self.Q)

        P_pred = F_P.dot(F_trans) + G_Q.dot(G_trans)

        H_P_pred = self.H.dot(P_pred)
        H_trans = self.H.transpose()
        H_P_pred_H_trans_R = H_P_pred.dot(H_trans) + self.R
        H_P_pred_H_trans_R_inv = np.linalg.inv(H_P_pred_H_trans_R)
        P_pred_H = P_pred.dot(self.H.transpose())
        
        K = P_pred_H.dot(H_P_pred_H_trans_R_inv)

        H_xErr_pred = self.H.dot(xErr_pred)

        xErr = xErr_pred + K.dot(measure-H_xErr_pred)

        K_H = K.dot(self.H)

        P = (np.eye(15)-K_H).dot(P_pred)

        self.xErr = xErr
        self.P = P

# k_f = KalmanFilterComplex() # экхземпляр класса фильтра

# num = 1000 # количество измерений
# noize = [] 
# filt = []
# ar = []

# for i in range(num):
#     noize.append(np.random.normal(0, 0.5)) # генерация шума по нормальному распределению с мат ожижанием и отклонением
#     k_f.kalmanUpdate([noize[i], 0, 0], [0,0,0]) # обновление фильтра калманаю Подается вектор разности измерений 3х1 и вектор углов ориентации [roll, pitch, yaw]
#     filt.append(k_f.xErr[0]) # получение необходимого значени из вектора состоянияя
#     ar.append(i+1)
#     time.sleep(0.01)

# plt.plot(ar, filt)
# plt.show()