import numpy as np
from numpy.linalg import inv


class KalmanF(object):
    def __init__ (self,dt,s_frame,u,acc_noise,measurement_noise,ez,ex,p,a,b,c ):
        
        self.dt=dt #Kalman step process
        self.s_frame=s_frame #starting frame
        self.u=u # magnitude accelration start
        self.acc_nois= acc_noise # accelration noise
        self.measurement_noise=measurement_noise
        self.ez=ez
        self.ex= ex
        self.p=p
        self.a=a  #starte update matrix
        self.b= b
        self.c=c
        self.q_estimate=np.zeros([2000,4])
        
    def predictK(self,tracke):
        
        self.q_estimate[tracke,:] = np.dot(self.a, self.q_estimate[tracke,:] ) + self.b.T * self.u
        
    def calcKalmanGain(self):
        
        return self.p * self.c.T *inv(self.c*self.p*self.c.T+self.ez) 
    
    
    def calcCoVariance(self):
        self.p =self.a*self.p*self.a.T + self.ex
    
    def kalmanEstUpdate(self,meas,m_ind,est_ind,k):
        
        y= np.matrix(meas[m_ind,:]).T -  self.c*np.matrix(self.q_estimate[est_ind,:]).T
        self.q_estimate[est_ind,:]= np.matrix(np.matrix(self.q_estimate[est_ind,:]).T+k*y).T
        self.q_estimate[est_ind,0:2]=np.ceil(self.q_estimate[est_ind,0:2])
        
    def kalmanCoVarianceUpdate(self,k):
        self.p=(np.eye(4)-k*self.c)*self.p