import cv2
import numpy as np
import rospy

class tracker():
    def __init__(self,x,y):
        self.kf = cv2.KalmanFilter(4,2,0) # Kalman filter with states[x,y,dx,dy] and measurements x,y and no control

        #Define the matrices of the Kalman Filter
        self.kf.processNoiseCov=np.array([  [1e-2, 0, 0, 0],
                                            [0, 1e-2, 0, 0],
                                            [0, 0, 5, 0],
                                            [0, 0, 0, 5]],dtype=np.float32) # noise values from https://github.com/Myzhar/simple-opencv-kalman-tracker/blob/master/source/opencv-kalman.cpp
        cv2.setIdentity(self.kf.measurementNoiseCov, 1e-1)
        self.kf.measurementMatrix=np.array([    [1, 0, 0, 0],
                                                [0, 1, 0, 0]],dtype=np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                        [0, 1, 0, 1],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]],np.float32)
        cv2.setIdentity(self.kf.errorCovPost, 1)

        #Define State and measurement vector
        self.state=np.zeros(4, dtype=np.float32)
        self.ms = np.zeros(2, dtype=np.float32)


    
    def predict(self,detectionStamp):
        '''This function is supposed to return the predicted state, such that the tracker can be assigned to the correct detection using hungarian method'''
        dt=detectionStamp-self.timestamp    #calculate the timedelta between the last prediction and this one

        self.kf.transitionMatrix[0,2] = dt  #update dt of kalman filter
        self.kf.transitionMatrix[1,3] = dt

        self.state = self.kf.predict()      #predict the current position

        return self.state                   #return predicted state

    def update(self,x,y,detectionStamp):
        
        if self.initFlag==False:
            #init kalman filter with first measurment so it doesn start at 0
            self.kf.statePost = np.array([x, y, 0, 0], dtype=np.float32)
            self.kf.statePre = np.array([x, y, 0, 0], dtype=np.float32)

            self.kf.errorCovPre=np.eye(4)
            self.initFlag=False
            #init tracker with its creation time
            self.timestamp=rospy.Time.now()

        else:
            '''After successful assignment update the tracker'''
            dt=detectionStamp-self.timestamp    #calculate the timedelta between the last prediction and this one   

            self.timestamp=detectionStamp       #update the timestamp of this detector

            self.kf.transitionMatrix[0,2] = dt  #update dt of kalman filter
            self.kf.transitionMatrix[1,3] = dt

            self.state = self.kf.predict()      #predict the current position

            self.ms = np.float32([x,y])
            self.kf.correct(self.ms)            #update the tracker with the current measurement

        return self.kf.statePost            #return updated current state

        