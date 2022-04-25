import pandas as pd
import numpy as np
import matplotlib.pyplot as plt



meanDist=[]
meanAngle=[]


def addAngleData(name,meanDist,meanAngle,value):
    df = pd.read_csv (name + ".csv")
    meanDist.append(df['distance'].mean())#append mean distance of this measurement
    meanAngle.append(np.degrees(df['angle'].mean()))#append mean angle of this measurement
    return meanDist,meanAngle

def addDistanceData(name,meanDist,meanAngle):
    print("--------")
    addAngleData(name + "-20deg", meanDist, meanAngle, -20)
    addAngleData(name + "-10deg", meanDist, meanAngle, -10)
    addAngleData(name + "0deg", meanDist, meanAngle, 0)
    addAngleData(name + "+10deg", meanDist, meanAngle, 10)
    addAngleData(name + "+20deg", meanDist, meanAngle, 20)
    
    return meanDist,meanAngle



meanDist,meanAngle=addDistanceData("test_0,7m", meanAngle, meanDist)
meanDist,meanAngle=addDistanceData("test_1m", meanAngle, meanDist)
meanDist,meanAngle=addDistanceData("test_1,5m", meanAngle, meanDist)
meanDist,meanAngle=addDistanceData("test_2m", meanAngle, meanDist)
meanDist,meanAngle=addDistanceData("test_3m", meanAngle, meanDist)
meanDist,meanAngle=addDistanceData("test_4m", meanAngle, meanDist)

print(meanDist)
print(meanAngle)