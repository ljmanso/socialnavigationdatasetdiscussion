import sys, os, Ice

Ice.loadSlice("-I . --all Pose3D.ice")

from RoboCompPose3D import *

class Pose3DI(Pose3D):
    def __init__(self, worker):
        self.worker = worker

    def getskeletons(self, c):
        return self.worker.getskeletons()

