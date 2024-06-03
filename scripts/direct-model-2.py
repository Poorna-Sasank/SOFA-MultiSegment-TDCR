from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile

import Sofa
import Sofa.Gui

from app import *
from math import sin, cos, pi


def Finger(parentNode=None, name="Finger", rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0], fixingBox=[-10, -4, -10, 10, 4, 10]):
    finger = parentNode.addChild(name)
    eobject = ElasticMaterialObject(finger,
                                    volumeMeshFileName='/path/to/model/robot.msh',
                                    poissonRatio=0.3897,
                                    youngModulus=2410,
                                    totalMass=1.0,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName='/path/to/model/robot.stl',
                                    rotation=rotation,
                                    translation=translation)
    finger.addChild(eobject)

    FixedBox(eobject,
             doVisualization=True,
             atPositions=fixingBox)

    cables = []
    idx = 1
    for i in range(4):
        cable1 = PullingCable(eobject, name= 'segment1_cable' + str(idx), pullPointLocation=[6*cos((45 + 90*i) * pi/180), -(i+4), 6*sin((45 + 90*i) * pi/180)], cableGeometry=loadPointListFromFile(
            f"/path/to/cables/cs1_{i+1}.json"))

        cable2 = PullingCable(eobject, name= 'segment2_cable' + str(idx), pullPointLocation=[6*cos((45 + 90*i) * pi/180), -(i+8), 6*sin((45 + 90*i) * pi/180)], cableGeometry=loadPointListFromFile(
            f"/path/to/cables/cs2_{i+1}.json"))
        
        cable3 = PullingCable(eobject, name= 'segment3_cable' + str(idx), pullPointLocation=[6*cos((45 + 90*i) * pi/180), -(i+10), 6*sin((45 + 90*i) * pi/180)], cableGeometry=loadPointListFromFile(
            f"/path/to/cables/cs3_{i+1}.json"))
        
        idx += 1
        
        cables.append(cable1)
        cables.append(cable2)
        cables.append(cable3)

    controller = CableController(*cables)
    app = App(controller)
    controller.app = app

    finger.addObject(controller)
    CollisionMesh(eobject, surfaceMeshFileName='/path/to/model/robot.stl',
                  name="flexer", collisionGroup=1)


def createScene(rootNode):
    
    # rootNode.addObject('AttachBodyButtonSetting', stiffness=100)
    from stlib3.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -9.81, 0.0], dt=1.0,
               plugins=["SoftRobots", "SoftRobots.Inverse", "Sofa.Component.Topology.Container.Dynamic",
                        "Sofa.Component.AnimationLoop", "Sofa.Component.Collision.Geometry",
                        "Sofa.Component.Collision.Detection.Algorithm", "Sofa.Component.Collision.Detection.Intersection",
                        "Sofa.Component.Collision.Response.Contact", "Sofa.Component.Constraint.Lagrangian.Correction",
                        "Sofa.Component.Constraint.Lagrangian.Solver", "Sofa.Component.Engine.Select",
                        "Sofa.Component.LinearSolver.Direct", "Sofa.Component.Mapping.NonLinear", "Sofa.Component.Mass",
                        "Sofa.Component.ODESolver.Backward", "Sofa.Component.SolidMechanics.FEM.Elastic",
                        "Sofa.Component.SolidMechanics.Spring", "Sofa.Component.StateContainer", "Sofa.Component.Visual",
                        "Sofa.Component.Topology.Container.Constant", "Sofa.GL.Component.Rendering3D", "Sofa.Component.IO.Mesh"])

    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    ContactHeader(rootNode, alarmDistance=1, contactDistance=1, frictionCoef=0.08)
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver',
                       maxIterations=100000, tolerance=1e-12)

    Finger(rootNode)

    return rootNode


def main():
    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 800)

    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


if __name__ == "__main__":
    main()
