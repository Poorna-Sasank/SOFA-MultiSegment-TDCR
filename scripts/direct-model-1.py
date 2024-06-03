from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import Sofa
import Sofa.Gui

from app import *
#from math import sin, cos, pi


def Finger(parentNode=None, name="finger"):
    finger = parentNode.addChild(name)
    finger.addObject('EulerImplicitSolver', firstOrder=True, rayleighMass=0.1, rayleighStiffness=0.1)
    finger.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    finger.addObject('MeshGmshLoader', name='loader', filename='/path/to/model/robot.msh')
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='mecha', template='Vec3d', showObject='1')
    finger.addObject('UniformMass', totalMass=0.2)
    finger.addObject('TetrahedronFEMForceField', poissonRatio=0.3897, youngModulus=2410, computeVonMisesStress="0", showVonMisesStressPerNodeColorMap="false")
    finger.addObject('BoxROI', name='ROI1', box=[-20, -6, -20, 20, 4, 20], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)
    finger.addObject('LinearSolverConstraintCorrection')

    
    ##########################################
    # Visualization                          #
    ##########################################
    fingerVisu = finger.addChild('visu')
    fingerVisu.addObject('MeshSTLLoader', filename='/path/to/model/robot.stl', name="loader")
    fingerVisu.addObject('OglModel', src="@loader", template='Vec3d', color=[0.0, 0.7, 0.7, 1])
    fingerVisu.addObject('BarycentricMapping')
    
    #single segment
    cables = []
    idx = 1
    for i in range(1,5):
        cable = PullingCable(finger, pullPointLocation=[0, -20, 0], name="cable" + str(idx), cableGeometry=loadPointListFromFile(f"/path/to/cables/cable{i}.json"))
        cables.append(cable)
        idx += 1
        
    #Multi-Segment
    '''
    	To-Do: The multi-segment cable routing is not entirely accurate try to identify the problems.
    '''
    # cables = []
    # idx = 1
    # for i in range(1, 4):
    #     for j in range(1, 5):
    #         cable = PullingCable(finger, pullPointLocation=[
    #                              0, -20, 0], name="cable"+str(idx),cableGeometry=loadPointListFromFile(f"/home/comrade/sofa_codes/src/test/cables/cs{i}_{j}.json"))
    #         cables.append(cable)
    #         idx += 1
    
    
    cables[0].addObject('WriteState', name="exporter", filename='cablePos.txt', writeX=True, writeV=False, writeF=False, time='0', period='1.0')
    
    
    '''
        : Study about "Monitor"
    '''
    # cable1.addObject('Monitor', name="positions", template="Vec3d", listening="1",
    #                      indices=[-20, -6, -20, 20, 4, 20],
    #                      showPositions="1", PositionsColor="1 1 0 1",
    #                      showVelocities="0", VelocitiesColor="1 1 0 1",
    #                      showForces="0", ForcesColor="1 0 0 0.5",
    #                      showMinThreshold="0.01", sizeFactor="0.5", ExportPositions='true',
    #                      TrajectoriesPrecision="0.1", TrajectoriesColor="1 1 0 1")
    
    '''
        : Exports all the mesh nodes data
        : Study about VTKExporter too
    '''
    
    #finger.addObject('VTKExporter', listening="true", name='exporter', filename='poses.vtk', pointsDataFields='@MechanicalObject.position', exportAtEnd="1")
    
    controller = CableController(*cables)
    app = App(controller)
    controller.app = app

    finger.addObject(controller)
    
    
    CollisionMesh(finger, surfaceMeshFileName='/path/to/robot.stl',
                  name="flexer", collisionGroup=1)


def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, 0.0, -9.81], dt=1.0,
               plugins=["SoftRobots", "SoftRobots.Inverse", "Sofa.Component.Topology.Container.Dynamic",
                        "Sofa.Component.AnimationLoop", "Sofa.Component.Playback", "Sofa.Component.Collision.Geometry",
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
                       maxIterations=1000, tolerance=1e-12)

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
