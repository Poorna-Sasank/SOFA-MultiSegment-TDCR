# -*- coding: utf-8 -*-
import Sofa.Core
from splib3.loaders import loadPointListFromFile
import Sofa

class DataSender(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.node = kwargs["node"]
        self.robot = self.node.getChild("robot")
        self.cables = {}
        for i in range(1, 13):
            self.cables[f'cable{i}'] = self.robot.getChild(f'cable{i}').cable

    def onAnimateBeginEvent(self, event):
        cable_displacements = {}
        for cable_name, cable in self.cables.items():
            cable_displacements[cable_name] = cable.getData('displacement').value
        dispmat = list(cable_displacements.values())
        print(dispmat)

         

def createScene(rootNode):
    #TODO: Add a collision mesh
    rootNode.addObject('RequiredPlugin',
                       pluginName='SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields showInteractionForceFields '
                                    'hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=False)

    rootNode.gravity = [0, -981.0, 0]
    rootNode.dt = 1.0
    


    # FEM Model
    robot = rootNode.addChild('robot')
    robot.addObject('EulerImplicitSolver', firstOrder=True,
                     rayleighMass=0.1, rayleighStiffness=0.1)
    robot.addObject('SparseLDLSolver',
                     template="CompressedRowSparseMatrixMat3x3d")
    robot.addObject('MeshGmshLoader', name='loader', filename='/path/to/model/robot.msh')
    robot.addObject('MeshTopology', src='@loader', name='container')
    robot.addObject('MechanicalObject', name='myMechObject',
                     template='Vec3d', showObject='1')
    robot.addObject('UniformMass', totalMass=0.2)
    robot.addObject('TetrahedronFEMForceField', poissonRatio=0.3897, youngModulus=2410, computeVonMisesStress="0", showVonMisesStressPerNode="false")
    robot.addObject('BoxROI', name='ROI1',
                     box=[-20, -6, -20, 20, 4, 20], drawBoxes=True)
    robot.addObject('RestShapeSpringsForceField',
                     points='@ROI1.indices', stiffness=1e12)
    robot.addObject('LinearSolverConstraintCorrection')

    #Visual Model
    robotVisu = robot.addChild('visu')
    robotVisu.addObject('MeshSTLLoader', filename='/path/to/model/robot.stl', name="loader")
    robotVisu.addObject('OglModel', src="@loader", color=[0.0, 0.7, 0.7, 1])
    robotVisu.addObject('BarycentricMapping')

    #Cables
    '''
	: There is a problem with cable mapping with the model 
    '''
    idx = 1
    cables = []
    for i in range(1, 4):
        for j in range(1, 5):
            cable = robot.addChild(f'cable{idx}')
            cable.addObject('MechanicalObject', position=loadPointListFromFile(
                f"/path/to/cables/cs{i}_{j}.json"))
            cable.addObject('CableActuator', name='cable', indices=list(range(16)), maxPositiveDisp=5, maxDispVariation=0.5, minForce=0)
            cable.addObject('BarycentricMapping')
            cables.append(cable)
            idx += 1
            
    # Effector Goal
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=1000,
                   tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO',
                   position=[0, 170, 0])
    goal.addObject('SphereCollisionModel', radius=5)
    goal.addObject('UncoupledConstraintCorrection', defaultCompliance=1e-5)
    

    #Effector
    effector = robot.addChild('fingertip')
    effector.addObject('MechanicalObject', position=([0, 168, 0]))
    effector.addObject('PositionEffector', template='Vec3d',
                       indices=0,
                       effectorGoal="@../../goal/goalMO.position")
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
    
    
    

    rootNode.addObject(DataSender(node=rootNode))
