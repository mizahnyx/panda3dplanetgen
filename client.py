from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3, GeomTriangles, Geom, GeomVertexData
from panda3d.core import GeomVertexFormat, GeomVertexWriter, GeomNode
from panda3d.core import PointLight
from math import sin, cos, pi
from array import array
from direct.task import Task
from random import random
from panda3d.core import TextNode, TransformState
from direct.gui.DirectGui import *
from panda3d.core import TransparencyAttrib
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode, BulletRigidBodyNode, BulletGhostNode
from panda3d.bullet import BulletTriangleMesh, BulletTriangleMeshShape
from panda3d.bullet import BulletBoxShape, BulletSphereShape
from panda3d.bullet import BulletCapsuleShape
from panda3d.bullet import BulletGenericConstraint
from panda3d.core import LVector3f, Point3
from panda3d.core import NodePathCollection
from panda3d.core import Quat
from mecha01 import Planet, CharacterController

class Mecha01Client(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
        self.plight = PointLight('plight')
        self.pnlp = self.camera.attachNewNode(self.plight)
        render.setLight(self.pnlp)


        self.debugNode = BulletDebugNode('Debug')
        self.debugNode.showWireframe(True)
        self.debugNode.showConstraints(True)
        self.debugNode.showBoundingBoxes(False)
        self.debugNode.showNormals(False)
        self.debugNP = render.attachNewNode(self.debugNode)
        self.debugNP.show()

        self.world = BulletWorld()
        self.world.setDebugNode(self.debugNode)

        self.planet = Planet()
        self.planet.perlin_terrain(5, 0.5)
        self.planet.build_node_path(render, self.world)

        shape = BulletCapsuleShape(
            self.planet.get_height_unit() / 2,
            self.planet.get_height_unit())
        node = BulletRigidBodyNode('Capsule')
        node.addShape(shape)
        np = render.attachNewNode(node)
        np.setPos(0, 1.1, 1.1)
        self.np01 = np
        self.world.attachRigidBody(node)

        #node.applyCentralImpulse(Vec3(0.4, 0.6, 1.0))

        self.taskMgr.add(self.physicsUpdateTask, "PhysicsUpdateTask")

        self.accept('arrow_up', self.test01, ["shalala"])

    def test01(self, x):
        print x

    def gravity(self, position):
        down_vector = Vec3(0, 0, 0) - position
        down_vector.normalize()
        gravity = LVector3f(down_vector*9.81)
        
    def physicsUpdateTask(self, task):
        dt = globalClock.getDt()

        # simulating spherical gravity
        node = self.np01.getNode(0)
        pos = self.np01.getPos()
        gravity = self.gravity(pos)
        self.np01.setQuat(Quat(0, 1.1, 1.1, 0))

        self.world.doPhysics(dt)

        return Task.cont

    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(
            100.0 * sin(angleRadians),
            -100.0 * cos(angleRadians), 0.0)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

app = Mecha01Client()
app.run()
