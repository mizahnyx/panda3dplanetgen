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
from panda3d.bullet import BulletGenericConstraint
from panda3d.core import LVector3f, Point3
from mecha01 import Planet


class Mecha01Client(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        planet = Planet(32, 16, 6, 0.5)
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
        self.world.attachRigidBody(planet.node)
        self.world.attachGhost(planet.oceanNode)

        shape = BulletBoxShape(Vec3(0.02, 0.02, 0.02))
        node = BulletRigidBodyNode('Box')
        node.setMass(1.0)
        node.addShape(shape)
        np = render.attachNewNode(node)
        np.setPos(0, 1.5, 0)
        self.np01 = np
        self.world.attachRigidBody(node)
        node.applyCentralImpulse(Vec3(0.4, 0.6, 1.0))

        self.taskMgr.add(self.physicsUpdateTask, "PhysicsUpdateTask")
        
    def physicsUpdateTask(self, task):
        dt = globalClock.getDt()
        self.world.doPhysics(dt)

        # simulating spherical gravity
        node = self.np01.getNode(0)
        pos = self.np01.getPos()
        down_vector = Vec3(0, 0, 0) - pos
        down_vector.normalize()
        gravity = LVector3f(down_vector*9.81)
        node.setGravity(gravity)

        return Task.cont

    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(
            5.0 * sin(angleRadians),
            -5.0 * cos(angleRadians), 0.0)
        self.camera.setHpr(angleDegrees, 0, 0)
        return Task.cont

app = Mecha01Client()
app.run()
