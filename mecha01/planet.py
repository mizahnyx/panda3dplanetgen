from array import array
from random import random
from panda3d.core import Vec3, GeomTriangles, Geom, GeomVertexData
from panda3d.core import GeomVertexFormat, GeomVertexWriter, GeomNode
from panda3d.core import PointLight
from math import sin, cos, pi
from panda3d.bullet import BulletDebugNode, BulletRigidBodyNode, BulletGhostNode
from panda3d.bullet import BulletTriangleMesh, BulletTriangleMeshShape
from panda3d.bullet import BulletBoxShape, BulletSphereShape
from panda3d.core import TransparencyAttrib
from panda3d.core import Vec4

class Planet(object):
    def __init__(self):
        """
        Initializes the planet with default values.
        """
        self.__n = 32
        n1 = self.__n + 1
        self.__radius = (self.__n*2) / pi
        self.__heightmap = array('f', [0.0 for i in range(6*n1*n1)])
        self.__height_factor = 16
        self.__height_unit = pi / (2*self.__n)

        self.__color_gradient = {
            0.00: (22, 8, 64),
            0.25: (125, 196, 255),
            0.50: (240, 228, 180),
            0.75: (87, 212, 116),
            1.00: (255, 255, 255)
        }

        self.__ocean_color = Vec4(125/255.0, 196/255.0, 255/255.0, 0.75)
        self.__node_path = None
        
    def get_n(self): return self.__n
    def set_n(self, n): 
        self.__n = n
        self.__height_unit = pi / (2*self.__n)

    def get_radius(self): return self.__radius
    def set_radius(self, radius): self.__radius = radius

    def get_gradient(self): return self.__color_gradient
    def set_gradient(self, gradient): self.__color_gradient = gradient

    def get_height(self, face, i, j):
        return self.__heightmap[self.__height_i(face, i, j)]

    def set_height(self, face, i, j, height):
        self.__heightmap[self.__height_i(face, i, j)] = height

    def get_height_factor(self):
        return self.__height_factor

    def get_height_unit(self):
        return self.__height_unit

    def set_height_factor(self, height_factor):
        self.__height_factor = height_factor

    def get_ocean_color(self):
        return self.__ocean_color

    def set_ocean_color(self, ocean_color):
        self.__ocean_color = ocean_color

    def get_node_path(self): return self.__node_path


    def perlin_terrain(self, octave_count, persistence):
        n1 = self.__n + 1
        white_noise = array('f', [random() for i in range(6*n1*n1)])
        self.__stitch_edges(white_noise)
        smooth_noise = array('f', [0.0 for i in range(6*n1*n1)])
        total_amplitude = 0.0
        amplitude = 1.0
        for octave in range(octave_count - 1, 0, -1):
            amplitude *= persistence
            total_amplitude += amplitude
            self.__smooth_noise(white_noise, smooth_noise, octave, amplitude)
        for i in range(6*n1*n1):
            smooth_noise[i] /= total_amplitude
        self.__stitch_edges(smooth_noise)
        self.__heightmap = smooth_noise

    def __build_ocean_mesh(self):
        vdata = GeomVertexData('data', GeomVertexFormat.getV3n3c4t2(),
                               Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        texcoord = GeomVertexWriter(vdata, 'texcoord')
        color = GeomVertexWriter(vdata, 'color')

        axes = [Vec3.unitX(), Vec3.unitY(), Vec3.unitZ()]
        face = 0

        for x in range(3):
            for s in [-1, 1]:
                for i in range(self.__n + 1):
                    for j in range(self.__n + 1):
                        a = (i*1.0/self.__n)*(pi/2) - (pi/4)
                        b = (j*1.0/self.__n)*(pi/2) - (pi/4)
                        xAxis = axes[(x + 3) % 3]
                        yAxis = axes[(x + 4) % 3]
                        zAxis = axes[(x + 5) % 3]
                        v = (xAxis*(-cos(a)*sin(b)) +
                             yAxis*(sin(a)*cos(b)) + 
                             zAxis*(cos(a)*cos(b))) * s
                        v.normalize()
                        normal.addData3f(v)
                        vertex.addData3f(v*self.__radius)
                        texcoord.addData2f(i*1.0, j*1.0)
                        color.addData4f(self.__ocean_color)
                face = face + 1
        prim = self.__heightmap_primitive()
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        return geom

    def __build_land_mesh(self):
        vdata = GeomVertexData('data', GeomVertexFormat.getV3n3c4t2(),
                               Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        texcoord = GeomVertexWriter(vdata, 'texcoord')
        color = GeomVertexWriter(vdata, 'color')

        axes = [Vec3.unitX(), Vec3.unitY(), Vec3.unitZ()]
        face = 0
        
        for x in range(3):
            for s in [-1, 1]:
                for i in range(self.__n + 1):
                    for j in range(self.__n + 1):
                        a = (i*1.0/self.__n)*(pi/2) - (pi/4)
                        b = (j*1.0/self.__n)*(pi/2) - (pi/4)
                        xAxis = axes[(x + 3) % 3]
                        yAxis = axes[(x + 4) % 3]
                        zAxis = axes[(x + 5) % 3]
                        v = (xAxis*(-cos(a)*sin(b)) +
                             yAxis*(sin(a)*cos(b)) + 
                             zAxis*(cos(a)*cos(b))) * s
                        v.normalize()
                        normal.addData3f(v)
                        index = self.__height_i(face, i, j)
                        v = v * (1.0 + (self.__height_factor*self.__height_unit*(self.__heightmap[index] - 0.5))) * self.__radius
                        vertex.addData3f(v)
                        texcoord.addData2f(i*1.0, j*1.0)
                        c = self.__gradient(self.__color_gradient, self.__heightmap[index])
                        color.addData4f(
                            c[0] / 255.0,
                            c[1] / 255.0,
                            c[2] / 255.0, 1.0)
                face = face + 1

        prim = self.__heightmap_primitive()
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        return geom

    def __heightmap_primitive(self):
        n1 = self.__n + 1
        n12 = n1*n1
        prim = GeomTriangles(Geom.UHStatic)

        for x in range(3):
            for s in range(2):
                for i in range(self.__n):
                    for j in range(self.__n):
                        if (s == 0):
                            prim.addVertex(x*2*n12 + s*n12 + (j + 0)*n1 + i + 0)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 0)*n1 + i + 1)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 1)*n1 + i + 1)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 0)*n1 + i + 0)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 1)*n1 + i + 1)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 1)*n1 + i + 0)
                        else:
                            prim.addVertex(x*2*n12 + s*n12 + (j + 0)*n1 + i + 0)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 1)*n1 + i + 1)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 0)*n1 + i + 1)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 0)*n1 + i + 0)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 1)*n1 + i + 0)
                            prim.addVertex(x*2*n12 + s*n12 + (j + 1)*n1 + i + 1)
        return prim

    def build_node_path(self, parent_node_path, bullet_world):
        land_geom = self.__build_land_mesh()
        ocean_geom = self.__build_ocean_mesh()
        land_mesh = BulletTriangleMesh()
        land_mesh.addGeom(land_geom)
        land_shape = BulletTriangleMeshShape(land_mesh, dynamic=False)
        ocean_shape = BulletSphereShape(self.__radius)

        land_bullet_node = BulletRigidBodyNode('land collider')
        land_bullet_node.addShape(land_shape)
        bullet_world.attachRigidBody(land_bullet_node)
        land_bullet_node_path = parent_node_path.attachNewNode(land_bullet_node)

        ocean_bullet_node = BulletGhostNode('ocean collider')
        ocean_bullet_node.addShape(ocean_shape)
        bullet_world.attachGhost(ocean_bullet_node)
        ocean_bullet_node_path = land_bullet_node_path.attachNewNode(ocean_bullet_node)

        land_node = GeomNode('land')
        land_node.addGeom(land_geom)
        land_node_path = land_bullet_node_path.attachNewNode(land_node)
        ocean_node = GeomNode('ocean')
        ocean_node.addGeom(ocean_geom)
        ocean_node_path = ocean_bullet_node_path.attachNewNode(ocean_node)
        ocean_node_path.setTransparency(TransparencyAttrib.MAlpha)

        self.__node_path = land_bullet_node_path
        land_bullet_node.setPythonTag('planet', self)
        return land_bullet_node_path
        #self.tex = loader.loadTexture('maps/noise.rgb')
        #self.nodePath.setTexture(self.tex)

    # http://devmag.org.za/2009/04/25/perlin-noise/
    def __interpolate(self, x0, x1, alpha):
        return x0*(1 - alpha) + alpha*x1

    def __smooth_noise(self, base, destination, octave, amplitude):
        n1 = self.__n + 1
        sample_period = 2**octave
        sample_frequency = 1.0 / sample_period

        for face in range(6):
            for i in range(n1):
                sample_i0 = (i / sample_period) * sample_period
                sample_i1 = (sample_i0 + sample_period) % n1
                horizontal_blend = (i - sample_i0) * sample_frequency

                for j in range(n1):
                    sample_j0 = (j / sample_period) * sample_period
                    sample_j1 = (sample_j0 + sample_period) % n1
                    vertical_blend = (j - sample_j0) * sample_frequency

                    top = self.__interpolate(
                        base[self.__height_i(face, sample_i0, sample_j0)],
                        base[self.__height_i(face, sample_i1, sample_j0)],
                        horizontal_blend)
                    bottom = self.__interpolate(
                        base[self.__height_i(face, sample_i0, sample_j1)],
                        base[self.__height_i(face, sample_i1, sample_j1)],
                        horizontal_blend)
                    v = self.__interpolate(top, bottom, vertical_blend)
                    destination[self.__height_i(face, i, j)] += v*amplitude

    def __height_i(self, face, i, j):
        n1 = self.__n + 1
        return int(face*n1*n1 + i*n1 + j)

    def __average_values(self, base, *vtxspecs):
        v = 0
        for vtxspec in vtxspecs:
            v = v + base[self.__height_i(vtxspec[0], vtxspec[1], vtxspec[2])]
        v = v / len(vtxspecs)
        for vtxspec in vtxspecs:
            base[self.__height_i(vtxspec[0], vtxspec[1], vtxspec[2])] = v

    def __stitch_edges(self, base):
        n = self.__n
        n1 = self.__n + 1

        self.__average_values(base, (1, 0, 0), (4, 0, n), (3, n, n))
        self.__average_values(base, (1, 0, n), (4, n, n), (2, 0, 0))
        self.__average_values(base, (5, 0, 0), (1, n, n), (2, 0, n))
        self.__average_values(base, (5, n, 0), (1, n, 0), (3, n, 0))
        self.__average_values(base, (0, 0, 0), (5, 0, n), (2, n, n))
        self.__average_values(base, (0, 0, n), (5, n, n), (3, 0, 0))
        self.__average_values(base, (4, 0, 0), (0, n, n), (3, 0, n))
        self.__average_values(base, (4, n, 0), (0, n, 0), (2, n, 0))
        for i in range(n1):
            self.__average_values(base, (1, 0, i), (4, i, n))
            self.__average_values(base, (5, i, 0), (1, n, n - i))
            self.__average_values(base, (0, 0, i), (5, i, n))
            self.__average_values(base, (4, i, 0), (0, n, n - i))

            self.__average_values(base, (5, 0, i), (2, i, n))
            self.__average_values(base, (3, i, 0), (5, n, n - i))
            self.__average_values(base, (4, 0, i), (3, i, n))
            self.__average_values(base, (2, i, 0), (4, n, n - i))

            self.__average_values(base, (0, i, 0), (2, n, n - i))
            self.__average_values(base, (2, 0, i), (1, i, n))
            self.__average_values(base, (1, i, 0), (3, n, n - i))
            self.__average_values(base, (3, 0, i), (0, i, n))

    def __gradient(self, gradient, value):
        keys = list(gradient.keys())
        keys.sort()
        for i in range(len(keys) - 1):
            if value >= keys[i] and value < keys[i + 1]:
                blend = (value - keys[i]) / (keys[i + 1] - keys[i])
                return (
                    self.__interpolate(gradient[keys[i]][0],
                                       gradient[keys[i + 1]][0], blend),
                    self.__interpolate(gradient[keys[i]][1],
                                       gradient[keys[i + 1]][1], blend),
                    self.__interpolate(gradient[keys[i]][2],
                                       gradient[keys[i + 1]][2], blend))
        return gradient[keys[0]]

