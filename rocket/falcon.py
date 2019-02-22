from direct.showbase.ShowBase import ShowBase
from direct.showbase.Loader import Loader
from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexWriter, GeomTriangles, GeomNode, NodePath, LODNode, Material
import math

def loadFalcon(nodePath, radius, height):

  uSteps = 32
  vSteps = 64
  
  falconData = GeomVertexData('falcon', GeomVertexFormat.getV3n3t2(), Geom.UHStatic)
  # falconData.setNumRows(4)
  falconDataVertexWriter = GeomVertexWriter(falconData, 'vertex')
  falconDataNormalWriter = GeomVertexWriter(falconData, 'normal')
  falconDataTexcoordWriter = GeomVertexWriter(falconData, 'texcoord')

  prim = GeomTriangles(Geom.UHStatic)

  i = 0

  for u in range(uSteps):
    y0 = math.sin(2 * math.pi * u / uSteps)
    x0 = math.cos(2 * math.pi * u / uSteps)
    x1 = math.cos(2 * math.pi * (u+1) / uSteps)
    y1 = math.sin(2 * math.pi * (u+1) / uSteps)

    falconDataVertexWriter.addData3f(x0*radius, y0*radius, -height*0.5)
    falconDataVertexWriter.addData3f(x1*radius, y1*radius, -height*0.5)
    falconDataVertexWriter.addData3f(0, 0, -height*0.5)

    falconDataVertexWriter.addData3f(x0*radius, y0*radius, height*0.5)
    falconDataVertexWriter.addData3f(x1*radius, y1*radius, height*0.5)
    falconDataVertexWriter.addData3f(0, 0, height*0.5)

    falconDataNormalWriter.addData3f(0, 0, -1)
    falconDataNormalWriter.addData3f(0, 0, -1)
    falconDataNormalWriter.addData3f(0, 0, -1)

    falconDataNormalWriter.addData3f(0, 0, 1)
    falconDataNormalWriter.addData3f(0, 0, 1)
    falconDataNormalWriter.addData3f(0, 0, 1)

    prim.addVertices(i, i+2, i+1)
    prim.addVertices(i+3, i+4, i+5)
    i = i + 6

    for v in range(0, vSteps):
      z0 = 1.0 * v / vSteps * height - height*0.5
      z1 = 1.0 * (v+1) / vSteps * height - height*0.5

      falconDataVertexWriter.addData3f(x0*radius, y0*radius, z0)
      falconDataVertexWriter.addData3f(x0*radius, y0*radius, z1)
      falconDataVertexWriter.addData3f(x1*radius, y1*radius, z1)
      falconDataVertexWriter.addData3f(x1*radius, y1*radius, z0)

      falconDataNormalWriter.addData3f(x0, y0, 0)
      falconDataNormalWriter.addData3f(x0, y0, 0)
      falconDataNormalWriter.addData3f(x1, y1, 0)
      falconDataNormalWriter.addData3f(x1, y1, 0)

      # falconDataTexcoordWriter.addData2f(1.0*u/uSteps, 1.0*v/vSteps)
      # falconDataTexcoordWriter.addData2f(1.0*u/uSteps, 1.0*v/vSteps)
      # falconDataTexcoordWriter.addData2f(1.0*u/uSteps, 1.0*v/vSteps)
      # falconDataTexcoordWriter.addData2f(1.0*u/uSteps, 1.0*v/vSteps)

    # Change rotation if triangles face the wrong way
      prim.addVertices(i, i+2, i+1)
      prim.addVertices(i, i+3, i+2)
      i = i + 4


  prim.close_primitive()

  geom = Geom(falconData)
  geom.addPrimitive(prim)

  node = GeomNode('node1')
  node.addGeom(geom)

  # This is required to make the scenery fit nicely within the depth buffer (can't figure out how to change the depth buffer)
  # nodePath.setScale(0.05,0.05,0.05)

  # loader = Loader(0)
  # texture = loader.loadTexture('imagery/falcon.png')
  path = NodePath(node)
  # path.setTexture(texture)
    
  
  path.reparentTo(nodePath)

  myMaterial = Material()
  myMaterial.setDiffuse(0.8)
  myMaterial.setAmbient((0.6,0.6,0.6,1))
  myMaterial.setSpecular(0.8)
  myMaterial.setShininess(4)
  
  path.setMaterial(myMaterial) #Apply the material to this nodePath
  
    
