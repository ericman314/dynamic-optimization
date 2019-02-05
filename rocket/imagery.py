from direct.showbase.ShowBase import ShowBase
from direct.showbase.Loader import Loader
from panda3d.core import Geom, GeomVertexData, GeomVertexFormat, GeomVertexWriter, GeomTriangles, GeomNode, NodePath, LODNode


def loadScenery(nodePath):


  imageryCoords = [[ -46.52, -47.76, 92.32, 92.32, 1000, 0 ],
                    [ -884.0, -256.9, 1456.7, 745.2, 20000, 0 ],
                    [ -2889, -1830, 5403, 3586, 50000, -1 ],
                    [ -25264,	-9341,	42873,	19580, 100000, -5 ],
                    [ -152996, -72479, 292008, 143725, 10000000, -100 ],
                    [ -704530, -320217, 1303922, 643019, 10000000, -300 ]]


  for i in range(1,6):
      

    terrainData = GeomVertexData('terrain1', GeomVertexFormat.getV3t2(), Geom.UHStatic)
    terrainData.setNumRows(4)
    terrainDataVertexWriter = GeomVertexWriter(terrainData, 'vertex')
    terrainDataTexcoordWriter = GeomVertexWriter(terrainData, 'texcoord')

    x = imageryCoords[i][0]
    y = imageryCoords[i][1]
    w = imageryCoords[i][2]
    h = imageryCoords[i][3]
    l = imageryCoords[i][4]
    z = imageryCoords[i][5]
    
    # To make a curved surface, loop over i,j and subdivide the triangles
    terrainDataVertexWriter.addData3f(x, y, z)
    terrainDataVertexWriter.addData3f(x + w, y, z)
    terrainDataVertexWriter.addData3f(x + w, y + h, z)
    terrainDataVertexWriter.addData3f(x, y + h, z)

    terrainDataTexcoordWriter.addData2f(0, 0)
    terrainDataTexcoordWriter.addData2f(1, 0)
    terrainDataTexcoordWriter.addData2f(1, 1)
    terrainDataTexcoordWriter.addData2f(0, 1)

    prim = GeomTriangles(Geom.UHStatic)
    # Change rotation if triangles face the wrong way
    prim.addVertices(0, 1, 2)
    prim.addVertices(0, 2, 3)
    prim.close_primitive()

    geom = Geom(terrainData)
    geom.addPrimitive(prim)

    node = GeomNode('node1')
    node.addGeom(geom)

    # This is required to make the scenery fit nicely within the depth buffer (can't figure out how to change the depth buffer)
    # nodePath.setScale(0.05,0.05,0.05)
  
    loader = Loader(0)
    texture = loader.loadTexture('imagery/lz1-scale' + str(i) + '.png')
    path = NodePath(node)
    path.setTexture(texture)
    
    lod = LODNode('lod node')
    lod.addSwitch(l, 0.0)
    lod_np = NodePath(lod)

    path.reparentTo(lod_np)

    lod_np.reparentTo(nodePath)
    
    
