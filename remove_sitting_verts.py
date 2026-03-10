import bpy
import bmesh
import math
import mathutils
import numpy as np
import time

class ParentEdgeInfo:
    setSittingVertCustomIndices = None
    lstSittingVerts = None
    setSubdivVertCustomIndices = None
    lstSubdivVerts = None
    referenceVertCustomIndex = 0
    def __init__(self, setSittingVertCustomIndices =None, referenceVertCustomIndex =None):
        self.setSittingVertCustomIndices = setSittingVertCustomIndices
        self.referenceVertCustomIndex = referenceVertCustomIndex
        self.setSubdivVertCustomIndices = set()
        self.lstSittingVerts = []
        self.lstSubdivVerts = []

##################################################################################################


def get_edge_length(edg):

    vrt0, vrt1 = edg.verts
    return (vrt0.co - vrt1.co).length
##################################################################################


def get_vert_cell(vrt, cell_size):
    
    coords = vrt.co
    x = math.floor(coords.x / cell_size)
    y = math.floor(coords.y / cell_size)
    z = math.floor(coords.z / cell_size)
    return (x,y,z)
########################################################################################################


def get_dict_of_cells_and_verts(bm, lstVerts, cell_size):
    print("Running get_dict_of_cells_and_verts()")
    
    dictOfCells = {}
    setOfCells = set()
    custom_vert_index = bm.verts.layers.int.get("Custom_Vert_Index")
    
    max_customIndex = 0
    for vrt in lstVerts:
        vrt.select = False
        max_customIndex += 1
        vrt[custom_vert_index] = max_customIndex
        cell = get_vert_cell(vrt,cell_size)
        
        if cell in setOfCells:
            dictOfCells[cell].add(vrt)
        else:
            dictOfCells[cell] = set([vrt])
            setOfCells.add(cell)
    
    print("After first assignment of custom index, the max_customIndex is:", max_customIndex)
    return dictOfCells, setOfCells, max_customIndex
###############################################################################################################


def get_aabb_of_edge(edg, min_dist):
    
    vrt0, vrt1 = edg.verts
    coords = [vrt0.co, vrt1.co]
    
    x_min_index = 0 if coords[0].x < coords[1].x else 1
    x_max_index = abs(x_min_index - 1)
    y_min_index = 0 if coords[0].y < coords[1].y else 1
    y_max_index = abs(y_min_index - 1)
    z_min_index = 0 if coords[0].z < coords[1].z else 1
    z_max_index = abs(z_min_index - 1)
    
    aabb_min = mathutils.Vector([coords[x_min_index].x-min_dist, 
                                 coords[y_min_index].y-min_dist, 
                                 coords[z_min_index].z-min_dist])
    aabb_max = mathutils.Vector([coords[x_max_index].x+min_dist, 
                                 coords[y_max_index].y+min_dist, 
                                 coords[z_max_index].z+min_dist])
    return aabb_min, aabb_max
###############################################################################################################


def get_distance_of_vert_to_edge(edg,vrt):
    
    vrtA, vrtB = edg.verts
    A = vrtA.co
    B = vrtB.co
    P = vrt.co - A
    direction = (B-A).normalized()
    P_dir = P.dot(direction)
    
    if P_dir <= 0:
        return P.length
    if P_dir >= (B-A).length:
        return (vrt.co-B).length
    
    dist = (P - P_dir*direction).length
    return dist
######################################################################################################################


def get_edge_sitting_verts_custom_indices(bm, edg, dictOfCells, setOfCells, cell_size, min_dist):
    
    aabb_min, aabb_max = get_aabb_of_edge(edg,min_dist)
    aabb_min_cell = (math.floor(aabb_min.x / cell_size), 
                    math.floor(aabb_min.y / cell_size), 
                    math.floor(aabb_min.z / cell_size))
    aabb_max_cell = (math.ceil(aabb_max.x / cell_size), 
                    math.ceil(aabb_max.y / cell_size), 
                    math.ceil(aabb_max.z / cell_size))
    
    setSittingVertCustomIndices = set()
    custom_vert_index = bm.verts.layers.int.get("Custom_Vert_Index")

    for i in range(aabb_min_cell[0],aabb_max_cell[0]):
        for j in range(aabb_min_cell[1],aabb_max_cell[1]):
            for k in range(aabb_min_cell[2],aabb_max_cell[2]):
                cell = (i,j,k)                                  #These are all cells that intersect the AABB
                if not cell in setOfCells:
                    continue
                for vrt in dictOfCells[cell]:
                    if vrt in edg.verts:
                        continue
                    dist = get_distance_of_vert_to_edge(edg,vrt)
                    if dist < min_dist:
                        setSittingVertCustomIndices.add(vrt[custom_vert_index])

    return setSittingVertCustomIndices
###################################################################################################################


def find_sitting_verts(bm, setEdges, lstVerts, min_dist):
    print("Running find_sitting_verts()")

    lstEdgeLengths = [get_edge_length(edg) for edg in setEdges]
    cell_size = np.median(np.array(lstEdgeLengths))
    
    dictOfCells, setOfCells, max_customIndex = get_dict_of_cells_and_verts(bm, lstVerts, cell_size)
    dictOfNumbersOfSittingVerts = {}
    
    totalNumSittingVerts = 0
    numParentEdges = 0

    parent_edge_index = bm.edges.layers.int.get("Parent_Edge_Index")
    num_edge_sitting_verts = bm.edges.layers.int.get("Edge_Sitting_Verts_Number")
    custom_vert_index = bm.verts.layers.int.get("Custom_Vert_Index")

    for edg in bm.edges:
        if not edg in setEdges:
            edg[num_edge_sitting_verts] = 0
            continue
        
        setSittingVertCustomIndices = get_edge_sitting_verts_custom_indices(bm, edg, dictOfCells, setOfCells, cell_size, min_dist)
        numSittingVerts = len(setSittingVertCustomIndices)
        
        if numSittingVerts == 0:
            edg[num_edge_sitting_verts] = 0
            continue
        
        edg[num_edge_sitting_verts] = numSittingVerts
        parentEdgeIndex = edg.index
        edg[parent_edge_index] = parentEdgeIndex
        totalNumSittingVerts += numSittingVerts
        numParentEdges += 1
        vrt0, vrt1 = edg.verts
        if numSittingVerts in dictOfNumbersOfSittingVerts.keys():
            dictOfParentEdgeIndices = dictOfNumbersOfSittingVerts[numSittingVerts]
            dictOfParentEdgeIndices[parentEdgeIndex] = ParentEdgeInfo(setSittingVertCustomIndices,vrt0[custom_vert_index])
        else:
            newDictOfParentEdgeIndices = {}
            newDictOfParentEdgeIndices[parentEdgeIndex] = ParentEdgeInfo(setSittingVertCustomIndices,vrt0[custom_vert_index])
            dictOfNumbersOfSittingVerts[numSittingVerts] = newDictOfParentEdgeIndices
    
    print("Found ", totalNumSittingVerts, " sitting Verts on ", numParentEdges, " Edges.")
    return dict(sorted(dictOfNumbersOfSittingVerts.items())), max_customIndex
###################################################################################################################
            

def subdivide_edges(bm, dictOfNumbersOfSittingVerts, max_customIndex):
    print("Running subdivide_edges()")

    num_edge_sitting_verts = bm.edges.layers.int.get("Edge_Sitting_Verts_Number")

    for num, dictOfParentEdgeIndices in dictOfNumbersOfSittingVerts.items():
        lstEdgesToDivide = [edg for edg in bm.edges if edg[num_edge_sitting_verts] == num]
        
        print("Now subdividing ", len(lstEdgesToDivide), " Edges with ", num, " Cuts each.")
        subdiv_result = bmesh.ops.subdivide_edges(bm,edges=lstEdgesToDivide,cuts=num)
        
        lstNewEdgesFromSubdiv = [item for item in subdiv_result["geom_split"] if isinstance(item,bmesh.types.BMEdge)]
        setNewVertsFromSubdiv = set(item for item in subdiv_result["geom_inner"] if isinstance(item,bmesh.types.BMVert))

        max_customIndex = update_parentEdge_infos(bm,dictOfParentEdgeIndices,lstNewEdgesFromSubdiv,setNewVertsFromSubdiv,max_customIndex)

    print("After second assignment of custom index, the max_customIndex is:", max_customIndex)
    return max_customIndex
######################################################################################################################


def update_parentEdge_infos(bm, dictOfParentEdgeIndices, lstNewEdgesFromSubdiv, setNewVertsFromSubdiv, max_customIndex):

    parent_edge_index = bm.edges.layers.int.get("Parent_Edge_Index")
    custom_vert_index = bm.verts.layers.int.get("Custom_Vert_Index")
    pre_subdiv_max_customIndex = max_customIndex

    for edg in lstNewEdgesFromSubdiv:
        parentEdgeIndex = edg[parent_edge_index]
        if not parentEdgeIndex in dictOfParentEdgeIndices.keys():
            print("Warning in subdivide_edges(): Found new Edge with no Ineritance from Parent-Edges!")
            edg.select = True                               #select problematic Edges, which will not be cured by this program.
            continue

        info = dictOfParentEdgeIndices[parentEdgeIndex]
        for vrt in edg.verts:
            vrt_customIndex = vrt[custom_vert_index]
            if (vrt_customIndex > pre_subdiv_max_customIndex) or (not vrt in setNewVertsFromSubdiv):
                continue
            else:
                max_customIndex += 1
                vrt[custom_vert_index] = max_customIndex
                info.setSubdivVertCustomIndices.add(max_customIndex)
        
    return max_customIndex
##########################################################################################################################


def order_sitting_verts(bm, dictOfNumbersOfSittingVerts):
    print("Running order_sitting_verts()")

    custom_vert_index = bm.verts.layers.int.get("Custom_Vert_Index")
    dictOfCustomVertIndices = {}
    for v in bm.verts:
        dictOfCustomVertIndices[v[custom_vert_index]] = v
    
    dictOfVertsToMerge = {}

    for num, dictOfParentEdgeIndices in dictOfNumbersOfSittingVerts.items():
        for info in dictOfParentEdgeIndices.values():

            if len(info.setSubdivVertCustomIndices) != num:
                print("Warning in merge_verts(): Found Edge with Number of Sitting Verts ,", num, ", not matching Number of New Verts, ", len(info.lstNewVerts), "!")
                continue

            refVert = dictOfCustomVertIndices[info.referenceVertCustomIndex]
            info.lstSittingVerts = [dictOfCustomVertIndices[i] for i in info.setSittingVertCustomIndices]
            info.lstSubdivVerts = [dictOfCustomVertIndices[i] for i in info.setSubdivVertCustomIndices]
            info.lstSittingVerts.sort(key = lambda v: (v.co - refVert.co).length_squared)
            info.lstSubdivVerts.sort(key = lambda v: (v.co - refVert.co).length_squared)

            update_dictOfVertsToMerge(num, dictOfVertsToMerge, info.lstSittingVerts, info.lstSubdivVerts)
                
    return dictOfVertsToMerge
############################################################################################################################


def update_dictOfVertsToMerge(num, dictOfVertsToMerge, lstSittingVerts, lstSubdivVerts):

    for i in range(0, num):
        vrtToStay = lstSittingVerts[i]
        vrtToMerge = lstSubdivVerts[i]

        if vrtToStay in dictOfVertsToMerge.keys():
            dictOfVertsToMerge[vrtToStay].append(vrtToMerge)
        else:
            newList = [vrtToMerge]
            dictOfVertsToMerge[vrtToStay] = newList
    
    return
############################################################################################################################


def merge_sitting_verts(bm, dictOfVertsToMerge):
    print("Running merge_sitting_verts()")

    numMergedVerts = 0

    for vrtToStay, lstVertsToMerge in dictOfVertsToMerge.items():
        bmesh.ops.pointmerge(bm,verts=[vrtToStay] + lstVertsToMerge,merge_co=vrtToStay.co)
        numMergedVerts += len(lstVertsToMerge)

    return numMergedVerts
###############################################################################################################################


def remove_sitting_verts(bm, min_dist):
    print("Running remove_sitting_verts()")
    startTime = time.perf_counter()

    custom_vert_index = bm.verts.layers.int.new("Custom_Vert_Index")
    parent_edge_index = bm.edges.layers.int.new("Parent_Edge_Index")
    num_edge_sitting_verts = bm.edges.layers.int.new("Edge_Sitting_Verts_Number")

    setEdges = set(edg for edg in bm.edges if edg.select)
    lstVerts = [vrt for vrt in bm.verts if vrt.select]

    for face in bm.faces:
        face.select = False

    dictOfNumbersOfSittingVerts, max_customIndex = find_sitting_verts(bm,setEdges,lstVerts,min_dist)
    subdivide_edges(bm,dictOfNumbersOfSittingVerts, max_customIndex)
    dictOfVertsToMerge = order_sitting_verts(bm,dictOfNumbersOfSittingVerts)
    numMergedVerts = merge_sitting_verts(bm,dictOfVertsToMerge)

    endTime = time.perf_counter()
    elapsedTime = endTime - startTime
    print("merged", numMergedVerts, "Verts in elapsedTime_2:", elapsedTime)

    bm.verts.layers.int.remove(custom_vert_index)
    bm.edges.layers.int.remove(parent_edge_index)
    bm.edges.layers.int.remove(num_edge_sitting_verts)
    
    return numMergedVerts
######################################################################################################################


def main(self, context):
    print("Running main()")

    if context.mode != 'EDIT_MESH':
        raise TypeError("Error! This Operation works only in Edit Mode!")
    
    min_dist = context.scene.distance_tolerance
    numMergedVerts = 0

    if len(context.selected_objects) == 0:
        raise TypeError("Error: No Object is selected!")

    for object in context.selected_objects:
        if object.type != 'MESH':
            self.report({'WARNING'},"Object ",object.name," is not of type 'MESH'!")
            continue
        mesh = object.data
        bm = bmesh.from_edit_mesh(mesh)
#For this operator to work properly, its important to remove double verts with distance less than the min_dist used in find_sitting_verts().
#Otherwhise it may appear as if not all sitting verts where removed.
#Note the Factor 1.1 here. That is due to inaccuraciy in Blender's remove_doubles() function.
#With min_dist = 0.0001, it misses points of distance 0.000095 (probably rounds up to 0.0001).
        bmesh.ops.remove_doubles(bm, verts = bm.verts, dist = 1.1*min_dist)
        numMergedVerts += remove_sitting_verts(bm, min_dist)
        bmesh.update_edit_mesh(mesh)
        bm.free()
    
    self.report({'INFO'},"Removed " +str(numMergedVerts) +" Sitting Verts")
    return
####################################################################################################################


class MESH_OT_remove_sitting_verts(bpy.types.Operator):
    bl_idname = "mesh.remove_sitting_verts"
    bl_label = "Remove Sitting Verts"
    bl_options = {'REGISTER','UNDO'}

    def execute(self, context):
        main(self,context)
        return {'FINISHED'}
####################################################################################################################

#The panel class is only needed if this operator shall be used as a stand alone script. Not if it is part of an add on package. Then only the operator class is needed.
# class PANEL_remove_sitting_verts(bpy.types.Panel):
#      bl_idname = "panel_remove_sitting_verts"
#      bl_label = "Remove Sitting Verts"
#      bl_space_type = "VIEW_3D"
#      bl_region_type = "UI"
#      bl_category = "Moritz Tools"

#      def draw(self,context):
#          layout = self.layout
#          layout.operator("mesh.remove_sitting_verts", text="Remove Sitting Verts")


def register():
#    bpy.utils.register_class(PANEL_remove_sitting_verts)
    bpy.utils.register_class(MESH_OT_remove_sitting_verts)

def unregister():
    bpy.utils.unregister_class(MESH_OT_remove_sitting_verts)
#    bpy.utils.unregister_class(PANEL_remove_sitting_verts)

#register()