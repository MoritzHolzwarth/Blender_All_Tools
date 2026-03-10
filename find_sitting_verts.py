import bpy
import bmesh
import math
import mathutils
import numpy as np


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


def get_dict_of_cells_and_verts(lstVerts, cell_size):
    print("Running get_dict_of_cells_and_verts()")
    
    dictOfCells = {}
    setOfCells = set()
    
    for vrt in lstVerts:
        vrt.select = False
        cell = get_vert_cell(vrt,cell_size)
        
        if cell in setOfCells:
            dictOfCells[cell].add(vrt)
        else:
            dictOfCells[cell] = set([vrt])
            setOfCells.add(cell)
    
    return dictOfCells, setOfCells
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
    
    
def check_cell_intersects_aabb(cell, aabb_min_cell, aabb_max_cell):
    
    if cell[0] >= aabb_min_cell[0] and cell[1] >= aabb_min_cell[1] and cell[2] >= aabb_min_cell[2]:
        if cell[0]+1 <= aabb_max_cell[0] and cell[1]+1 <= aabb_max_cell[1] and cell[2]+1 <= aabb_max_cell[2]:
            return True
    
    return False
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


def get_edge_sitting_verts(bm, edg, dictOfCells, setOfCells, cell_size, min_dist):
    
    aabb_min, aabb_max = get_aabb_of_edge(edg,min_dist)
    aabb_min_cell = (math.floor(aabb_min.x / cell_size), 
                    math.floor(aabb_min.y / cell_size), 
                    math.floor(aabb_min.z / cell_size))
    aabb_max_cell = (math.ceil(aabb_max.x / cell_size), 
                    math.ceil(aabb_max.y / cell_size), 
                    math.ceil(aabb_max.z / cell_size))
    
    setSittingVerts = set()

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
                        setSittingVerts.add(vrt)

    return setSittingVerts
###################################################################################################################


def find_sitting_verts(bm, lstEdges, lstVerts, min_dist):
    print("Running find_sitting_verts()")

    for face in bm.faces:
        face.select = False
    
    lstEdgeLengths = [get_edge_length(edg) for edg in lstEdges]
    cell_size = np.median(np.array(lstEdgeLengths))
    
    dictOfCells, setOfCells = get_dict_of_cells_and_verts(lstVerts, cell_size)
    
    setSittingVerts = set()
    for edg in lstEdges:
        setSittingVerts.update(get_edge_sitting_verts(bm, edg, dictOfCells, setOfCells, cell_size, min_dist))
    
    for vrt in setSittingVerts:
        vrt.select = True
    
    return len(setSittingVerts)
###################################################################################################################


def main(self, context):
    print("Running main()")

    if context.mode != 'EDIT_MESH':
        raise TypeError("Error! This Operation works only in Edit Mode!")
    
    min_dist = context.scene.distance_tolerance
    numSittingVerts = 0

    for object in context.selected_objects:
        if object.type != 'MESH':
            self.report({'WARNING'},"Object ",object.name," is not of type 'MESH'!")
            continue
        mesh = object.data
        bm = bmesh.from_edit_mesh(mesh)
        lstEdges = [edg for edg in bm.edges if edg.select]
        lstVerts = [vrt for vrt in bm.verts if vrt.select]
        numSittingVerts += find_sitting_verts(bm, lstEdges, lstVerts, min_dist)
        bmesh.update_edit_mesh(mesh)
        bm.free()
    
    self.report({'INFO'},"Found " +str(numSittingVerts) +" Sitting Verts")
    return
####################################################################################################################


class MESH_OT_find_sitting_verts(bpy.types.Operator):
    bl_idname = "mesh.find_sitting_verts"
    bl_label = "Find Sitting Verts"
    bl_options = {'REGISTER','UNDO'}

    def execute(self, context):
        main(self,context)
        return {'FINISHED'}
####################################################################################################################


# class PANEL_find_sitting_verts(bpy.types.Panel):
#     bl_idname = "panel_find_sitting_verts"
#     bl_label = "Find Sitting Verts"
#     bl_space_type = "VIEW_3D"
#     bl_region_type = "UI"
#     bl_category = "Moritz Tools"

#     def draw(self,context):
#         layout = self.layout
#         layout.operator("mesh.find_sitting_verts", text="Find Sitting Verts")
#######################################################################################################################        


def register():
#    bpy.utils.register_class(PANEL_find_sitting_verts)
    bpy.utils.register_class(MESH_OT_find_sitting_verts)

def unregister():
    bpy.utils.unregister_class(MESH_OT_find_sitting_verts)
#    bpy.utils.unregister_class(PANEL_find_sitting_verts)

#register()