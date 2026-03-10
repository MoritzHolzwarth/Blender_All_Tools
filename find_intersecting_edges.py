import bpy
import bmesh
import math
import mathutils
import numpy as np


def get_edge_length(edg):

    vrt0, vrt1 = edg.verts
    return (vrt0.co - vrt1.co).length
##################################################################################


#This Fuction checks if two edges intersect, 
#but only in a way where the point of intersection is withing the open intervalls of both edges.
#Cases where the interection point is an end point of an edge,
#or where the two edges are parallel, are considered non-intersecting in this Function.
def check_edges_intersect(edg1, edg2, min_dist):

    vrtP1, vrtQ1 = edg1.verts
    vrtP2, vrtQ2 = edg2.verts
    
    if (vrtP1 in [vrtP2, vrtQ2]) or (vrtQ1 in [vrtP2, vrtQ2]):
        return False
    
    p1 = vrtP1.co; q1 = vrtQ1.co
    p2 = vrtP2.co; q2 = vrtQ2.co

    v_q1p1 = q1-p1
    v_q2p2 = q2-p2
    v_p2p1 = p2-p1
    
    d_q1p1_q1p1 = v_q1p1.dot(v_q1p1)
    d_q2p2_q2p2 = v_q2p2.dot(v_q2p2)
    d_q2p2_q1p1 = v_q2p2.dot(v_q1p1)
    d_p2p1_q1p1 = v_p2p1.dot(v_q1p1)
    d_q2p2_p2p1 = v_q2p2.dot(v_p2p1)
    d_p2p1_p2p1 = v_p2p1.dot(v_p2p1)

    denom = d_q1p1_q1p1*d_q2p2_q2p2 - d_q2p2_q1p1*d_q2p2_q1p1
    #case where edg1 and edg2 are parallel
    if denom == 0: 
        return False
    
    t1 = (-d_q2p2_q1p1*d_q2p2_p2p1 + d_p2p1_q1p1*d_q2p2_q2p2) / denom
    t2 = (d_q2p2_q1p1*d_p2p1_q1p1 - d_q2p2_p2p1*d_q1p1_q1p1) / denom
    
    #case where closest points are within the edges
    if 0 < t1 and t1 < 1 and 0 < t2 and t2 < 1:
        dist_squared = max(0, d_p2p1_p2p1 + (t2**2)*d_q2p2_q2p2 + (t1**2)*d_q1p1_q1p1 + 2*(t2*d_q2p2_p2p1 - t1*d_p2p1_q1p1 - t1*t2*d_q2p2_q1p1))
        dist = math.sqrt(dist_squared)
        return dist < min_dist

    #case where at least one closest point is an end point
    return False
##########################################################################################################


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
######################################################################################################


def get_dict_of_cells_and_edges(lstEdges, min_dist, cell_size):
    print("Running get_dict_of_cells_and_edges()")

    setOfCells = set()
    dictOfCells = {}

    for edg in lstEdges:
        aabb_min, aabb_max = get_aabb_of_edge(edg,min_dist)
        aabb_min_cell = (math.floor(aabb_min.x / cell_size), 
                    math.floor(aabb_min.y / cell_size), 
                    math.floor(aabb_min.z / cell_size))
        aabb_max_cell = (math.ceil(aabb_max.x / cell_size), 
                    math.ceil(aabb_max.y / cell_size), 
                    math.ceil(aabb_max.z / cell_size))
        for i in range(aabb_min_cell[0],aabb_max_cell[0]):
            for j in range(aabb_min_cell[1],aabb_max_cell[1]):
                for k in range(aabb_min_cell[2],aabb_max_cell[2]):
                    cell = (i,j,k)
                    if cell in setOfCells:
                        dictOfCells[cell].add(edg)
                    else:
                        setOfCells.add(cell)
                        dictOfCells[cell] = set([edg])
    
    return dictOfCells
##########################################################################################

                    
def get_intersecting_edges(lstEdges, dictOfCells, min_dist, cell_size):
    print("Running get_intersecting_edges()")

    setOfEdgesWithIntersect = set()

    for edg in lstEdges:
        edg.select = False
        edgHasIntersects = False
        aabb_min, aabb_max = get_aabb_of_edge(edg,min_dist)
        aabb_min_cell = (math.floor(aabb_min.x / cell_size), 
                    math.floor(aabb_min.y / cell_size), 
                    math.floor(aabb_min.z / cell_size))
        aabb_max_cell = (math.ceil(aabb_max.x / cell_size), 
                    math.ceil(aabb_max.y / cell_size), 
                    math.ceil(aabb_max.z / cell_size))
                    
        setOfCloseEdges = set()
        for i in range(aabb_min_cell[0],aabb_max_cell[0]):
            for j in range(aabb_min_cell[1],aabb_max_cell[1]):
                for k in range(aabb_min_cell[2],aabb_max_cell[2]):
                    cell = (i,j,k)
                    setOfCloseEdges.update(dictOfCells[cell])
                    
        for closeEdge in setOfCloseEdges:
            if closeEdge.index <= edg.index:
                continue
            if check_edges_intersect(edg,closeEdge,min_dist):
                setOfEdgesWithIntersect.add(closeEdge)
                edgHasIntersects = True
                
        if edgHasIntersects:
            setOfEdgesWithIntersect.add(edg)
    
    return setOfEdgesWithIntersect
#####################################################################################################


def select_intersecting_edges(bm, min_dist):
    print("Running select_intersecting_edges()")

    lstEdges = [edg for edg in bm.edges if edg.select]
    lstEdgeLengths = [get_edge_length(edg) for edg in lstEdges]
    cell_size = np.median(np.array(lstEdgeLengths))

    for face in bm.faces:
        face.select = False

    dictOfCells = get_dict_of_cells_and_edges(lstEdges,min_dist,cell_size)
    setOfEdgesWithIntersect = get_intersecting_edges(lstEdges,dictOfCells,min_dist,cell_size)

    for edg in setOfEdgesWithIntersect:
        edg.select = True
    
    return len(setOfEdgesWithIntersect)
#####################################################################################################


def main(self, context):
    print("Running main()")

    if context.mode != 'EDIT_MESH':
        raise TypeError("Error! This Operation works only in Edit Mode!")
    
    min_dist = context.scene.distance_tolerance
    totalNumOfEdgesWithIntersect = 0

    for object in context.selected_objects:
        if object.type != 'MESH':
            self.report({'WARNING'},"Object ",object.name," is not of type 'MESH'!")
            continue
        mesh = object.data
        bm = bmesh.from_edit_mesh(mesh)
        totalNumOfEdgesWithIntersect += select_intersecting_edges(bm,min_dist)
        bmesh.update_edit_mesh(mesh)
        bm.free()
    
    self.report({'INFO'},"Found " + str(totalNumOfEdgesWithIntersect) + " Intersecting Edges")
    return
########################################################################################################


class MESH_OT_find_intersecting_edges(bpy.types.Operator):
    bl_idname = "mesh.find_intersecting_edges"
    bl_label = "Find Intersecting Edges"
    bl_options = {'REGISTER','UNDO'}

    def execute(self, context):
        main(self,context)
        return {'FINISHED'}
####################################################################################################################

# class PANEL_find_intersecting_edges(bpy.types.Panel):
#      bl_idname = "panel_find_intersecting_edges"
#      bl_label = "Find Intersecting Edges"
#      bl_space_type = "VIEW_3D"
#      bl_region_type = "UI"
#      bl_category = "Moritz Tools"

#      def draw(self,context):
#          layout = self.layout
#          layout.operator("mesh.find_intersecting_edges", text="Find Intersecting Edges")


def register():
#    bpy.utils.register_class(PANEL_find_intersecting_edges)
    bpy.utils.register_class(MESH_OT_find_intersecting_edges)

def unregister():
    bpy.utils.unregister_class(MESH_OT_find_intersecting_edges)
#    bpy.utils.unregister_class(PANEL_find_intersecting_edges)

#register()