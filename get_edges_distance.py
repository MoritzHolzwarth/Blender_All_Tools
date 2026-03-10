import bpy
import bmesh
import math
import mathutils
import numpy as np

def get_distance_of_edges(edg1, edg2):

    vrtP1, vrtQ1 = edg1.verts
    vrtP2, vrtQ2 = edg2.verts
    p1 = vrtP1.co; q1 = vrtQ1.co
    p2 = vrtP2.co; q2 = vrtQ2.co

    v_q1p1 = q1-p1
    v_q2p2 = q2-p2
    v_p2p1 = p2-p1
    v_q2p1 = q2-p1
    v_p2q1 = p2-q1
    
    d_q1p1_q1p1 = v_q1p1.dot(v_q1p1)
    d_q2p2_q2p2 = v_q2p2.dot(v_q2p2)
    d_q2p2_q1p1 = v_q2p2.dot(v_q1p1)
    d_p2p1_q1p1 = v_p2p1.dot(v_q1p1)
    d_q2p1_q1p1 = v_q2p1.dot(v_q1p1)
    d_q2p2_p2p1 = v_q2p2.dot(v_p2p1)
    d_q2p2_p2q1 = v_q2p2.dot(v_p2q1)
    d_p2p1_p2p1 = v_p2p1.dot(v_p2p1)
    d_q2p1_q2p1 = v_q2p1.dot(v_q2p1)
    d_p2q1_p2q1 = v_p2q1.dot(v_p2q1)

    denom = d_q1p1_q1p1*d_q2p2_q2p2 - d_q2p2_q1p1*d_q2p2_q1p1
    #case where edg1 and edg2 are parallel
    if denom == 0: 
        #consider all 4 distances between any 2 end points
        t1_for_p2 = min( max(d_p2p1_q1p1 / d_q1p1_q1p1, 0), 1)
        t1_for_q2 = min( max(d_q2p1_q1p1 / d_q1p1_q1p1, 0), 1)
        t2_for_p1 = min( max(-d_q2p2_p2p1 / d_q2p2_q2p2, 0), 1)
        t2_for_q1 = min( max(-d_q2p2_p2q1 / d_q2p2_q2p2, 0), 1)

        a1_for_p2 = d_p2p1_p2p1 + (t1_for_p2**2)*d_q1p1_q1p1 - 2*t1_for_p2*d_p2p1_q1p1
        a1_for_q2 = d_q2p1_q2p1 + (t1_for_q2**2)*d_q1p1_q1p1 - 2*t1_for_q2*d_q2p1_q1p1
        a2_for_p1 = d_p2p1_p2p1 + (t2_for_p1**2)*d_q2p2_q2p2 + 2*t2_for_p1*d_q2p2_p2p1
        a2_for_q1 = d_p2q1_p2q1 + (t2_for_q1**2)*d_q2p2_q2p2 + 2*t2_for_q1*d_q2p2_p2q1

        dist = math.sqrt(min([a1_for_p2, a1_for_q2, a2_for_p1, a2_for_q1]))
        return dist
    
    t1 = (-d_q2p2_q1p1*d_q2p2_p2p1 + d_p2p1_q1p1*d_q2p2_q2p2) / denom
    t2 = (d_q2p2_q1p1*d_p2p1_q1p1 - d_q2p2_p2p1*d_q1p1_q1p1) / denom
    
    #case where closest points are within the edges
    if 0 <= t1 and t1 <= 1 and 0 <= t2 and t2 <= 1:
        dist = math.sqrt( d_p2p1_p2p1 + (t2**2)*d_q2p2_q2p2 + (t1**2)*d_q1p1_q1p1 + 2*(t2*d_q2p2_p2p1 - t1*d_p2p1_q1p1 - t1*t2*d_q2p2_q1p1) )
        return dist
    
    #case where at least one closest point is an end point
    t1_for_p2 = min( max(d_p2p1_q1p1 / d_q1p1_q1p1, 0), 1)
    t1_for_q2 = min( max(d_q2p1_q1p1 / d_q1p1_q1p1, 0), 1)
    t2_for_p1 = min( max(-d_q2p2_p2p1 / d_q2p2_q2p2, 0), 1)
    t2_for_q1 = min( max(-d_q2p2_p2q1 / d_q2p2_q2p2, 0), 1)

    a1_for_p2 = d_p2p1_p2p1 + (t1_for_p2**2)*d_q1p1_q1p1 - 2*t1_for_p2*d_p2p1_q1p1
    a1_for_q2 = d_q2p1_q2p1 + (t1_for_q2**2)*d_q1p1_q1p1 - 2*t1_for_q2*d_q2p1_q1p1
    a2_for_p1 = d_p2p1_p2p1 + (t2_for_p1**2)*d_q2p2_q2p2 + 2*t2_for_p1*d_q2p2_p2p1
    a2_for_q1 = d_p2q1_p2q1 + (t2_for_q1**2)*d_q2p2_q2p2 + 2*t2_for_q1*d_q2p2_p2q1

    dist = math.sqrt(min([a1_for_p2, a1_for_q2, a2_for_p1, a2_for_q1]))
    return dist
#######################################################################################################


def main(self, context):
    print("Running main()")

    if context.mode != 'EDIT_MESH':
        raise TypeError("Error! This Operation works only in Edit Mode!")

    if len(context.selected_objects) == 0:
        raise TypeError("Error: No Object is selected!")
    
    lstEdges = []

    for object in context.selected_objects:
        if object.type != 'MESH':
            self.report({'WARNING'},"Object ",object.name," is not of type 'MESH'!")
            continue
        mesh = object.data
        bm = bmesh.from_edit_mesh(mesh)
        lstEdges = lstEdges + [edg for edg in bm.edges if edg.select]

    if len(lstEdges) != 2:
        raise TypeError("This Operation works only if exactly 2 Edges are selected!")
    
    dist = get_distance_of_edges(lstEdges[0], lstEdges[1])
    self.report({'INFO'},"Distance: " + str(dist))
    
    return
#################################################################################################


class MESH_OT_get_edges_distance(bpy.types.Operator):
    bl_idname = "mesh.get_edges_distance"
    bl_label = "Get Distance of two Edges"
    bl_options = {'REGISTER','UNDO'}

    def execute(self, context):
        main(self,context)
        return {'FINISHED'}
####################################################################################################################


# class PANEL_get_edges_distance(bpy.types.Panel):
#     bl_idname = "panel_get_edges_distance"
#     bl_label = "Get Distance of two Edges"
#     bl_space_type = "VIEW_3D"
#     bl_region_type = "UI"
#     bl_category = "Moritz Tools"

#     def draw(self,context):
#         layout = self.layout
#         layout.operator("mesh.get_edges_distance", text="Get Distance of two Edges")
###############################################################################################################


def register():
    #bpy.utils.register_class(PANEL_get_edges_distance)
    bpy.utils.register_class(MESH_OT_get_edges_distance)

def unregister():
    bpy.utils.unregister_class(MESH_OT_get_edges_distance)
    #bpy.utils.unregister_class(PANEL_get_edges_distance)

#register()