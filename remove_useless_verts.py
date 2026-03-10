import bpy
import bmesh
import mathutils
import math

def check_is_useless_vert(vrt, min_angle):

    lstEdges = list(vrt.link_edges)
    if len(lstEdges) != 2:
        return False
    v1 = lstEdges[0].other_vert(vrt).co - vrt.co
    v2 = lstEdges[1].other_vert(vrt).co - vrt.co
    if v1.length_squared == 0 or v2.length_squared == 0:
        raise TypeError("Error in check_is_useless_vert(): Vert has Edge of zero Length!")
    
    angle = v1.angle(v2)
    return abs(abs(angle) - math.pi) < min_angle
####################################################################################################

def remove_useless_verts(bm, min_angle):
    print("Running remove_useless_verts()")

    lstVerts = [v for v in bm.verts if v.select]

    for face in bm.faces:
        face.select = False

    num = 0
    for vrt in lstVerts:
        vrt.select = False
        if check_is_useless_vert(vrt,min_angle):
            num += 1
            neighbourVrt = vrt.link_edges[0].other_vert(vrt)
            bmesh.ops.pointmerge(bm,verts=[neighbourVrt,vrt],merge_co=neighbourVrt.co)

    return num
#####################################################################################################


def main(self, context):
    print("Running main()")

    if context.mode != 'EDIT_MESH':
        raise TypeError("Error! This Operation works only in Edit Mode!")
    
    min_angle = context.scene.angle_tolerance
    min_distance = context.scene.distance_tolerance
    bpy.ops.mesh.remove_doubles(threshold=min_distance,use_unselected=True)

    numRemovedVerts = 0
    for object in context.selected_objects:
        if object.type != 'MESH':
            self.report({'WARNING'},"Object ",object.name," is not of type 'MESH'!")
            continue
        mesh = object.data
        bm = bmesh.from_edit_mesh(mesh)
        numRemovedVerts += remove_useless_verts(bm,min_angle)
        bmesh.update_edit_mesh(mesh)
        bm.free()
    
    self.report({'INFO'},"Removed "+ str(numRemovedVerts) +" Useless Verts")
    return
####################################################################################################################


class MESH_OT_remove_useless_verts(bpy.types.Operator):
    bl_idname = "mesh.remove_useless_verts"
    bl_label = "Remove Useless Verts"
    bl_options = {'REGISTER','UNDO'}

    def execute(self, context):
        main(self,context)
        return {'FINISHED'}
####################################################################################################################


# class PANEL_remove_useless_verts(bpy.types.Panel):
#     bl_idname = "panel_remove_useless_verts"
#     bl_label = "Remove Useless Verts"
#     bl_space_type = "VIEW_3D"
#     bl_region_type = "UI"
#     bl_category = "Moritz Tools"

#     def draw(self,context):
#         layout = self.layout
#         layout.operator("mesh.remove_useless_verts", text="Remove Useless Verts")
##################################################################################################################


def register():
#    bpy.utils.register_class(PANEL_remove_useless_verts)
    bpy.utils.register_class(MESH_OT_remove_useless_verts)

def unregister():
    bpy.utils.unregister_class(MESH_OT_remove_useless_verts)
#    bpy.utils.unregister_class(PANEL_remove_useless_verts)

#register()