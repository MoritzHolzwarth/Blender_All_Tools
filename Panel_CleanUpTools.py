import bpy

class VIEW3D_PT_CleanUpTools(bpy.types.Panel):
    bl_idname = "VIEW3D_PT_AllMoritzTools"
    bl_label = "All Moritz Tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Moritz Tools"
    
    def draw(self, context):
        layout = self.layout
        layout.operator("mesh.find_useless_verts", text="Find Useless Verts")
        layout.operator("mesh.remove_useless_verts", text="Remove Useless Verts")
        layout.operator("mesh.find_sitting_verts", text="Find Sitting Verts")
        layout.operator("mesh.remove_sitting_verts", text="Remove Sitting Verts")
        layout.operator("mesh.find_intersecting_edges", text="Find Intersecting Edges")
        layout.operator("mesh.get_edges_distance", text="Get smallest Distance of two Edges")
        layout.operator("mesh.cut_rectangles", text = "Cut Rectangles")
        layout.operator("mesh.find_direct_path", text="Find Direct Path")
        layout.prop(context.scene, "distance_tolerance")
        layout.prop(context.scene, "angle_tolerance")

def register():
    bpy.utils.register_class(VIEW3D_PT_CleanUpTools)
        
def unregister():
    bpy.utils.unregister_class(VIEW3D_PT_CleanUpTools)