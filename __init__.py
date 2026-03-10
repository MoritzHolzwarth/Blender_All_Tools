bl_info = {
    "name" : "All_Moritz_Tools",
    "author" : "Moritz Holzwarth",
    "blender" : (4,5,1),
    "category" : "Mesh" }

from . import (
float_properties,
find_useless_verts,
remove_useless_verts,
find_sitting_verts,
remove_sitting_verts,
find_intersecting_edges,
get_edges_distance,
find_direct_path,
cut_Rectangles,
Panel_CleanUpTools )

def register():
    float_properties.register()
    find_useless_verts.register()
    remove_useless_verts.register()
    find_sitting_verts.register()
    remove_sitting_verts.register()
    find_intersecting_edges.register()
    get_edges_distance.register()
    find_direct_path.register()
    cut_Rectangles.register()
    Panel_CleanUpTools.register()
    
    
def unregister():
    Panel_CleanUpTools.unregister()
    cut_Rectangles.unregister()
    find_direct_path.unregister()
    get_edges_distance.unregister()
    find_intersecting_edges.unregister()
    remove_sitting_verts.unregister()
    find_sitting_verts.unregister()
    remove_useless_verts.unregister()
    find_useless_verts.unregister()
    float_properties.unregister()
    
if __name__ == "__main__":
    register()