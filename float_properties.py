import bpy

def register():
    bpy.types.Scene.distance_tolerance = bpy.props.FloatProperty(
        name = "Distance Tolerance",
        description = "Elements with Distance below this Tolerance are considered touching",
        default = 0.0001,
        min = 0,
        max = 10,
        precision = 5
    )
    bpy.types.Scene.angle_tolerance = bpy.props.FloatProperty(
        name = "Angle Tolerance (Rad)",
        description = "Edges with Angle below this Tolerance are considered parallel",
        default = 0.01,
        min = 0,
        max = 10,
        precision = 4
    )

def unregister():
    del bpy.types.Scene.distance_tolerance
    del bpy.types.Scene.angle_tolerance