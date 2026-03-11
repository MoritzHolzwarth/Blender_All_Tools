import bpy
import bmesh
import mathutils
import math

#from . import remove_sitting_verts
#from . import remove_useless_verts


def check_vert_move_is_safe(vrt, newLoc, min_dist):

    location = vrt.co
    vecV = newLoc - location
    vecV_len = vecV.length
    if vecV_len < min_dist:
        return True
    
    vecV = vecV/vecV_len
    lstNeighbours = [edg.other_vert(vrt) for edg in vrt.link_edges]
    
    for v1 in lstNeighbours:
        vecS = v1.co - location
        vecSvecV = vecS.dot(vecV)
        if 0 < vecSvecV:
            for v2 in lstNeighbours:
                if v2 == v1:
                    continue
                vecP = v2.co - location
                vecW = vecS - vecSvecV * vecV
                vecPvecW = vecP.dot(vecW)
                vecW_lensq = vecW.length_squared
                x = vecP.dot(vecV)- vecSvecV
                if x < 0 and vecSvecV < vecV_len:
                    if vecPvecW > vecW_lensq*(1 + x/(vecSvecV - vecV_len)):
                        print("vert move not safe!")
                        return False
                elif x > 0:
                    if vecPvecW > vecW_lensq*(1 + x/vecSvecV):
                        print("vert move not safe!")
                        return False

    return True
########################################################################################################


def get_lineSegment_vert_distance(vrt, startCo, endCo, vecDirection):
    
    vecStartToVrt = vrt.co - startCo
    vecEndToVrt = vrt.co - endCo
    
    if vecStartToVrt.dot(vecDirection) < 0:
        return vecStartToVrt.length
    
    elif vecEndToVrt.dot(vecDirection) > 0:
        return vecEndToVrt.length
    
    else:
        vecProjection = vecStartToVrt.dot(vecDirection)*vecDirection
        return (vecStartToVrt - vecProjection).length
#######################################################################################################


def get_face_center_point(face):

    center =  mathutils.Vector((0,0,0))
    for vrt in face.verts:
        center += vrt.co
        
    center /= len(face.verts)
    return center
############################################################################################################


def poke_and_get_closest_verts(bm, lstInitCoords, lstNewVerts, min_dist):
    print("Running poke_and_get_closest_verts()")
    
    bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)
    
    lstClosestMeshLocations = []
    lstClosestFaces = []
    lstClosestFacesCenters = []
    
    kdtree = mathutils.kdtree.KDTree(len(bm.verts))
    for vrt in bm.verts:
        kdtree.insert(vrt.co, vrt.index)
    
    kdtree.balance()

    for p in lstInitCoords:
        location, normal, face_index, distance = bvh.find_nearest(p)
        lstClosestMeshLocations.append(location)
        
        closest_vrt_coords, closest_vrt_index, distance = kdtree.find(location)
        closest_vrt = bm.verts[closest_vrt_index]
        if check_vert_move_is_safe(closest_vrt, location, min_dist): #If the closest vert can be moved safely to the location, do that!
            lstClosestFacesCenters.append(closest_vrt_coords)
            continue
        
        print("poke face")
        closest_face = bm.faces[face_index]                          #Otherwise, poke the closest face and use the created vert as closest vert
        closest_face.select = True
        if not closest_face in lstClosestFaces:
            lstClosestFaces.append(closest_face)
        else:
            print("Warning in poke_and_get_closest_verts(): found same closest Face twice!")
            return False
        
        center =  get_face_center_point(closest_face)
        lstClosestFacesCenters.append(center)
    
    poke_result = bmesh.ops.poke(bm, faces = lstClosestFaces)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    
    kdtree = mathutils.kdtree.KDTree(len(bm.verts))
    for vrt in bm.verts:
        kdtree.insert(vrt.co, vrt.index)
    
    kdtree.balance()
    for i in range(0, len(lstInitCoords)):
        closestMeshLocation = lstClosestMeshLocations[i]
        center = lstClosestFacesCenters[i]
        vrt_coords, center_index, distance = kdtree.find(center)
        center_vert = bm.verts[center_index]
        center_vert.co = closestMeshLocation
        lstNewVerts.append(center_vert)
    
    return True
######################################################################################################################


def getNextVrt(currentVrt, startVrt, endVrt, pathDirection):

    lstNeighbours = [edg.other_vert(currentVrt) for edg in currentVrt.link_edges]
    nextVrt = currentVrt
    blFoundRightDirection = False
    minDist = float('inf')
    vecRightDirection = (endVrt.co - currentVrt.co).normalized()
    
    for v in lstNeighbours:
        
        if v == endVrt:
            nextVrt = v
            minDist = 0
            blFoundRightDirection = True
            break
        
        if v.select and not any(edg.is_boundary for edg in v.link_edges):        #Dont slect any Vert that were already selected in the current Path, except if they lie on a boundary because then it may be topologically inevitable
            continue
        
        if (v.co - currentVrt.co).dot(vecRightDirection) > 0: #This ckecks if the next-Vert-Candidate v goes in the "right" Direction relative to currentVrt. 
                                                                  #Such "right Direction"-Verts are prefered as candidates for the next Vert. ("right" Direction means "from startVrt to endVrt")
                                                                  #If any "right Direction"-Verts exist, we pick the one with the shortest Distance from the startVrt-endVrt-Linesegment
            dist = get_lineSegment_vert_distance(v, startVrt.co, endVrt.co, pathDirection)
            
            if not blFoundRightDirection: #If this is the first found Vert that points in rigth Direction, switch the Flag and update nextVert and minDist
                                          #Note: minDist gets updated here, even if dist > minDist, because the current minDist belongs to a Vert that does not "point" in right Direction and is therefore discarded.
                minDist = dist
                nextVrt = v
                blFoundRightDirection = True
                
            elif dist < minDist:        #Otherwise, only update based on the distance
                minDist = dist
                nextVrt = v
    
        elif not blFoundRightDirection: #Only if no Verts pointing in right Direction are found yet, we cosider any non-right-Direction Verts as nextVrt candidates...
            dist = get_lineSegment_vert_distance(v, startVrt.co, endVrt.co, pathDirection)
            
            if dist < minDist:          #..And chose them based on their distance from the line segment
                minDist = dist
                nextVrt = v
            
    return nextVrt
###############################################################################################################################


def selectDirectPath(startVrt, endVrt):
    print("Running selectDirectPath()")
    
    pathDirection = (endVrt.co - startVrt.co).normalized()
    vrt = startVrt
    setPathVerts = set()
    
    while True:
        vrt.select = True
        setPathVerts.add(vrt)
        
        if(vrt == endVrt): #Successfully reached the endVert (Path is complete)
            print("Reached End Vert")
            break
        
        newVrt = getNextVrt(vrt, startVrt, endVrt, pathDirection)
                
        if newVrt == vrt: #This means that the Slection of a new Vert failed, in which case we give up
            print("Found no new Vert in Path")
            break
        
        for edg in newVrt.link_edges:
            if edg.other_vert(newVrt) == vrt:
                edg.select = True
                
        vrt = newVrt
        
    return setPathVerts
#####################################################################################################################################


def projectPath(setPathVrts, startCo, endCo, min_dist, setProblemVerts):
    print("Running projectPath()")
    
    pathDirection = (endCo - startCo).normalized()
    
    for v in setPathVrts:
        if any(edg.is_boundary for edg in v.link_edges): #Dont project verts that lie on the mesh boundary
            continue
        newCO = startCo + (v.co - startCo).dot(pathDirection)*pathDirection
        if not check_vert_move_is_safe(v, newCO, min_dist):
            setProblemVerts.add(v)
        
        v.co = newCO
    
    return True
##################################################################################################################


def get_closest_object(context, point):
    print("Running get_closest_object()")
    
    closestObject = None
    minDist = float('inf')
    
    for obj in bpy.context.view_layer.objects:
        if obj.type != 'MESH':
            continue
        if obj.hide_get(): #ignore hidden objects
            continue
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.context.view_layer.update() # Might be neccessary in combination with above, context.view_layer.objects.active = obj
        
        bpy.ops.object.mode_set(mode='EDIT')
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        bvh = mathutils.bvhtree.BVHTree.FromBMesh(bm)
        obj_center = obj.matrix_world.translation.copy()
        location, normal, face_index, distance = bvh.find_nearest(point-obj_center)
        bm.free()
        bpy.ops.object.mode_set(mode='OBJECT')
        obj.select_set(False)
        
        if distance < minDist:
            closestObject = obj
            minDist = distance
    
    if closestObject == None:
        raise TypeError("Error in get_closest_object(): No Mesh-Type Object Found!")
    
    return closestObject
###############################################################################################################


def get_mean_vector(lst_vectors):

    av = mathutils.Vector((0,0,0))
    num = len(lst_vectors)
    if num == 0:
        return av

    for v in lst_vectors:
        av += v
    
    return av/num
##############################################################################################


def make_selection_flat(bm, normalAxis):
    print("Running make_selection_flat()")
    
    lstSelectedVerts = [v for v in bm.verts if v.select]
    if len(lstSelectedVerts) == 0:
        return
    mean_normal_value = 0
    for v in lstSelectedVerts:
        mean_normal_value += v.co[normalAxis]
    
    mean_normal_value /= len(lstSelectedVerts)
    for v in lstSelectedVerts:
        if any(edg.is_boundary for edg in v.link_edges):
            continue
        v.co[normalAxis] = mean_normal_value
#################################################################################################


def cut_rectangle(init_coords, context, normalAxis): #Four corners in clockwise order
    print("Running cut_rectangle()")
   
    
    object = get_closest_object(context, get_mean_vector(init_coords))
    bpy.context.view_layer.objects.active = object
    object.select_set(True)
    bpy.ops.object.transform_apply(location = True, rotation = True, scale = True)
    
    bpy.ops.object.mode_set(mode='EDIT')
    mesh = object.data
    bm = bmesh.from_edit_mesh(mesh)

#Triangulate the whole mesh
    #bmesh.ops.triangulate(bm, faces=bm.faces, quad_method='BEAUTY', ngon_method='BEAUTY')
    bm.faces.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    lstNewVerts = []
    min_dist = 0.0001

    if not poke_and_get_closest_verts(bm, init_coords, lstNewVerts, min_dist):
        bmesh.update_edit_mesh(mesh)
        bm.free()
        return

    if len(lstNewVerts) != len(init_coords):
        raise TypeError("Error in cut_rectangle(): Lost Corner Vertices! Found", len(lstNewVerts), " corner Verts!" )
    
    bpy.ops.mesh.select_all(action='DESELECT')
    lstOfPathVrtSets = []
    setProblemVerts = set()
    for i in range(0,4):
        startVrt = lstNewVerts[i]
        endVrt = lstNewVerts[(i+1)%4]
        setPathVerts = selectDirectPath(startVrt,endVrt)
        projectPath(setPathVerts, startVrt.co, endVrt.co, context.scene.distance_tolerance, setProblemVerts)
        lstOfPathVrtSets.append(setPathVerts)
    
    if len(setProblemVerts) == 0:
        print("No problematic Vert Moves :)")
#Selecting the loop's inner region (works based on edge selection)
        bpy.ops.mesh.loop_to_region()
        make_selection_flat(bm, normalAxis)
    else:
        print("Problematic Vert Moves: ", len(setProblemVerts))
    
        
    bmesh.update_edit_mesh(mesh)
    return
    
    #Undo the Triangulation for the whole mesh, execpt for faces touching the rectangle path
    #Select all faces touching the rectangle path
    for setPathVerts in lstOfPathVrtSets:
        for v in setPathVerts:
            for f in v.link_faces:
                f.select = True

#switch to Face-Selection-Mode
    bpy.context.tool_settings.mesh_select_mode = (False,False,True)

#Invert the selection to select All except the path-related faces
    bpy.ops.mesh.select_all(action='INVERT')
#Now convert the selcted faces back to Quads
    bpy.ops.mesh.tris_convert_to_quads(
        face_threshold=3.14159,  # 180 degrees in radians
        shape_threshold=3.14159
    )
#Invert Back to select only the path related faces
    bpy.ops.mesh.select_all(action='INVERT')
    bmesh.update_edit_mesh(mesh)
    
#Now do remove_sitting_verts(), and remove_useless_verts()
    #remove_useless_verts.remove_useless_verts(bm, context.scene.angle_tolerance, False) #'False' parameter for Not unselecting everything, because next operation also requires selection
    #remove_sitting_verts.remove_sitting_verts(bm,  context.scene.distance_tolerance)

    bmesh.update_edit_mesh(mesh)
    bm.free()
    print("Finished cut_rectangle()")
    
########################################################################################################################################


def main(self, context):
    print("Running main")
    
    # A = mathutils.Vector((-0.38, 0.471, 0.01))
    # B = mathutils.Vector((0.51, 0.57, -0.2))
    # C = mathutils.Vector((0.84, -0.22, -0.3))
    # D = mathutils.Vector((-0.6, -0.8, 0))
    xvec = mathutils.Vector((1,0,0))
    yvec = mathutils.Vector((0,1,0))
    zvec = mathutils.Vector((0,0,1))
    canonicalBasis = [xvec,yvec,zvec]

    corner1 = mathutils.Vector((0,0,1.75)) - 0.02*zvec    #After closer inspectino, it appears that I calculated the z-values in this model 2 cm to high
    corner2 = mathutils.Vector((0,0.3,2.05)) - 0.02*zvec
    
    minCoords = mathutils.Vector((min(corner1.x, corner2.x), min(corner1.y, corner2.y), min(corner1.z, corner2.z)))
    maxCoords = mathutils.Vector((max(corner1.x, corner2.x), max(corner1.y, corner2.y), max(corner1.z, corner2.z)))
    minmaxCoords_Diff = maxCoords - minCoords

    min_MinMaxDiff = float('inf')
    normalAxis = None
    for i in range(0,3):
        if minmaxCoords_Diff[i] < min_MinMaxDiff:
            min_MinMaxDiff = minmaxCoords_Diff[i]
            normalAxis = i
    
    print("found normalAxis", normalAxis)
    
    axis1 = (normalAxis + 1) % 3
    axis2 = (normalAxis + 2) % 3
    A = minCoords
    B = A + minmaxCoords_Diff[axis1]*canonicalBasis[axis1]
    D = A + minmaxCoords_Diff[axis2]*canonicalBasis[axis2]
    C = maxCoords
    
    cut_rectangle([A,B,C,D], context, normalAxis)
    print("finished")
###########################################################
    

class MESH_OT_cut_Rectangles(bpy.types.Operator):

    bl_idname = "mesh.cut_rectangles"
    bl_label = "Cut Rectangles"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self,context):
        main(self, context)
        return {'FINISHED'}
###########################################################################################################################


class VIEW3D_PT_cut_Rectangles(bpy.types.Panel):
    bl_idname = "VIEW3D_PT_cut_Rectangles"
    bl_label = "Cut Rectangles"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Cut Rectangles"
    
    def draw(self, context):
        layout = self.layout
        layout.operator("mesh.cut_rectangles", text="Cut Rectangles")
#########################################################################################################################


def register():
    bpy.utils.register_class(MESH_OT_cut_Rectangles)
    bpy.utils.register_class(VIEW3D_PT_cut_Rectangles)

def unregister():
    bpy.utils.unregister_class(VIEW3D_PT_cut_Rectangles)
    bpy.utils.unregister_class(MESH_OT_cut_Rectangles)

register()