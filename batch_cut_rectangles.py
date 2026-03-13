import bpy
import bmesh
import numpy as np
import mathutils

class rectangle:

    index = -1
    lstOriginalCoords = None
    lstCornerVerts = None
    normalAxis = None
    issafe = True
    failedPathIndices = []

    def __init__(self, index, lstCoords, normalAxis):
        self.index = index
        self.lstOriginalCoords = lstCoords
        self.normalAxis = normalAxis
#######################################################################################################################


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
                vecT = vecS - vecV*vecV_len
                vecQ = vecP - vecS
                vecQvecT = vecQ.dot(vecT)
                vecQvecS = vecQ.dot(vecS)
                vecSvecT = vecS.dot(vecT)
                vecQ_len = vecQ.length
                vecS_len = vecS.length
                vecT_len = vecT.length
                
                if vecQvecT/vecQ_len > vecSvecT/vecS_len and vecQvecS/vecQ_len > vecSvecT/vecT_len:
                    v1.select = True
                    v2.select = True
                    return False

    return True


def check_vert_move_is_safe_old(vrt, newLoc, min_dist):

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
                        v1.select = True
                        v2.select = True
                        return False
                elif x > 0:
                    if vecPvecW > vecW_lensq*(1 + x/vecSvecV):
                        if vecSvecV < vecV_len: #Extra Condition
                            v1.select = True
                            v2.select = True
                            return False
                        elif vecPvecW < vecW_lensq*(1 + x/(vecSvecV - vecV_len)):
                            v1.select = True
                            v2.select = True
                            return False

    return True
########################################################################################################


def get_closest_object(context, point):
    print("Running get_closest_object()")
    
    closestObject = None
    minDist = float('inf')
    
    for obj in context.view_layer.objects:
        if obj.type != 'MESH':
            continue
        if obj.hide_get(): #ignore hidden objects
            continue
        context.view_layer.objects.active = obj
        obj.select_set(True)
        context.view_layer.update() # Might be neccessary in combination with above, context.view_layer.objects.active = obj
        
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
    
    print("Found closest Object", closestObject, "for position", point)
    return closestObject
###################################################################################################################


def get_rectangle_coords(corner1, corner2, canonicalBasis):

    minCoords = mathutils.Vector((min(corner1.x, corner2.x), min(corner1.y, corner2.y), min(corner1.z, corner2.z)))
    maxCoords = mathutils.Vector((max(corner1.x, corner2.x), max(corner1.y, corner2.y), max(corner1.z, corner2.z)))
    minmaxCoords_Diff = maxCoords - minCoords

    min_MinMaxDiff = float('inf')
    normalAxis = None
    for i in range(0,3):
        if minmaxCoords_Diff[i] < min_MinMaxDiff:
            min_MinMaxDiff = minmaxCoords_Diff[i]
            normalAxis = i
    
    axis1 = (normalAxis + 1) % 3
    axis2 = (normalAxis + 2) % 3
    A = minCoords
    B = A + minmaxCoords_Diff[axis1]*canonicalBasis[axis1]
    D = A + minmaxCoords_Diff[axis2]*canonicalBasis[axis2]
    C = maxCoords
    return [A,B,C,D], normalAxis
########################################################################################################


def get_mean_vector(lst_vectors):

    av = mathutils.Vector((0,0,0))
    num = len(lst_vectors)
    if num == 0:
        return av

    for v in lst_vectors:
        av += v
    
    return av/num
#####################################################################################################


def get_dict_of_objects_and_rects(context, data):
    print("Running get_dict_of_objects_and_rects()")

    xvec = mathutils.Vector((1,0,0))
    yvec = mathutils.Vector((0,1,0))
    zvec = mathutils.Vector((0,0,1))
    canonicalBasis = [xvec,yvec,zvec]

    dictOfObjectsAndRects = {}

    rect_index = -1
    for row in data:
        rect_index += 1
        corner1 = mathutils.Vector(row[0:3]) - 0.02*zvec
        corner2 = mathutils.Vector(row[3:6]) - 0.02*zvec
        rect_coords, rect_normalAxis = get_rectangle_coords(corner1, corner2, canonicalBasis)
        closest_object = get_closest_object(context, get_mean_vector(rect_coords))

        if closest_object in dictOfObjectsAndRects.keys():
            lstRectangles = dictOfObjectsAndRects[closest_object]
            lstRectangles.append( rectangle(rect_index, rect_coords, rect_normalAxis) )
        else:
            lstRectangles = [rectangle(rect_index, rect_coords, rect_normalAxis)]
            dictOfObjectsAndRects[closest_object] = lstRectangles

    print("Found closest objects", dictOfObjectsAndRects.keys())
    return dictOfObjectsAndRects
###########################################################################################################


def get_face_center_point(face):

    center =  mathutils.Vector((0,0,0))
    for vrt in face.verts:
        center += vrt.co
        
    center /= len(face.verts)
    return center
############################################################################################################


def assign_rectangle_corner_verts(bm, lstOfRects, min_dist):
    print("Running assign_rectangle_corner_verts()")

    bvhtree = mathutils.bvhtree.BVHTree.FromBMesh(bm)
    kdtree = mathutils.kdtree.KDTree(len(bm.verts))
    for v in bm.verts:
        kdtree.insert(v.co, v.index)
    kdtree.balance()

    lstMeshLocationsLists = []
    lstVertLocationLists = []
    lstPokedFaces = []

    for rect in lstOfRects:

        lstMeshLocations = []
        lstVertLocations = []

        for p in rect.lstOriginalCoords:

            location, normal, face_index, loc_distance = bvhtree.find_nearest(p)
            lstMeshLocations.append(location)
            closest_vrt_coords, closest_vrt_index, closest_vrt_distance = kdtree.find(location)
            closest_vrt = bm.verts[closest_vrt_index]

            if check_vert_move_is_safe(closest_vrt, location, min_dist):
                lstVertLocations.append(closest_vrt_coords) #This Vert Location belongs to closest_vrt (which is a valid vert)
            else:
                closest_face = bm.faces[face_index]
                if closest_face in lstPokedFaces: #Any closest_face can only correspond to exactly one point! Only the first assignment of a closest_face is valid.
                    print("Warning! found same closest Face for two different locations!")
                    rect.issafe = False
                else:
                    lstPokedFaces.append(closest_face)
                    lstVertLocations.append(get_face_center_point(closest_face)) #This Vert Location belongs to a vert that does not yet exist. It will be created in the center of corresponding face by poking it.
            
        lstMeshLocationsLists.append(lstMeshLocations)
        lstVertLocationLists.append(lstVertLocations)
    
    if len(lstPokedFaces) == 0:
        print("No Faces to be poked!")
    else:
        print("Poking", len(lstPokedFaces), "Faces")
        bmesh.ops.poke(bm, faces = lstPokedFaces)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()

        kdtree = mathutils.kdtree.KDTree(len(bm.verts))
        for v in bm.verts:
            kdtree.insert(v.co, v.index)
        kdtree.balance()

    for i in range(0,len(lstOfRects)):
        rect = lstOfRects[i]
        if not rect.issafe:
            continue
        lstMeshLocations = lstMeshLocationsLists[i]
        lstVertLocations = lstVertLocationLists[i]
        if len(lstMeshLocations) != len(lstVertLocations):
            raise TypeError("Error in assign_verts_to_rectangle_corners(): Lists of Mesh Locations and Verts do not match in count!")
        
        lstCornerVerts = []
        for j in range(0,len(lstMeshLocations)):
            mesh_loc = lstMeshLocations[j]
            vrt_loc = lstVertLocations[j]
            vrt_coords, vrt_index, vrt_distance = kdtree.find(vrt_loc)
            vrt = bm.verts[vrt_index]
            vrt.co = mesh_loc
            lstCornerVerts.append(vrt)
        
        rect.lstCornerVerts = lstCornerVerts
######################################################################################################################################


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
        
        if v.select and not any(edg.is_boundary for edg in v.link_edges):   #Dont slect any Vert that were already selected in the current Path, except if they lie on a boundary because then it may be topologically inevitable
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


def selectDirectPath(startVrt, endVrt, lstAllPathVertSets):
    
    pathDirection = (endVrt.co - startVrt.co).normalized()
    vrt = startVrt
    
    safetyCounter = 0

    setPathVerts = set()
    while True:
        safetyCounter += 1
        vrt.select = True
        setPathVerts.add(vrt)
        
        if(vrt == endVrt): #Successfully reached the endVert (Path is complete)
            break
        
        newVrt = getNextVrt(vrt, startVrt, endVrt, pathDirection)
                
        if newVrt == vrt: #This means that the Slection of a new Vert failed, in which case we give up
            lstAllPathVertSets.append(setPathVerts)
            return False
        
        if safetyCounter > 10000:
            print("SafetyCounter > 10000! Exit Loop!")
            lstAllPathVertSets.append(setPathVerts)
            return False
        
        for edg in newVrt.link_edges:
            if edg.other_vert(newVrt) == vrt:
                edg.select = True

        vrt = newVrt
        
    lstAllPathVertSets.append(setPathVerts)
    return True
###########################################################################################################################################


def projectPath(setPathVrts, startCo, endCo, min_dist):
    
    pathDirection = (endCo - startCo).normalized()
    NoUnsafeVertMoves = True
    
    for v in setPathVrts:
        if any(edg.is_boundary for edg in v.link_edges): #Dont project verts that lie on the mesh boundary
            continue
        newCO = startCo + (v.co - startCo).dot(pathDirection)*pathDirection
        if not check_vert_move_is_safe(v, newCO, min_dist):
            NoUnsafeVertMoves = False
        
        v.co = newCO
    
    return NoUnsafeVertMoves
##################################################################################################################


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

def cut_rectangle(bm, rect, min_dist, bl_object):
    print("Running cut_rectangle() with rectangle", rect.index)

    if not rect.issafe:
        return
    
    lstAllPathVertSets = []
    for i in range(0,4):
        startVrt = rect.lstCornerVerts[i]
        endVrt = rect.lstCornerVerts[(i+1) % 4]
        if not selectDirectPath(startVrt, endVrt, lstAllPathVertSets):
            print("Warning: Problem at Selecting Rectangle Path!")
            pathIndices = [v.index for setPathVerts in lstAllPathVertSets for v in setPathVerts]
            rect.failedPathIndices = pathIndices
            rect.issafe = False
            return
    
    allProjectionsSafe = True
    for i in range(0,4):
        startVrt = rect.lstCornerVerts[i]
        endVrt = rect.lstCornerVerts[(i+1) % 4]
        setPathVerts = lstAllPathVertSets[i]
        if not projectPath(setPathVerts, startVrt.co, endVrt.co, min_dist):
            allProjectionsSafe = False
    
    if not allProjectionsSafe:
        print("Warning: Problem at Straightening Rectangle!")
        pathIndices = [v.index for setPathVerts in lstAllPathVertSets for v in setPathVerts]
        rect.failedPathIndices = pathIndices
        rect.issafe = False
        return
   
#Selecting the loop's inner region (works based on edge selection)
    bpy.ops.mesh.loop_to_region()
    make_selection_flat(bm, rect.normalAxis)
    setObjectsBefore = set(bpy.context.scene.objects)
    bpy.ops.mesh.separate(type = 'SELECTED')
    setObjectsAfter = set(bpy.context.scene.objects)
    newObject = (setObjectsAfter - setObjectsBefore).pop()
    newObject.name = bl_object.name + "_Platte" + str(rect.index)
    print("Finished cut_rectangle()")
####################################################################################################


def batch_cut_rectangles_main(self, context):

    min_dist = 0.0001

    if self.filepath == None:
        raise TypeError("Error in batch_cut_rectangles_main(): TXT File Path is None!")

    try:
        data = np.loadtxt(self.filepath)
    
    except:
        raise TypeError("Error in batch_cut_rectangles_main(): Failed to load TXT File!")
    
    if len(data) == 0:
        raise TypeError("Error in batch_cut_rectangles_main(): Txt File is empty!")
    
    if data.ndim == 1:
        data = np.array([list(data)])
    
    if data.shape[1] != 6:
        raise TypeError("Error in batch_cut_rectangles_main(): Number of Columns must be 6!")
    
    print("Read Rectangle Data", data)
    
    dictOfObjectsAndRects = get_dict_of_objects_and_rects(context, data)

    numFailedRectangles = 0
    for obj, lstOfRects in dictOfObjectsAndRects.items():
        print("Now handling object", obj)
        context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.transform_apply(location = True, rotation = True, scale = True)
        bpy.ops.object.mode_set(mode='EDIT')
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        assign_rectangle_corner_verts(bm, lstOfRects, min_dist)
        
        for rect in lstOfRects:
            bpy.ops.mesh.select_all(action='DESELECT')
            cut_rectangle(bm, rect, min_dist, obj)
            if not rect.issafe:
                numFailedRectangles += 1

        bmesh.update_edit_mesh(obj.data)
        obj.select_set(False)
        bm.free()
        bpy.ops.object.mode_set(mode='OBJECT')
        
        for rect in lstOfRects:
            if not rect.issafe:
                vertGroup = obj.vertex_groups.new(name = "Unfinished_Platte_"+str(rect.index))
                vertGroup.add(rect.failedPathIndices, 1.0, 'REPLACE')
    
    
    if numFailedRectangles > 0:
        print("Warning: " + str(numFailedRectangles) + " Rectangles failed to be cut.")
        for obj in dictOfObjectsAndRects.keys():
            bpy.ops.object.mode_set(mode='EDIT')
            for vertGroup in obj.vertex_groups:
                if vertGroup.name.startswith("Unfinished_Platte_"):
                    obj.vertex_groups.active = vertGroup
                    bpy.ops.object.vertex_group_select()
            
            bpy.ops.object.mode_set(mode='OBJECT')


    print("Finished batch_cut_rectangles_main()")
############################################################################################################################



class WM_OT_Batch_Cut_Rectangles(bpy.types.Operator):
    
    bl_idname = "wm.batch_cut_rectangles"
    bl_label = "Load TXT File"
    
    filepath: bpy.props.StringProperty(subtype = 'FILE_PATH')
    
    filter_glob: bpy.props.StringProperty(default = '*.txt', options = {'HIDDEN'})
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
    
    def execute(self, context):
        batch_cut_rectangles_main(self, context)
        return {'FINISHED'}
##################################################################################################################
    

class VIEW3D_PT_Batch_Cut_Rectangles(bpy.types.Panel):
    bl_idname = "VIEW3D_PT_batch_cut_rectangles"
    bl_label = "Batch Cut Rectangles"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Batch Cut Rectangles"
    
    def draw(self, context):
        layout = self.layout
        layout.operator("wm.batch_cut_rectangles", text="Batch Cut Rectangles")
#########################################################################################################################

    
def register():
    bpy.utils.register_class(WM_OT_Batch_Cut_Rectangles)
    bpy.utils.register_class(VIEW3D_PT_Batch_Cut_Rectangles)

def unregister():
    bpy.utils.unregister_class(VIEW3D_PT_Batch_Cut_Rectangles)
    bpy.utils.unregister_class(WM_OT_Batch_Cut_Rectangles)
    
register()