import bpy
import bmesh
import mathutils
from bpy_extras import view3d_utils


def getKDTree(bm):
    
    kdTree = mathutils.kdtree.KDTree(len(bm.verts))
    for i, v in enumerate(bm.verts):
        kdTree.insert(v.co, i)
    kdTree.balance()
    return kdTree
########################################################################################################################


def getClosestVrtToMouse(context, self, event):
    print("getting closest Vert to Mouse")
    
    # Get the active region and space
    region = self.region
    rv3d = self.region3D
    coord = event.mouse_region_x, event.mouse_region_y

    # Convert 2D mouse position to a 3D ray
    view_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    view_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)

    # Raycast into the scene
    depsgraph = context.evaluated_depsgraph_get()
    result, location, normal, index, obj, matrix = context.scene.ray_cast(
        depsgraph, view_origin, view_vector
    )

    if not result or obj.type != 'MESH':
        return None

    # Transform hit location to object space
    hit_local = self.object.matrix_world.inverted() @ location
    
    closest_vert_coords, closest_vert_index, closest_vert_dist = self.kdTree.find(hit_local)
    
    return self.bm.verts[closest_vert_index]
#################################################################################################################


def getDistanceFromLineSegmentToVrt(vrt, startCo, endCo, vecDirection):
    
    vecStartToVrt = vrt.co - startCo
    vecEndToVrt = vrt.co - endCo
    
    if vecStartToVrt.dot(vecDirection) < 0:
        return vecStartToVrt.length
    
    elif vecEndToVrt.dot(vecDirection) > 0:
        return vecEndToVrt.length
    
    else:
        vecProjection = vecStartToVrt.dot(vecDirection)*vecDirection
        return (vecStartToVrt - vecProjection).length
#################################################################################################################


def getNextVrt(currentVrt, lstNeighbours, self):
    nextVrt = currentVrt
    startCo = self.currentStartVrt.co
    endCo = self.currentEndVrt.co
    blFoundRightDirection = False
    minDist = float('inf')
    vecRightDirection = (endCo - currentVrt.co).normalized()

    
    for v in lstNeighbours:
        
        if v == self.currentEndVrt:
            nextVrt = v
            break
        
        if v.select: #Dont slect any Vert that were already selected in the current Path
            continue
        
        if (v.co - currentVrt.co).dot(vecRightDirection) > 0: #This ckecks if the next-Vert-Candidate v goes in the "right" Direction relative to currentVrt. 
                                                                  #Such "right Direction"-Verts are prefered as candidates for the next Vert. ("right" Direction means "from startVrt to endVrt")
                                                                  #If any "right Direction"-Verts exist, we pick the one with the shortest Distance from the startVrt-endVrt-Linesegment
            dist = getDistanceFromLineSegmentToVrt(v, startCo, endCo, self.currentDirection)
            
            if not blFoundRightDirection: #If this is the first found Vert that points in rigth Direction, switch the Flag and update nextVert and minDist
                                          #Note: minDist gets updated here, even if dist > minDist, because the current minDist belongs to a Vert that does not "point" in right Direction and is therefore discarded.
                minDist = dist
                nextVrt = v
                blFoundRightDirection = True
                
            elif dist < minDist:        #Otherwise, only update based on the distance
                minDist = dist
                nextVrt = v
            
        elif not blFoundRightDirection: #Only if no Verts pointing in right Direction are found yet, we cosider any non-right-Direction Verts as nextVrt candidates...
            dist = getDistanceFromLineSegmentToVrt(v, startCo, endCo, self.currentDirection)
            
            if dist < minDist:         #..And chose them based on their distance from the line segment
                minDist = dist
                nextVrt = v
    
    return nextVrt
###############################################################################################################################


def selectDirectPath(self):
    print("selecting Path")
    
    startVrtCo = self.currentStartVrt.co 
    vrt = self.currentStartVrt
    
    while True:
        vrt.select = True
        self.setOfCurrentPathVrts.add(vrt)
        
        if(vrt == self.currentEndVrt): #Successfully reached the endVert (Path is complete)
            print("Reached End Vert")
            return
        
        lstNeighbours = [edg.other_vert(vrt) for edg in vrt.link_edges]
        newVrt = getNextVrt(vrt, lstNeighbours, self)
                
        if newVrt == vrt: #This means that the Slection of a new Vert failed, in which case we give up
            print("Found no new Vert in Path")
            return
        
        vrt = newVrt
######################################################################################################################


def updateCurrentPath(self, context, event):
    print("updating current Path")
    
    self.currentEndVrt = getClosestVrtToMouse(context, self, event)
    
    if (self.currentEndVrt is None) or (self.currentEndVrt == self.currentStartVrt):
        print("Didnt find closest Vert to Mouse!")
        return False
    
    self.currentDirection = (self.currentEndVrt.co - self.currentStartVrt.co).normalized()
    
    #Unselect current path, since a new path will be selected...
    for v in self.setOfCurrentPathVrts:
        if v in self.setOfFinishedPathVrts: #..But don't unselect Verts that belong to any already finished Path.
            continue
        v.select = False
    
    self.setOfCurrentPathVrts.clear()
    selectDirectPath(self)
    bmesh.update_edit_mesh(self.mesh)
    return True
################################################################################################################


def updateLstOfPaths(self):
    print("updating Path List")

    self.lstOfFinishedPathSets.append(self.setOfCurrentPathVrts.copy())
    self.setOfFinishedPathVrts.update(self.setOfCurrentPathVrts.copy())
    self.lstOfStartAndEndVrts.append(self.currentEndVrt)
    self.currentStartVrt = self.currentEndVrt #endVert of finished Path becomes startVert of next Path
    self.setOfCurrentPathVrts.clear()
#############################################################################################################


def projectAllFinishedPaths(self):
    print("projecting finished Paths")

    for i in range(0, len(self.lstOfFinishedPathSets)):
        print("i = ", i)
        print(len(self.lstOfFinishedPathSets[i]))
        path = self.lstOfFinishedPathSets[i]
        startVrt = self.lstOfStartAndEndVrts[i]
        endVrt = self.lstOfStartAndEndVrts[i+1]
        projectPath(path, startVrt.co, endVrt.co,)
        
    bmesh.update_edit_mesh(self.mesh)
####################################################################################################################


def projectPath(path, startCo, endCo):
    
    vecDirection = (endCo - startCo).normalized()
    
    for v in path:
        if any(edg.is_boundary for edg in v.link_edges): #Dont project verts that lie on the mesh boundary
            continue
        
        newCO = startCo + (v.co - startCo).dot(vecDirection)*vecDirection
        v.co = newCO
##################################################################################################################


def finish(self):

    print("finish")
    print("Hello for Test")
    for v in self.setOfCurrentPathVrts:
        v.select = False
        
    for path in self.lstOfFinishedPathSets:
        for v in path:
            for e in v.link_edges:
                other_v = e.other_vert(v)
                
                if other_v in path:
                    e.select = True
                    
    bmesh.update_edit_mesh(self.mesh)
########################################################################################################################


def quit(self):

    print("quit")

    for v in self.setOfCurrentPathVrts:
        v.select = False
        
    for path in self.lstOfFinishedPathSets:
        for v in path:
            v.select = False
                    
    bmesh.update_edit_mesh(self.mesh)
###########################################################################################################################


def undo_last_path(self):

    setOfLastPathVrts = self.lstOfFinishedPathSets[-1]
    self.setOfFinishedPathVrts.difference_update(setOfLastPathVrts)
    self.lstOfFinishedPathSets.pop()
    self.lstOfStartAndEndVrts.pop()
    self.currentStartVrt = self.lstOfStartAndEndVrts[-1]
    for vrt in setOfLastPathVrts:
        vrt.select = False

    bmesh.update_edit_mesh(self.mesh)
#############################################################################################################################


class MESH_OT_find_direct_path(bpy.types.Operator):
    bl_idname = "mesh.find_direct_path"
    bl_label = "Find Direct Path"
    bl_options = {'REGISTER', 'UNDO'}
    
    def invoke(self, context, event):
        if context.object.mode != "EDIT":
            raise TypeError("This Tool only works in Edit Mode!")
    
        lstActiveObs = list(context.objects_in_mode)
        if len(lstActiveObs) > 1 or len(lstActiveObs) == 0:
            raise TypeError("This Tool only works on a single active object!")
        
        self.object = lstActiveObs[0]
        if self.object.type != "MESH":
            raise TypeError("This Tool only works on objects of Type 'MESH'!")
        
        self.mesh = self.object.data
        self.bm = bmesh.from_edit_mesh(self.mesh)
        self.kdTree = getKDTree(self.bm)
        
        lstSelectedVrts = [vrt for vrt in self.bm.verts if vrt.select]
        if len(lstSelectedVrts) > 1 or len(lstSelectedVrts) == 0:
            raise TypeError("More than one vertex selected! This Tool only works for one initially selected Vertex!")
            
        for region in context.area.regions:
            if region.type == 'WINDOW':
                self.region = region
                self.region3D = context.space_data.region_3d
                break
        
        if self.region == None:
            raise TypeError("Could not find area.region of Type 'WINDOW'!")
        
        self.currentStartVrt = lstSelectedVrts[0]
        self.currentEndVrt = lstSelectedVrts[0]
        self.currentDirection = mathutils.Vector((0,0))
        self.setOfCurrentPathVrts = set()
        self.setOfFinishedPathVrts = set()
        self.lstOfStartAndEndVrts = [self.currentStartVrt]
        self.lstOfFinishedPathSets = []
        
        updateCurrentPath(self, context, event)
        
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
    def modal(self, context, event):
        if event.type == 'MOUSEMOVE':
            updateCurrentPath(self, context, event)
            return {'RUNNING_MODAL'}
        
        if event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
            if updateCurrentPath(self, context, event):
                updateLstOfPaths(self)
            return {'RUNNING_MODAL'}
        
        if event.type == 'SPACE':
            projectAllFinishedPaths(self)
            return {'RUNNING_MODAL'}

        if event.type == 'RET':
            finish(self)
            return {'CANCELLED'}
        
        if event.type == 'RIGHTMOUSE' and event.value == 'PRESS':
            if len(self.lstOfFinishedPathSets) == 0:
                return {'RUNNING_MODAL'}
            
            undo_last_path(self)
            updateCurrentPath(self, context, event)
            return {'RUNNING_MODAL'}
        
        if event.type == 'ESC':
            quit(self)
            return {'CANCELLED'}

        
        return {'PASS_THROUGH'}


def register():
    bpy.utils.register_class(MESH_OT_find_direct_path)

def unregister():
    bpy.utils.unregister_class(MESH_OT_find_direct_path)