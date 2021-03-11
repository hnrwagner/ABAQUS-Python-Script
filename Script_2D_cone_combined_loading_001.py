#python #abaqus #abaqustutorial #hnrwagner 

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------


from abaqus import *
from abaqusConstants import *
import regionToolset
import __main__
import section
import regionToolset
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import connectorBehavior
import odbAccess
from operator import add
import numpy as np


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------

# functions

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------

def Create_Part_3D_Cylinder(radius,length,thickness,part,model):
    s1 = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius, 0.0))
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius-thickness, 0.0))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseSolidExtrude(sketch=s1, depth=length)
    s1.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']
    

def Create_Part_2D_Cylinder(radius,length,part,model):
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius, 0.0))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseShellExtrude(sketch=s, depth=length)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']
#------------------------------------------------------------------------------

def Create_Part_2D_Cone(radiusR,height,angle,part,model):
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
    s.FixedConstraint(entity=g[2])
    s.Line(point1=(radiusR, 0.0), point2=(radiusR+height*tan(angle*np.pi/180), height))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseShellRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']

#------------------------------------------------------------------------------

def Create_Datum_Plane_by_Principal(type_plane,part,model,offset_plane):
    p = mdb.models[model].parts[part]
    myPlane = p.DatumPlaneByPrincipalPlane(principalPlane=type_plane, offset=offset_plane)
    myID = myPlane.id
    return myID


def Create_Set_All_Cells(model,part,set_name):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    p.Set(cells=c, name=set_name)

def Create_Set_All_Faces(model,part,set_name):
    p = mdb.models[model].parts[part]
    f = p.faces[:]
    p.Set(faces=f, name=set_name)

def Create_Material_Data(model,material_name,e11,e22,e33,nu12,nu13,nu23,g12,g13,g23,lts,lcs,tts,tcs,lss,tss):
    mdb.models[model].Material(name=material_name)
    mdb.models[model].materials[material_name].Elastic(type=ENGINEERING_CONSTANTS, table=((e11,e22,e33,nu12,nu13,nu23,g12,g13,g23), ))
    mdb.models[model].materials[material_name].HashinDamageInitiation(table=((lts,lcs,tts,tcs,lss,tss), ))

#------------------------------------------------------------------------------

def Create_Material_Data_2D(model,material_name,e11,e22,nu12,g12,g13,g23):
    mdb.models[model].Material(name=material_name)
    mdb.models[model].materials[material_name].Elastic(type=LAMINA, table=((e11,e22,nu12,g12,g13,g23), ))
#------------------------------------------------------------------------------

def Create_Set_Face(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    f = p.faces
    myFace = f.findAt((x,y,z),)
    face = face + (f[myFace.index:myFace.index+1], )
    p.Set(faces=face, name=set_name)
    return myFace

def Create_Set_Edge(x,y,z,model,part,set_name):
    edge = ()
    p = mdb.models[model].parts[part]
    e = p.edges
    myEdge = e.findAt((x,y,z),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    return myEdge


#-----------------------------------------------------------------------------
def Create_Set_Vertice(x,y,z,model,part,set_name):
    vertice = ()
    p = mdb.models[model].parts[part]
    v = p.vertices
    myVertice = v.findAt((x,y,z),)
    vertice = vertice + (v[myVertice.index:myVertice.index+1], )
    p.Set(vertices=vertice, name=set_name)  

#-----------------------------------------------------------------------------
def Create_Set_Vertice_2(x,y,z,model,part,set_name):
    vertice = ()
    a = mdb.models[model].rootAssembly
    v = a.instances[part].vertices
    myVertice = v.findAt((x,y,z),)
    vertice = vertice + (v[myVertice.index:myVertice.index+1], )
    a.Set(vertices=vertice, name=set_name)

#-----------------------------------------------------------------------------

def Create_Set_Internal_Surface(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    s = p.faces
    myFace = s.findAt((x,y,z),)
    face = face + (s[myFace.index:myFace.index+1], )
    p.Surface(side2Faces=face, name=set_name)
#-----------------------------------------------------------------------------

def Create_Set_External_Surface(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    s = p.faces
    myFace = s.findAt((x,y,z),)
    face = face + (s[myFace.index:myFace.index+1], )
    p.Surface(side1Faces=face, name=set_name)

#-----------------------------------------------------------------------------

def Create_Assembly(model,part,instance_name):
    a = mdb.models[model].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models[model].parts[part]
    a.Instance(name=instance_name, part=p, dependent=ON)

#-------------------------------------------------------------

def Create_Reference_Point(x,y,z,model,setname):
    a = mdb.models[model].rootAssembly
    myRP = a.ReferencePoint(point=(x, y, z))
    r = a.referencePoints
    myRP_Position = r.findAt((x, y, z),)
    refPoints1=(myRP_Position, )
    a.Set(referencePoints=refPoints1, name=setname)
    return myRP,myRP_Position

def Create_Constraint_Equation(model,constraint_name,set_name,set_name_rp):
    mdb.models[model].Equation(name=constraint_name, terms=((1.0, set_name, 3), (-1.0, set_name_rp, 3)))


def Create_Boundary_Condition_by_Instance(model,instance_name,set_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].sets[set_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  


def Create_Boundary_Condition_for_Assembly(model,set_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.sets[set_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  

def Create_Boundary_Condition_by_RP(model,RP_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.sets[RP_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  

#-----------------------------------------------------------------------------
def Create_Analysis_Step(model,step_name,pre_step_name,Initial_inc,Max_inc,Min_inc,Inc_Number,NL_ON_OFF):
    a = mdb.models[model].StaticStep(name=step_name, previous=pre_step_name, initialInc=Initial_inc, maxInc=Max_inc, minInc=Min_inc)
    a = mdb.models[model].steps[step_name].setValues(maxNumInc=Inc_Number)
    a = mdb.models[model].steps[step_name].setValues(nlgeom=NL_ON_OFF)
    #a = mdb.models[model].steps[step_name].setValues(stabilizationMagnitude=1E-009, stabilizationMethod=DAMPING_FACTOR, continueDampingFactors=False, adaptiveDampingRatio=None)
    a = mdb.models[model].fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'P', 'CSTRESS', 'CDISP'))
#-----------------------------------------------------------------------------

def Create_Partion_by_Plane(model,part,id_plane):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[id_plane], cells=c)


def Create_Partion_by_Plane_2D(model,part,id_plane):
    p = mdb.models[model].parts[part]
    f = p.faces[:]
    d = p.datums
    p.PartitionFaceByDatumPlane(datumPlane=d[id_plane], faces=f)

#-----------------------------------------------------------------------------

def Create_Composite_Layup(model,part,set_name,composite_name,number,material,thickness,angle):
    layupOrientation = None
    p = mdb.models[model].parts[part]
    region1=p.sets[set_name]
    normalAxisRegion = p.surfaces['Outer_Surface']
    primaryAxisRegion = p.sets['Set-Top-Edge']
    compositeLayup = mdb.models[model].parts[part].CompositeLayup(name=composite_name, description='', elementType=CONTINUUM_SHELL, symmetric=False)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF)
    for i in range(0,number,1):
        compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material, thicknessType=SPECIFY_THICKNESS, thickness=thickness, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
        compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
#-----------------------------------------------------------------------------

def Create_Composite_Layup_2D(model,part,set_name,composite_name,number,material,thickness,angle):
    layupOrientation = None
    p = mdb.models[model].parts[part]
    region1=p.sets[set_name]
    normalAxisRegion = p.surfaces['Outer_Surface']
    primaryAxisRegion = p.sets['Set-Top-Edge']
    compositeLayup = mdb.models[model].parts[part].CompositeLayup(name=composite_name, description='', elementType=SHELL, symmetric=False)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF)
    for i in range(0,number,1):
        compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material, thicknessType=SPECIFY_THICKNESS, thickness=thickness, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
        compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
     
#----------------------------------------------------------------------------

def Create_Mesh(model,part,size):
    p = mdb.models[model].parts[part]
    elemType1 = mesh.ElemType(elemCode=SC8R, elemLibrary=STANDARD, secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=SC6R, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=UNKNOWN_TET, elemLibrary=STANDARD)
    cells = p.cells[:]
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.seedPart(size=size, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

def Create_Mesh_Solid(model,part,size):
    p = mdb.models[model].parts[part]
    elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
    cells = p.cells[:]
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.seedPart(size=size, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()
 
#------------------------------------------------------------------------------

def Create_Mesh_Shell(model,part,size):
    p = mdb.models[model].parts[part]
    elemType1 = mesh.ElemType(elemCode=S4R, elemLibrary=STANDARD, secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=S3, elemLibrary=STANDARD)
    faces = p.faces[:]
    pickedRegions =(faces, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))
    p.seedPart(size=size, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

def Create_SPLA(model,instance_name,set_name,load_name,step_name,load):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].sets[set_name]
    mdb.models[model].ConcentratedForce(name=load_name, createStepName=step_name, region=region, cf1=-load, distributionType=UNIFORM, field='', localCsys=None)

#------------------------------------------------------------------------------
def Create_CF(model,set_name,load_name,step_name,load):
    a = mdb.models[model].rootAssembly
    region = a.sets[set_name]
    mdb.models[model].ConcentratedForce(name=load_name, createStepName=step_name, region=region, cf1=UNSET,cf2=UNSET,cf3=load, distributionType=UNIFORM, field='', localCsys=None)

#------------------------------------------------------------------------------

def Create_Pressure_Load(model,instance_name,load_name,step_name,surface,load):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].surfaces[surface]
    mdb.models[model].Pressure(name=load_name, createStepName=step_name, region=region, distributionType=UNIFORM, field='', magnitude=load, amplitude=UNSET)    

def CreateCutout(model,part,radius_cutout,id_plane,edge,x,y,z):
    p = mdb.models[model].parts[part]
    e, d = p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=d[id_plane], sketchUpEdge=edge, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(x, y, z))
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=2000.0, gridSpacing=20.0, transform=t)
    g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius_cutout, 0.0))
    p.CutExtrude(sketchPlane=d[id_plane], sketchUpEdge=edge, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()
    del mdb.models[model].sketches['__profile__']

def AssignStack(model,part,face):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    p.assignStackDirection(referenceRegion=face, cells=c)

def CreateJob(model,job_name,cpu):
    a = mdb.models[model].rootAssembly
    mdb.Job(name=job_name, model=model, description='', type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=cpu, numDomains=cpu, numGPUs=0)

def SubmitJob(job_name):
    mdb.jobs[job_name].submit()
    mdb.jobs[job_name].waitForCompletion()


def Create_Material_Isotropic(model,material_name_isotropic,E,nu,Y):
    mdb.models[model].Material(name=material_name_isotropic)
    mdb.models[model].materials[material_name_isotropic].Elastic(table=((E, nu), ))
    mdb.models[model].materials[material_name_isotropic].Plastic(table=((Y, 0.0), ))

def Create_Isotropic_Section(model,section_name,material_name_isotropic):
    mdb.models[model].HomogeneousSolidSection(name=section_name, material=material_name_isotropic, thickness=None)

def Create_Isotropic_Section_2D(model,section_name,material_name_isotropic,thickness):
    mdb.models[model].HomogeneousShellSection(name=section_name, preIntegrate=OFF, material=material_name_isotropic, thicknessType=UNIFORM, thickness=thickness, thicknessField='', nodalThicknessField='', idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)

def Assign_Isotropic_Material(model,part,set_name,section_name):
    p = mdb.models[model].parts[part]
    region = p.sets[set_name]
    p.SectionAssignment(region=region, sectionName=section_name, offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

def Deactivate_BC(model,BC_name,step_name):
    mdb.models[model].boundaryConditions[BC_name].deactivate(step_name)

def Rotate_Instance(model,instance,x,y,z,angle):
    a = mdb.models[model].rootAssembly
    a.rotate(instanceList=(instance, ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(x, y, z), angle=angle)

#------------------------------------------------------------------------------

def Boolean_Merge_and_Remove(model,part,instance_merge,instance_1,instance_2):
    a = mdb.models[model].rootAssembly
    a.InstanceFromBooleanMerge(name=instance_merge, instances=(a.instances[instance_1], a.instances[instance_2], ), keepIntersections=ON, originalInstances=SUPPRESS, domain=GEOMETRY)
    p = mdb.models[model].parts[instance_merge]
    f = p.faces
    p.RemoveFaces(faceList = f[1:2], deleteCells=False)    
    del mdb.models[model].parts[part]
    mdb.models[model].parts.changeKey(fromName=instance_2, toName=part)
#------------------------------------------------------------------------------

def Open_ODB_and_Write_NodeSet_data_to_text(model,step_name,variable_name,set_name,Variable_component):
    # open ODB file - ABAQUS Result file
    odb = session.openOdb(str(model)+'.odb')
    
    # list for the VARIABLE you want to evaluate
    Variable_v = []
    
    # analysis step for your VARIABLE
    lastStep=odb.steps[step_name]
    
    #loop over all increments of the analysis step and save VARIABLE information from each increment
    for x in range(len(lastStep.frames)):
        lastFrame = lastStep.frames[x]
        Variable = lastFrame.fieldOutputs[variable_name]
        center = odb.rootAssembly.nodeSets[set_name]
        centerRForce = Variable.getSubset(region=center)
       
        # loop over the VARIABLE and save component (x,y,z - 0,1,2) to list
        for i in centerRForce.values:
            Variable_vr = [i.data[Variable_component]]
            Variable_v = Variable_v + Variable_vr
    
    # Max value of Variable_v
    Max_Variable = [np.max(Variable_v)] 
    Max_Variable_v = [Max_Variable]
            
    # write VARIABLE - component to text file
    
    np.savetxt(str(variable_name)+'_'+str(myString)+'.txt',Variable_v)
    return Max_Variable

#------------------------------------------------------------------------------

def Write_Variable_to_text(variable,variable_name):
         
    # write VARIABLE - component to text file
    
    np.savetxt(str(variable_name)+'_'+str(myString)+'.txt',variable)    
    
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------

# variables

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------


# This script can only create trunecated cones, 
# so check if myLength is long enough (or the absolute of mySemi_Vertex_Angle is small enough)
# myRadius_r cannot be zero or negativ

myRadius = 100.61
myThickness = 2.91
myLength = 100.11
mySemi_Vertex_Angle = -26.65
mySlant_Length = myLength/np.cos(mySemi_Vertex_Angle*np.pi/180.0)
myRadius_r = myRadius+myLength*tan(mySemi_Vertex_Angle*np.pi/180.0)


myPart = "Cylinder"
myPart_2 = "Cone"
# material parameters

myE = 210490
myNu = 0.281
myY = 230.6



myAngle = [45,-45,0,90,90,0,-45,45]

myPlyNumber = len(myAngle)


Mesh_Size = 1.5

N = []
P = []
for ic in range(1,2,1):
    
    myString = "GNIA_eP_aC_Loop_"+str(ic)
    mdb.Model(name=myString)
    
    # Imperfection Parameter
    myPerturbation = 0.3*ic
    
    
    #------------------------------------------------------------------------------
    #------------------------------------------------------------------------------
    
    # create model
    
    #------------------------------------------------------------------------------
    #------------------------------------------------------------------------------
    
    Create_Part_2D_Cylinder(myRadius,myLength,myPart,myString)
    Create_Set_All_Faces(myString,myPart,"Cylinder_2D")
    Create_Part_2D_Cone(myRadius,myLength,mySemi_Vertex_Angle,myPart_2,myString)
    
    Create_Assembly(myString,myPart,"Cylinder-1")
    Create_Assembly(myString,myPart_2,"Cone-1")
    Rotate_Instance(myString,"Cone-1",1,0,0,90.0)
    Boolean_Merge_and_Remove(myString,myPart_2,"Cone-1","Cylinder-1","Cone-1")
    
    myID_1 = Create_Datum_Plane_by_Principal(XZPLANE,myPart_2,myString,0.0)
    myID_2 = Create_Datum_Plane_by_Principal(XYPLANE,myPart_2,myString,myLength/2.0)
    myID_3 = Create_Datum_Plane_by_Principal(YZPLANE,myPart_2,myString,0.0)
    
    Create_Set_Edge(myRadius,0.0,0.0,myString,myPart_2,"Set-RP-1")
    Create_Set_Edge(myRadius_r,0.0,myLength,myString,myPart_2,"Set-RP-2")
    Create_Set_External_Surface((myRadius+myRadius_r)/2.0,0.0,myLength/2.0,myString,myPart_2,"Outer_Surface")
    Create_Set_Internal_Surface((myRadius+myRadius_r)/2.0,0.0,myLength/2.0,myString,myPart_2,"Internal_Surface")
    Create_Set_All_Faces(myString,myPart_2,"Cone_2D")
    
    # Create_Material_Data_2D(myString,"CFRP",myE11,myE22,myNu12,myG12,myG13,myG23)
    Create_Material_Isotropic(myString,"Steel",myE,myNu,myY)
    myRP1,myRP_Position1 = Create_Reference_Point(0.0,0.0,0.0,myString,"RP-1")
    #myRP2,myRP_Position2 = Create_Reference_Point(0.0,0.0,myLength,myString,"RP-2")
    
    Create_Constraint_Equation(myString,"Constraint-RP-1","Cone-1-1."+str("Set-RP-1"),"RP-1")
    #Create_Constraint_Equation(myString,"Constraint-RP-2","Cylinder-1."+str("Set-RP-2"),"RP-2")
    
    Create_Analysis_Step(myString,"Step-1","Initial",0.1,0.1,1E-05,300,ON)
    Create_Analysis_Step(myString,"Step-2","Step-1",0.01,0.01,1E-05,300,ON)
    
    Create_Boundary_Condition_by_Instance(myString,"Cone-1-1","Set-RP-1","BC-Set-RP-1","Initial",SET,SET,UNSET,SET,SET,SET)
    Create_Boundary_Condition_by_Instance(myString,"Cone-1-1","Set-RP-2","BC-Set-RP-2","Initial",SET,SET,SET,SET,SET,SET)
    # Create_Boundary_Condition_by_RP(myString,"RP-1","Displacement_Load","Step-2",UNSET,UNSET,5,UNSET,UNSET,UNSET)
    
    
    Create_Partion_by_Plane_2D(myString,myPart_2,myID_3)
    myEdge = Create_Set_Edge(0.0,(myRadius+myRadius_r)/2.0,myLength/2.0,myString,myPart_2,"Set-Top-Edge")
    Create_Partion_by_Plane_2D(myString,myPart_2,myID_1)
    Create_Partion_by_Plane_2D(myString,myPart_2,myID_2)
    
    
    Create_Set_Vertice_2((myRadius+myRadius_r)/2.0,0.0,myLength/2.0,myString,"Cone-1-1","SPLA_Point")
    Create_Boundary_Condition_for_Assembly(myString,"SPLA_Point","SPDA-Imperfection","Step-1",-myPerturbation,UNSET,UNSET,UNSET,UNSET,UNSET)
    Deactivate_BC(myString,"SPDA-Imperfection","Step-2")
    Create_CF(myString,"RP-1","CF","Step-1",100.0)
    Create_Pressure_Load(myString,"Cone-1-1","External_Pressure","Step-2","Outer_Surface",10.0)
    
    # Create_Composite_Layup_2D(myString,myPart_2,"Cone_2D","An_Isotropic",myPlyNumber,"CFRP",myThickness/myPlyNumber,myAngle)
    Create_Isotropic_Section_2D(myString,"Steel_section","Steel",myThickness)
    Assign_Isotropic_Material(myString,myPart_2,"Cone_2D","Steel_section")
    
    Create_Mesh_Shell(myString,myPart_2,Mesh_Size)
    myFace = Create_Set_Face((myRadius+myRadius_r)/2.0,0.0,myLength/2.0,myString,myPart_2,"Outer_Surface")
    # AssignStack(myString,myPart_2,myFace)
    
    #------------------------------------------------------------------------------
    #------------------------------------------------------------------------------
    
    # create Job for analysis
    
    #------------------------------------------------------------------------------
    #------------------------------------------------------------------------------
    
    CreateJob(myString,myString,8)
    
    # SubmitJob(myString)
    
    #------------------------------------------------------------------------------
    #------------------------------------------------------------------------------
    
    # evaluate ABAQUS results
    
    #------------------------------------------------------------------------------
    #------------------------------------------------------------------------------
    
    N.append(Open_ODB_and_Write_NodeSet_data_to_text(myString,"Step-2","RF","RP-1",2))
    P.append(Open_ODB_and_Write_NodeSet_data_to_text(myString,"Step-2","RF","SPLA_POINT",0))
                                                                           
    Write_Variable_to_text(N,"Buckling Load")
    Write_Variable_to_text(P,"Perturbation Reaction Force")