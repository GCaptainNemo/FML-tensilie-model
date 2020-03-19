# -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior

def Tensile_model(Metal_text, FRP_text, k1, table_val):
    """
    table_val:table value
    k1:part name
    """
    # depth
    obj_lst = []
    depth = 0
    for _ in table_val:
        if _[0] != _[1]:
            obj_lst.append(_)
            depth += _[2]
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.Line(point1=(0.0, 40.0), point2=(10.0, 40.0))
    s1.HorizontalConstraint(entity=g[2], addUndoState=False)
    s1.Line(point1=(10.0, 40.0), point2=(10.0, 15.0))
    s1.VerticalConstraint(entity=g[3], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[2], entity2=g[3], addUndoState=False)
    s1.Line(point1=(10.0, 15.0), point2=(5.0, 0.0))
    s1.Line(point1=(5.0, 0.0), point2=(5.0, -50.0))
    s1.VerticalConstraint(entity=g[5], addUndoState=False)
    s1.FilletByRadius(radius=100.0, curve1=g[4], nearPoint1=(7.35569190979004, 
        7.46994590759277), curve2=g[5], nearPoint2=(5.0147647857666, 
        -7.51091575622559))
    s1.Line(point1=(10.0, 15.0), point2=(10.0, 14.646689414978))
    s1.VerticalConstraint(entity=g[7], addUndoState=False)
    s1.ParallelConstraint(entity1=g[3], entity2=g[7], addUndoState=False)
    s1.autoTrimCurve(curve1=g[4], point1=(10.0378875732422, 15.1052093505859))
    s1.autoTrimCurve(curve1=g[6], point1=(10.0462617874146, 15.1093778610229))
    s1.autoTrimCurve(curve1=g[7], point1=(10.0016756057739, 14.7144193649292))
    s1.Line(point1=(0.0, 40.0), point2=(0.0, -50.0))
    s1.VerticalConstraint(entity=g[10], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[2], entity2=g[10], addUndoState=False)
    s1.Line(point1=(0.0, -50.0), point2=(5.0, -50.0))
    s1.HorizontalConstraint(entity=g[11], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[10], entity2=g[11], addUndoState=False)
    s1.setAsConstruction(objectList=(g[10], ))
    s1.setAsConstruction(objectList=(g[11], ))
    s1.copyMirror(mirrorLine=g[10], objectList=(g[2], g[3], g[5], g[8], g[9]))
    s1.copyMirror(mirrorLine=g[11], objectList=(g[2], g[3], g[5], g[8], g[9], 
        g[10], g[12], g[13], g[14], g[15], g[16]))


    p = mdb.models['Model-1'].Part(name=k1, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[k1]
    # p.BaseSolidExtrude(sketch=s1, depth=float(k2))
    p.BaseSolidExtrude(sketch=s1, depth=depth)
    s1.unsetPrimaryObject()
    p = mdb.models['Model-1'].parts[k1]
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Partition_model(k1, depth)
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Materials_model()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Create_section(tuple(obj_lst), k1, Metal_text, FRP_text)
    # Assembly and step ~~~~~~~~~~~~~~~~~~~~~~~~~
    Assembly_Step(k1)
    # Load and Mest
    Load_Mesh(k1)

def Partition_model(partname, depth):
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e, d = p.edges, p.datums
    pickedEdges =(e[7], )
    p.PartitionCellByExtrudeEdge(line=e[0], cells=pickedCells, edges=pickedEdges, 
        sense=REVERSE)
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e1, d1 = p.edges, p.datums
    pickedEdges =(e1[30], )
    p.PartitionCellByExtrudeEdge(line=e1[24], cells=pickedCells, edges=pickedEdges, 
        sense=REVERSE)
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#2 ]', ), )
    e, d = p.edges, p.datums
    pickedEdges =(e[47], )
    p.PartitionCellByExtrudeEdge(line=e[3], cells=pickedCells, edges=pickedEdges, 
        sense=FORWARD)
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    e1, d1 = p.edges, p.datums
    pickedEdges =(e1[51], )
    p.PartitionCellByExtrudeEdge(line=e1[14], cells=pickedCells, edges=pickedEdges, 
        sense=FORWARD)
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Create reference point
    p = mdb.models['Model-1'].parts[partname]
    p.ReferencePoint(point=(0, 40, depth / 2))

def Create_section(obj_lst, name, metal_text, frp_text):
    layup = []
    num = 1
    for _ in obj_lst:
        if _[0]:
            layup.append(section.SectionLayer(material=metal_text,
                                              thickness=_[2], orientAngle=_[3], numIntPts=3, plyName="p%d"%num))
        else:
            layup.append(section.SectionLayer(material=frp_text,
                                              thickness=_[2], orientAngle=_[3], numIntPts=3, plyName="p%d"%num))
        num += 1
    mdb.models['Model-1'].CompositeShellSection(name='Section-1', preIntegrate=OFF, idealization=NO_IDEALIZATION, symmetric=False, thicknessType=UNIFORM, poissonDefinition=DEFAULT, thicknessModulus=None,
                                                temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON, layup=tuple(layup, ))
    p = mdb.models['Model-1'].parts[name]
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#1f ]', ), )
    region = regionToolset.Region(cells=cells)
    p = mdb.models['Model-1'].parts[name]
    p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)

def Assembly_Step(partname):
    a = mdb.models['Model-1'].rootAssembly
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts[partname]
    a.Instance(name=partname + '-1', part=p, dependent=ON)
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Interaction
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.instances[partname + '-1'].referencePoints
    refPoints1 = (r1[6],)
    region1 = regionToolset.Region(referencePoints=refPoints1)
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances[partname + '-1'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#1000000 ]',), )
    region2 = regionToolset.Region(faces=faces1)
    mdb.models['Model-1'].Coupling(name='Constraint-1', controlPoint=region1,
                                   surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC,
                                   localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)
    a = mdb.models['Model-1'].rootAssembly
    a.makeIndependent(instances=(a.instances[partname + '-1'],))




def Load_Mesh(partname):
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances[partname + '-1'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#400 ]',), )
    region = regionToolset.Region(faces=faces1)
    mdb.models['Model-1'].EncastreBC(name='BC-1', createStepName='Initial',
                                     region=region, localCsys=None)
    a = mdb.models['Model-1'].rootAssembly
    f1 = a.instances[partname + '-1'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#1000000 ]',), )
    region = regionToolset.Region(faces=faces1)
    mdb.models['Model-1'].DisplacementBC(name='BC-2', createStepName='Initial',
                                         region=region, u1=SET, u2=UNSET, u3=SET, ur1=SET, ur2=SET, ur3=SET,
                                         amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
    a = mdb.models['Model-1'].rootAssembly
    r1 = a.instances[partname + '-1'].referencePoints
    refPoints1 = (r1[6],)
    region = regionToolset.Region(referencePoints=refPoints1)
    mdb.models['Model-1'].ConcentratedForce(name='Load-1', createStepName='Step-1',
                                            region=region, cf2=1500.0, distributionType=UNIFORM, field='',
                                            localCsys=None)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Mesh
    # a = mdb.models['Model-1'].rootAssembly
    partInstances = (a.instances[partname + '-1'],)
    a.seedPartInstance(regions=partInstances, size=1.7, deviationFactor=0.1,
                       minSizeFactor=0.1)
    # a = mdb.models['Model-1'].rootAssembly
    partInstances = (a.instances[partname + '-1'],)
    a.generateMesh(regions=partInstances)







