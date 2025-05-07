#ifndef MESH_FUNCTIONS_HPP
#define MESH_FUNCTIONS_HPP


#include <TDF_LabelSequence.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_Label.hxx>
#include <TDF_Tool.hxx>
#include <TDocStd_Document.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Handle.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <map>


#include <TCollection_ExtendedString.hxx>
#include <TCollection_AsciiString.hxx>

#include <iostream>
#include <algorithm>
#include <cctype>
#include <string>


#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Xw_Window.hxx> // On Windows; on Linux use Xw_Window

#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx> // Example shape

#include <StlAPI_Writer.hxx>

#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>

#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Copy.hxx>

#include <TDF_ChildIterator.hxx>

#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>

#include <string>
#include <codecvt>
#include <locale>  // if you use codecvt

#include <BRepAlgoAPI_Section.hxx>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_Orientation.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <iostream>
#include <Standard_Type.hxx>

#include <BRepExtrema_DistShapeShape.hxx>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <STEPControl_Writer.hxx>

#include <gp_GTrsf.hxx>
#include <BRepBuilderAPI_GTransform.hxx>

#include <Standard_Real.hxx>

#include <BRepBuilderAPI_MakeFace.hxx>


#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakePrism.hxx>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>

// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/Polyhedron_3.h>
// #include <CGAL/Bbox_3.h>
// #include <CGAL/Polyhedral_mesh_domain_3.h>
// #include <CGAL/Polyhedron_incremental_builder_3.h>
// #include <CGAL/Aff_transformation_3.h>
// #include <CGAL/AABB_tree.h>
// #include <CGAL/AABB_traits_3.h>
// #include <CGAL/AABB_face_graph_triangle_primitive.h>
// #include <CGAL/intersections.h>
// #include <CGAL/Nef_polyhedron_3.h>
// #include <CGAL/convex_decomposition_3.h>
// #include <CGAL/Polygon_mesh_processing/repair.h>
// #include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
// #include <CGAL/Polygon_mesh_processing/compute_normal.h>
// #include <CGAL/boost/graph/Euler_operations.h>  // For remove_face()
// #include <CGAL/Surface_mesh.h>
// #include <CGAL/boost/graph/IO/polygon_mesh_io.h>
// #include <CGAL/IO/STL.h>
// #include <CGAL/Surface_mesh/IO/3MF.h>
// //#include <CGAL/IO/read_mesh.h>

#include <fstream>

// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef Kernel::Point_3 Point;
// typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
// typedef Kernel::Vector_3 Vector;
// typedef Kernel::Triangle_3 Triangle;
// typedef CGAL::Aff_transformation_3<Kernel> Transformation;
// typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
// typedef CGAL::AABB_traits_3<Kernel, Primitive> AABB_traits;
// typedef CGAL::AABB_tree<AABB_traits> AABB_tree;
// typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;
// typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
// typedef CGAL::Bbox_3 BoundingBox;

gp_Pnt ShapeCentroid(TopoDS_Shape shape);

TopoDS_Shape ShapeSetCentroid(TopoDS_Shape shape, gp_Pnt position);

gp_Pnt ShapeCenterOfMass(TopoDS_Shape shape);

Bnd_Box ShapeBoundingBox(TopoDS_Shape shape);

TopoDS_Shape ShapeBoundingBoxShape(TopoDS_Shape shape);

TopoDS_Shape ShapeHighBoundingBoxShape(TopoDS_Shape shape, Standard_Real height);

Standard_Real ShapeLowestPoint(TopoDS_Shape shape);

Standard_Real ShapeHighestPoint(TopoDS_Shape shape);

Standard_Real ShapeAxisSize(TopoDS_Shape shape, int axis);

TopoDS_Shape TranslateShape(TopoDS_Shape shape, gp_Vec vec);

TopoDS_Shape NonUniformScaleShape(TopoDS_Shape shape, gp_Pnt scaling);

TopoDS_Shape UniformScaleShape(TopoDS_Shape shape, Standard_Real scaling);

TopoDS_Shape SubtractShapeBFromA(TopoDS_Shape shape_A, TopoDS_Shape shape_B);

void SaveShapeAsSTL(TopoDS_Shape shape, std::string filename);

//void ProjectedContourFromShape(TopoDS_Shape shape, gp_Pnt origin, gp_Dir normal);

// BoundingBox meshBoundingBox(std::shared_ptr<Polyhedron> mesh);

// Point facetLowestPoint(Polyhedron::Facet_handle facet);

// double meshLowestPoint(std::shared_ptr<Polyhedron> mesh);

// bool saveMesh(std::shared_ptr<Polyhedron> mesh, std::string filename);

// bool saveMesh(std::shared_ptr<SurfaceMesh> mesh, std::string filename);

// void debugMesh(std::shared_ptr<Polyhedron> poly, std::string name);

// void debugNefMesh(Nef_polyhedron mesh, std::string name);

// void positionMesh(std::shared_ptr<Polyhedron> mesh, Point target_position);

// Point meshCenter(std::shared_ptr<Polyhedron> mesh);

// void scaleMesh(std::shared_ptr<Polyhedron> mesh, double scale_factor_x, double scale_factor_y, double scale_factor_z);

// void translateMesh(std::shared_ptr<Polyhedron> mesh, Vector translation);

#endif