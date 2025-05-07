#include "assembler/MeshFunctions.hpp"

gp_Pnt ShapeCentroid(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return gp_Pnt((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2);
}

TopoDS_Shape ShapeSetCentroid(TopoDS_Shape shape, gp_Pnt position)
{
    gp_Pnt current_centroid = ShapeCentroid(shape);

    gp_Vec move(position, current_centroid);

    TopoDS_Shape moved_shape = TranslateShape(shape, move);

    return moved_shape;
}

gp_Pnt ShapeCenterOfMass(TopoDS_Shape shape)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);

    return props.CentreOfMass();
}

Bnd_Box ShapeBoundingBox(TopoDS_Shape shape)
{
    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);

    return bbox;
}

TopoDS_Shape ShapeBoundingBoxShape(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    TopoDS_Shape bbox_shape = BRepPrimAPI_MakeBox(xmax - xmin, ymax - ymin, zmax - zmin).Shape();

    gp_Pnt shape_centroid = ShapeCentroid(shape);

    gp_Pnt bbox_centroid = ShapeCentroid(bbox_shape);

    gp_Vec move(bbox_centroid, shape_centroid);

    bbox_shape = TranslateShape(bbox_shape, move);

    return bbox_shape;
}

TopoDS_Shape ShapeHighBoundingBoxShape(TopoDS_Shape shape, Standard_Real height)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    TopoDS_Shape bbox_shape = BRepPrimAPI_MakeBox(xmax - xmin, ymax - ymin, height).Shape();

    gp_Pnt shape_centroid = ShapeCentroid(shape);

    gp_Pnt bbox_centroid = ShapeCentroid(bbox_shape);

    Standard_Real bbox_zmin = ShapeLowestPoint(bbox_shape);

    gp_Vec move(shape_centroid.X() - bbox_centroid.X(), shape_centroid.Y() - bbox_centroid.Y(), zmin - bbox_zmin);
    
    bbox_shape = TranslateShape(bbox_shape, move);

    return bbox_shape;
}

Standard_Real ShapeLowestPoint(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return zmin;
}

Standard_Real ShapeHighestPoint(TopoDS_Shape shape)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return zmax;
}

Standard_Real ShapeAxisSize(TopoDS_Shape shape, int axis)
{
    Bnd_Box bbox = ShapeBoundingBox(shape);

    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    if (axis == 0)
        return xmax - xmin;

    if (axis == 1)
        return ymax - ymin;

    if (axis == 2)
        return zmax - zmin;

    std::cerr << "Axis not available" << std::endl;

    return 0;
}

TopoDS_Shape TranslateShape(TopoDS_Shape shape, gp_Vec vec)
{
    gp_Trsf move_trsf;

    move_trsf.SetTranslation(vec);

    return shape.Moved(move_trsf);
}

TopoDS_Shape NonUniformScaleShape(TopoDS_Shape shape, gp_Pnt scaling)
{
    std::cout << "Scaling shape" << std::endl;

    gp_GTrsf gtrsf;
    gtrsf.SetValue(1, 1, scaling.X());
    gtrsf.SetValue(2, 2, scaling.Y());
    gtrsf.SetValue(3, 3, scaling.Z());

    return BRepBuilderAPI_GTransform(shape, gtrsf, true).Shape();
}

TopoDS_Shape UniformScaleShape(TopoDS_Shape shape, Standard_Real scaling)
{
    gp_Pnt scaleCenter = ShapeCentroid(shape);

    gp_Trsf trsf;
    trsf.SetScale(scaleCenter, scaling);

    return BRepBuilderAPI_Transform(shape, trsf, true).Shape();
}

TopoDS_Shape SubtractShapeBFromA(TopoDS_Shape shape_A, TopoDS_Shape shape_B)
{
    // Perform the subtraction: A - B
    BRepAlgoAPI_Cut cut(shape_A, shape_B);

    cut.Build();

    if (!cut.IsDone()) {
        std::cerr << "Cut operation failed!" << std::endl;
    }

    return cut.Shape();
}

void SaveShapeAsSTL(TopoDS_Shape shape, std::string filename)
{
    BRepMesh_IncrementalMesh(shape, 0.1);  // Mesh with a 0.1 tolerance

    // Export the result to STL
    StlAPI_Writer writer;
    writer.Write(shape, filename.c_str());
}

// void ProjectedContourFromShape(TopoDS_Shape shape, gp_Pnt origin, gp_Dir normal)
// {
//     gp_Pln plane(origin, normal);

//     TopoDS_Face planeFace = BRepBuilderAPI_MakeFace(plane);
//     BRepAlgoAPI_Section sectionAlgo(shape, planeFace, false);
//     sectionAlgo.ComputePCurveOn1(Standard_True);
//     sectionAlgo.Build();

//     TopoDS_Shape sectionShape = sectionAlgo.Shape();  // Contains edges on the projection plane

//     std::cout << "check1" << std::endl;

//     // 2. Collect edges
//     BRepBuilderAPI_MakeWire wireBuilder;
//     for (TopExp_Explorer ex(sectionShape, TopAbs_EDGE); ex.More(); ex.Next()) {
//         TopoDS_Edge edge = TopoDS::Edge(ex.Current());
//         wireBuilder.Add(edge);
//     }

//     std::cout << "check2" << std::endl;

//     TopoDS_Wire wire = wireBuilder.Wire();

//     std::cout << "check3" << std::endl;

//     // 3. Create face
//     TopoDS_Face face = BRepBuilderAPI_MakeFace(wire);

//     std::cout << "check4" << std::endl;

//     // 4. Extrude
//     gp_Vec extrusionVec(0, 0, 10);  // change vector as needed
//     TopoDS_Shape prism = BRepPrimAPI_MakePrism(face, extrusionVec);

//     std::cout << "check5" << std::endl;

//     SaveShapeAsSTL(prism, "section_test.stl");

//     std::cout << "check6" << std::endl;
// }






// Point facetLowestPoint(Polyhedron::Facet_handle facet)
// {
//     auto halfedge = facet->halfedge();  // Get one halfedge of the facet
//     auto start = halfedge;  // Store the starting halfedge

//     auto lowest_z = halfedge->vertex()->point().z();

//     Point lowest_point = halfedge->vertex()->point();

//     do {
//         auto z = halfedge->vertex()->point().z();
        
//         if (z < lowest_z)
//         {
//             lowest_z = z;
//             lowest_point = halfedge->vertex()->point();
//         }

//         halfedge = halfedge->next();  // Move to the next halfedge in the facet
//     } while (halfedge != start);  // Stop when we loop back to the start

//     return lowest_point;
// }

// double meshLowestPoint(std::shared_ptr<Polyhedron> mesh)
// {
//     if (mesh->size_of_facets() == 0)
//     {
//         std::cout << "Mesh has no facets" << std::endl;
//         return 0;
//     }

//     return meshBoundingBox(mesh).zmin();
// }

// bool saveMesh(std::shared_ptr<Polyhedron> mesh, std::string filename)
// {
//     std::ofstream out(filename);
//     if (!out) {
//         throw std::runtime_error("Failed to open output file.");
//         return false;
//     }

//     out << "solid polyhedron\n";
//     for (auto f = mesh->facets_begin(); f != mesh->facets_end(); ++f) {
//         // Get three vertices of the face (assuming triangular faces)
//         auto h = f->facet_begin();
//         Point p1 = h->vertex()->point();
//         Point p2 = (++h)->vertex()->point();
//         Point p3 = (++h)->vertex()->point();

//         // Compute a simple normal (not necessarily unit length)
//         Kernel::Vector_3 normal = CGAL::cross_product(p2 - p1, p3 - p1);

//         out << "  facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
//         out << "    outer loop\n";
//         out << "      vertex " << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
//         out << "      vertex " << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
//         out << "      vertex " << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
//         out << "    endloop\n";
//         out << "  endfacet\n";
//     }
//     out << "endsolid polyhedron\n";

//     out.close();

//     return true;
// }

// bool saveMesh(std::shared_ptr<SurfaceMesh> mesh, std::string filename)
// {
//     std::ofstream out(filename, std::ios::binary);
//     if (!out) {
//         std::cerr << "Error: couldn't open output file." << std::endl;
//         return 1;
//     }

//     if (!CGAL::IO::write_STL(out, *mesh)) {
//         std::cerr << "Error: failed to write STL file." << std::endl;
//         return 1;
//     }

//     std::cout << "Mesh successfully written to output.stl" << std::endl;
//     return 0;
// }

// void debugMesh(std::shared_ptr<Polyhedron> poly, std::string name)
// {
//     std::cout << std::endl << name << " Information:\n";
    
//     // Print total number of vertices and faces
//     std::cout << "Total vertices: " << poly->size_of_vertices() << "\n";
//     std::cout << "Total faces: " << poly->size_of_facets() << "\n";

//     // Create a mapping from vertex handles to indices
//     std::map<Polyhedron::Vertex_const_handle, int> vertex_index;
//     int index = 0;
    
//     // Print indexed list of vertices
//     std::cout << "\nVertices (index: x, y, z):\n";
//     for (auto v = poly->vertices_begin(); v != poly->vertices_end(); ++v, ++index) {
//         vertex_index[v] = index;
//         std::cout << index << ": (" << v->point().x() << ", " 
//                                   << v->point().y() << ", " 
//                                   << v->point().z() << ")\n";
//     }

//     // Print faces with indexed vertices
//     std::cout << "\nFaces (vertex indices):\n";
//     index = 0;
//     for (auto f = poly->facets_begin(); f != poly->facets_end(); ++f, ++index) {
//         std::cout << "Face " << index << ": ";

//         auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(f, *poly);
//             std::cout << "Facet normal: " 
//                       << normal.x() << " " 
//                       << normal.y() << " " 
//                       << normal.z() << std::endl;
//         // Iterate over the halfedges of the face
//         auto h = f->halfedge();
//         do {
//             std::cout << vertex_index[h->vertex()] << " ";
//             h = h->next();
//         } while (h != f->halfedge());
        
//         std::cout << "\n";
//     }
// }

// void debugNefMesh(Nef_polyhedron mesh, std::string name)
// {
//     std::cout << std::endl << "Nef mesh: " << name << std::endl;

//     for (Nef_polyhedron::SFace_const_iterator sface_it = mesh.sfaces_begin(); sface_it != mesh.sfaces_end(); ++sface_it) {
//         std::cout << "New facet: " << std::endl;

//         // Iterate over the halfedges that form the boundary of the facet
//         Nef_polyhedron::SHalfedge_const_handle she = sface_it->sface_cycles_begin();
//         if (she == nullptr) continue; // Skip empty faces

//         Nef_polyhedron::SHalfedge_const_handle start = she; // Store start to detect loops
//         do {
//             Kernel::Point_3 p = she->source()->point();
//             std::cout << "  Vertex: (" << CGAL::to_double(p.x()) << ", "
//                                         << CGAL::to_double(p.y()) << ", "
//                                         << CGAL::to_double(p.z()) << ")" << std::endl;

//             she = she->next(); // Move to next halfedge in the facet loop
//         } while (she != start);
//     }

//     // for (auto f = result.facets_begin(); f != cradle->facets_end(); ++f) {
//     //     // Get three vertices of the face (assuming triangular faces)
//     //     auto h = f->facet_begin();
//     //     Point p1 = h->vertex()->point();
//     //     Point p2 = (++h)->vertex()->point();
//     //     Point p3 = (++h)->vertex()->point();

//     //     // Compute a simple normal (not necessarily unit length)
//     //     Kernel::Vector_3 normal = CGAL::cross_product(p2 - p1, p3 - p1);

//     //     out << "  facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
//     //     out << "    outer loop\n";
//     //     out << "      vertex " << p1.x() << " " << p1.y() << " " << p1.z() << "\n";
//     //     out << "      vertex " << p2.x() << " " << p2.y() << " " << p2.z() << "\n";
//     //     out << "      vertex " << p3.x() << " " << p3.y() << " " << p3.z() << "\n";
//     //     out << "    endloop\n";
//     //     out << "  endfacet\n";
//     // }



//     // for (auto it = result.vertices_begin(); it != result.vertices_end(); ++it) {
//     //     const Point& p = it->point();
//     //     std::cout << "Vertex: (" << CGAL::to_double(p.x()) << ", " 
//     //               << CGAL::to_double(p.y()) << ", " 
//     //               << CGAL::to_double(p.z()) << ")\n";
//     // }
// }

// void positionMesh(std::shared_ptr<Polyhedron> mesh, Point target_position)
// {
//     Point initial_center = meshCenter(mesh);

//     Vector delta = target_position - initial_center;

//     //Substract initial center from each vertex to create new center of 0,0,0

//     for (auto vertex = mesh->vertices_begin(); vertex != mesh->vertices_end(); vertex++)
//     {
//         vertex->point() += delta;
//     }
// }

// Point meshCenter(std::shared_ptr<Polyhedron> mesh)
// {
//     if (mesh == nullptr)
//         return Point(0, 0, 0);

//     if (mesh->size_of_vertices() == 0)
//         return Point(0, 0, 0);

//     if (!mesh->is_closed())
//     {
//         std::cout << "Can't find centroid of open mesh" << std::endl;

//         return Point(0, 0, 0);
//     }

//     BoundingBox bbox = meshBoundingBox(mesh);

//     double cx = (bbox.xmin() + bbox.xmax()) / 2.0;
//     double cy = (bbox.ymin() + bbox.ymax()) / 2.0;
//     double cz = (bbox.zmin() + bbox.zmax()) / 2.0;

//     return Point(cx, cy, cz);
// }

// void scaleMesh(std::shared_ptr<Polyhedron> mesh, double scale_factor_x, double scale_factor_y, double scale_factor_z)
// {

//     Transformation scaleTransform(  Kernel::RT(scale_factor_x), Kernel::RT(0), Kernel::RT(0), Kernel::RT(0),
//                                     Kernel::RT(0), Kernel::RT(scale_factor_y), Kernel::RT(0), Kernel::RT(0),
//                                     Kernel::RT(0), Kernel::RT(0), Kernel::RT(scale_factor_z), Kernel::RT(0));
    
    
    



//     //Transformation scaleTransform(CGAL::SCALING, Kernel::RT(scale_factor_x))

//     for (auto v = mesh->points_begin(); v != mesh->points_end(); ++v) {
//         *v = scaleTransform(*v);  // Apply transformation
//     }
// }

// BoundingBox meshBoundingBox(std::shared_ptr<Polyhedron> mesh)
// {
//     if (mesh->size_of_vertices() == 0)
//         return CGAL::Bbox_3(0, 0, 0, 0, 0, 0);  // Return empty bbox if mesh is empty

//     double xmin = std::numeric_limits<double>::max();
//     double ymin = std::numeric_limits<double>::max();
//     double zmin = std::numeric_limits<double>::max();
//     double xmax = std::numeric_limits<double>::lowest();
//     double ymax = std::numeric_limits<double>::lowest();
//     double zmax = std::numeric_limits<double>::lowest();

//     for (auto v = mesh->vertices_begin(); v != mesh->vertices_end(); ++v) {
//         const Point& p = v->point();
//         xmin = std::min(xmin, CGAL::to_double(p.x()));
//         ymin = std::min(ymin, CGAL::to_double(p.y()));
//         zmin = std::min(zmin, CGAL::to_double(p.z()));
//         xmax = std::max(xmax, CGAL::to_double(p.x()));
//         ymax = std::max(ymax, CGAL::to_double(p.y()));
//         zmax = std::max(zmax, CGAL::to_double(p.z()));
//     }

//     return CGAL::Bbox_3(xmin, ymin, zmin, xmax, ymax, zmax);
// }

// void translateMesh(std::shared_ptr<Polyhedron> mesh, Vector translation)
// {
//     Transformation moveTransform(CGAL::TRANSLATION, translation);

//     for (auto v = mesh->points_begin(); v != mesh->points_end(); ++v) {
//         *v = moveTransform(*v);  // Apply transformation
//     }
// }

