#ifndef MESH_FUNCTIONS_HPP
#define MESH_FUNCTIONS_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/intersections.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/boost/graph/Euler_operations.h>  // For remove_face()
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/IO/polygon_mesh_io.h>

#include <fstream>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Triangle_3 Triangle;
typedef CGAL::Aff_transformation_3<Kernel> Transformation;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> AABB_tree;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef CGAL::Bbox_3 BoundingBox;


BoundingBox meshBoundingBox(std::shared_ptr<Polyhedron> mesh);

Point facetLowestPoint(Polyhedron::Facet_handle facet);

double meshLowestPoint(std::shared_ptr<Polyhedron> mesh);

bool saveMesh(std::shared_ptr<Polyhedron> mesh, std::string filename);

void debugMesh(std::shared_ptr<Polyhedron> poly, std::string name);

void debugNefMesh(Nef_polyhedron mesh, std::string name);

void positionMesh(std::shared_ptr<Polyhedron> mesh, Point target_position);

Point meshCenter(std::shared_ptr<Polyhedron> mesh);

void scaleMesh(std::shared_ptr<Polyhedron> mesh, double scale_factor_x, double scale_factor_y, double scale_factor_z);

void translateMesh(std::shared_ptr<Polyhedron> mesh, Vector translation);

#endif