#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <memory>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <CGAL/Polygon_mesh_processing/repair.h>

#include <CGAL/Polygon_mesh_processing/border.h>

#include <CGAL/Polygon_mesh_processing/stitch_borders.h>

#include <boost/property_map/property_map.hpp>



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

#include "assembler/Part.hpp"

class Substrate;

class Assembly;

struct TriangleMesh {
    std::vector<CGAL::Point_3<Kernel>> vertices;
    std::vector<std::array<int, 3>> faces; // Indices into vertices list
};

struct PointComparator {
    bool operator()(const gp_Pnt& p1, const gp_Pnt& p2) const {
        if (p1.X() != p2.X()) return p1.X() < p2.X();
        if (p1.Y() != p2.Y()) return p1.Y() < p2.Y();
        return p1.Z() < p2.Z();
    }
};

struct NamedPolyhedron {
    std::shared_ptr<Polyhedron> polyhedron;
    std::string name;
};

class ModelLoader {
public:
    explicit ModelLoader();
    std::shared_ptr<Assembly> loadModel(const std::string& filename);

    std::shared_ptr<Substrate> loadSubstrate(const std::string& filename);

    //std::shared_ptr<Assembly> loadSTEP(const std::string& filename);

    std::vector<std::shared_ptr<NamedPolyhedron>> loadSTEP(const std::string& filename);

    std::string GetShapeName(const TDF_Label& label);

    TriangleMesh ExtractMeshFromShape(const TopoDS_Shape& shape);

    Polyhedron ConvertToPolyhedron(const TriangleMesh& mesh);

    //std::string GetShapeName(const TopoDS_Shape& shape, const Handle(XCAFDoc_ShapeTool)& shapeTool);

    //std::string GetShapeName(const TDF_Label& label);

private:
    Assimp::Importer importer_;
};

template <class HDS>
class BuildPolyhedron : public CGAL::Modifier_base<HDS> {
public:
    const aiMesh* ai_mesh;

    BuildPolyhedron(const aiMesh* mesh) : ai_mesh(mesh) {}

    void operator()(HDS& hds) {
        typedef typename HDS::Vertex::Point Point;  // Ensure correct type resolution

        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
        builder.begin_surface(ai_mesh->mNumVertices, ai_mesh->mNumFaces);

        // Add vertices
        for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
            aiVector3D v = ai_mesh->mVertices[i];
            builder.add_vertex(Point(v.x, v.y, v.z));  // Use the correct Point type
        }

        // Add faces
        for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
            const aiFace& face = ai_mesh->mFaces[i];
            if (face.mNumIndices == 3) {  // Ensure it's a triangle
                builder.begin_facet();
                builder.add_vertex_to_facet(face.mIndices[0]);
                builder.add_vertex_to_facet(face.mIndices[1]);
                builder.add_vertex_to_facet(face.mIndices[2]);
                builder.end_facet();
            }
        }

        builder.end_surface();
    }
};

template <class HDS>
class BuildPolyhedron2 : public CGAL::Modifier_base<HDS> {
    const TriangleMesh& mesh;

public:
    BuildPolyhedron2(const TriangleMesh& mesh) : mesh(mesh) {}

    void operator()(HDS& hds) {
        CGAL::Polyhedron_incremental_builder_3<HDS> builder(hds, true);
        builder.begin_surface(mesh.vertices.size(), mesh.faces.size());

        // Add vertices
        for (const auto& v : mesh.vertices) {
            builder.add_vertex(v);
        }

        // Add faces
        for (const auto& f : mesh.faces) {
            //builder.add_facet(f[0], f[1], f[2]);

            builder.begin_facet();
            builder.add_vertex_to_facet(f[0]);
            builder.add_vertex_to_facet(f[1]);
            builder.add_vertex_to_facet(f[2]);
            builder.end_facet();
        }

        builder.end_surface();
    }
};

#endif  // MODEL_LOADER_HPP