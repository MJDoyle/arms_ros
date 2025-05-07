#ifndef PART_HPP
#define PART_HPP

#include <fstream>

#include "assembler/MeshFunctions.hpp"



class Part
{
public:

    enum PART_TYPE {
        NONE,
        INTERNAL,
        EXTERNAL,
        SCREW
    };
    
    Part(std::shared_ptr<TopoDS_Shape> shape, PART_TYPE type, size_t id, std::string name) : shape_(shape), type_(type), id_(id), name_(name) {}

    Part() {}

    //Copy constructor
    Part(const Part& other) : shape_(std::make_shared<TopoDS_Shape>(*other.shape_)), id_(other.id_), type_(other.type_), name_(other.name_) {}

    // Deep Copy Function (returns a shared_ptr to the new Part)
    std::shared_ptr<Part> clone() const {
        return std::make_shared<Part>(*this);  // Uses copy constructor
    }

    void translate(gp_Vec translation) {shape_ = std::make_shared<TopoDS_Shape>(TranslateShape(*shape_, translation));}

    bool collide(std::shared_ptr<Part> otherPart);

    void setCentroidPosition(gp_Pnt position) {shape_ = std::make_shared<TopoDS_Shape>(ShapeSetCentroid(*shape_, position));}

    gp_Pnt createNegative();

    void saveShape(std::string filename) {SaveShapeAsSTL(*shape_, filename);}

    gp_Pnt                          getCentroid()       { return ShapeCentroid(*shape_); }
    Standard_Real                          getHighestPoint()   { return ShapeHighestPoint(*shape_);}

    PART_TYPE                       getType()           { return type_; }
    size_t                          getId()             { return id_; }
    std::string                     getName()           { return name_; }
    std::shared_ptr<TopoDS_Shape>   getShape()          { return shape_; }

private:

    std::shared_ptr<TopoDS_Shape> shape_;

    PART_TYPE type_;

    size_t id_;

    std::string name_;
};







// class MeshObject
// {

// public:

//     MeshObject() {}

//     MeshObject(std::shared_ptr<TopoDS_Shape> shape) : shape_(shape) {}

//     MeshObject(std::shared_ptr<Polyhedron> mesh) : mesh_(mesh) {}

//     MeshObject(const MeshObject& other) : mesh_(std::make_shared<Polyhedron>(*other.mesh_)) {}

//     std::shared_ptr<Polyhedron>     getMesh()       { return mesh_; }

//     Point                           getCentroidPosition() { return meshCenter(mesh_); }

//     double                          getMeshMaxZ() { return meshBoundingBox(mesh_).zmax(); }

//     void setMesh(std::shared_ptr<Polyhedron> mesh) { mesh_ = mesh; }

//     void setCentroidPosition(Point position) { positionMesh(mesh_, position); }

//     //OCC stuff
//     void setShape(std::shared_ptr<TopoDS_Shape> shape) { shape_ = shape; }

//     std::shared_ptr<TopoDS_Shape> getShape()    { return shape_; }
 
// protected:

//     std::shared_ptr<Polyhedron> mesh_;

//     //OCC stuff
//     std::shared_ptr<TopoDS_Shape> shape_;
// };   


// class Part : public MeshObject
// {
// public:

//     enum PART_TYPE {
//         NONE,
//         INTERNAL,
//         EXTERNAL,
//         SCREW
//     };

//     Part(std::shared_ptr<Polyhedron> mesh, PART_TYPE type, size_t id, std::string name) : MeshObject(mesh), type_(type), id_(id), name_(name) {}

//     Part(std::shared_ptr<TopoDS_Shape> shape, PART_TYPE type, size_t id, std::string name) : MeshObject(shape), type_(type), id_(id), name_(name) {}


//     Part() {}

//     void translate(Vector translation);

//     bool collide(std::shared_ptr<Part> otherPart);

//     Point createNegative(std::shared_ptr<MeshObject> substrate, std::string filename);

//     PART_TYPE                       getType()       { return type_; }
//     size_t                          getId()         { return id_; }
//     std::string                     getName()       { return name_; }

//     //Copy constructor
//     Part(const Part& other) : MeshObject(other), id_(other.id_), type_(other.type_), name_(other.name_) {}

//     // Deep Copy Function (returns a shared_ptr to the new Part)
//     std::shared_ptr<Part> clone() const {
//         return std::make_shared<Part>(*this);  // Uses copy constructor
//     }

// private:

//     PART_TYPE type_;

//     size_t id_;

//     std::string name_;

// };

#endif