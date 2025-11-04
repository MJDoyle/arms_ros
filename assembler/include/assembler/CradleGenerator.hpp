#ifndef CRADLEGENERATOR_HPP
#define CRADLEGENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class CradleGenerator
{
    public:

    CradleGenerator(std::string name, TopoDS_Shape shape) : shape_(shape), name_(name) {}

    void createNegative();

    void createNegative2();

    TopTools_ListOfShape bottomEdgesFlatHorizontalFaces();

    gp_Vec outwardsNormalOfEdge(TopoDS_Edge edge, TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap);

    TopoDS_Shape createBoxWithPose(double width, double height, double length, gp_Pnt target_center, gp_Dir target_direction);

    private:
    
        TopoDS_Shape shape_;

        std::string name_;


};


#endif