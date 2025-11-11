#ifndef VACUUM_GRASP_GENERATOR_HPP
#define VACUUM_GRASP_GENERATOR_HPP

#include "assembler/MeshFunctions.hpp"

class Part;

class VacuumGraspGenerator
{
    public:

    static gp_Pnt generate(std::shared_ptr<Part> part);
};

#endif