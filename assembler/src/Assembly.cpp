#include "assembler/Assembly.hpp"

#include "assembler/Part.hpp"

#include <iostream>

#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <StlAPI_Writer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

Assembly::Assembly()
{
    std::cout << "Creating assembly" << std::endl;
}

void Assembly::saveAsSTL(std::string filename)
{
    std::cout << "Saving assembly as STL" << std::endl;

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    for (std::shared_ptr<Part> part : parts_)
    {
        std::cout << "Adding part to builder" << std::endl;

        builder.Add(compound, *(part->getShape()));
    }
        
    BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;
    writer.Write(compound, filename.c_str());

}


std::vector<size_t> Assembly::getPartIds()
{
    std::vector<size_t> part_ids;

    for (std::shared_ptr<Part> part : parts_)
    {
        part_ids.push_back(part->getId());
    }

    return part_ids;
}

std::shared_ptr<Part> Assembly::getPartById(size_t id)
{
    for (std::shared_ptr<Part> part : parts_)
    {
        if (part->getId() == id)
            return part;
    }

    std::cerr << "No part of this ID present in assembly" << std::endl;

    return std::shared_ptr<Part>(new Part());
}

int Assembly::getNumInternalParts()
{
    int num_internal_parts = 0;

    for (std::shared_ptr<Part> part : parts_)
    {
        if (part->getType() == Part::INTERNAL)
            num_internal_parts ++;
    }

    return num_internal_parts;
}

void Assembly::alignToPart(std::shared_ptr<Part> part)
{
    // std::shared_ptr<Part> thisPart = getPartById(part->getId());

    // if (thisPart == nullptr)
    // {
    //     std::cerr << "Corresponding part can't be found in assembly" << std::endl;
    //     return;
    // }

    // Vector delta = part->getCentroidPosition() - thisPart->getCentroidPosition();

    // for (std::shared_ptr<Part> part_ : parts_)
    // {
    //     part_->translate(delta);
    // }
}

void Assembly::placeOnPoint(gp_Pnt point)
{
    //Find x,y center and lowest z point
    std::cout << std::endl << "Placing on point: " << point.X() << " " << point.Y() << " " << point.Z() << std::endl;

    //Find the bounding box of the entire assembly
    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double zmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = std::numeric_limits<double>::lowest();
    double zmax = std::numeric_limits<double>::lowest();

    for (std::shared_ptr<Part> part : parts_)
    {
        Bnd_Box bbox = ShapeBoundingBox(*(part->getShape()));

        Standard_Real bb_xmin, bb_ymin, bb_zmin, bb_xmax, bb_ymax, bb_zmax;

        bbox.Get(bb_xmin, bb_ymin, bb_zmin, bb_xmax, bb_ymax, bb_zmax);

        xmin = std::min(xmin, bb_xmin);
        ymin = std::min(ymin, bb_ymin);
        zmin = std::min(zmin, bb_zmin);
        xmax = std::max(xmax, bb_xmax);
        ymax = std::max(ymax, bb_ymax);
        zmax = std::max(zmax, bb_zmax);
    }

    double lowest_z = zmin;

    gp_Pnt center((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0);

    //Move al parts so that x,y lies on point.x and point.y, and lowest z point lies on zero (or adjust for bed heighgt)

    gp_Vec translation(point, gp_Pnt(center.X(), center.Y(), lowest_z));

    for (std::shared_ptr<Part> part : parts_)
    {
        part->translate(translation);
    }
}
