#include "assembler/VacuumGraspGenerator.hpp"

#include "assembler/Logger.hpp"

/*  
    Creates a simulated vacuum nozzle and searches for locations on the top face of a part where the nozzle has a strong overlap
*/
static gp_Pnt VacuumGraspGenerator::generate(std::shared_ptr<Part> part)
{
    RCLCPP_INFO(logger(), "Generating vacuum grasp");

    TopoDS_Shape shape = *(part->getShape())
    TopoDS_Shape nozzle = BRepPrimAPI_MakeCylinder(4, 1);
    double nozzle_full_volume = ShapeVolume(nozzle);
    gp_Pnt part_com = ShapeCenterOfMass(shape);
    Standard_Real part_highest_point = ShapeHighestPoint(shape);
    double largest_shape_axis = std::max(ShapeAxisSize(shape, 0), ShapeAxisSize(shape, 1));
    double best_fit = 0;
    int best_r = 0;
    int best_th = 0;

    //Iterature over radius
    for (int r = 0; r < largest_shape_axis; r ++)
    {
        //Iterate over angle
        for (int th = 0; th < 360; th += 90)
        {
            gp_Pnt new_nozzle_pos(part_com.X() + r * cos(th * 3.14159 / 180),
                                part_com.Y() + r * sin(th * 3.14159 / 180),
                                part_highest_point - 0.51);

            TopoDS_Shape moved_nozzle = ShapeSetCentroid(nozzle, new_nozzle_pos);

            //Intersect the nozzle and the part
            TopoDS_Shape intersection = ShapeIntersection(moved_nozzle, shape);

            if (intersection.IsNull())
            {
                continue;
            }

            //Check the volume of the intersection
            double intersection_volume = ShapeVolume(intersection);

            double intersection_ratio = intersection_volume / nozzle_full_volume;

            if (intersection_ratio > best_fit + 0.01)
            {
                best_fit = intersection_ratio;
                best_r = r;
                best_th = th;
            }           
        }
    }

    TopoDS_Shape visual_nozzle = BRepPrimAPI_MakeCylinder(4, 8);

    gp_Pnt visual_nozzle_pos(part_com.X() + best_r * cos(best_th * 3.14159 / 180),
                            part_com.Y() + best_r * sin(best_th * 3.14159 / 180),
                            part_highest_point - 0.51);

    visual_nozzle = ShapeSetCentroid(visual_nozzle, visual_nozzle_pos);

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    builder.Add(compound, shape);

    builder.Add(compound, visual_nozzle);

    BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << WORKING_DIR << part->getName() << "_vacuum_grasp.stl";

    writer.Write(compound, ss.str().c_str());

    return gp_Pnt(best_r * cos(best_th * 3.14159 / 180), best_r * sin(best_th * 3.14159 / 180), part_highest_point - part_com.Z());

}