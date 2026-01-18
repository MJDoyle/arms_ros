#include "assembler/VacuumGraspGenerator.hpp"

#include "assembler/Logger.hpp"

#include "assembler/Part.hpp"

#include "assembler/Config.hpp"


gp_Pnt VacuumGraspGenerator::generate(std::shared_ptr<Part> part)
{
    RCLCPP_INFO(logger(), "Generating vacuum grasp");

    struct CandidateGrasp {
        double com_distance;
        gp_Pnt position;
        float nozzle_intersection;
        float tip_intersection;
    };

    TopoDS_Shape shape = *(part->getShape());
    double nozzle_height = 20;
    double nozzle_tip_height = 1;
    double nozzle_radius = 4.2;
    TopoDS_Shape nozzle_tip = BRepPrimAPI_MakeCylinder(nozzle_radius, nozzle_tip_height);
    TopoDS_Shape nozzle = BRepPrimAPI_MakeCylinder(nozzle_radius, nozzle_height);
    double nozzle_volume = ShapeVolume(nozzle);
    double nozzle_tip_volume = ShapeVolume(nozzle_tip);
    gp_Pnt shape_com = ShapeCenterOfMass(shape);
    gp_Pnt shape_centroid = ShapeCentroid(shape);

    std::vector<CandidateGrasp> candidate_grasps;

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopoDS_Face face = TopoDS::Face(exp.Current());

        gp_Dir normal = outwardFaceNormal(face);

        //Ignore faces that are not planar
        if (!BRep_Tool::Surface(face)->IsKind(STANDARD_TYPE(Geom_Plane)))
            continue;

        //Ignore faces with a not vertical normal
        if (normal.Angle(UPWARDS) > 0.05)
            continue;

        //Ignore faces that are too small
        if (faceArea(face) < 50)
            continue;



 
        Handle(Geom_Plane) gpln = Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(face));
        gp_Pln plane = gpln->Pln();
        
        double largest_face_axis = std::max(ShapeAxisSize(face, 0), ShapeAxisSize(face, 1));  //TODO

        gp_Pnt face_centroid = ShapeCentroid(face);

        Standard_Real face_highest_point = ShapeHighestPoint(face);

        RCLCPP_INFO(logger(), "Checking face of area %f and highest point %f", faceArea(face), face_highest_point);

        double nozzle_z = face_highest_point + 0.5 * nozzle_height;
        double nozzle_tip_z = face_highest_point - 0.5 * nozzle_tip_height;

        RCLCPP_INFO(logger(), "Nozzle CoM z position %f and tip CoM z position %f", nozzle_z, nozzle_tip_z);

        for (int r = 0; r < largest_face_axis; r ++)
        {
            //Iterate over angle
            for (int th = 0; th < 360; th += 90)
            {
                double nozzle_x = face_centroid.X() + r * cos(th * M_PI / 180);
                double nozzle_y = face_centroid.Y() + r * sin(th * M_PI / 180);                

                nozzle = ShapeSetCentroid(nozzle, gp_Pnt(nozzle_x, nozzle_y, nozzle_z));

                nozzle_tip = ShapeSetCentroid(nozzle_tip, gp_Pnt(nozzle_x, nozzle_y, nozzle_tip_z));


                gp_Pnt graspPosition = gp_Pnt(nozzle_x - shape_centroid.X(), nozzle_y - shape_centroid.Y(), nozzle_z - shape_centroid.Z() - 0.5 * nozzle_height);
                RCLCPP_INFO(logger(), "Testing grasp | Position: %f %f %f", graspPosition.X(), graspPosition.Y(), graspPosition.Z());



                //Intersect the nozzle and all the solids within the part - more reliable to do them one at a time than all together

                double nozzle_intersection_ratio = 0;

                for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) {
                    TopoDS_Shape sub_shape = ex.Current();

                    TopoDS_Shape nozzle_intersection = ShapeIntersection(nozzle, sub_shape);

                    if (nozzle_intersection.IsNull())
                        continue;
                
                    nozzle_intersection_ratio += ShapeVolume(nozzle_intersection) / nozzle_volume;
                    
                }

                if (nozzle_intersection_ratio > 0.0001)
                    continue;




                // TopoDS_Shape nozzle_intersection = ShapeIntersection(nozzle, shape);

                // double nozzle_intersection_ratio = 0;

                // if (!nozzle_intersection.IsNull())
                // {
                //     RCLCPP_INFO(logger(), "Not null nozzle intersection");

                //     nozzle_intersection_ratio = ShapeVolume(nozzle_intersection) / nozzle_volume;

                //     //Nozzle clashes
                //     if (nozzle_intersection_ratio > 0.0001)
                //         continue;
                // }

                RCLCPP_INFO(logger(), "Nozzle intersection ok");



                //Intersect the nozzle tip and the part
                TopoDS_Shape nozzle_tip_intersection = ShapeIntersection(nozzle_tip, shape);

                if (nozzle_tip_intersection.IsNull())
                    continue;

                double nozzle_tip_intersection_ratio = ShapeVolume(nozzle_tip_intersection) / nozzle_tip_volume;

                if (nozzle_tip_intersection_ratio < 0.5)
                    continue;

                gp_Vec com_delta = gp_Vec(nozzle_x - shape_com.X(), nozzle_y - shape_com.Y(), 0);

                CandidateGrasp grasp;

                grasp.com_distance = com_delta.Magnitude();

                grasp.position = gp_Pnt(nozzle_x - shape_centroid.X(), nozzle_y - shape_centroid.Y(), nozzle_z - shape_centroid.Z() - 0.5 * nozzle_height); //Relative to shape centroid

                grasp.nozzle_intersection = nozzle_intersection_ratio;

                grasp.tip_intersection = nozzle_tip_intersection_ratio;

                candidate_grasps.push_back(grasp);

                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);

                builder.Add(compound, shape);

                builder.Add(compound, nozzle);

                BRepMesh_IncrementalMesh mesher(compound, 0.1);

                StlAPI_Writer writer;

                std::stringstream ss;

                ss << WORKING_DIR << part->getName() << "_vacuum_grasp_pos_" << nozzle_x - shape_centroid.X() << "_" << nozzle_y - shape_centroid.Y() << "_" << nozzle_z - shape_centroid.Z() - 0.5 * nozzle_height << "_nozzleintersection_" << nozzle_intersection_ratio << ".stl";

                writer.Write(compound, ss.str().c_str());

                RCLCPP_INFO(logger(), "Candidate grasp | Position: %f %f %f, nozzle intersection %f, and tip intersection %f", grasp.position.X(), grasp.position.Y(), grasp.position.Z(), nozzle_intersection_ratio, nozzle_tip_intersection_ratio);
            }
        }      
    }
    
    if (candidate_grasps.size() == 0)
    {
        RCLCPP_FATAL(logger(), "No vacuum grasp found");
        rclcpp::shutdown();
        return gp_Pnt();
    }

    auto best_grasp_it = std::min_element(
            candidate_grasps.begin(), candidate_grasps.end(),
            [](auto& a, auto& b) { return a.com_distance < b.com_distance; }
        );

    RCLCPP_INFO(logger(), "Grasp position %f %f %f, nozzle intersection %f, tip intersection %f", best_grasp_it->position.X(), best_grasp_it->position.Y(), best_grasp_it->position.Z(), best_grasp_it->nozzle_intersection, best_grasp_it->tip_intersection);

    TopoDS_Shape visual_nozzle = BRepPrimAPI_MakeCylinder(nozzle_radius, nozzle_height);

    visual_nozzle = ShapeSetCentroid(visual_nozzle, gp_Pnt(best_grasp_it->position.X() + shape_centroid.X(), best_grasp_it->position.Y() + shape_centroid.Y(), best_grasp_it->position.Z() + shape_centroid.Z() + 0.5 * nozzle_height));

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

    return best_grasp_it->position.Translated(gp_Vec(0, 0, - nozzle_height * 0.5));
}


/*  
    Creates a simulated vacuum nozzle and searches for locations on the top face of a part where the nozzle has a strong overlap
*/
gp_Pnt VacuumGraspGenerator::generate_simple(std::shared_ptr<Part> part)
{
    RCLCPP_INFO(logger(), "Generating vacuum grasp");

    TopoDS_Shape shape = *(part->getShape());
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