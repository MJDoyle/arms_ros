#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"

#include "assembler/Config.hpp"

#include <cmath>

/*  Input: ptr to another part

    Output: collision between parts yes/no

    First check the bounding boxes of the two parts to see if they intersect. If they do, use an OCC function
    to check the closest distance between them
*/
bool Part::collide(std::shared_ptr<Part> otherPart)
{
    float collisionThreshold = 0.01;                                
    Bnd_Box this_bb = ShapeBoundingBox(*shape_);
    Bnd_Box other_bb = ShapeBoundingBox(*(otherPart->getShape()));

    if (this_bb.IsOut(other_bb))
        return false;

    BRepExtrema_DistShapeShape distCalc(*shape_, *(otherPart->getShape()));
    
    if (!distCalc.IsDone()) 
    {
        std::cerr << "Collision can't be calculated" << std::endl;

        return true;  // Distance can't be computed - assume collision
    }

    if (distCalc.Value() < collisionThreshold)
        return true;
    
    else
        return false;
}





void Part::positionPartInBay(std::vector<std::vector<bool>>& occupancy)
{
    double substrate_depth = 6;   
    int bay_size_index = 0;
    int bay_index = -1;                                 //The bay index for a given size
    Standard_Real x_size = ShapeAxisSize(*shape_, 0);
    Standard_Real y_size = ShapeAxisSize(*shape_, 1);

    //Select correct bay size depending on size of part
    if (x_size > 38 || y_size > 38)
    {
        bay_size_index = 1;
    }

    //Find a free, appropriately sized bay (TODO ideally need to do this before creating the substrate so that you can use a larger bay if available)
    for (int i = 0; i < occupancy[bay_size_index].size(); i ++)
    {
        if (occupancy[bay_size_index][i] == false)
        {
            bay_index = i;
            break;
        }
    }

    if (bay_index == -1)
    {
        std::cerr << "No free bay" << std::endl;
        return;
    }

    occupancy[bay_size_index][bay_index] = true;

    bay_index_ = bay_index;

    bay_size_index_ = bay_size_index;

    //Position the shape in the parts bay - base of the shape should be at -1 mm - TODO this needs to be calibrated
    
    *shape_ = ShapeSetCentroid(*shape_, gp_Pnt(PARTS_BAY_POSITIONS[bay_size_index][bay_index].X(),
                                               PARTS_BAY_POSITIONS[bay_size_index][bay_index].X(),
                                               ShapeAxisSize(*shape_, 2) / 2 - 1));


}

/*  
    Creates a simulated vacuum nozzle and searches for locations on the top face of a part where the nozzle has a strong overlap
*/
void Part::generateVacuumGraspPosition()
{
    TopoDS_Shape nozzle = BRepPrimAPI_MakeCylinder(4, 1);
    double nozzle_full_volume = ShapeVolume(nozzle);
    gp_Pnt part_com = ShapeCenterOfMass(*shape_);
    Standard_Real part_highest_point = ShapeHighestPoint(*shape_);
    double largest_shape_axis = std::max(ShapeAxisSize(*shape_, 0), ShapeAxisSize(*shape_, 1));
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
            TopoDS_Shape intersection = ShapeIntersection(moved_nozzle, *shape_);

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

    //std::cout << "Intersection ratio: " << best_fit << std::endl;

    TopoDS_Shape visual_nozzle = BRepPrimAPI_MakeCylinder(4, 8);

    gp_Pnt visual_nozzle_pos(part_com.X() + best_r * cos(best_th * 3.14159 / 180),
                            part_com.Y() + best_r * sin(best_th * 3.14159 / 180),
                            part_highest_point - 0.51);

    visual_nozzle = ShapeSetCentroid(visual_nozzle, visual_nozzle_pos);

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    builder.Add(compound, *shape_);

    builder.Add(compound, visual_nozzle);

    BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << WORKING_DIR << name_ << "_vacuum_grasp.stl";

    writer.Write(compound, ss.str().c_str());

    vacuum_grasp_position_ = gp_Pnt(best_r * cos(best_th * 3.14159 / 180), best_r * sin(best_th * 3.14159 / 180), part_highest_point - part_com.Z());

    std::cout << "Finished grasp checking" << std::endl;
}


/*  
    TODO this should be applied to initial part not target part
    ultimately you'll have to generate different grasps for different contexts
*/
void Part::generatePPGGraspPosition()
{
    struct Grasp
    {
        gp_Pnt center1;
        gp_Pnt center2;

        gp_Dir normal1;
        gp_Dir normal2;

        float com_xy_magnitude;

        TopoDS_Compound padels_compound;
    };


    if (type_ == PART_TYPE::SCREW)
        return;

    std::cout << "Generating PPG grasp" << std::endl;

    //Iterate through the faces of the part and get their normals AND antinormals

    //Disregard normals that don't have close to zero value in z
    //Check all remaining normals against one another to find pairs that align, that are close to the CoM, and that don't cause collisions

    std::vector<gp_Pnt> centers;
    std::vector<gp_Dir> normals;

    for (TopExp_Explorer faceExp(*shape_, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        TopoDS_Face face = TopoDS::Face(faceExp.Current());
        BRepAdaptor_Surface surf(face);

        GeomAbs_SurfaceType surfType = surf.GetType();

        Standard_Real u1, u2, v1, v2;
        BRepTools::UVBounds(face, u1, u2, v1, v2);

        // Midpoint in parameter space
        Standard_Real uMid = (u1 + u2) / 2.0;
        Standard_Real vMid = (v1 + v2) / 2.0;

        // Compute derivatives
        gp_Pnt center;
        gp_Vec d1u, d1v;
        surf.D1(uMid, vMid, center, d1u, d1v);

        // Compute normal
        gp_Vec normal = d1u.Crossed(d1v);
        if (normal.Magnitude() < 1e-6) continue; // Skip degenerate

        normal.Normalize();

        // Create a small arrow representing the normal
        Standard_Real length = 10.0;  // Adjust length as needed
        gp_Pnt end = center.Translated(normal.Scaled(length));

        gp_Pnt otherEnd = center.Translated(-normal.Scaled(length));

        TopoDS_Edge normalEdge = BRepBuilderAPI_MakeEdge(center, end);
        Handle(AIS_Shape) aisNormal = new AIS_Shape(normalEdge);

        centers.push_back(center);
        normals.push_back(normal);

        centers.push_back(center);
        normals.push_back(-normal);
    }

    bool grasp_found = false;

    Grasp best_grasp;    

    for (int a = 0; a < normals.size(); a++)
    {
        for (int b = 0; b < normals.size(); b++)
        {
            Standard_Real normal_normal_angle = normals[a].Angle(normals[b]);

            //Check the the two normals are anti-aligned
            if (normal_normal_angle < 0.99 * 3.14159)
                continue;

            gp_Vec delta(centers[a], centers[b]);

            //Check the points are far enough away to grasp
            if (delta.Magnitude() < 1)
                continue;

            Standard_Real delta_normal_angle = gp_Vec(normals[b]).Angle(delta);

            //Check if the normal and the delta are misaligned
            if (delta_normal_angle > 0.01 * 3.14159)
                continue;

            //Check x y distance from center of delta to CoM
            gp_Pnt delta_center = centers[a].Translated(0.5 * delta);

            gp_Vec com_xy_distance(delta_center.X() - getCoM().X(), delta_center.Y() - getCoM().Y(), 0);

            float com_xy_mag = com_xy_distance.Magnitude();

            if (com_xy_mag > 5)
                continue;

            TopoDS_Shape gripper_plate_1 = GenerateGripperPlate(normals[a], centers[a]);
            TopoDS_Shape gripper_plate_2 = GenerateGripperPlate(normals[b], centers[b]);

            // //Create padle models and check for collisions
            // TopoDS_Shape paddle_1 = BRepPrimAPI_MakeBox(10, 2, 10).Shape();
            // TopoDS_Shape paddle_2 = BRepPrimAPI_MakeBox(10, 2, 10).Shape();

            // //1st rotation
            // gp_Dir source_normal_1(0, 1, 0);
            // gp_Dir target_normal_1 = normals[a];

            // gp_Pnt source_point_1(5, 0, 5);
            // gp_Pnt target_point_1 = centers[a].Translated(1.1 * gp_Vec(normals[a]));

            // gp_Vec rotation_axis_1;

            // if ((gp_Vec(source_normal_1) ^ gp_Vec(target_normal_1)).Z() > 0)
            //     rotation_axis_1 = gp_Vec(0, 0, 1);

            // else
            //     rotation_axis_1 = gp_Vec(0, 0, -1);

            // Standard_Real angle_1 = source_normal_1.Angle(target_normal_1); 

            // gp_Trsf rotation_1;

            // gp_Ax1 axis_1(source_point_1, gp_Dir(rotation_axis_1));  // rotate around axis passing through source point
            // rotation_1.SetRotation(axis_1, angle_1);

            // BRepBuilderAPI_Transform rotTransformer_1(rotation_1);
            // rotTransformer_1.Perform(paddle_1);
            // paddle_1 = rotTransformer_1.Shape();

            // gp_Pnt rotated_source_point_1 = source_point_1.Transformed(rotation_1);  // new location of P1 after rotation TODO requierd?

            // gp_Vec translation_vector_1(rotated_source_point_1, target_point_1);

            // gp_Trsf translation_1;
            // translation_1.SetTranslation(translation_vector_1);

            // BRepBuilderAPI_Transform transTransformer_1(translation_1);
            // transTransformer_1.Perform(paddle_1);
            // paddle_1 = transTransformer_1.Shape();







            // //Second rotation
            // gp_Dir source_normal_2(0, 1, 0);
            // gp_Dir target_normal_2 = normals[b];

            // gp_Pnt source_point_2(5, 0, 5);
            // gp_Pnt target_point_2 = centers[b].Translated(1.1 * gp_Vec(normals[b]));

            // gp_Vec rotation_axis_2;

            // if ((gp_Vec(source_normal_2) ^ gp_Vec(target_normal_2)).Z() > 0)
            //     rotation_axis_2 = gp_Vec(0, 0, 1);

            // else
            //     rotation_axis_2 = gp_Vec(0, 0, -1);

            // Standard_Real angle_2 = source_normal_2.Angle(target_normal_2); 

            // gp_Trsf rotation_2;

            // gp_Ax1 axis_2(source_point_2, gp_Dir(rotation_axis_2));  // rotate around axis passing through source point
            // rotation_2.SetRotation(axis_2, angle_2);

            // BRepBuilderAPI_Transform rotTransformer_2(rotation_2);
            // rotTransformer_2.Perform(paddle_2);
            // paddle_2 = rotTransformer_2.Shape();

            // gp_Pnt rotated_source_point_2 = source_point_2.Transformed(rotation_2);  // new location of P2 after rotation TODO requierd?

            // gp_Vec translation_vector_2(rotated_source_point_2, target_point_2);

            // gp_Trsf translation_2;
            // translation_2.SetTranslation(translation_vector_2);

            // BRepBuilderAPI_Transform transTransformer_2(translation_2);
            // transTransformer_2.Perform(paddle_2);
            // paddle_2 = transTransformer_2.Shape();

            //Intersect the paddles and the part
            TopoDS_Shape intersection_1 = ShapeIntersection(gripper_plate_1, *shape_);

            TopoDS_Shape intersection_2 = ShapeIntersection(gripper_plate_2, *shape_);

            if ((!intersection_1.IsNull() && ShapeVolume(intersection_1) > 0.01) || (!intersection_2.IsNull() && ShapeVolume(intersection_2) > 0.01))
            {   
                //Collision between paddles and part, continue

                continue;
            }

    

            TopoDS_Compound compound;
            BRep_Builder builder;
            builder.MakeCompound(compound);

            builder.Add(compound, *shape_);

            builder.Add(compound, gripper_plate_1);

            builder.Add(compound, gripper_plate_2);

            // BRepMesh_IncrementalMesh mesher(compound, 0.1);

            // StlAPI_Writer writer;

            // std::stringstream ss;

            // ss << WORKING_DIR << name_ << "_ppg_grasp_" << a << "_" << b << ".stl";

            // writer.Write(compound, ss.str().c_str());



            std::cout << std::endl << "GRASP: " << std::endl;
            std::cout << "Face A center: " << centers[a].X() << ", " << centers[a].Y() << ", " << centers[a].Z() << std::endl;
            std::cout << "Face A normal: " << normals[a].X() << ", " << normals[a].Y() << ", " << normals[a].Z() << std::endl;
            std::cout << "Face B center: " << centers[b].X() << ", " << centers[b].Y() << ", " << centers[b].Z() << std::endl;
            std::cout << "Face B normal: " << normals[b].X() << ", " << normals[b].Y() << ", " << normals[b].Z() << std::endl << std::endl;;

            if (!grasp_found)
            {
                best_grasp = Grasp();

                best_grasp.center1 = centers[a];
                best_grasp.center2 = centers[b];

                best_grasp.normal1 = normals[a];
                best_grasp.normal2 = normals[b];

                best_grasp.padels_compound = compound;

                best_grasp.com_xy_magnitude = com_xy_mag;
            }

            //Compare grasps
            else if (com_xy_mag < best_grasp.com_xy_magnitude)
            {
                best_grasp = Grasp();

                best_grasp.center1 = centers[a];
                best_grasp.center2 = centers[b];

                best_grasp.normal1 = normals[a];
                best_grasp.normal2 = normals[b];

                best_grasp.padels_compound = compound;

                best_grasp.com_xy_magnitude = com_xy_mag;
            }



            grasp_found = true;


        }
    }

    if (grasp_found)
    {
        gp_Vec grasp_center = 0.5 * SumPoints(best_grasp.center2, best_grasp.center1);

        gp_Vec default_angle_vector(0, 1, 0);

        Standard_Real grasp_angle = grasp_center.Angle(default_angle_vector);
        
        Standard_Real grasp_width = SubtractPoints(best_grasp.center2, best_grasp.center1).Magnitude();

        Standard_Real grasp_height = grasp_center.Z() - 7.5;    //TODO set size of gripper

        Standard_Real shape_lowest_point = ShapeLowestPoint(*shape_);

        Standard_Real lowest_point_delta = shape_lowest_point - grasp_height;


        if (lowest_point_delta > -0.5)
        {
            grasp_height += lowest_point_delta + 0.5;
        }



        ppg_grasp_position_ = SubtractPoints(gp_Pnt(grasp_center.X(), grasp_center.Y(), grasp_height), getCoM());

        ppg_grasp_rotation_ = grasp_angle;

        ppg_grasp_width_ = grasp_width;

        std::cout << "Grasp pos: " << ppg_grasp_position_.X() << " " << ppg_grasp_position_.Y() << " " << ppg_grasp_position_.Z() << std::endl;
        std::cout << "Grasp angle: " << grasp_angle << std::endl;
        std::cout << "Grasp width: " << grasp_width << std::endl;
 
 

        BRepMesh_IncrementalMesh mesher(best_grasp.padels_compound, 0.1);

        StlAPI_Writer writer;

        std::stringstream ss;

        ss << WORKING_DIR << name_ << "_ppg_grasp.stl";

        writer.Write(best_grasp.padels_compound, ss.str().c_str());
    }

    //TODO need to check collisions and 'size' of grasp
}

TopoDS_Shape Part::GenerateGripperPlate(gp_Dir normal, gp_Pnt center)
{
    TopoDS_Shape gripper_plate = BRepPrimAPI_MakeBox(10, 2, 15).Shape();
    gp_Dir source_normal(0, 1, 0);
    gp_Dir target_normal = normal;
    gp_Pnt source_point(5, 0, 5);
    gp_Pnt target_point = center.Translated(1.1 * gp_Vec(normal));
    gp_Vec rotation_vector;
    Standard_Real rotation_angle;
    gp_Trsf rotation;
    gp_Trsf translation;

    if ((gp_Vec(source_normal) ^ gp_Vec(target_normal)).Z() > 0)
        rotation_vector = gp_Vec(0, 0, 1);

    else
        rotation_vector = gp_Vec(0, 0, -1);

    rotation_angle = source_normal.Angle(target_normal); 

    gp_Ax1 rotation_axis(source_point, gp_Dir(rotation_vector));  // rotate around axis passing through source point
    rotation.SetRotation(rotation_axis, rotation_angle);

    BRepBuilderAPI_Transform rotTransformer(rotation);
    rotTransformer.Perform(gripper_plate);
    gripper_plate = rotTransformer.Shape();

    gp_Pnt rotated_source_point = source_point.Transformed(rotation);  // new location of P1 after rotation TODO requierd?

    gp_Vec translation_vector(rotated_source_point, target_point);

    translation.SetTranslation(translation_vector);

    BRepBuilderAPI_Transform transTransformer(translation);
    transTransformer.Perform(gripper_plate);
    gripper_plate = transTransformer.Shape();

    //Check bottom of gripper plate and move it up slightly above part
    Standard_Real shape_lowest_point = ShapeLowestPoint(*shape_);

    Standard_Real plate_lowest_point = (ShapeLowestPoint(gripper_plate));

    Standard_Real lowest_point_delta = shape_lowest_point - plate_lowest_point;

    if (lowest_point_delta > -0.5)
    {
        gp_Vec raise_vector(0, 0, lowest_point_delta + 0.5);

        gp_Trsf raise_translation;

        raise_translation.SetTranslation(raise_vector);

        BRepBuilderAPI_Transform raise_transformer(raise_translation);
        raise_transformer.Perform(gripper_plate);
        gripper_plate = raise_transformer.Shape();
    }

    return gripper_plate;
}