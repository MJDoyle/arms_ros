#include "assembler/Part.hpp"

bool Part::collide(std::shared_ptr<Part> otherPart)
{
    BRepExtrema_DistShapeShape distCalc(*shape_, *(otherPart->getShape()));
    if (distCalc.IsDone()) {

        std::cout << "Intersection: " << distCalc.Value() << std::endl;

        if (distCalc.Value() < 0.01)
            return true;
        
        else
            return false;
    }

    std::cerr << "Collision can't be calculated" << std::endl;

    return false;  // If the distance couldn't be computed
}

gp_Pnt Part::createNegative()
{
    TopoDS_Shape substrate = BRepPrimAPI_MakeBox(40, 40, 6).Shape();    //TODO: box shape depends on part

    //Get the substrate centroid
    gp_Pnt substrate_centroid = ShapeCentroid(substrate);

    //Get the shape centroid
    gp_Pnt shape_centroid = ShapeCentroid(*shape_);

    //Get the shape min z
    Standard_Real shape_min_z = ShapeLowestPoint(*shape_);

    //Get the substrate min z
    Standard_Real substrate_min_z = ShapeLowestPoint(substrate);

    //Move the substrate so that the X and Y centers align and the base is 2mm below the shape base

    gp_Vec substrate_move(shape_centroid.X() - substrate_centroid.X(), 
                          shape_centroid.Y() - substrate_centroid.Y(), 
                          shape_min_z - substrate_min_z - 2);
    
    substrate = TranslateShape(substrate, substrate_move);

    Standard_Real step_size = 1;

    //Get bounding box of part
    Bnd_Box shape_bbox = ShapeBoundingBox(*shape_);

    //Create two boxes that are slightly larger than this
    TopoDS_Shape top_bound = BRepPrimAPI_MakeBox(ShapeAxisSize(*shape_, 0) + 1, ShapeAxisSize(*shape_, 1) + 1, ShapeAxisSize(*shape_, 2) + 1).Shape();
    TopoDS_Shape bottom_bound = BRepPrimAPI_MakeBox(ShapeAxisSize(*shape_, 0) + 1, ShapeAxisSize(*shape_, 1) + 1, ShapeAxisSize(*shape_, 2) + 1).Shape();

    //Set one box bottom to bottom of part and XY centroid pos to part XY centroid pos
    gp_Pnt top_centroid = ShapeCentroid(top_bound);
    Standard_Real top_min_z = ShapeLowestPoint(top_bound);

    gp_Vec top_move(shape_centroid.X() - top_centroid.X(), 
                    shape_centroid.Y() - top_centroid.Y(), 
                    shape_min_z - top_min_z);
    
    top_bound = TranslateShape(top_bound, top_move);

    //Set other box top to bottom of part minus step size, and XY centroid pos to part XY centroid pos
    gp_Pnt bottom_centroid = ShapeCentroid(bottom_bound);
    Standard_Real bottom_max_z = ShapeHighestPoint(bottom_bound);

    gp_Vec bottom_move(shape_centroid.X() - bottom_centroid.X(), 
                    shape_centroid.Y() - bottom_centroid.Y(), 
                    shape_min_z - bottom_max_z - step_size);
    
    bottom_bound = TranslateShape(bottom_bound, bottom_move);

    std::vector<TopoDS_Shape> slivers;

    //Step the boxes through the step size, cut the part with both of them
    //Find the bounding box of the sliver, scale it
    //Make sure the the x and y sizes are at least as big as the previous sliver
    //Add to the list of slivers
    for (int i = 0; i < 5; i ++)
    {
        std::cout << "making cuts" << std::endl;

        top_bound = TranslateShape(top_bound, gp_Vec(0, 0, step_size));
        bottom_bound = TranslateShape(bottom_bound, gp_Vec(0, 0, step_size));

        // Perform the first subtraction: A - B
        TopoDS_Shape first_cut = SubtractShapeBFromA(*shape_, bottom_bound);

        //Perform the second subtraction
        TopoDS_Shape both_cuts = SubtractShapeBFromA(first_cut, top_bound);


        std::stringstream ss;

        ss << "sliver" << i << ".stl";

        SaveShapeAsSTL(both_cuts, ss.str());



        TopoDS_Shape both_cuts_bbox_shape = ShapeHighBoundingBoxShape(both_cuts, 5);

        std::stringstream ss2;

        ss2 << "sliver_bbox" << i << ".stl";

        SaveShapeAsSTL(both_cuts_bbox_shape, ss2.str());

        Standard_Real x_size = ShapeAxisSize(both_cuts_bbox_shape, 0);

        Standard_Real y_size = ShapeAxisSize(both_cuts_bbox_shape, 1);

        Standard_Real scaling_factor;

        if (x_size < y_size)
        {
            scaling_factor = (x_size + 0.8) / x_size;
        }

        else
        {
            scaling_factor = (y_size + 0.8) / y_size;
        }

        TopoDS_Shape scaled_sliver = UniformScaleShape(both_cuts_bbox_shape, scaling_factor);

        //Now extrude upwards

        slivers.push_back(scaled_sliver);
    }

    int s = 0;

    for (TopoDS_Shape sliver : slivers)
    {
        std::cout << "subtracting sliver" << std::endl;

        substrate = SubtractShapeBFromA(substrate, sliver);

        s++;
    }



    BRepMesh_IncrementalMesh(substrate, 0.1);  // Mesh with a 0.1 tolerance

    // Export the result to STL
    StlAPI_Writer substrate_writer;
    substrate_writer.Write(substrate, (name_ + "_sliced_cradle.stl").c_str());

    return gp_Pnt(0, 0, 0);
}









// void Part::translate(Vector translation)
// {   
//     translateMesh(mesh_, translation);
// }


// bool Part::collide(std::shared_ptr<Part> otherPart)
// {
//     //AABB_tree tree1(faces(*mesh_).first, faces(*mesh_).second, *mesh_);
//     AABB_tree tree2(faces(*(otherPart->getMesh())).first, faces(*(otherPart->getMesh())).second, *(otherPart->getMesh()));

//     //tree1.accelerate_distance_queries();
//     tree2.accelerate_distance_queries();

//     // Iterate through the faces of one polyhedron
//     for (auto f1 = mesh_->facets_begin(); f1 != mesh_->facets_end(); ++f1) {
//         // Convert face into a triangle
//         auto h = f1->halfedge();
//         Triangle tri1(h->vertex()->point(), h->next()->vertex()->point(), h->next()->next()->vertex()->point());

//         // Check if this triangle intersects with tree2
//         if (tree2.do_intersect(tri1)) {
//             return true;  // Collision detected
//         }
//     }

//     return false;  // No collision
// }


// Point Part::createNegative(std::shared_ptr<MeshObject> substrate, std::string filename)
// {
//     std::cout << "Creating negative" << std::endl;

//     Point final_position(0, 0, 0);

//     // Convert Polyhedra to Nef Polyhedra

//     if (substrate == nullptr)
//     {
//         std::cout << "substrate null!" << std::endl;
//         return final_position;
//     }

//     if (substrate->getMesh() == nullptr)
//     {
//         std::cout << "substrate mesh null!" << std::endl;
//         return final_position;
//     }

//     //Copy the mesh and operate on the copy
//     std::shared_ptr<Polyhedron> scaledMesh = std::make_shared<Polyhedron>(*mesh_);

//     //Determine scaling factor from desired clearance
//     double clearance = 1;

//     BoundingBox bbox = meshBoundingBox(scaledMesh);

//     auto x_size = bbox.x_span();

//     auto y_size = bbox.y_span();

//     auto z_size = bbox.z_span();

//     auto target_x_size = x_size + clearance;

//     auto target_y_size = y_size + clearance;
    
//     auto target_z_size = z_size + clearance;

//     auto x_scale_ratio = target_x_size / x_size;

//     auto y_scale_ratio = target_y_size / y_size;

//     auto z_scale_ratio = target_z_size / z_size;

//     std::cout << "Scaling ratios: " <<  x_scale_ratio << " " << y_scale_ratio << " " << z_scale_ratio << std::endl;


//     scaleMesh(scaledMesh, x_scale_ratio, y_scale_ratio, z_scale_ratio);   
//                         //TODO we need to make the object orthogonally convex first! Maybe we just do this by hand for now
//                         //TODO or maybe eve just convex? That might suffice for most objects


//     //Find the lowest point of the substrate
//     double substrate_lowest_z = meshLowestPoint(substrate->getMesh());

//     //Find the lowest point of the part
//     double part_lowest_z = meshLowestPoint(scaledMesh);

//     //Move the part so that it's lowest point is 2mm above the lowest point of the substrate

//     translateMesh(scaledMesh, Vector(0, 0, substrate_lowest_z + 2 - part_lowest_z));

//     auto distance_moved = substrate_lowest_z + 2 - part_lowest_z;

//     //Now, loop

//     int i = 0;

//     while (true)
//     {
//         std::cout << "Distance moved: " << distance_moved << std::endl;

//         //Create the nefs and do the substraction

//         SurfaceMesh mesh;
//         CGAL::copy_face_graph(*(substrate->getMesh()), mesh);

//         Nef_polyhedron nef_substrate(mesh);

//         SurfaceMesh mesh2;
//         CGAL::copy_face_graph(*scaledMesh, mesh2);

//         Nef_polyhedron nef_part(mesh2);

//         if (!nef_substrate.is_simple()) std::cout << "Error: Nef_substrate not simple!" << std::endl;
//         if (!nef_part.is_simple()) std::cout << "Error: Nef_part not simple!" << std::endl;


//         if (nef_substrate.is_empty()) std::cout << "Error: nef_substrate is empty!" << std::endl;
//         if (nef_part.is_empty()) std::cout << "Error: nef_part is empty!" << std::endl;

//         // Perform subtraction (poly1 - poly2)
//         Nef_polyhedron result = nef_substrate.difference(nef_part);

//         result.regularization();  // Improves numerical stability
//         result.simplify();        // Removes unnecessary small faces

//         // Convert back to Polyhedron
//         std::shared_ptr<Polyhedron> cradle = std::shared_ptr<Polyhedron>(new Polyhedron());
//         if (result.is_simple()) {
//             result.convert_to_polyhedron(*cradle);
//         } else {
//             throw std::runtime_error("Resulting shape is not a valid polyhedron.");
//         }


//         //Check if there are any faces pointing downwards
//         int downwards_faces = 0;

//         for (auto facet = cradle->facets_begin(); facet != cradle->facets_end(); facet++)
//         {
//             Vector normal = CGAL::Polygon_mesh_processing::compute_face_normal(facet, *cradle);

//             if (normal.z() < -0.001 && facetLowestPoint(facet).z() > substrate_lowest_z + 1) {  // Face is downward-facing
//                 downwards_faces++;
//             }
//         }

//         std::cout << "Num downwards faces: " << downwards_faces << std::endl;

//         // std::stringstream ss;
        
//         // ss << filename << "_partial_" << distance_moved << std::endl;

//         // saveMesh(cradle, ss.str());

//         //If there are no downward faces, save the mesh
//         if (downwards_faces == 0)
//         {
//             //Save mesh
//             saveMesh(cradle, filename);

//             std::cout << std::endl << std::endl << "SAVED AT " << distance_moved << std::endl << std::endl;

//             break;
//         }

//         translateMesh(scaledMesh, Vector(0, 0, 0.1));

//         distance_moved += 0.1;
//     }

//     final_position += Vector(0, 0, distance_moved); 

//     return final_position;
// }