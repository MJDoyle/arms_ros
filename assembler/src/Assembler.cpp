#include "assembler/Assembler.hpp"

#include "assembler/Assembly.hpp"

#include "assembler/CradleGenerator.hpp"


#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "yaml-cpp/yaml.h"

//#include <ament_index_cpp/get_package_share_directory.hpp>

#include "assembler/Config.hpp"

#include <iostream>
#include <fstream>
#include <queue>
#include <set>
#include <map>

#include <cstdlib>

Assembler::Assembler()
{
    //TODO
    // output_path_ = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";
    // input_path_ = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";

    // std::cout << "Ouput path: " << output_path_ << std::endl;
    // std::cout << "Input path: " << input_path_ << std::endl;

    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

    
    for (std::vector<gp_Pnt> bays : PARTS_BAY_POSITIONS)
    {
        bay_occupancy_.push_back(std::vector<bool>());

        for (gp_Pnt bay_position : bays)
        {
            bay_occupancy_.back().push_back(false);
        }
    }
}

void Assembler::generateAssemblySequence() 
{
    if (target_assembly_ == nullptr)
        return;

    generateGrasps();

    generateInitialAssembly();

    generateNegatives();

    std::cout << "Generating assembly sequence" << std::endl;

    std::vector<std::shared_ptr<AssemblyNode>> path = breadthFirstZAssembly();

    std::vector<size_t> ordered_part_additions;

    std::cout << std::endl << "Ordered part list: " << std::endl;

    for (std::shared_ptr<AssemblyNode> node : path)
    {
        for (size_t part_id : node->assembly_->getPartIds())
        {
            bool part_present = false;

            for (size_t id : ordered_part_additions)
            {
                if (id == part_id)
                    part_present = true;
            }

            if (!part_present)
            {
                ordered_part_additions.push_back(part_id);

                std::cout << "Part: " << part_id << std::endl;

                continue;
            }
        }
    }

    if (path.size() < 2)
        return;

    //TODO janky
    std::shared_ptr<Part> target_base_part = path[1]->assembly_->getParts()[0];

    //If initial (or target) assembly has internal parts, do slicer stuff and then set target_assembly position
    if (initial_assembly_->getNumInternalParts() != 0)
    {
        //Arrange the internal parts on the bed
        if (!arrangeInternalParts())
        {
            std::cerr << "Parts cannot be arranged" << std::endl;

            return;
        }

        //Get the slicing GCODE from prusa slicer
        generateSlicerGcode();

        //Set the target assembly position
        
        //If base part is internal, build on that
        if (target_base_part->getType() == Part::INTERNAL)
        {
            std::shared_ptr<Part> initial_base_part = initial_assembly_->getPartById(target_base_part->getId());

            target_assembly_->alignToPart(initial_base_part);
        }
        //Otherwise, we need to build in a free area
        else    
        {
            std::cerr << "base part not internal" << std::endl;

            return;
        }
    }

    
    //If initial (or target) assembly has no internal parts, set target_assembly position to middle of bed
    else
    {
        //set target assembly positions at bed center
        target_assembly_->placeOnPoint(gp_Pnt(PRINT_BED_CENTER[0], PRINT_BED_CENTER[1], PRINT_BED_HEIGHT));
    }





    //In these commands for now let's give the part-height as the height of the pnp location relative to 0,
    //and the z location as the height of the pnp placement location relative to 0
    //You then might need to offset by the vacuum toolhead offset at some point


    YAML::Node root;

    YAML::Node commands = YAML::Node(YAML::NodeType::Sequence);

    std::vector<std::shared_ptr<Part>> initial_parts = initial_assembly_->getParts();

    //TODO might get rid of locating external parts
    // for (std::shared_ptr<Part> part : initial_parts)
    // {        
    //     if (!part->getType() == Part::EXTERNAL)
    //         continue;

    //     YAML::Node detect_part_command;
    //     detect_part_command["command-type"] = "LOCATE_EXTERNAL_PART";
    //     detect_part_command["command-properties"]["part-description"] = ""; //TODO
    //     detect_part_command["command-properties"]["part-name"] = ""; //TODO
    //     detect_part_command["command-properties"]["part-id"] = part->getId();
    //     detect_part_command["command-properties"]["part-height"] = part->getMeshMaxZ(); //This is the height from 0, accounting for the cradle etc

    //     commands.push_back(detect_part_command);
    // }



    //Designate internal parts
    for (std::shared_ptr<Part> initial_part : initial_parts)
    {
        if (initial_part->getType() != Part::INTERNAL)
            continue;

        std::shared_ptr<Part> target_part = target_assembly_->getPartById(initial_part->getId());

        gp_Pnt pick_position(initial_part->getCentroid().X(), 
                             initial_part->getCentroid().Y(),
                             initial_part->getHighestPoint());

        gp_Pnt place_position(target_part->getCentroid().X(), 
                              target_part->getCentroid().Y(),
                              target_part->getHighestPoint());

        gp_Vec grasp_position = SumPoints(initial_part->getCoM(), target_part->getPPGGraspPosition());
        Standard_Real grasp_rotation = target_part->getPPGGraspRotation();  //TODO need to handle rotation of parts eventually too
        Standard_Real grasp_width = target_part->getPPGGraspWidth();

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_INTERNAL_PART";
        designate_part_command["command-properties"]["part-id"] = initial_part->getId();
        designate_part_command["command-properties"]["part-pick-height"] = pick_position.Z();
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z();
        designate_part_command["command-properties"]["part-pick-pos-x"] = pick_position.X();
        designate_part_command["command-properties"]["part-pick-pos-y"] = pick_position.Y();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();
        designate_part_command["command-properties"]["part-grasp-height"] = grasp_position.Z();
        designate_part_command["command-properties"]["part-grasp-pos-x"] = grasp_position.X();
        designate_part_command["command-properties"]["part-grasp-pos-y"] = grasp_position.Y();
        designate_part_command["command-properties"]["part-grasp-angle"] = grasp_rotation;
        designate_part_command["command-properties"]["part-grasp-width"] = grasp_width;

        commands.push_back(designate_part_command);
    }

    //Designate external parts
    for (std::shared_ptr<Part> initial_part : initial_parts)
    {        
        if (initial_part->getType() != Part::EXTERNAL)
            continue;

        std::shared_ptr<Part> target_part = target_assembly_->getPartById(initial_part->getId());

        gp_Vec pick_position = SumPoints(initial_part->getCoM(), initial_part->getVacuumGrasp());

        gp_Vec place_position = SumPoints(target_part->getCoM(), target_part->getVacuumGrasp());

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_EXTERNAL_PART";
        designate_part_command["command-properties"]["part-id"] = initial_part->getId();
        designate_part_command["command-properties"]["part-pick-height"] = pick_position.Z(); //This is the height from 0, accounting for the cradle etc TODO
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z(); //This is the height from 0, accounting for the cradle etc
        designate_part_command["command-properties"]["part-pick-pos-x"] = pick_position.X();
        designate_part_command["command-properties"]["part-pick-pos-y"] = pick_position.Y();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();

        commands.push_back(designate_part_command);
    }

    //Designate screws
    for (std::shared_ptr<Part> initial_part : initial_parts)
    {
        if (initial_part->getType() != Part::SCREW)
            continue;

        std::shared_ptr<Part> target_part = target_assembly_->getPartById(initial_part->getId());

        gp_Pnt place_position(target_part->getCoM().X(), target_part->getCoM().Y(), target_part->getHighestPoint());

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_SCREW";
        designate_part_command["command-properties"]["part-id"] = initial_part->getId();
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();

        commands.push_back(designate_part_command);
    }

    //Print all internal parts together
    if (initial_assembly_->getNumInternalParts() != 0)
    {
        YAML::Node direct_print_command;
        direct_print_command["command-type"] = "DIRECT_PRINT";

        // Add "gcode" sequence to the first command
        YAML::Node gcode = YAML::Node(YAML::NodeType::Sequence);

        for (std::string gcode_line : slicer_gcode_)
        {
            gcode.push_back(gcode_line);
        }

        direct_print_command["command-properties"]["gcode"] = gcode;

        commands.push_back(direct_print_command);
    }


    //Iterate through each of the added parts in the path
    for (size_t part_id : ordered_part_additions)
    {
        std::cout << "Adding PLACE_PART command" << std::endl;

        Part::PART_TYPE part_type = initial_assembly_->getPartById(part_id)->getType();

        std::cout << "Part type: " << part_type << std::endl;

        std::cout << "Part name: " << initial_assembly_->getPartById(part_id)->getName();

        //Do nothing with the base object if it's internal  //TODO janky
        if (part_id == path[1]->assembly_->getPartIds()[0] && path[1]->assembly_->getParts()[0]->getType() == Part::INTERNAL)
        {
            std::cout << "base object - skipping" << std::endl;
 
            continue;
        }

        YAML::Node place_part_command;

        if (part_type == Part::SCREW)
            place_part_command["command-type"] = "PLACE_SCREW";

        else
            place_part_command["command-type"] = "PLACE_PART";

        place_part_command["command-properties"]["part-id"] = part_id;

        //If it's an internal part, add the command to vibrate the part first
        if (part_type == Part::INTERNAL)
        {
            YAML::Node vibrate_part_command;

            vibrate_part_command["command-type"] = "VIBRATE_PART";

            vibrate_part_command["command-properties"]["part-id"] = part_id;

            commands.push_back(vibrate_part_command);
        }

        commands.push_back(place_part_command);
    }

    root["commands"] = commands;

    std::ofstream fout(OUTPUT_DIR + "assembly_plan.yaml");


    std::cout << "Output path: " << OUTPUT_DIR + "assembly_plan.yaml" << std::endl;

    fout << root;

    fout.close();
}

bool Assembler::arrangeInternalParts()
{
    double currentY = PRINT_BED_BOTTOM_LEFT[1];

    double currentX = PRINT_BED_TOP_RIGHT[0];

    double nextY = PRINT_BED_BOTTOM_LEFT[1];

    for (std::shared_ptr<Part> part : initial_assembly_->getParts())
    {
        if (part->getType() != Part::INTERNAL)
            continue;

        while (true)
        {
            TopoDS_Shape shape = *part->getShape();

            gp_Pnt part_position(currentX - ShapeAxisSize(shape, 0) / 2, currentY + ShapeAxisSize(shape, 1) / 2, ShapeAxisSize(shape, 2) / 2);

            double nextX = currentX - ShapeAxisSize(shape, 0) - PRINT_MIN_SPACING;

            double topY = currentY + ShapeAxisSize(shape, 1) + PRINT_MIN_SPACING;

            std::cout << "nextX: " << nextX << "    topyY: " << topY << std::endl;

            //Check the new position is within parts bay bounds
            if (topY > PRINT_BED_TOP_RIGHT[1])
            {
                //Parts can't fit, return false
                return false;
            }

            else if (nextX < PRINT_BED_BOTTOM_LEFT[0])
            {
                //Start a new y layer, try again with this part
                currentY = nextY;
                currentX = PRINT_BED_TOP_RIGHT[0];

                continue;
            }

            //Otherwise, part fits

            part->setCentroidPosition(part_position);

            currentX = nextX;

            nextY = std::max(topY, nextY);
            
            break;
        }
    }

    return true;
}

void Assembler::generateSlicerGcode()
{
    //Find internal parts
    //They will already be in the correct position from previously arranging them
    //Save each as its own stl file

    int i = 0;

    std::vector<std::string> filenames;

    for (std::shared_ptr<Part> part : initial_assembly_->getParts())
    {
        if (part->getType() != Part::INTERNAL)
            continue;

        std::stringstream filename_ss;

        filename_ss << WORKING_DIR << "internal_part_" << i << ".stl";

        std::stringstream filename_local_ss;

        filename_local_ss << "internal_part_" << i << ".stl";

        part->saveShape(filename_ss.str());

        filenames.push_back(filename_local_ss.str());

        i ++;        
    }

    //Call Prusa Slicer with the stl files and the correct settings (don't allow rearranging)

    std::stringstream command_ss;

    command_ss << "(cd " << WORKING_DIR << " && " << "prusa-slicer --export-gcode --dont-arrange --merge --output assembler.gcode --load arms_prusa_config.ini";

    for (std::string filename : filenames)
        command_ss << " " << filename;

    command_ss << ")";

    std::cout << "Slicing command: " << command_ss.str() << std::endl;

    int ret = std::system(command_ss.str().c_str());

    if (ret == 0) {
        std::cout << "Slicing completed successfully!" << std::endl;
    } else {
        std::cerr << "Error: PrusaSlicer execution failed!" << std::endl;
    }

    std::stringstream gcode_ss;

    gcode_ss << WORKING_DIR << "assembler.gcode";

    //Load the GCode that Prusa writes
    std::ifstream gcodeFile(gcode_ss.str());
    if (!gcodeFile) {
        std::cerr << "Error: Cannot open G-code file." << std::endl;
        return;
    }

    std::string line;
    while (std::getline(gcodeFile, line)) 
    {
        slicer_gcode_.push_back(line);

        // if (line == "; Filament-specific end gcode")
        //     break;
    }

    //Iterate through gcode in reverse and delete up to and including the END_PRINT macro
    for (int i = slicer_gcode_.size() - 1; i >= 0; i--)
    {
        if (slicer_gcode_[i] == "END_PRINT")
        {
            slicer_gcode_.pop_back();
            break;   
        }

        else
        {
            slicer_gcode_.pop_back();
        }
    }
}


std::vector<std::shared_ptr<AssemblyNode>> Assembler::breadthFirstZAssembly()
{
    std::vector<std::shared_ptr<AssemblyNode>> path;

    std::shared_ptr<AssemblyNode> target_node;

    std::shared_ptr<AssemblyNode> first_part_node;

    std::shared_ptr<AssemblyNode> base_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

    base_node->assembly_ = target_assembly_;

    base_node->id_ = nodeIdGenerator(target_assembly_->getPartIds());

    std::queue<std::shared_ptr<AssemblyNode>> queue;

    std::set<std::shared_ptr<AssemblyNode>> visited_nodes;

    std::map<std::shared_ptr<AssemblyNode>, std::shared_ptr<AssemblyNode>> parents;

    queue.push(base_node);

    visited_nodes.emplace(base_node);

    std::cout << "starting breadth search" << std::endl;

    while (queue.size() > 0)
    {
        std::shared_ptr<AssemblyNode> current_node = queue.front();

        std::cout << std::endl << std::endl << "Current node: " << current_node->id_ << std::endl;

        for (std::shared_ptr<Part> part : current_node->assembly_->getParts())
            std::cout << part->getId() << std::endl;

        std::stringstream ss;

        ss << WORKING_DIR << "node_" << current_node->id_ << ".stl";

        current_node->assembly_->saveAsSTL(ss.str().c_str());
 
        queue.pop();

        //Check if current node is target node (e.g., has no meshes)
        if (current_node->assembly_->getParts().size() == 0)               
            target_node = current_node;

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        for (std::shared_ptr<AssemblyNode> neighbour : neighbours)
        {
            //std::cout << "neighbour: " << neighbour->id_ << std::endl; 

            //If neighbour has already been visited, move on
            if (visited_nodes.find(neighbour) != visited_nodes.end())
                continue;

            //std::cout << "not visited" << std::endl;

            visited_nodes.emplace(neighbour);

            queue.push(neighbour);

            parents[neighbour] = current_node;
        }
    }

    if (!parents.count(target_node))
    {
        std::cout << "No path found" << std::endl;

        return path;
    }

    std::shared_ptr<AssemblyNode> path_node = target_node;

    path.push_back(target_node);

    while (parents.count(path_node))
    {
        path_node = parents[path_node];

        path.push_back(path_node);
    }

    //TODO - find internal first node

    std::cout << std::endl << "Found path" << std::endl << std::endl; 

    int n = 0;

    for (std::shared_ptr<AssemblyNode> node : path)
    {
        std::stringstream ss;

        ss << WORKING_DIR << "assembly_node_" << n << ".stl";

        node->assembly_->saveAsSTL(ss.str());

        n ++;
    }

    return path;
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::findNodeNeighbours(std::shared_ptr<AssemblyNode> node)
{
    std::vector<std::shared_ptr<AssemblyNode>> neighbours;

    std::vector<std::shared_ptr<Part>> parts = node->assembly_->getParts();

    // //Iterate through each part
    // //Try to move the part vertically over 10cm
    // //Every 0.1cm check collisions with every other part
    // //If no collisions take place, create a new assemblynode with this part removed and add it to neighbours list
    float step_size = 1.0f;


    for (std::shared_ptr<Part> part : parts)
    {
        bool collides = false;

        float step_distance = 0;

        for (int num_steps = 0; num_steps <= 5; ++num_steps)
        {   
            //TODO for now assume that bolts don't collide. This needs to be handled correctly
            if (part->getType() == Part::SCREW)
                break;

            part->translate(gp_Vec(0, 0, step_size));

            step_distance += step_size;

            std::cout << "stepping!" << std::endl;

            for (std::shared_ptr<Part> otherPart : parts)
            {
                //Don't try to collide with self
                if (otherPart->getId() == part->getId())
                    continue;

                //std::cout << "Checking collision: " << part->getId() << " and "  << otherPart->getId() << std::endl;

                if (part->collide(otherPart))
                {
                    //std::cout << "Collision" << std::endl;

                    collides = true;
                    break;
                }
            }

            if (collides)
                break;
        }

        //TODO for now assume that bolts don't collide. This needs to be handled correctly
        if (part->getType() == Part::SCREW)
            collides = false;

        //std::cout << "Moved " << num_steps << " steps" << std::endl;

        //Put the part back where it was
        part->translate(gp_Vec(0, 0, -step_distance));

        if (collides)
            continue;

        //std::cout << std::endl;

        //Create new assembly node
        std::shared_ptr<Assembly> neighbour_assembly = std::shared_ptr<Assembly>(new Assembly());

        std::shared_ptr<AssemblyNode> neighbour_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

        for (std::shared_ptr<Part> part_to_add : parts)
        {
            //Don't copy over part to be removed
            if (part_to_add->getId() == part->getId())
                continue;

            neighbour_assembly->addPart(part_to_add->clone());
        }

        neighbour_node->assembly_ = neighbour_assembly;

        neighbour_node->id_ = nodeIdGenerator(neighbour_assembly->getPartIds());

        neighbours.push_back(neighbour_node);
        
    } 

    return neighbours;
}

size_t Assembler::nodeIdGenerator(std::vector<size_t> object_ids)
{
    for (auto const& [id, obj_ids] : node_id_map_)
    {
        if (object_ids.size() != obj_ids.size())
            continue;
        
        bool allFound = true;

        for (size_t obj_id_1 : object_ids)
        {
            bool found = false;

            for (size_t obj_id_2 : obj_ids)
            {
                if (obj_id_1 == obj_id_2)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                allFound = false;

                break;
            }
        }

        if (allFound)
            return id;
    }

    //Not found, new ID
    node_id_map_[next_node_ID_++] = object_ids;

    return next_node_ID_ - 1;
}

void Assembler::generateInitialAssembly()
{
    std::cout << std::endl << "Generating initial assembly" << std::endl;

    initial_assembly_ = std::shared_ptr<Assembly>(new Assembly());

    for (std::shared_ptr<Part> part : target_assembly_->getParts())
    {
        std::cout << "Target Part: " << part->getId() << std::endl;

        initial_assembly_->addPart(part->clone());

        std::cout << "Initial Part: " << initial_assembly_->getParts().back()->getId() << std::endl;
    }
}

void Assembler::generateNegatives()
{
    std::cout << "Generating negatives" << std::endl;

    for (std::shared_ptr<Part> part : initial_assembly_->getParts())
    {
        std::cout << "Part type: " << part->getType() << std::endl;

        //Only create negatives or external parts
        if (part->getType() != Part::EXTERNAL)
            continue;

        std::cout << "Creating part negative" << std::endl;

        part->positionPartInBay(bay_occupancy_);

        CradleGenerator cradle_gen(part->getName(), *part->getShape());

        cradle_gen.createNegative();

        //part->createNegative();

        //part->createNegativeAndPositionPart(bay_occupancy_);
    }
}

void Assembler::generateGrasps()
{
    std::cout << "Generating grasps" << std::endl;

    for (std::shared_ptr<Part> part : target_assembly_->getParts())
    {
        if (part->getType() == Part::INTERNAL)
            part->generatePPGGraspPosition();

        part->generateVacuumGraspPosition();
    }
}