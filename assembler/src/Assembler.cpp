#include "assembler/Assembler.hpp"

#include "assembler/Assembly.hpp"

#include "assembler/CradleGenerator.hpp"

#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"

#include "yaml-cpp/yaml.h"

#include "assembler/Config.hpp"

#include <iostream>
#include <fstream>
#include <queue>
#include <set>
#include <map>

#include <cstdlib>

#include "assembler/Part.hpp"

#include "assembler/Logger.hpp"

#include "assembler/VacuumGraspGenerator.hpp"

#include "assembler/PPGGraspGenerator.hpp"

#include "assembler/GCodeGenerator.hpp"

Assembler::Assembler()
{  
    initialisePartBays();
}

/*  Initialise the 2D occupancy values for the grid of parts bays - outer vector corresponds to bays of different sizes, inner vector to individual bays
*/
void Assembler::initialisePartBays()
{
    for (std::vector<gp_Pnt> bays : PARTS_BAY_POSITIONS)
    {
        bay_occupancy_.push_back(std::vector<bool>());

        for (gp_Pnt bay_position : bays)
        {
            bay_occupancy_.back().push_back(false);
        }
    }
}

/*  The main assembler function. Generates grasps and cradles for each part as appropriate, determines the correct order of assembly, and produces a .yaml command file to be sent to ARMS
*/
void Assembler::generateAssemblySequence() 
{
    RCLCPP_INFO(logger(), "Starting assembler analysis");

    if (target_assembly_ == nullptr)
    {
        RCLCPP_FATAL(logger(), "No target assembly");
        rclcpp::shutdown();
        return;
    }

    //For now, at least one part must be internal (printed)
    if (target_assembly_->getNumInternalParts() == 0)
    {
        RCLCPP_FATAL(logger(), "No internal parts in target assembly");
        rclcpp::shutdown();
        return;
    }

    generateInitialAssembly();  //Initial part positions not set here

    assembly_path_ = breadthFirstZAssembly();   //All parts in the path are in the target_assembly position

    if (assembly_path_.size() < 2)
    {
        RCLCPP_WARN(logger(), "Only one assembly state found");
        return;
    }

    //First node on assembly path has no assembled parts
    //Second node has one assembled part - this should be internal and is the base part

    if (assembly_path_[1]->assembly_->getAssembledPartTransforms().size() != 1)
    {
        RCLCPP_FATAL(logger(), "Second assembly node doesn't have one assembled part");
        rclcpp::shutdown();
        return;
    }

    if (assembly_path_[1]->assembly_->getAssembledPartTransforms().begin()->first->getType() != Part::PART_TYPE::INTERNAL)
    {
        RCLCPP_FATAL(logger(), "Second assembly node part isn't internal");
        rclcpp::shutdown();
        return;
    }

    base_part_ = assembly_path_[1]->assembly_->getAssembledPartTransforms().begin()->first;  //Set base part

    generateInitialPartPositions(); //Initial part positions are now set here   Sets inital_assembly positions

    generateGrasps();

    generateNegatives();

    generateSlicerGcode();  //Uses initial_assembly positions

    alignAssemblyPathToInitialAssembly();

    //alignTargetAssemblyToInitialAssembly();



    debugState();


    std::vector<size_t> part_addition_order = generatePartAdditionOrder();

    generateGCodeFile(part_addition_order);

    generateCommandFile(part_addition_order);
}

void Assembler::debugState()
{
    RCLCPP_INFO(logger(), "Base part ID: %ld", base_part_->getId());

    RCLCPP_INFO(logger(), "INITIAL STATE");

    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        RCLCPP_INFO(logger(), "Part Id: %ld Position: %f, %f, %f", part->getId(), initial_transform.X(), initial_transform.Y(), initial_transform.Z());
    }

    RCLCPP_INFO(logger(), "TARGET STATE");

    for (auto const& [part, target_transform] : target_assembly_->getAssembledPartTransforms())  //All parts in target assembly are assembled
    {
        RCLCPP_INFO(logger(), "Part Id: %ld Position: %f, %f, %f", part->getId(), target_transform.X(), target_transform.Y(), target_transform.Z());
    }

    RCLCPP_INFO(logger(), "ASSEMBLY PATH");

    for (std::shared_ptr<AssemblyNode> node : assembly_path_)
    {
        RCLCPP_INFO(logger(), "NODE");

        for (auto const& [part, transform] : node->assembly_->getUnassembledPartTransforms())  
        {
            RCLCPP_INFO(logger(), "Unassembled part Id: %ld Position: %f, %f, %f", part->getId(), transform.X(), transform.Y(), transform.Z());
        }

        for (auto const& [part, transform] : node->assembly_->getAssembledPartTransforms()) 
        {
            RCLCPP_INFO(logger(), "Assembled part Id: %ld Position: %f, %f, %f", part->getId(), transform.X(), transform.Y(), transform.Z());
        }
    }

}

/*
Generates a direct GCode file that can be uploaded to ARMS and 'printed' directly
*/
void Assembler::generateGCodeFile(std::vector<size_t> part_addition_order)
{
    GCodeGenerator::generate(initial_assembly_, target_assembly_, base_part_, part_addition_order, slicer_gcode_);
}

/*  Uses the grasps, part positions and path to generate the command file to be sent to ARMS
*/
void Assembler::generateCommandFile(std::vector<size_t> part_addition_order)
{
    //In these commands for now let's give the part-height as the height of the pnp location relative to 0,
    //and the z location as the height of the pnp placement location relative to 0
    //You then might need to offset by the vacuum toolhead offset at some point

    YAML::Node root;

    YAML::Node commands = YAML::Node(YAML::NodeType::Sequence);


    //Designate internal parts
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        if (part->getType() != Part::INTERNAL)
            continue;   

        gp_Vec pick_position = SumPoints(initial_transform, part->getVacuumGrasp());

        gp_Vec place_position = SumPoints(target_assembly_->getAssembledPartTransforms()[part], part->getVacuumGrasp());

        PPGGrasp ppg_grasp = part->getPPGGrasp();

        gp_Vec grasp_position = SumPoints(initial_transform, ppg_grasp.position_);

        RCLCPP_INFO(logger(), "Pick pos: %f, %f, %f", pick_position.X(), pick_position.Y(), pick_position.Z());

        RCLCPP_INFO(logger(), "Place pos: %f, %f, %f", place_position.X(), place_position.Y(), place_position.Z());

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_INTERNAL_PART";
        designate_part_command["command-properties"]["part-id"] = part->getId();
        designate_part_command["command-properties"]["part-pick-height"] = pick_position.Z();
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z();
        designate_part_command["command-properties"]["part-pick-pos-x"] = pick_position.X();
        designate_part_command["command-properties"]["part-pick-pos-y"] = pick_position.Y();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();
        designate_part_command["command-properties"]["part-grasp-height"] = grasp_position.Z();
        designate_part_command["command-properties"]["part-grasp-pos-x"] = grasp_position.X();
        designate_part_command["command-properties"]["part-grasp-pos-y"] = grasp_position.Y();
        designate_part_command["command-properties"]["part-grasp-angle"] = ppg_grasp.rotation_;
        designate_part_command["command-properties"]["part-grasp-width"] = ppg_grasp.width_;

        commands.push_back(designate_part_command);
    }

    //Designate external parts
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {        
        if (part->getType() != Part::EXTERNAL)
            continue;

        gp_Vec pick_position = SumPoints(initial_transform, part->getVacuumGrasp());

        gp_Vec place_position = SumPoints(target_assembly_->getAssembledPartTransforms()[part], part->getVacuumGrasp());

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_EXTERNAL_PART";
        designate_part_command["command-properties"]["part-id"] = part->getId();
        designate_part_command["command-properties"]["part-pick-height"] = pick_position.Z(); //This is the height from 0, accounting for the cradle etc TODO
        designate_part_command["command-properties"]["part-place-height"] = place_position.Z(); //This is the height from 0, accounting for the cradle etc
        designate_part_command["command-properties"]["part-pick-pos-x"] = pick_position.X();
        designate_part_command["command-properties"]["part-pick-pos-y"] = pick_position.Y();
        designate_part_command["command-properties"]["part-place-pos-x"] = place_position.X();
        designate_part_command["command-properties"]["part-place-pos-y"] = place_position.Y();

        commands.push_back(designate_part_command);
    }

    //Designate screws
    for (auto const& [part, initial_transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        if (part->getType() != Part::SCREW)
            continue;

        gp_Pnt place_position(target_assembly_->getAssembledPartTransforms()[part].X(), 
                                target_assembly_->getAssembledPartTransforms()[part].Y(), 
                                target_assembly_->getAssembledPartTransforms()[part].Z() + 5); //TODO need to work out what to do with screw z offset

        YAML::Node designate_part_command;
        designate_part_command["command-type"] = "DESIGNATE_SCREW";
        designate_part_command["command-properties"]["part-id"] = part->getId();
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
    for (size_t part_id : part_addition_order)
    {
        Part::PART_TYPE part_type = initial_assembly_->getPartById(part_id)->getType();

        RCLCPP_DEBUG(logger(), "Adding PLACE_PART with type %d and name %s", static_cast<int>(part_type), initial_assembly_->getPartById(part_id)->getName().c_str());

        //Do nothing with the base object if it's internal  //TODO which it must be at the moment - this must be the first part in the list right?
        if (part_id == base_part_->getId() && base_part_->getType() == Part::INTERNAL)
        {
            RCLCPP_DEBUG(logger(), "Skipping base object");
 
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

/*  
    Generate the initial internal part positions by populating the print bed, and generate the initial external part positions using the bay occupancy
*/
void Assembler::generateInitialPartPositions()
{
    RCLCPP_INFO(logger(), "Generating ordered part addition");

    //Arrange the internal parts on the bed
    if (!arrangeInternalParts())
    {
        RCLCPP_FATAL(logger(), "Internal parts cannot be arranged on bed");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(logger(), "Generating external part bay positions");

    //External initial part
    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms())  //All parts in initial assembly are unassembled
    {
        //Only create negatives or external parts
        if (part->getType() != Part::EXTERNAL)
            continue;

        initial_assembly_->setUnassembledPart(part, part->generateBayPosition(bay_occupancy_));
    }
}

void Assembler::alignAssemblyPathToInitialAssembly()
{
    RCLCPP_INFO(logger(), "Aligning assembly path with initial assembly base part");

    gp_Pnt target_base_part_pos = target_assembly_->getAssembledPartTransforms()[base_part_]; //TODO need to check this part exists

    gp_Pnt initial_base_part_pos = initial_assembly_->getUnassembledPartTransforms()[base_part_];

    gp_Vec delta(target_base_part_pos, initial_base_part_pos);

    for (std::shared_ptr<AssemblyNode> node : assembly_path_)
    {
        //Unassembled parts get set to initial positions
        for (auto const& [part, transform] : node->assembly_->getUnassembledPartTransforms())  
        {
            node->assembly_->setUnassembledPart(part, initial_assembly_->getUnassembledPartTransforms()[part]);
        }

        for (auto const& [part, transform] : node->assembly_->getAssembledPartTransforms()) 
        {
            node->assembly_->setAssembledPart(part, transform.Translated(delta));
        }
    }
}

// /*
//     Align the positions of the assembled parts in the target assembly to the position of the base part as given by the intial assembly
// */
// void Assembler::alignTargetAssemblyToInitialAssembly()
// {
//     RCLCPP_INFO(logger(), "Aligning target assembly with initial assembly base part");
    
//     gp_Pnt target_base_part_pos = target_assembly_->getAssembledPartTransforms()[base_part_]; //TODO need to check this part exists

//     gp_Pnt initial_base_part_pos = assembly_path_[1]->assembly_->getAssembledPartTransforms()[base_part_];

//     RCLCPP_INFO(logger(), "Target base part pos: %f, %f, %f", target_base_part_pos.X(), target_base_part_pos.Y(), target_base_part_pos.Z());

//     RCLCPP_INFO(logger(), "Initial base part pos: %f, %f, %f", initial_base_part_pos.X(), initial_base_part_pos.Y(), initial_base_part_pos.Z());

//     gp_Vec delta(initial_base_part_pos, target_base_part_pos);

//     RCLCPP_INFO(logger(), "Delta: %f, %f, %f", delta.X(), delta.Y(), delta.Z());

//     for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms())  //All parts in target assembly are assemble
//     {
//         target_assembly_->setAssembledPart(part, transform.Translated(delta));
//     }

//     target_base_part_pos = target_assembly_->getAssembledPartTransforms()[base_part_]; //TODO need to check this part exists

//     initial_base_part_pos = assembly_path_[1]->assembly_->getAssembledPartTransforms()[base_part_];

//     RCLCPP_INFO(logger(), "Target base part pos: %f, %f, %f", target_base_part_pos.X(), target_base_part_pos.Y(), target_base_part_pos.Z());

//     RCLCPP_INFO(logger(), "Initial base part pos: %f, %f, %f", initial_base_part_pos.X(), initial_base_part_pos.Y(), initial_base_part_pos.Z());
// }



/* Output: vector of parts ids in the order that they must be assembled to go from starting assembly to target assembly

    Iterates through the assembly states and determines which part must be added to go from one state to the next
*/
std::vector<size_t> Assembler::generatePartAdditionOrder()
{
    RCLCPP_INFO(logger(), "Generating ordered part addition");

    std::vector<size_t> ordered_part_additions;

    for (std::shared_ptr<AssemblyNode> node : assembly_path_)
    {
        for (size_t part_id : node->assembly_->getAssembledPartIds())
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

                RCLCPP_DEBUG(logger(), "Part %ld", part_id);

                continue;
            }
        }
    }

    return ordered_part_additions;
}

/* Arrange internal parts on the print bed
*/
bool Assembler::arrangeInternalParts()
{
    RCLCPP_INFO(logger(), "Arranging initial internal parts on bed");

    double currentY = PRINT_BED_BOTTOM_LEFT[1];

    double currentX = PRINT_BED_TOP_RIGHT[0];

    double nextY = PRINT_BED_BOTTOM_LEFT[1];

    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms())    //All parts unassembled in initial assembly
    {
        if (part->getType() != Part::INTERNAL)
            continue;

        while (true)
        {
            TopoDS_Shape shape = *part->getShape();

            gp_Pnt part_position(currentX - ShapeAxisSize(shape, 0) / 2, currentY + ShapeAxisSize(shape, 1) / 2, ShapeAxisSize(shape, 2) / 2);

            double nextX = currentX - ShapeAxisSize(shape, 0) - PRINT_MIN_SPACING;

            double topY = currentY + ShapeAxisSize(shape, 1) + PRINT_MIN_SPACING;

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
            initial_assembly_->setUnassembledPart(part, part_position);

            currentX = nextX;

            nextY = std::max(topY, nextY);
            
            break;
        }
    }

    return true;
}

/* Generate GCode for the internal parts, the print positions of which are given by the initial assembly
*/
void Assembler::generateSlicerGcode()
{
    RCLCPP_INFO(logger(), "Generating Slicer Gcode for internal parts");

    //Find internal parts
    //They will already be in the correct position from previously arranging them
    //Save each as its own stl file

    int i = 0;

    std::vector<std::string> filenames;

    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms()) //No assembled parts in target assembly
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
    //command_ss << "(cd " << WORKING_DIR << " && " << "slic3r --dont-arrange --merge --output assembler.gcode --load arms_prusa_config.ini";

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
        RCLCPP_FATAL(logger(), "Cannot open Gcode file");
        rclcpp::shutdown();
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
    RCLCPP_INFO(logger(), "Generating assembly path");

    std::vector<std::shared_ptr<AssemblyNode>> path;

    std::shared_ptr<AssemblyNode> target_node;

    std::shared_ptr<AssemblyNode> first_part_node;

    std::shared_ptr<AssemblyNode> base_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

    base_node->assembly_ = target_assembly_;

    base_node->id_ = nodeIdGenerator(target_assembly_->getAssembledPartIds());

    std::queue<std::shared_ptr<AssemblyNode>> queue;

    std::set<std::shared_ptr<AssemblyNode>> visited_nodes;

    std::map<std::shared_ptr<AssemblyNode>, std::shared_ptr<AssemblyNode>> parents;

    queue.push(base_node);

    visited_nodes.emplace(base_node);

    while (queue.size() > 0)
    {
        std::shared_ptr<AssemblyNode> current_node = queue.front();
 
        queue.pop();

        //Check if current node is target node (e.g., has no assembled parts)
        if (current_node->assembly_->getAssembledPartTransforms().size() == 0)               
            target_node = current_node;

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        for (std::shared_ptr<AssemblyNode> neighbour : neighbours)
        {
            //If neighbour has already been visited, move on
            if (visited_nodes.find(neighbour) != visited_nodes.end())
                continue;

            visited_nodes.emplace(neighbour);

            queue.push(neighbour);

            parents[neighbour] = current_node;
        }
    }

    if (!parents.count(target_node))
    {
        RCLCPP_FATAL(logger(), "No assembly path found");
        rclcpp::shutdown();
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

    RCLCPP_INFO(logger(), "Assembly path found");

    return path;
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::findNodeNeighbours(std::shared_ptr<AssemblyNode> node)
{
    // Set each part to its position within the assembly
    node->assembly_->setPartTransforms();

    std::vector<std::shared_ptr<AssemblyNode>> neighbours;

    // //Iterate through each part
    // //Try to move the part vertically over 10cm
    // //Every 0.1cm check collisions with every other part
    // //If no collisions take place, create a new assemblynode with this part removed and add it to neighbours list
    float step_size = 1.0f;

    for (auto const& [part, transform] : node->assembly_->getAssembledPartTransforms())
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

            for (auto const& [other_part, other_transform] : node->assembly_->getAssembledPartTransforms())
            {
                //Don't try to collide with self
                if (other_part->getId() == part->getId())
                    continue;

                if (part->collide(other_part))
                {
                    collides = true;
                    break;
                }
            }

            if (collides)
                break;
        }

        //Put the part back where it was
        part->translate(gp_Vec(0, 0, -step_distance));

        if (collides)
            continue;

        //Create new assembly node
        std::shared_ptr<Assembly> neighbour_assembly = std::shared_ptr<Assembly>(new Assembly());

        std::shared_ptr<AssemblyNode> neighbour_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());


        for (auto const& [part_to_add, transform_to_add] : node->assembly_->getUnassembledPartTransforms())
        {
            neighbour_assembly->setUnassembledPart(part_to_add, transform_to_add);
        }


        for (auto const& [part_to_add, transform_to_add] : node->assembly_->getAssembledPartTransforms())
        {
            if (part_to_add->getId() != part->getId())
                neighbour_assembly->setAssembledPart(part_to_add, transform_to_add);

            //For the remove part, add it to the unassemlbed parts map and set the transform as the transofmr of the part in the initial assembly
            else
                neighbour_assembly->setUnassembledPart(part_to_add, initial_assembly_->getUnassembledPartTransforms()[part_to_add]);  
            
        }

        neighbour_node->assembly_ = neighbour_assembly;

        neighbour_node->id_ = nodeIdGenerator(neighbour_assembly->getAssembledPartIds());

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

/* Create a new set assembly state, for now just copying over the part positions
*/
void Assembler::generateInitialAssembly()
{
    RCLCPP_INFO(logger(), "Generating initial assembly");

    initial_assembly_ = std::shared_ptr<Assembly>(new Assembly());

    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms()) //No unassembled parts in target assembly
    {
        initial_assembly_->setUnassembledPart(part, transform);
    }
}

/* Create printable jigs for each external part to fit int he parts bay. Also allocates each external
part to a bay in the parts bay and sets the transforms of the parts in the initial assembly
*/
void Assembler::generateNegatives()
{
    RCLCPP_INFO(logger(), "Generating external part jigs");

    for (auto const& [part, transform] : initial_assembly_->getUnassembledPartTransforms())
    {
        //Only create negatives or external parts
        if (part->getType() != Part::EXTERNAL)
            continue;

        CradleGenerator cradle_gen(part->getName(), *part->getShape());

        float part_jig_z_offset = cradle_gen.createSimpleNegative(BAY_SIZES[part->getBaySizeIndex()]);

        initial_assembly_->setUnassembledPart(part, gp_Pnt(transform.X(), transform.Y(), JIG_CENTER_Z + part_jig_z_offset));

    }
}

/* Generate both vacuum grasps and parallel plate gripper grasps for parts
*/
void Assembler::generateGrasps()
{
    for (auto const& [part, transform] : target_assembly_->getAssembledPartTransforms()) //No unassembled parts in target assembly
    {
        //if (part->getType() == Part::INTERNAL)
        //    part->setPPGGrasp(PPGGraspGenerator::generate(part));

        //if (part->getType() == Part::EXTERNAL)
        //    part->setVacuumGrasp(VacuumGraspGenerator::generate(part));
    }
}