#include "assembler/Assembler.hpp"

#include "assembler/Assembly.hpp"


#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "yaml-cpp/yaml.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <fstream>
#include <queue>
#include <set>
#include <map>

Assembler::Assembler()
{
    //TODO
    output_path_ = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";
    input_path_ = ament_index_cpp::get_package_share_directory("assembler") + "/../../../../";

    std::cout << "Ouput path: " << output_path_ << std::endl;
    std::cout << "Input path: " << input_path_ << std::endl;

    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());
}

void Assembler::generateAssemblySequence() 
{
    if (target_assembly_ == nullptr)
        return;

    generateInitialAssembly();

    generateNegatives();

    //If initial (or target) assembly has internal parts, do slicer stuff and then set target_assembly position
    if (initial_assembly_->getNumInternalParts() != 0)
    {
        //TODO - get the GCODE and positions/poses of internal parts from prusa slicer

        //TODO - set target assembly positions based on internal part positions
    }

    //If initial (or target) assembly has no internal parts, set target_assembly position to middle of bed
    else
    {
        //set target assembly positions at bed center
        target_assembly_->placeOnPoint(Point(PRINT_BED_CENTER[0], PRINT_BED_CENTER[1], PRINT_BED_HEIGHT));
    }

    //At this point we have the target assembly, with the components in the correct positions on the print bed that
    //they will be in the finished design, and the initial assembly, with the external parts in their parts bay positions,
    //the screws placed nowhere, and the internal parts needed fetching from Prusa

    std::cout << "Generating assembly" << std::endl;

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


    //In these commands for now let's give the part-height as the height of the pnp location relative to 0,
    //and the z location as the height of the pnp placement location relative to 0
    //You then might need to offset by the vacuum toolhead offset at some point


    YAML::Node root;

    YAML::Node commands = YAML::Node(YAML::NodeType::Sequence);

    std::vector<std::shared_ptr<Part>> initial_parts = initial_assembly_->getParts();

    //TODO might get rid of locating external parts
    for (std::shared_ptr<Part> part : initial_parts)
    {        
        if (!part->getType() == Part::EXTERNAL)
            continue;

        YAML::Node detect_part_command;
        detect_part_command["command-type"] = "LOCATE_EXTERNAL_PART";
        detect_part_command["command-properties"]["part-description"] = ""; //TODO
        detect_part_command["command-properties"]["part-name"] = ""; //TODO
        detect_part_command["command-properties"]["part-id"] = part->getId();
        detect_part_command["command-properties"]["part-height"] = part->getMeshMaxZ(); //This is the height from 0, accounting for the cradle etc

        commands.push_back(detect_part_command);
    }


    //Iterate through each of the added parts in the path
    for (size_t part_id : ordered_part_additions)
    {
        std::cout << "Adding PLACE_PART command" << std::endl;

        std::cout << "Part type: " << initial_assembly_->getPartById(part_id)->getType() << std::endl;

        //Do nothing with the base object if it's internal  //TODO janky
        if (part_id == path[1]->assembly_->getPartIds()[0] && path[1]->assembly_->getParts()[0]->getType() == Part::INTERNAL)
            continue;

        Point target_position = target_assembly_->getPartById(part_id)->getCentroidPosition();

        YAML::Node place_part_command;

        place_part_command["command-type"] = "PLACE_PART";
        place_part_command["command-properties"]["part-name"] = ""; //TODO
        place_part_command["command-properties"]["part-id"] = part_id;
        place_part_command["command-properties"]["x-target-pos"] = CGAL::to_double(target_position.x());
        place_part_command["command-properties"]["y-target-pos"] = CGAL::to_double(target_position.y());                    
        place_part_command["command-properties"]["z-target-pos"] = target_assembly_->getPartById(part_id)->getMeshMaxZ();
        
        commands.push_back(place_part_command);
    }

    root["commands"] = commands;



    std::ofstream fout(Assembler::output_path_ + "assembly_plan.yaml");

    fout << root;

    fout.close();


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

        std::cout << "Current node: " << current_node->id_ << std::endl;
 
        queue.pop();

        //Check if current node is target node (e.g., has no meshes)
        if (current_node->assembly_->getParts().size() == 0)               
            target_node = current_node;

        std::vector<std::shared_ptr<AssemblyNode>> neighbours = findNodeNeighbours(current_node);

        for (std::shared_ptr<AssemblyNode> neighbour : neighbours)
        {
            std::cout << "neighbour: " << neighbour->id_ << std::endl; 

            //If neighbour has already been visited, move on
            if (visited_nodes.find(neighbour) != visited_nodes.end())
                continue;

            std::cout << "not visited" << std::endl;

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

    for (std::shared_ptr<AssemblyNode> node : path)
    {
        std::cout << "node id: " << node->id_ << " parts:" << std::endl;
        
        for (std::shared_ptr<Part> part : node->assembly_->getParts())
        {
            Point centroid = part->getCentroidPosition();

            std::cout << centroid.x() << " " << centroid.y() << " " << centroid.z() << std::endl;
        }

        std::cout << std::endl;
    }

    return path;
}

std::vector<std::shared_ptr<AssemblyNode>> Assembler::findNodeNeighbours(std::shared_ptr<AssemblyNode> node)
{
    std::vector<std::shared_ptr<AssemblyNode>> neighbours;


    std::vector<std::shared_ptr<Part>> parts = node->assembly_->getParts();

    //Iterate through each part
    //Try to move the part vertically over 10cm
    //Every 0.1cm check collisions with every other part
    //If no collisions take place, create a new assemblynode with this part removed and add it to neighbours list
    float step_size = 0.1f;


    for (std::shared_ptr<Part> part : parts)
    {
        bool collides = false;

        int num_steps = 0;

        for (int i = 0; i != 100; i ++)
        {   
            num_steps ++;

            part->translate(Vector(0, 0, step_size));

            for (std::shared_ptr<Part> otherPart : parts)
            {
                //Don't try to collide with self
                if (otherPart->getId() == part->getId())
                    continue;

                Point cd1 = part->getCentroidPosition();

                Point cd2 = otherPart->getCentroidPosition();

                std::cout << "Checking collision: " << part->getId() << " and "  << otherPart->getId() << std::endl;

                std::cout << "Z Positions: " << cd1.z() << " | " << cd2.z() << std::endl;

                if (part->collide(otherPart))
                {
                    std::cout << "Collision" << std::endl;

                    collides = true;
                    break;
                }
            }

            if (collides)
                break;
        }

        //Put the part back where it was
        part->translate(Vector(0, 0, -num_steps * step_size));

        if (collides)
            continue;

        std::cout << std::endl;

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



    // float max_z = 0;
    // std::shared_ptr<Part> max_z_part;

    // for (std::shared_ptr<Part> part : parts)
    // {
    //     if (part->getCentroid().z() >= max_z)
    //     {
    //         max_z = part->getCentroid().z();

    //         max_z_part = part;
    //     }
    // }

    // //No assembly can be a neighbour
    // if (max_z_part == nullptr)
    //     return neighbours;

    // std::shared_ptr<Assembly> neighbour_assembly = std::shared_ptr<Assembly>(new Assembly());

    // std::shared_ptr<AssemblyNode> neighbour_node = std::shared_ptr<AssemblyNode>(new AssemblyNode());

    // for (std::shared_ptr<Part> part : parts)
    // {
    //     //Don't copy over part to be removed
    //     if (part == max_z_part)
    //         continue;

    //     neighbour_assembly->addPart(part->clone());
    // }

    // neighbour_node->assembly_ = neighbour_assembly;

    // neighbour_node->id_ = nodeIdGenerator(neighbour_assembly->getPartIds());

    // neighbours.push_back(neighbour_node);


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
    negative_substrate_->setCentroidPosition(Point(0, 0, 0));

    int i = 0;

    for (std::shared_ptr<Part> part : initial_assembly_->getParts())
    {
        //Only create negatives or external parts
        if (!part->getType() == Part::EXTERNAL)
            continue;

        std::stringstream ss;

        ss << "negative_" << i << "_" << part->getName() << ".stl";

        part->setCentroidPosition(Point(0, 0, 0));

        Point bay_displacement = part->createNegative(negative_substrate_, ss.str());

        //Correct for parts bed height
        bay_displacement += Vector(0, 0, PARTS_BED_HEIGHT);

        //Set to bay position
        bay_displacement += Vector(PARTS_BAY_POSITIONS[i][0], PARTS_BAY_POSITIONS[i][1], 0);

        //Then place part in the correct position
        part->setCentroidPosition(bay_displacement);

        i ++;
    }
}