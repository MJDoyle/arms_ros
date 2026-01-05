#ifndef ASSEMBLER_HPP
#define ASSEMBLER_HPP


#include <string>
#include <memory>
#include <vector>
#include <map>


class aiScene;

class Assembly;

class Part;

class AssemblyNode;

class Assembler {

public:
    explicit Assembler();

    void generateAssemblySequence();

    void setTargetAssembly(std::shared_ptr<Assembly> target_assembly) { target_assembly_ = target_assembly; }

    std::shared_ptr<Assembly> getInitialAssembly() { return initial_assembly_; }

private:

    void initialisePartBays();

    void generateInitialAssembly();

    void generateInitialPartPositions();

    void generateGCodeFile(std::vector<size_t> part_addition_order);

    void generateCommandFile(std::vector<size_t> part_addition_order);

    std::vector<size_t> generatePartAdditionOrder();

    size_t nodeIdGenerator(std::vector<size_t> object_ids);

    std::map<size_t, std::vector<size_t>> node_id_map_;

    size_t next_node_ID_ = 0;

    void generateNegatives();

    void generateSlicerGcode();

    void generateGrasps();

    void debugState();

    bool arrangeInternalParts();

    //void alignTargetAssemblyToInitialAssembly();

    void alignAssemblyPathToInitialAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> breadthFirstZAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> findNodeNeighbours(std::shared_ptr<AssemblyNode> node);

    std::shared_ptr<Assembly> initial_assembly_;

    std::shared_ptr<Assembly> target_assembly_;

    std::vector<std::shared_ptr<AssemblyNode>> assembly_path_;

    std::shared_ptr<Part> base_part_;

    std::string output_path_;
    std::string input_path_;

    std::vector<std::string> slicer_gcode_;

    std::vector<std::vector<bool>> bay_occupancy_;

};

#endif  // ASSEMBLER_HPP