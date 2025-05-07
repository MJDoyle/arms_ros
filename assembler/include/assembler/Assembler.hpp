#ifndef ASSEMBLER_HPP
#define ASSEMBLER_HPP


#include <string>
#include <memory>
#include <vector>
#include <map>


class aiScene;

class Assembly;

class AssemblyNode;

class MeshObject;

class Assembler {

public:
    explicit Assembler();

    void generateAssemblySequence();

    void setTargetAssembly(std::shared_ptr<Assembly> target_assembly) { target_assembly_ = target_assembly; }

    void setSubstrate(std::shared_ptr<MeshObject> substrate) { negative_substrate_ = substrate; }

private:

    void generateInitialAssembly();

    size_t nodeIdGenerator(std::vector<size_t> object_ids);

    std::map<size_t, std::vector<size_t>> node_id_map_;

    size_t next_node_ID_ = 0;

    void generateNegatives();

    void generateSlicerGcode();

    bool arrangeInternalParts();

    std::vector<std::shared_ptr<AssemblyNode>> breadthFirstZAssembly();

    std::vector<std::shared_ptr<AssemblyNode>> findNodeNeighbours(std::shared_ptr<AssemblyNode> node);

    std::shared_ptr<Assembly> initial_assembly_;

    std::shared_ptr<Assembly> target_assembly_;

    std::shared_ptr<MeshObject> negative_substrate_;

    std::string output_path_;
    std::string input_path_;

    std::vector<std::string> slicer_gcode_;

};

#endif  // ASSEMBLER_HPP