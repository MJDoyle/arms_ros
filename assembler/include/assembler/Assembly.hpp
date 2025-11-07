#ifndef ASSEMBLY_HPP
#define ASSEMBLY_HPP

#include "assembler/MeshFunctions.hpp"

#include <memory>
#include <vector>
#include <set>
#include <map>

class Part;

class Assembly {

public:
    explicit Assembly() {}

    void setAssembledPart(std::shared_ptr<Part> part, gp_Pnt position) { assembled_part_transforms_[part] = position; }

    void setUnassembledPart(std::shared_ptr<Part> part, gp_Pnt position) { unassembled_part_transforms_[part] = position; }
 
    std::map<std::shared_ptr<Part>, gp_Pnt> getAssembledPartTransforms() { return assembled_part_transforms_; }

    std::map<std::shared_ptr<Part>, gp_Pnt> getUnassembledPartTransforms() { return unassembled_part_transforms_; }

    void setPartTransforms();

    //void setParts(std::map<std::shared_ptr<Part>, gp_Pnt> parts) { parts_ = parts; }

    std::shared_ptr<Part> getPartById(size_t id);   //TODO replace with a dictionary of ID to part in assembler?

    std::vector<size_t> getAssembledPartIds();

    int getNumInternalParts();

    // void placeOnPoint(gp_Pnt point);

    // void alignToPart(std::shared_ptr<Part> part);

    // void saveAsSTL(std::string filename);


private:

    std::map<std::shared_ptr<Part>, gp_Pnt> unassembled_part_transforms_;

    std::map<std::shared_ptr<Part>, gp_Pnt> assembled_part_transforms_;
};

struct AssemblyNode {

    std::shared_ptr<Assembly> assembly_;

    size_t id_;

    std::set<size_t> parent_ids_;

    bool operator <(const AssemblyNode& rhs) const              //TODO this is janky
    {
        return id_ < rhs.id_;
    }

    bool operator ==(const AssemblyNode& rhs) const
    {
        return id_ == rhs.id_;
    }
};

#endif  // ASSEMBLY_HPP