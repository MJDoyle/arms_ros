#ifndef ASSEMBLY_HPP
#define ASSEMBLY_HPP

#include "assembler/MeshFunctions.hpp"

#include <memory>
#include <vector>
#include <set>

class Part;

class Assembly {

public:
    explicit Assembly();

    void addPart(std::shared_ptr<Part> part) { parts_.push_back(part); }
 
    std::vector<std::shared_ptr<Part>> getParts() { return parts_; }

    std::shared_ptr<Part> getPartById(size_t id);

    std::vector<size_t> getPartIds();

    int getNumInternalParts();

    void placeOnPoint(gp_Pnt point);

    void alignToPart(std::shared_ptr<Part> part);

    void saveAsSTL(std::string filename);


private:
    std::vector<std::shared_ptr<Part>> parts_;

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