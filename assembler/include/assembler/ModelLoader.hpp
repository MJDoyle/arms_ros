#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include <memory>

#include <boost/property_map/property_map.hpp>

#include "assembler/Part.hpp"

class MeshObject;

class Assembly;

class ModelLoader {

    public:

        explicit ModelLoader() {}
        std::shared_ptr<Assembly> loadModel(const std::string& filename);

    private:

        std::vector<std::shared_ptr<Part>> loadSTEP(const std::string& filename);

        std::string GetShapeName(const TDF_Label& label);

        void RecurrentAddPart(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level);

        int next_id_ = 0;
};

#endif  // MODEL_LOADER_HPP