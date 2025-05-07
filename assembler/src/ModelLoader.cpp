#include "assembler/ModelLoader.hpp"
#include "assembler/Assembly.hpp"


std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filename) {

    //Create new assembly
    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());

    std::vector<std::shared_ptr<Part>> parts = loadSTEP(filename);

    for (std::shared_ptr<Part> part : parts)
        assembly->addPart(part);

    assembly->saveAsSTL("target_assembly.stl");

    return assembly;
}

std::string ModelLoader::GetShapeName(const TDF_Label& label) {
    Handle(TDataStd_Name) nameAttr;
    if (!label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
        return "Unnamed Shape";
    }

    // Convert ExtendedString to ASCII
    TCollection_AsciiString asciiName(nameAttr->Get());

    return asciiName.ToCString();
}

std::vector<std::shared_ptr<Part>> ModelLoader::loadSTEP(const std::string& filename)
{
    std::vector<std::shared_ptr<Part>> parts;

    // Load STEP file using STEPCAFControl_Reader (handles metadata properly)
    Handle(TDocStd_Document) doc = new TDocStd_Document("STEP");
    STEPCAFControl_Reader stepReader;

    if (stepReader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
        std::cerr << "Error: Failed to read STEP file." << std::endl;
        return parts;
    }

    // Transfer to document structure
    stepReader.Transfer(doc);
    
    // Get the shape tool (manages multiple parts)
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    if (shapeTool.IsNull()) {
        std::cerr << "Error: Unable to retrieve shape tool from document!" << std::endl;
        return parts;
    }

    TDF_LabelSequence freeShapes;

    shapeTool->GetFreeShapes(freeShapes);  // top-level shapes (assemblies or parts)

    if (freeShapes.Length() != 1)
        std::cerr << "Incorrect number of free shapes" << std::endl;

    TDF_Label assemblyLabel = freeShapes.Value(1);

    RecurrentAddPart(assemblyLabel, shapeTool, parts, 0);

    return parts;
}

void ModelLoader::RecurrentAddPart(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level)
{
    std::string name = GetShapeName(label);

    Part::PART_TYPE type;

    if (name.find("internal") != std::string::npos)
        type = Part::INTERNAL;

    else if (name.find("external") != std::string::npos)
        type = Part::EXTERNAL;

    else if (name.find("screw") != std::string::npos)
        type = Part::SCREW;

    else
        type = Part::NONE;

    if (type != Part::NONE)
        parts.push_back(std::shared_ptr<Part>(new Part(std::make_shared<TopoDS_Shape>(shapeTool->GetShape(label)), type, next_id_++, name)));

    std::cout << level << " " << name << std::endl;

    TDF_LabelSequence children;

    shapeTool->GetComponents(label, children);

    for (Standard_Integer i = 1; i <= children.Length(); ++i)
    {
        RecurrentAddPart(children.Value(i), shapeTool, parts, level + 1);
    }
}