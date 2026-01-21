#include "assembler/ModelLoader.hpp"

#include "assembler/Part.hpp"

#include "assembler/Assembly.hpp"

#include "assembler/Logger.hpp"


int ModelLoader::next_id_ = 0;

/*  Input: string filepath of .step file

    Output: target assembly to be assembled

    Generates a new assembly and then populates the parts via inspecting the .step file
*/
std::shared_ptr<Assembly> ModelLoader::loadModel(const std::string& filename) {

    RCLCPP_INFO(logger(), "Loading model");

    std::shared_ptr<Assembly> assembly = std::shared_ptr<Assembly>(new Assembly());
    std::vector<std::shared_ptr<Part>> parts = loadParts(filename);

    next_id_ = 0;

    for (std::shared_ptr<Part> part : parts)
    {
        assembly->setAssembledPart(part, ShapeCentroid(*(part->getShape())));
    }

    RCLCPP_INFO(logger(), "Model loaded");

    return assembly;
}

/*  Input: string filepath of .step file

    Output: list of parts to be added to the target assembly

    Creates an OpenCascade doc reader and shape tool, and uses them to read the individual parts
    OCC parts are referred to as shapes, and labelled
*/
std::vector<std::shared_ptr<Part>> ModelLoader::loadParts(const std::string& filename)
{
    std::vector<std::shared_ptr<Part>> parts;                       //List of parts to be returned
    Handle(TDocStd_Document) doc = new TDocStd_Document("STEP");    //OCC document to analyse .step file
    STEPCAFControl_Reader stepReader;                               //OCC .step reader
    Handle(XCAFDoc_ShapeTool) shapeTool;                            //OCC shape tool for analysing individual shapes
    TDF_LabelSequence shapes;                                   //nested OCC sequence of parts 
    TDF_Label topLevelShape;                                        //OCC label for the highest level part

    if (stepReader.ReadFile(filename.c_str()) != IFSelect_RetDone) {
        RCLCPP_FATAL(logger(), "Failed to read STEP file");
        rclcpp::shutdown();
        return parts;
    }

    // Transfer to document structure
    stepReader.Transfer(doc);
    
    // Get the shape tool (manages multiple parts)
    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    if (shapeTool.IsNull()) {
        RCLCPP_FATAL(logger(), "Unable to retrieve shape tool from document");
        rclcpp::shutdown();
        return parts;
    }

    shapeTool->GetFreeShapes(shapes);

    //Top level should just be a single shape
    if (shapes.Length() != 1)
    {
        RCLCPP_FATAL(logger(), "Incorrect number of free shapes");
        rclcpp::shutdown();
        return parts;
    }

    //Indexing starts from 1
    topLevelShape = shapes.Value(1);

    RecurrentAddPart(topLevelShape, shapeTool, parts, 0);

    return parts;
}


/*  Input: current shape to analyse, shape tool, list of parts, and current level of recursion

    Classifies the current shape based on its name and adds it to the parts list, then finds all children of the shape
    and calls itself on each of them, incrementing the recursion level
*/
void ModelLoader::RecurrentAddPart(const TDF_Label& currentShape, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level)
{
    std::string shapeName = GetShapeName(currentShape);
    Part::PART_TYPE shapeType;                                   //The type of part, as extracted from the shape name
    TDF_LabelSequence childShapes;
    TopoDS_Shape shape = shapeTool->GetShape(currentShape);

    RCLCPP_DEBUG(logger(), "Adding part %s on level %d", shapeName.c_str(), level);

    if (shapeName.find("internal") != std::string::npos)
        shapeType = Part::INTERNAL;

    else if (shapeName.find("external") != std::string::npos)
        shapeType = Part::EXTERNAL;

    else if (shapeName.find("screw") != std::string::npos || shapeName.find("bolt") != std::string::npos)
        shapeType = Part::SCREW;

    else
        shapeType = Part::NONE;


    BRepCheck_Analyzer ana(shape);

    if (!ana.IsValid()) {

        RCLCPP_WARN(logger(), "Shape %s is invalid, attempting fix", shapeName.c_str());

        Handle(ShapeFix_Shape) aFixShape = new ShapeFix_Shape();
        aFixShape->Init(shape);

        aFixShape->Perform();

        shape = aFixShape->Shape();

        BRepCheck_Analyzer ana2(shape);

        if (!ana.IsValid())
        {
            RCLCPP_FATAL(logger(), "WARNING: Shape is still invalid after fix attempt");
        }
    }

    if (shapeType != Part::NONE)
        parts.push_back(std::shared_ptr<Part>(new Part(std::make_shared<TopoDS_Shape>(shape), shapeType, next_id_++, shapeName)));

    shapeTool->GetComponents(currentShape, childShapes);

    for (Standard_Integer i = 1; i <= childShapes.Length(); ++i)
    {
        RecurrentAddPart(childShapes.Value(i), shapeTool, parts, level + 1);
    }
}

/*  Input: shape label

    Output: the name of the shape from the .step file
*/
std::string ModelLoader::GetShapeName(const TDF_Label& shape) 
{
    Handle(TDataStd_Name) nameAttr;
    if (!shape.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
        return "Unnamed Shape";
    }

    TCollection_AsciiString asciiName(nameAttr->Get());

    return asciiName.ToCString();
}