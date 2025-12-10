#include "assembler/CradleGenerator.hpp"

#include "assembler/Config.hpp"

#include "assembler/Logger.hpp"

#include "assembler/quickhull/QuickHull.hpp"

struct HullMesh
{
  std::vector<gp_Pnt>    vertices;
  std::vector<uint32_t>  indices;  // 3 per triangle, 0-based
};


// Collect 3D points from a TopoDS_Shape
static void CollectPointsFromShape(const TopoDS_Shape& shape,
                                   std::vector<gp_Pnt>& outPoints,
                                   double deflection = 0.1)
{
  // Triangulate (if not already)
  BRepMesh_IncrementalMesh mesher(shape, deflection);

  TopExp_Explorer ex(shape, TopAbs_FACE);
  for (; ex.More(); ex.Next())
  {
    const TopoDS_Face& F = TopoDS::Face(ex.Current());
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(F, loc);
    if (tri.IsNull())
      continue;

    const gp_Trsf& T = loc.Transformation();

    const int nbNodes = tri->NbNodes();

    for (int i = 1; i <= nbNodes; ++i)
    {
        // Works on all versions because:
        // - In ≥7.5: Node(i) is an inline wrapper around Nodes()(i)
        // - In ≤7.4: Node(i) is the only API
        gp_Pnt p = tri->Node(i);

        if (!loc.IsIdentity())
            p.Transform(T);

        outPoints.push_back(p);
    }
  }
}

// Build convex hull triangle mesh from OCCT shape
static HullMesh BuildConvexHullMesh(const TopoDS_Shape& shape,
                                    double deflection = 0.1)
{
  std::vector<gp_Pnt> pts;
  CollectPointsFromShape(shape, pts, deflection);

  HullMesh out;
  if (pts.size() < 4)
    return out; // too few points for a 3D hull

  // Fill QuickHull point cloud
  std::vector<quickhull::Vector3<double>> cloud;
  cloud.reserve(pts.size());
  for (const gp_Pnt& p : pts)
    cloud.emplace_back(p.X(), p.Y(), p.Z());

  quickhull::QuickHull<double> qh;

  // CCW = true; useOriginalIndices = false (we get a compact vertex buffer)
  auto hull = qh.getConvexHull(cloud, /*CCW*/ true, /*useOriginalIndices*/ false);

  auto& vbuf = hull.getVertexBuffer(); // behaves like array of Vector3<double>
  auto& ibuf = hull.getIndexBuffer();  // std::vector<IndexType>

  out.vertices.reserve(vbuf.size());
  for (size_t i = 0; i < vbuf.size(); ++i)
  {
    const auto& v = vbuf[i];
    out.vertices.emplace_back(v.x, v.y, v.z);
  }

  out.indices.assign(ibuf.begin(), ibuf.end());
  return out;
}

// Build an OCCT shape representing the convex hull surface
TopoDS_Shape MakeConvexHullShape(const TopoDS_Shape& input,
                                 double deflection = 0.1)
{
  HullMesh mesh = BuildConvexHullMesh(input, deflection);
  if (mesh.vertices.empty() || mesh.indices.size() < 3)
    return TopoDS_Shape(); // null

  const Standard_Integer nbNodes = static_cast<Standard_Integer>(mesh.vertices.size());
  const Standard_Integer nbTris  = static_cast<Standard_Integer>(mesh.indices.size() / 3);

  // Fill OCCT arrays (1-based indexing!)
  TColgp_Array1OfPnt nodes(1, nbNodes);
  for (Standard_Integer i = 1; i <= nbNodes; ++i)
    nodes.SetValue(i, mesh.vertices[static_cast<size_t>(i - 1)]);

  Poly_Array1OfTriangle triangles(1, nbTris);
  for (Standard_Integer t = 1; t <= nbTris; ++t)
  {
    const uint32_t i0 = mesh.indices[3 * (t - 1) + 0];
    const uint32_t i1 = mesh.indices[3 * (t - 1) + 1];
    const uint32_t i2 = mesh.indices[3 * (t - 1) + 2];

    // QuickHull indices are 0-based → Poly_Triangle expects 1-based indices
    triangles.SetValue(t, Poly_Triangle(
        static_cast<Standard_Integer>(i0 + 1),
        static_cast<Standard_Integer>(i1 + 1),
        static_cast<Standard_Integer>(i2 + 1)));
  }

  Handle(Poly_Triangulation) tri =
      new Poly_Triangulation(nodes, Poly_Array1OfTriangle(triangles));

  // Wrap into a face and a shell
  BRep_Builder BB;
  TopoDS_Face hullFace;
  BB.MakeFace(hullFace, tri);      // face defined only by triangulation :contentReference[oaicite:4]{index=4}

  TopoDS_Shell shell;
  BB.MakeShell(shell);
  BB.Add(shell, hullFace);

  // Optionally: make a solid; many apps are fine just displaying the shell
  TopoDS_Solid solid;
  BB.MakeSolid(solid);
  BB.Add(solid, shell);

  return solid; // or `shell` if you only want the surface
}

void CradleGenerator::createSimpleNegative()
{

    double clearance = 2.0;



    gp_Pnt shape_position = ShapeCentroid(shape_);

    TopoDS_Shape jigBlock = BRepPrimAPI_MakeBox(shape_position, 60, 60, 40);

    jigBlock = ShapeSetCentroid(jigBlock, gp_Pnt(shape_position.X(), shape_position.Y(), shape_position.Z()));

    //Create an offset version of the shape for some clearance
    BRepOffsetAPI_MakeOffset offsetMaker(shape_, clearance, 1.0e-3, BRepOffset_Skin, Standard_True);

    offsetMaker.Build();

    TopoDS_Shape inflated_shape =offsetMaker.Shape();

    //Make the inflated shape convex
    TopoDS_Shape convex_shape = MakeConvexHullShape(inflated_shape);

    TopoDS_Shape final_jig = jigBlock;

    //Step the jig through the shape until no downwards faces are present

    for (float z = ShapeLowestPoint(shape_) - 21; z < ShapeLowestPoint(shape_); z += 1)
    {
          RCLCPP_INFO(logger(), "Iterating");

          jigBlock = ShapeSetCentroid(jigBlock, gp_Pnt(shape_position.X(), shape_position.Y(), z));

          BRepAlgoAPI_Cut cutter(jigBlock, shape_);
          TopoDS_Shape jig = cutter.Shape();

          int num_downwards_faces = 0;

         for (TopExp_Explorer exp(jig, TopAbs_FACE); exp.More(); exp.Next())
         {
             TopoDS_Face face = TopoDS::Face(exp.Current());

             gp_Dir normal = outwardFaceNormal(face);

             //Ignore faces that are not planar
             if (!BRep_Tool::Surface(face)->IsKind(STANDARD_TYPE(Geom_Plane)))   //TODO how do we handle curved faces?
             {
                 //RCLCPP_INFO(logger(), "Non planar face");

                 continue;
             }

             //Ignore faces that don't point downwards
             if (normal.Angle(UPWARDS) <= M_PI * 0.5001)
                 continue;

             RCLCPP_INFO(logger(), "Normal angle: %f", normal.Angle(UPWARDS));

             num_downwards_faces ++;

             //RCLCPP_INFO(logger(), "Downwards face");
         }

         RCLCPP_INFO(logger(), "Downwards faces: %d", num_downwards_faces);

         if (num_downwards_faces > 1)
             break;

         final_jig = jig;


    }

    BRepMesh_IncrementalMesh mesher(final_jig, 0.1);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << WORKING_DIR << name_ << "_jig.stl";

    writer.Write(final_jig, ss.str().c_str());

}

void CradleGenerator::createNegative()
{

    //Create box and position it on part

    TopoDS_Shape box = BRepPrimAPI_MakeBox(40, 40, 5).Shape();

    box = ShapeSetCentroid(box, ShapeCentroid(shape_));


    BRepMesh_IncrementalMesh mesher(shape_, 0.1, Standard_False, 0.1, Standard_True);

    BRepMesh_IncrementalMesh mesher2(box, 0.1, Standard_False, 0.1, Standard_True);



    BOPAlgo_BOP bop;
    bop.AddArgument(box);
    bop.AddArgument(shape_);
    bop.SetOperation(BOPAlgo_Operation::BOPAlgo_CUT);

    bop.Perform();

    box = bop.Shape();

    //box = BRepAlgoAPI_Cut(box, shape_).Shape();





    // TopoDS_Compound compound;
    // BRep_Builder builder;
    // builder.MakeCompound(compound);














    // BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << WORKING_DIR << name_ << "_new_cradle.stl";

    writer.Write(box, ss.str().c_str());

}









TopTools_ListOfShape CradleGenerator::bottomEdgesFlatHorizontalFaces()
{
    TopTools_ListOfShape result;

    Standard_Real lowestPoint = ShapeLowestPoint(shape_);

    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, edgeToFaceMap);

    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(shape_, TopAbs_EDGE, edgeMap);   // fills map with unique edges

    for (int i = 1; i <= edgeMap.Extent(); ++i) 
    {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeMap(i));

        if (getEdgeType(edge) != GeomAbs_Line)
            continue;

        gp_Pnt P0 = getEdgeStart(edge);
        gp_Pnt P1 = getEdgeEnd(edge);

        if (P0.Z() > lowestPoint + 1 || P1.Z() > lowestPoint + 1)
            continue;

        if (std::abs(P0.Z() - P1.Z()) > 0.01)
            continue;

        bool downFaceFound = false;

        if (edgeToFaceMap.Contains(edge)) 
        {
            const TopTools_ListOfShape& faces = edgeToFaceMap.FindFromKey(edge);

            // Iterate through associated faces
            for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) 
            {
                const TopoDS_Face& face = TopoDS::Face(it.Value());
                
                Standard_Real first, last;
                Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(edge, face, first, last);
                if (curve2d.IsNull()) {
                    RCLCPP_FATAL(logger(), "No 2D curve on surface found");
                    rclcpp::shutdown();
                    return result;
                }

                // Midpoint in parametric space of edge
                Standard_Real mid = (first + last) / 2.0;
                gp_Pnt2d uv = curve2d->Value(mid);

                // Evaluate surface at UV point
                BRepAdaptor_Surface surface(face);
                gp_Pnt surfPnt;
                gp_Vec dU, dV;
                surface.D1(uv.X(), uv.Y(), surfPnt, dU, dV);

                // Compute and normalize normal
                gp_Vec normal = dU.Crossed(dV);
                normal.Normalize();

                if (face.Orientation() == TopAbs_REVERSED) {
                    normal.Reverse();  // Flip inward-pointing normal to outward
                }

                //Check that the face is flat
                GeomAbs_SurfaceType surfType = BRepAdaptor_Surface(face).GetType();

                if (surfType != GeomAbs_Plane)
                    continue;

                //Check that the normal is aligned with the z direction
                if (normal.Angle(gp_Vec(0, 0, -1)) > 0.01 * M_PI)
                    continue;

                downFaceFound = true;

                break;
            }
        }

        if (downFaceFound)
            result.Append(edge);
    }

    return result;
}

TopoDS_Shape CradleGenerator::createBoxWithPose(double width, double height, double length, gp_Pnt target_center, gp_Dir target_direction)
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(length, width, height).Shape();

    //Rotate box
    gp_Dir  source_dir (1,0,0);             // X of the box
    gp_Vec  rot_vector = gp_Vec(source_dir) ^ gp_Vec(target_direction); // cross product

    if (rot_vector.Magnitude() > gp::Resolution()) 
    {        
        gp_Trsf rotation;

        Standard_Real rot_angle = source_dir.Angle(target_direction);      // radians

        gp_Ax1 rot_axis(gp_Pnt(length / 2, width / 2, height / 2), gp_Dir(rot_vector));

        rotation.SetRotation(rot_axis, rot_angle);

        BRepBuilderAPI_Transform rot_transformer(rotation);
        rot_transformer.Perform(box);
        box = rot_transformer.Shape();
    }

    //Get center of box
    gp_Pnt box_centroid = ShapeCentroid(box).Translated(gp_Vec(0, 0, height / 2));   //Add half of box height so that we get top face

    gp_Vec translation_vector(box_centroid, target_center);

    gp_Trsf translation;

    translation.SetTranslation(translation_vector);

    BRepBuilderAPI_Transform trans_transformer(translation);
    trans_transformer.Perform(box);
    box = trans_transformer.Shape();

    return box;
}

gp_Vec CradleGenerator::outwardsNormalOfEdge(TopoDS_Edge edge, TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap)
{
    gp_Vec outwards_normal(0, 0, 0);

    if (edgeToFaceMap.Contains(edge)) 
    {
        const TopTools_ListOfShape& faces = edgeToFaceMap.FindFromKey(edge);

        // Iterate through associated faces
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) 
        {
            const TopoDS_Face& face = TopoDS::Face(it.Value());
            
            Standard_Real first, last;
            Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(edge, face, first, last);
            if (curve2d.IsNull()) {
                RCLCPP_FATAL(logger(), "No 2D curve on surface found");
                rclcpp::shutdown();
                return outwards_normal;
            }

            // Midpoint in parametric space of edge
            Standard_Real mid = (first + last) / 2.0;
            gp_Pnt2d uv = curve2d->Value(mid);

            // Evaluate surface at UV point
            BRepAdaptor_Surface surface(face);
            gp_Pnt surfPnt;
            gp_Vec dU, dV;
            surface.D1(uv.X(), uv.Y(), surfPnt, dU, dV);

            // Compute and normalize normal
            gp_Vec normal = dU.Crossed(dV);
            normal.Normalize();

            if (face.Orientation() == TopAbs_REVERSED) {
                normal.Reverse();  // Flip inward-pointing normal to outward
            }

            //Select the normal that doesn't point straight donwards
            if (abs(normal.Z()) < 0.95)
            {
                outwards_normal = normal;
                outwards_normal.SetZ(0);
            }
        }
    }

    return outwards_normal;
}

// TopoDS_Shape Part::chamferCorrectEdge(TopoDS_Shape box, gp_Vec outwards_normal)
// {
//     TopoDS_Shape chamferCorrectEdge(TopoDS_Shape box, gp_Vec outward_normal);

// }

void CradleGenerator::createNegative2()
{

    // Display connection
    Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();

    // Graphic driver (OpenGL)
    Handle(Graphic3d_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);

    // Viewer
    Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
    viewer->SetDefaultLights();
    viewer->SetLightOn();

    // View
    Handle(V3d_View) view = viewer->CreateView();

    // Create an interactive context
    Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);

    // Wrap shape in an AIS_Shape
    // Handle(AIS_Shape) aisShape = new AIS_Shape(shape_);
    // context->Display(aisShape, Standard_True);

    // context->SetDisplayMode(aisShape, AIS_Shaded, Standard_True);




    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    // TopoDS_Shape base = BRepPrimAPI_MakeBox(50, 50, 2).Shape();

    // builder.Add(compound, base);

    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaceMap;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, edgeToFaceMap);

    TopTools_ListOfShape bottomEdges = bottomEdgesFlatHorizontalFaces();

    for (TopTools_ListIteratorOfListOfShape it(bottomEdges); it.More(); it.Next()) {
        const TopoDS_Shape& shape = it.Value();
        if (shape.ShapeType() != TopAbs_EDGE)
            continue;

        TopoDS_Edge edge = TopoDS::Edge(shape);

        gp_Pnt P0 = getEdgeStart(edge);
        gp_Pnt P1 = getEdgeEnd(edge);

        gp_Vec  edgeVec(P0, P1);
        Standard_Real length = edgeVec.Magnitude();

        //TODO are you sure
        if (length < 3)
            continue;

        gp_Dir  edgeDir(edgeVec);   
        gp_Pnt edge_centroid = P0.Translated(0.5 * edgeVec);

        double width  = 2.0;   // Y
        double height = 5.0;   // Z
        TopoDS_Shape box = BRepPrimAPI_MakeBox(length, width, height).Shape();




        box = createBoxWithPose(width, height, length, P0.Translated(0.5 * edgeVec), gp_Dir(edgeVec));


        Handle(AIS_Shape) aisBox = new AIS_Shape(box);

        aisBox->SetColor(Quantity_NOC_BLUE);

        context->Display(aisBox, Standard_True);





        gp_Vec outwards_normal = outwardsNormalOfEdge(edge, edgeToFaceMap);

        if (outwards_normal.Magnitude() < Precision::Confusion()) 
            continue;
        

        // if (outwards_normal.IsEqual(gp_Vec(0, 0, 0), Precision::Confusion()))
        //     continue;

        // TopoDS_Edge visual_normal = BRepBuilderAPI_MakeEdge(edge_centroid, edge_centroid.Translated(outwards_normal));

        // // `context` is your Handle(AIS_InteractiveContext) created earlier
        // Handle(AIS_Shape) aisEdge = new AIS_Shape(visual_normal);

        // // optional styling
        // aisEdge->SetColor(Quantity_NOC_RED);
        // aisEdge->SetWidth(2.0);  // line width in pixels

        // context->Display(aisEdge, /*updateViewer=*/ Standard_True);




        //box = chamferCorrectEdge(box, outwards_normal);




        // Build map from edges to adjacent faces
        TopTools_IndexedDataMapOfShapeListOfShape boxEdgeFaceMap;
        TopExp::MapShapesAndAncestors(box, TopAbs_EDGE, TopAbs_FACE, boxEdgeFaceMap);

        TopTools_IndexedMapOfShape boxEdgeMap;
        TopExp::MapShapes(box, TopAbs_EDGE, boxEdgeMap);   // fills map with unique edges

        for (int i = 1; i <= boxEdgeMap.Extent(); ++i) 
        {
            std::cout << std::endl;

            const TopoDS_Edge& boxEdge = TopoDS::Edge(boxEdgeMap(i));

            const TopTools_ListOfShape& boxFaces = boxEdgeFaceMap.FindFromKey(boxEdge);

            bool chamferedEdge = false;

            std::vector<gp_Vec> face_normals;

            //Get the two normals for the two faces attached to this edge

            for (TopTools_ListIteratorOfListOfShape it(boxFaces); it.More(); it.Next()) 
            {
                const TopoDS_Face& boxFace = TopoDS::Face(it.Value());
            
                Standard_Real first, last;
                Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(boxEdge, boxFace, first, last);
                if (curve2d.IsNull()) {
                    RCLCPP_FATAL(logger(), "No 2D curve on surface found");
                    rclcpp::shutdown();
                    return;
                }

                // Midpoint in parametric space of edge
                Standard_Real mid = (first + last) / 2.0;
                gp_Pnt2d uv = curve2d->Value(mid);

                // Evaluate surface at UV point
                BRepAdaptor_Surface surface(boxFace);
                gp_Pnt surfPnt;
                gp_Vec dU, dV;
                surface.D1(uv.X(), uv.Y(), surfPnt, dU, dV);

                // Compute and normalize normal
                gp_Vec normal = dU.Crossed(dV);
                normal.Normalize();

                if (boxFace.Orientation() == TopAbs_REVERSED) {
                    normal.Reverse();  // Flip inward-pointing normal to outward
                }

                face_normals.push_back(normal);
            }

            

            if (face_normals.size() != 2)
                continue;

            face_normals[0].Angle(gp_Vec(0, 0, 1));

            face_normals[1].Angle(outwards_normal);

            face_normals[1].Angle(gp_Vec(0, 0, 1));

            face_normals[0].Angle(outwards_normal);

            //Inspect the two normals, if one if pointing upwards and one is antiparalel to the shape outward_normal then
            //we have the correct face and edge. Otherwise continue

            int correct_face_index = -1;

            if (face_normals[0].Angle(gp_Vec(0, 0, 1)) < 0.1 * M_PI && face_normals[1].Angle(outwards_normal) > 0.9 * M_PI)
            {
                correct_face_index = 1;
            }

            else if (face_normals[1].Angle(gp_Vec(0, 0, 1)) < 0.1 * M_PI && face_normals[0].Angle(outwards_normal) > 0.9 * M_PI)
            {
                correct_face_index = 0;
            }

            else
            {
                continue;
            }

            //Use the normal index to select the correct face

            int normal_index = 0;

            for (TopTools_ListIteratorOfListOfShape it(boxFaces); it.More(); it.Next()) 
            {
                if (normal_index != correct_face_index)
                {
                    normal_index ++;
                    continue;
                }

                const TopoDS_Face& boxFace = TopoDS::Face(it.Value());

                // Create chamfer
                BRepFilletAPI_MakeChamfer chamferMaker(box);
                chamferMaker.Add(1.0, 1.0, boxEdge, boxFace);  // 5.0 is the chamfer distance
                chamferMaker.Build();

                if (chamferMaker.IsDone()) {
                    box = chamferMaker.Shape();
                    // Display or use chamferedShape...
                }

                break;
            }
        }

        builder.Add(compound, box);
    }
   

    BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << WORKING_DIR << name_ << "_new_cradle.stl";

    writer.Write(compound, ss.str().c_str());


    context->UpdateCurrentViewer();





    // Create a window (X11 version here; use WNT_Window for Windows)
    Handle(Xw_Window) window = new Xw_Window(displayConnection, "OpenCascade Viewer", 0, 0, 800, 600);
    view->SetWindow(window);
    view->SetBackgroundColor(Quantity_NOC_GRAY50);
    view->MustBeResized();
    view->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.1, V3d_ZBUFFER);
    view->SetProj(0, 0, -1);  // Top view
    view->FitAll();

    window->Map();

    // Start the event loop — OpenCascade has no built-in loop, so use your GUI framework (Qt, wxWidgets) or system events
    while (true) {


        char key;
        std::cin >> key;

        if (key == 't') {
            view->SetProj(0, 0, -1); // Top view
        }

        else if (key == 'b') {
            view->SetProj(0, 0, 1); // Bottom view
        }

        else if (key == 'f') {
            view->SetProj(0, -1, 0); // Front view
        }

        else if (key == 's') {
            view->SetProj(1, 0, 0); // Front view
        }

        else if (key == 'i') {
            view->SetProj(1, 1, 1); // Isometric
        }

        else if (key == 'c') {
            break;
        }

        view->FitAll();
        view->Redraw();
    }


}
