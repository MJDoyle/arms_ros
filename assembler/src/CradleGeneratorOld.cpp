#include "assembler/CradleGenerator.hpp"

#include "assembler/Config.hpp"

#include "assembler/Logger.hpp"

#include "assembler/quickhull/QuickHull.hpp"

#include "assembler/ARMSConfig.hpp"

TopoDS_Shape makeConvexHullSolid(const std::vector<gp_Pnt>& verts,
    const std::vector<uint32_t>& indices)
{
    if (verts.empty() || indices.empty() || indices.size() % 3 != 0) {
    std::cerr << "ERROR: Invalid hull mesh" << std::endl;
    return TopoDS_Shape();
    }

    // Sewing to assemble faces into a watertight shell
    BRepBuilderAPI_Sewing sewing(1.0e-6);

    for (size_t t = 0; t < indices.size(); t += 3)
    {
    gp_Pnt p0 = verts[indices[t]];
    gp_Pnt p1 = verts[indices[t+1]];
    gp_Pnt p2 = verts[indices[t+2]];

    // Create 3 edges
    TopoDS_Edge e0 = BRepBuilderAPI_MakeEdge(p0, p1);
    TopoDS_Edge e1 = BRepBuilderAPI_MakeEdge(p1, p2);
    TopoDS_Edge e2 = BRepBuilderAPI_MakeEdge(p2, p0);

    // Form a wire
    BRepBuilderAPI_MakeWire mkWire;
    mkWire.Add(e0);
    mkWire.Add(e1);
    mkWire.Add(e2);
    TopoDS_Wire wire = mkWire.Wire();

    // Build planar face
    TopoDS_Face face = BRepBuilderAPI_MakeFace(wire);

    // Add to sewing
    sewing.Add(face);
    }

    // Sew faces
    sewing.Perform();
    TopoDS_Shape stitched = sewing.SewedShape();

    // Extract stitched shell
    TopoDS_Shell shell;
    TopExp_Explorer ex(stitched, TopAbs_SHELL);
    if (ex.More()) {
    shell = TopoDS::Shell(ex.Current());
    } else {
    std::cerr << "ERROR: No shell produced by sewing!" << std::endl;
    return TopoDS_Shape();
    }

    // Build solid
    BRepBuilderAPI_MakeSolid mkSolid(shell);
    TopoDS_Shape solid = mkSolid.Shape();

    // Validate
    BRepCheck_Analyzer ana(solid);
    if (!ana.IsValid())
    std::cerr << "WARNING: Hull solid is not perfectly valid" << std::endl;

    return solid;
}





TopoDS_Shape safeCut(const TopoDS_Shape& A, const TopoDS_Shape& B)
{
    // Quick sanity check
    BRepCheck_Analyzer anaA(A);
    BRepCheck_Analyzer anaB(B);
    if (!anaA.IsValid()) {
        std::cerr << "Shape A is invalid before boolean\n";
    }
    if (!anaB.IsValid()) {
        std::cerr << "Shape B is invalid before boolean\n";
    }

    try
    {
        BRepAlgoAPI_Cut cut(A, B);
        cut.Build();

        if (!cut.IsDone()) {
            std::cerr << "Cut failed: not done\n";
            return TopoDS_Shape();
        }

        TopoDS_Shape res = cut.Shape();

        BRepCheck_Analyzer anaRes(res);
        if (!anaRes.IsValid()) {
            std::cerr << "Resulting shape is invalid\n";
        }

        return res;
    }
    catch (Standard_Failure& e)
    {
        std::cerr << "Boolean exception: " << e.GetMessageString() << "\n";
        return TopoDS_Shape();
    }
}

struct HullMesh
{
  std::vector<gp_Pnt>    vertices;
  std::vector<uint32_t>  indices;  // 3 per triangle, 0-based
};


// Collect 3D points from a TopoDS_Shape
static void CollectPointsFromShape(const TopoDS_Shape& shape,
                                   std::vector<gp_Pnt>& outPoints,
                                   double deflection = 0.01)
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
                                    double deflection = 0.01)
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


float CradleGenerator::createSimpleNegative(float bay_size)
{
    RCLCPP_INFO(logger(), "Shape: %s", name_.c_str());

    gp_Pnt shape_position = ShapeCentroid(shape_);
    TopoDS_Shape jigBlock = BRepPrimAPI_MakeBox(shape_position, bay_size, bay_size, JIG_HEIGHT);

    jigBlock = ShapeSetCentroid(jigBlock, gp_Pnt(shape_position.X(), shape_position.Y(), shape_position.Z()));

    //Cut a notch out of the jig to indicate the corner which goes toward the origin
    TopoDS_Shape notch = BRepPrimAPI_MakeCylinder(3, 4);

    notch = ShapeSetCentroid(notch, gp_Pnt(shape_position.X() - bay_size / 2, shape_position.Y() - bay_size / 2, shape_position.Z() + JIG_HEIGHT / 2));

    BRepAlgoAPI_Cut cutter(jigBlock, notch);
    jigBlock = cutter.Shape();

    //Create a scaled version of the shape for some clearance
    double largest_shape_axis = std::max(ShapeAxisSize(shape_, 0), ShapeAxisSize(shape_, 1));

    float scaling_distance = 0.6f;

    float scaling_factor = (largest_shape_axis + scaling_distance) / largest_shape_axis;

    TopoDS_Shape scaled_shape = UniformScaleShape(shape_, scaling_factor);

    RCLCPP_INFO(logger(), "Scaled shape");

    //Create a convex hull

    HullMesh hull = BuildConvexHullMesh(scaled_shape);

    RCLCPP_INFO(logger(), "Built convex hull of %ld vertices and %ld indices", hull.vertices.size(), hull.indices.size());

    TopoDS_Shape convex_shape = makeConvexHullSolid(hull.vertices, hull.indices);

    RCLCPP_INFO(logger(), "Made convex scaled shape");

    //Preemptively fix the convex hull shape

    Handle(ShapeFix_Shape) aFixShape = new ShapeFix_Shape();
    aFixShape->Init (convex_shape);

    aFixShape->Perform();

    convex_shape = aFixShape->Shape();

    RCLCPP_INFO(logger(), "Fixed convex scaled shape");




    TopoDS_Shape final_jig = jigBlock;


    SaveShapeAsSTL(convex_shape, WORKING_DIR + name_ + "_convex_scaled_shape.stl");

    SaveShapeAsSTL(shape_, WORKING_DIR + name_ + "_shape.stl");


    //Step the jig through the shape until no downwards faces are present

    int i = -1;

    float jig_part_z_offset = 0;

    for (float z = ShapeLowestPoint(shape_) - (JIG_HEIGHT * 0.5 + 1); z < ShapeLowestPoint(shape_); z += 1)
    {
        i ++;

        jigBlock = ShapeSetCentroid(jigBlock, gp_Pnt(shape_position.X(), shape_position.Y(), z));

        TopoDS_Shape compound = makeCompound({jigBlock, convex_shape});
        SaveShapeAsSTL(compound, WORKING_DIR + name_ + "_partial_compound_" + std::to_string(i) + ".stl");

        // BRepAlgoAPI_Cut cutter(jigBlock, convex_shape);
        // TopoDS_Shape jig = cutter.Shape();

        TopoDS_Shape jig = safeCut(jigBlock, convex_shape);

        RCLCPP_INFO(logger(), "Performed cut");

        SaveShapeAsSTL(jig, WORKING_DIR + name_ + "_partial_jig_" + std::to_string(i) + ".stl");

        int num_downwards_faces = 0;

        for (TopExp_Explorer exp(jig, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face face = TopoDS::Face(exp.Current());

            gp_Dir normal = outwardFaceNormal(face);

            //Ignore faces that are not planar
            if (!BRep_Tool::Surface(face)->IsKind(STANDARD_TYPE(Geom_Plane)))   //TODO how do we handle curved faces?
            {
                continue;
            }

            //Ignore faces that don't point downwards
            if (normal.Angle(UPWARDS) <= M_PI * 0.5001)
                continue;

            num_downwards_faces ++;
        }

        //RCLCPP_INFO(logger(), "Downwards faces: %d", num_downwards_faces);

        if (num_downwards_faces > 1)
            break;

        jig_part_z_offset = ShapeCentroid(shape_).Z() - ShapeCentroid(jig).Z();

        final_jig = jig;


    }

    BRepMesh_IncrementalMesh mesher(final_jig, 0.01);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << OUTPUT_DIR << name_ << "_jig.stl";

    writer.Write(final_jig, ss.str().c_str());


    return jig_part_z_offset;

}




