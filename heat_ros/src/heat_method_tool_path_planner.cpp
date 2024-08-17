#include <heat_ros/heat_method_tool_path_planner.h>
extern "C"
{
#include <libgeodesic/hmTriDistance.h>
#include <libgeodesic/hmContext.h>
#include <libgeodesic/hmUtility.h>
#include <libgeodesic/hmTriMesh.h>
}
#include <libgeodesic/hmHeatPath.hpp>

namespace heat_ros
{
std::vector<pcl::PCLPointField>::const_iterator findField(const std::string& name,
                                                          const std::vector<pcl::PCLPointField>& fields)
{
  auto it = std::find_if(fields.begin(), fields.end(), [&name](const pcl::PCLPointField& f){ return f.name == name; });
  if (it == fields.end())
    throw std::runtime_error("Failed to find field '" + name + "'");

  if (it->datatype != pcl::PCLPointField::FLOAT32)
    throw std::runtime_error("Field '" + name + "' is not float32 type");

  if (it->count != 1)
    throw std::runtime_error("Field '" + name + "' does not have a count of 1");

  return it;
}

hmTriMesh fromPCL(const pcl::PolygonMesh& mesh)
{
  hmTriMesh surface_;
  hmTriMeshInitialize(&surface_);

  // allocate
  surface_.nVertices = mesh.cloud.height * mesh.cloud.width;
  surface_.vertices = (double*)malloc(surface_.nVertices * 3 * sizeof(double));
  surface_.texCoords = (double*)malloc(surface_.nVertices * 2 * sizeof(double));

  surface_.nFaces = mesh.polygons.size();
  surface_.faces = (size_t*)malloc(surface_.nFaces * 3 * sizeof(size_t));

  auto x_it = findField("x", mesh.cloud.fields);
  auto y_it = findField("y", mesh.cloud.fields);
  auto z_it = findField("z", mesh.cloud.fields);

  double* v = surface_.vertices;
  for(std::size_t row = 0; row < mesh.cloud.height; ++row)
  {
    for(std::size_t col = 0; col < mesh.cloud.width; ++col)
    {
      auto offset = row * mesh.cloud.row_step + col * mesh.cloud.point_step;
      auto x = reinterpret_cast<const float*>(mesh.cloud.data.data() + offset + x_it->offset);
      auto y = reinterpret_cast<const float*>(mesh.cloud.data.data() + offset + y_it->offset);
      auto z = reinterpret_cast<const float*>(mesh.cloud.data.data() + offset + z_it->offset);

      v[0] = static_cast<double>(*x);
      v[1] = static_cast<double>(*y);
      v[2] = static_cast<double>(*z);
      v += 3;
    }
  }

  std::size_t* f = surface_.faces;
  for(std::size_t i = 0; i < mesh.polygons.size(); ++i)
  {
    const pcl::Vertices& poly = mesh.polygons[i];

    if(poly.vertices.size() != 3)
      throw std::runtime_error("Polygon is not a triangle");

    f[0] = static_cast<std::size_t>(poly.vertices[0]);
    f[1] = static_cast<std::size_t>(poly.vertices[1]);
    f[2] = static_cast<std::size_t>(poly.vertices[2]);
    f += 3;
  }

  hmTriMeshWriteOBJ(&surface_, "/tmp/test.obj");

  return surface_;
}

noether::ToolPaths HeatMethodToolPathPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  double smoothness_ = 0.0;
  double boundaryConditions_ = 0.0;
  bool verbose_ = false;
  double raster_spacing_ = 0.1;

  hmContext context_;
  hmContextInitialize(&context_);

  hmTriMesh surface_ = fromPCL(mesh);

  hmTriDistance distance_;
  hmTriDistanceInitialize(&distance_);
  distance_.surface = &surface_;

  /* set time for heat flow */
  hmTriDistanceEstimateTime(&distance_);

  if (smoothness_ > 0.0)
    distance_.time *= smoothness_;

  /* specify boundary conditions */
  if (boundaryConditions_ > 0.0)
    hmTriDistanceSetBoundaryConditions(&distance_, boundaryConditions_);

  /* specify verbosity */
  distance_.verbose = verbose_;

  /* compute distance */
  hmTriDistanceBuild(&distance_);

  int nv = surface_.nVertices;
  hmClearArrayDouble(distance_.isSource.values, nv, 0.0);

  // TODO: Set heat sources
  std::vector<int> source_indices = {0};

  // calculate the distances
  hmTriDistanceUpdate(&distance_);
  hmTriHeatPaths THP(&distance_, raster_spacing_);

  THP.compute(&distance_, source_indices);

  // copy THP.pose_arrays into paths
  noether::ToolPaths tool_paths;
  tool_paths.reserve(THP.pose_arrays_.size());

  for (std::size_t i = 0; i < THP.pose_arrays_.size(); ++i)
  {
    noether::ToolPathSegment segment;
    segment.reserve(THP.pose_arrays_[i].size());

    for (std::size_t j = 0; j < THP.pose_arrays_[i].size(); ++j)
    {
      Eigen::Isometry3d pose =
          Eigen::Translation3d(THP.pose_arrays_[i][j].x, THP.pose_arrays_[i][j].y, THP.pose_arrays_[i][j].z) *
          Eigen::Quaterniond(THP.pose_arrays_[i][j].qw, THP.pose_arrays_[i][j].qx, THP.pose_arrays_[i][j].qy, THP.pose_arrays_[i][j].qz);

      segment.push_back(pose);
    }

    noether::ToolPath tool_path;
    tool_path.push_back(segment);

    tool_paths.push_back(tool_path);
  }

  /* deallocate data structures*/
  // TODO there are known memory leaks fix them
  hmTriMeshDestroy(&surface_);
  hmTriDistanceDestroy(&distance_);
  hmContextDestroy(&context_);

  return tool_paths;
}

} // namespace heat_ros
