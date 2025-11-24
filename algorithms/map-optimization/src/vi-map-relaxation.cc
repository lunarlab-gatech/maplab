#include "map-optimization/vi-map-relaxation.h"

#include <functional>
#include <string>
#include <unordered_map>

// --- ADD THESE INCLUDES ---
#include <fstream>  // For saving the file
#include <set>      // For std::set
#include "json.hpp" // nlohmann::json header (maplab internal path)
// Include headers needed for export logic
#include <vi-map/edge.h>
#include <vi-map/vertex.h>
#include <vi-map/mission.h>
#include <vi-map/sensor-manager.h>
// --------------------------

#include <loop-closure-handler/loop-detector-node.h>
#include <map-optimization/augment-loopclosure.h>
#include <map-optimization/callbacks.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/solver.h>
#include <map-optimization/vi-optimization-builder.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>

DEFINE_bool(
    lc_relax_merge_landmarks, false,
    "If enabled, the loop closure part of the pose graph relaxation algorithm "
    "will also merge the landmarks in addition to adding temporary loop "
    "closure edges.");

// --- ADD using json ALIAS ---
// for convenience
using json = nlohmann::json;
// ----------------------------

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace map_optimization {

VIMapRelaxation::VIMapRelaxation(
    const visualization::ViwlsGraphRvizPlotter* plotter,
    bool signal_handler_enabled)
    : plotter_(plotter), signal_handler_enabled_(signal_handler_enabled) {}

void VIMapRelaxation::visualizePosegraph(const vi_map::VIMap& map) const {
  if (plotter_ == nullptr) {
    VLOG(3) << "You need to initialize with a plotter";
    return;
  }

  plotter_->visualizeMap(map);
}

// NOTE: This function ADDS the edges
void VIMapRelaxation::detectLoopclosures(
    vi_map::MissionIdSet mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  loop_detector_node::LoopDetectorNode loop_detector;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    loop_detector.addMissionToDatabase(mission_id, *map);
  }

  const bool kMergeLandmarks = FLAGS_lc_relax_merge_landmarks;
  // NOTE: This flag ensures edges are actually added by detectLoopClosuresMissionToDatabase
  constexpr bool kAddLoopclosureEdges = true;

  pose::Transformation T_G_M2;
  vi_map::LoopClosureConstraintVector inlier_constraints;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    loop_detector.detectLoopClosuresMissionToDatabase(
        mission_id, kMergeLandmarks, kAddLoopclosureEdges, map, &T_G_M2,
        &inlier_constraints);
  }
}

// Helper function to extract robot name from topic like "/mh1/cam0" -> "mh1"
std::string extractRobotNameFromTopic(const std::string& topic) {
  if (topic.empty() || topic[0] != '/') {
    return "";
  }
  
  // Find the second slash
  size_t next_slash = topic.find('/', 1);
  if (next_slash != std::string::npos && next_slash > 1) {
    // Extract between first and second slash: "/mh1/cam0" -> "mh1"
    return topic.substr(1, next_slash - 1);
  } else if (next_slash == std::string::npos && topic.length() > 1) {
    // Single-level topic "/mh1" -> "mh1"
    return topic.substr(1);
  }
  
  return "";
}

// This is the function called by the 'relax' command plugin
bool VIMapRelaxation::findLoopClosuresAndSolveRelaxation(
    const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();

  return findLoopClosuresAndSolveRelaxation(
      solver_options, mission_id_list, map);
}

// Overload function
bool VIMapRelaxation::findLoopClosuresAndSolveRelaxation(
    const ceres::Solver::Options& solver_options,
    const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  vi_map::MissionIdSet mission_ids(
      mission_id_list.begin(), mission_id_list.end());

  LOG(INFO) << "[Exporter] Removing " << numLoopclosureEdges(*map)
            << " pre-existing loop closure edges to ensure a clean export.";
  map->removeLoopClosureEdges();

  // Step 1: Find loop closures and add edges to the map
  detectLoopclosures(mission_ids, map);

  // --- Step 2: EXPORT LOOP CLOSURES ---
  // Build mission -> robot name mapping by examining vertices and their NCameras
  
  std::unordered_map<std::string, std::string> mission_to_robot;
  
  try {
    const vi_map::SensorManager& sensor_manager = map->getSensorManager();
    
    // For each mission, find its root vertex and extract robot name from NCamera topic
    for (const vi_map::MissionId& mission_id : mission_ids) {
      if (!map->hasMission(mission_id)) {
        LOG(WARNING) << "[Exporter] Map does not have mission " << mission_id.hexString();
        continue;
      }
      
      const vi_map::Mission& mission = map->getMission(mission_id);
      const pose_graph::VertexId& root_vertex_id = mission.getRootVertexId();
      
      if (!root_vertex_id.isValid() || !map->hasVertex(root_vertex_id)) {
        LOG(WARNING) << "[Exporter] Mission " << mission_id.hexString() 
                     << " has invalid root vertex";
        continue;
      }
      
      const vi_map::Vertex& root_vertex = map->getVertex(root_vertex_id);
      
      // Get the NCamera from the vertex
      if (root_vertex.hasVisualNFrame()) {
        aslam::NCamera::ConstPtr ncamera = root_vertex.getNCameras();
        
        if (ncamera != nullptr) {
          const aslam::SensorId& ncamera_id = ncamera->getId();
          
          // Look up this NCamera sensor in the sensor manager
          if (sensor_manager.hasSensor(ncamera_id)) {
            const aslam::NCamera::Ptr ncamera_from_manager = 
                sensor_manager.getSensorPtr<aslam::NCamera>(ncamera_id);
            
            if (ncamera_from_manager != nullptr) {
              // Get topic from the NCamera sensor
              const std::string& topic = ncamera_from_manager->getTopic();
              std::string robot_name = extractRobotNameFromTopic(topic);
              
              if (!robot_name.empty()) {
                mission_to_robot[mission_id.hexString()] = robot_name;
                LOG(INFO) << "[Exporter] Mission " << mission_id.hexString()
                          << " -> robot '" << robot_name << "' (from topic: " 
                          << topic << ")";
              } else {
                // Try getting topic from individual cameras
                if (ncamera_from_manager->getNumCameras() > 0) {
                  aslam::Camera::ConstPtr camera = ncamera_from_manager->getCameraShared(0);
                  if (camera != nullptr) {
                    const std::string& cam_topic = camera->getTopic();
                    robot_name = extractRobotNameFromTopic(cam_topic);
                    
                    if (!robot_name.empty()) {
                      mission_to_robot[mission_id.hexString()] = robot_name;
                      LOG(INFO) << "[Exporter] Mission " << mission_id.hexString()
                                << " -> robot '" << robot_name 
                                << "' (from camera topic: " << cam_topic << ")";
                    }
                  }
                }
              }
            }
          }
        }
      }
      
      // If we still don't have a name, log a warning
      if (mission_to_robot.find(mission_id.hexString()) == mission_to_robot.end()) {
        LOG(WARNING) << "[Exporter] Could not determine robot name for mission "
                     << mission_id.hexString() << ", will use mission ID";
      }
    }
    
  } catch (const std::exception& e) {
    LOG(ERROR) << "[Exporter] Exception while building mission->robot map: " 
               << e.what();
  } catch (...) {
    LOG(ERROR) << "[Exporter] Unknown exception while building mission->robot map.";
  }
  
  // Export loop closures
  pose_graph::EdgeIdList all_edge_ids;
  map->getAllEdgeIds(&all_edge_ids);
  json all_closures = json::array();
  size_t exported_count = 0;
  bool export_failed = false;

  LOG(INFO) << "[Exporter] Checking " << all_edge_ids.size()
            << " total edges for loop closures to export...";

  for (const pose_graph::EdgeId& edge_id : all_edge_ids) {
    if (!map->hasEdge(edge_id)) continue;
    const vi_map::Edge& edge_base = map->getEdgeAs<vi_map::Edge>(edge_id);

    if (edge_base.getType() == vi_map::Edge::EdgeType::kLoopClosure) {
      try {
        const vi_map::LoopClosureEdge& lc_edge =
            map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
        
        if (!map->hasVertex(lc_edge.from()) || !map->hasVertex(lc_edge.to())) {
          LOG(WARNING) << "[Exporter] Loop closure edge " << edge_id
                       << " points to non-existent vertex. Skipping.";
          continue;
        }

        const vi_map::Vertex& vertex_from = map->getVertex(lc_edge.from());
        const vi_map::Vertex& vertex_to = map->getVertex(lc_edge.to());
        const vi_map::MissionId& mission_id_from = vertex_from.getMissionId();
        const vi_map::MissionId& mission_id_to = vertex_to.getMissionId();

        // Use robot name if available, otherwise mission hex ID
        std::string name_from = mission_id_from.hexString();
        std::string name_to = mission_id_to.hexString();
        
        const std::string mid_from_hex = mission_id_from.hexString();
        const std::string mid_to_hex = mission_id_to.hexString();
        
        if (mission_to_robot.count(mid_from_hex)) {
          name_from = mission_to_robot[mid_from_hex];
        }
        if (mission_to_robot.count(mid_to_hex)) {
          name_to = mission_to_robot[mid_to_hex];
        }

        json lc_data;
        lc_data["names"] = {name_from, name_to};

        int64_t ts_from_ns = vertex_from.getMinTimestampNanoseconds();
        int64_t ts_to_ns = vertex_to.getMinTimestampNanoseconds();
        int64_t sec_from = ts_from_ns / 1000000000LL;
        int64_t ns_from = ts_from_ns % 1000000000LL;
        int64_t sec_to = ts_to_ns / 1000000000LL;
        int64_t ns_to = ts_to_ns % 1000000000LL;

        const aslam::Transformation& T_A_B = lc_edge.get_T_A_B();
        const Eigen::Vector3d& translation = T_A_B.getPosition();
        const aslam::Quaternion& rotation_quat = T_A_B.getRotation();

        lc_data["seconds"] = {sec_from, sec_to};
        lc_data["nanoseconds"] = {ns_from, ns_to};
        lc_data["translation"] = {translation.x(), translation.y(), translation.z()};
        lc_data["rotation"] =
            {rotation_quat.x(), rotation_quat.y(), rotation_quat.z(), rotation_quat.w()};
        lc_data["rotation_convention"] = "xyzw";

        all_closures.push_back(lc_data);
        exported_count++;
      } catch (const std::exception& e) {
        LOG(ERROR) << "[Exporter] Exception while exporting edge " << edge_id
                   << ": " << e.what();
        export_failed = true;
      } catch (...) {
        LOG(ERROR) << "[Exporter] Unknown exception while exporting edge " << edge_id;
        export_failed = true;
      }
    }
  }

  // Save the JSON file
  if (exported_count > 0 || !export_failed) {
    std::string output_filename = "loop_closures.json";
    try {
      std::ofstream output_file(output_filename);
      output_file << all_closures.dump(4);
      output_file.close();
      LOG(INFO) << "[Exporter] Success! Exported " << exported_count
                << " loop closures to " << output_filename;
    } catch (const std::exception& e) {
      LOG(ERROR) << "[Exporter] Failed to save JSON file '" << output_filename << "': " << e.what();
      export_failed = true;
    } catch (...) {
      LOG(ERROR) << "[Exporter] Unknown exception while saving JSON file '" << output_filename << "'.";
      export_failed = true;
    }
  } else if (exported_count == 0 && !export_failed) {
    LOG(INFO) << "[Exporter] No loop closure edges found in the map after detectLoopclosures. Nothing to export.";
  } else {
    LOG(ERROR) << "[Exporter] Export failed due to errors during edge processing or file saving.";
  }
  // --- END OF EXPORT CODE ---

  // Check if any edges were actually added before proceeding
  const int num_lc_edges = numLoopclosureEdges(*map);
  if (num_lc_edges == 0) {
    LOG(INFO) << "No loop closure edges present in map after filtering/export step. Relaxation cannot proceed with LC constraints.";
    return false;
  }
  LOG(INFO) << num_lc_edges
            << " loopclosure edges are present in map before solving relaxation.";

  // Step 3: Solve the relaxation problem
  return solveRelaxation(solver_options, mission_ids, map);
}

bool VIMapRelaxation::solveRelaxation(
    const ceres::Solver::Options& solver_options,
    const vi_map::MissionIdSet& mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();

  // Specific relaxation options.
  options.fix_accel_bias = true;
  options.fix_gyro_bias = true;
  options.fix_velocity = true;

  options.fix_intrinsics = true;
  options.fix_extrinsics_rotation = true;
  options.fix_extrinsics_translation = true;
  options.fix_landmark_positions = true;

  options.add_loop_closure_edges = true;

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          mission_ids, options, map));
  CHECK(optimization_problem);

  std::vector<std::shared_ptr<ceres::IterationCallback>> callbacks;
  if (FLAGS_ba_enable_signal_handler) {
    map_optimization::appendSignalHandlerCallback(&callbacks);
  }

  if (plotter_) {
    constexpr int kVisualizeEveryN = 1;
    map_optimization::appendVisualizationCallbacks(
        kVisualizeEveryN,
        *optimization_problem->getOptimizationStateBufferMutable(), *plotter_,
        map, &callbacks);
  }
  ceres::Solver::Options solver_options_with_callbacks = solver_options;
  map_optimization::addCallbacksToSolverOptions(
      callbacks, &solver_options_with_callbacks);

  map_optimization::solve(
      solver_options_with_callbacks, optimization_problem.get());

  visualizePosegraph(*map);

  const size_t number_of_loop_closure_edges_removed =
      map->removeLoopClosureEdges();
  LOG(INFO) << "Removed " << number_of_loop_closure_edges_removed
            << " loop closures edges.";

  return true;
}

}  // namespace map_optimizationac