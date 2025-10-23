#include <fstream>      
#include <glog/logging.h> 
#include <iostream>
#include <string>

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <vi-map/edge.h>   
#include <vi-map/mission.h> 
#include <vi-map/vertex.h>  
#include <vi-map/vi-map.h>

// --- NEW INCLUDES ---
#include <ceres/ceres.h> // Needed by relaxation
#include <map-optimization/solver-options.h> // Needed for solver options
#include <map-optimization/vi-map-relaxation.h> // The relaxation class
#include <visualization/viwls-graph-plotter.h> // Needed by relaxation
// --------------------

// Including the local json.hpp is correct
#include "json.hpp" 

// for convenience
using json = nlohmann::json;

// --- Define kSignalHandlerEnabled if not defined ---
// This constant is used by VIMapRelaxation but might not be visible.
#ifndef kSignalHandlerEnabled
constexpr bool kSignalHandlerEnabled = true;
#endif
// ---------------------------------------------------


class LoopClosureExporterPlugin : public common::ConsolePluginBase {
 public:
  // --- Need a plotter for VIMapRelaxation ---
  // We don't actually need to plot, so we'll create a dummy one.
  visualization::ViwlsGraphRvizPlotter dummy_plotter_;
  // -------------------------------------------

  std::string getPluginId() const override {
    return "loop_closure_exporter";
  }

  LoopClosureExporterPlugin(common::Console* console)
      : common::ConsolePluginBase(console) {

    // --- YOUR ORIGINAL EXPORT COMMAND (Unchanged) ---
    addCommand(
        {"export_loop_closures_json", "elcj"},
        [this]() -> int {
          std::string selected_map_key;
          if (!getSelectedMapKeyIfSet(&selected_map_key)) {
            return common::kStupidUserError;
          }
          vi_map::VIMapManager map_manager;
          vi_map::VIMapManager::MapWriteAccess map =
              map_manager.getMapWriteAccess(selected_map_key);
          
          json all_closures = json::array();
          pose_graph::EdgeIdList all_edge_ids; 
          map->getAllEdgeIds(&all_edge_ids);
          LOG(INFO) << "Iterating over " << all_edge_ids.size()
                    << " edges to find loop closures...";
          for (const pose_graph::EdgeId& edge_id : all_edge_ids) {
            const vi_map::Edge& edge = map->getEdgeAs<vi_map::Edge>(edge_id);
            if (edge.getType() == vi_map::Edge::EdgeType::kLoopClosure) {
              const vi_map::Vertex& vertex_from = map->getVertex(edge.from());
              const vi_map::Vertex& vertex_to = map->getVertex(edge.to());
              const vi_map::MissionId& mission_id_from = vertex_from.getMissionId();
              const vi_map::MissionId& mission_id_to = vertex_to.getMissionId();
              std::string name_from = mission_id_from.hexString();
              std::string name_to = mission_id_to.hexString();
              int64_t ts_from_ns = vertex_from.getMinTimestampNanoseconds();
              int64_t ts_to_ns = vertex_to.getMinTimestampNanoseconds();
              int64_t sec_from = ts_from_ns / 1'000'000'000;
              int64_t ns_from = ts_from_ns % 1'000'000'000;
              int64_t sec_to = ts_to_ns / 1'000'000'000;
              int64_t ns_to = ts_to_ns % 1'000'000'000;
              const vi_map::LoopClosureEdge& lc_edge =
                  static_cast<const vi_map::LoopClosureEdge&>(edge);
              const aslam::Transformation& T_A_B = lc_edge.get_T_A_B();
              const Eigen::Vector3d& translation = T_A_B.getPosition();
              const aslam::Quaternion& rotation_quat = T_A_B.getRotation(); 
              json lc_data;
              lc_data["seconds"] = {sec_from, sec_to};
              lc_data["nanoseconds"] = {ns_from, ns_to};
              lc_data["names"] = {name_from, name_to};
              lc_data["translation"] = {translation.x(), translation.y(), translation.z()};
              lc_data["rotation"] = {rotation_quat.x(), rotation_quat.y(), rotation_quat.z(), rotation_quat.w()};
              lc_data["rotation_convention"] = "xyzw";
              all_closures.push_back(lc_data);
            }
          }
          std::string output_filename = "loop_closures.json";
          std::ofstream output_file(output_filename);
          output_file << all_closures.dump(4); 
          output_file.close();
          LOG(INFO) << "Success! Exported " << all_closures.size()
                    << " loop closures to " << output_filename;
          return common::kSuccess;
        },
        "Exports all loop closure edges to a JSON file.",
        common::Processing::Sync);

    // --- NEW COMMAND: Relax and Keep Edges ---
    addCommand(
        {"relax_keep_edges", "rke"},
        [this]() -> int {
          std::string selected_map_key;
          if (!getSelectedMapKeyIfSet(&selected_map_key)) {
            return common::kStupidUserError;
          }
          vi_map::VIMapManager map_manager;
          vi_map::VIMapManager::MapWriteAccess map =
              map_manager.getMapWriteAccess(selected_map_key);

          // Instantiate the relaxation object (needs a plotter, even if dummy)
          map_optimization::VIMapRelaxation relaxation(
              &dummy_plotter_, kSignalHandlerEnabled);

          vi_map::MissionIdList mission_id_list;
          map->getAllMissionIds(&mission_id_list);
          vi_map::MissionIdSet mission_ids(
              mission_id_list.begin(), mission_id_list.end());

          // Step 1: Find Loop Closures and add edges (this is internal to VIMapRelaxation)
          // We call the public function that does this.
          const size_t num_lc_edges =
              relaxation.findLoopClosuresWithinMissions(mission_ids, map.get());
          
          if (num_lc_edges == 0u) {
            LOG(WARNING) << "No loop closures found by relax_keep_edges command.";
            // Still proceed to relaxation in case there were other constraints
          } else {
             LOG(INFO) << num_lc_edges << " loop closure edges found and added.";
          }

          // Step 2: Prepare and run the solver
          ceres::Solver::Options solver_options =
              map_optimization::initSolverOptionsFromFlags();
          
          bool success = relaxation.solveRelaxation(solver_options, mission_ids, map.get());

          // Step 3: CRUCIALLY, WE DO *NOT* CALL map->removeLoopClosureEdges() here!

          LOG(INFO) << "Relaxation finished. Loop closure edges were KEPT in the map.";

          return success ? common::kSuccess : common::kUnknownError;
        },
        "Relax posegraph like 'relax' but KEEP loop closure edges.",
        common::Processing::Sync);
  }
};

MAPLAB_CREATE_CONSOLE_PLUGIN(LoopClosureExporterPlugin);