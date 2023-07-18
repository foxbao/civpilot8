/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/adapters/planning_gflags.h"

#include <limits>

DEFINE_bool(planning_test_mode, false, "Enable planning test mode.");

// DEFINE_int32(planning_loop_rate, 10, "Loop rate for planning node");

// DEFINE_int32(history_max_record_num, 5,
//              "the number of planning history frame to keep");
// DEFINE_int32(max_frame_history_num, 1, "The maximum history frame number");

// // scenario related
// DEFINE_string(scenario_bare_intersection_unprotected_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/bare_intersection_unprotected_config.pb.txt",
//               "The bare_intersection_unprotected scenario configuration file");
// DEFINE_string(scenario_lane_follow_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/lane_follow_config.pb.txt",
//               "The lane_follow scenario configuration file");
// DEFINE_string(scenario_narrow_street_u_turn_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/narrow_street_u_turn_config.pb.txt",
//               "narrow_street_u_turn scenario config file");
// DEFINE_string(scenario_park_and_go_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/park_and_go_config.pb.txt",
//               "park_and_go scenario config file");
// DEFINE_string(scenario_pull_over_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/pull_over_config.pb.txt",
//               "The pull_over scenario configuration file");
// DEFINE_string(scenario_emergency_pull_over_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/emergency_pull_over_config.pb.txt",
//               "The emergency_pull_over scenario configuration file");
// DEFINE_string(scenario_emergency_stop_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/emergency_stop_config.pb.txt",
//               "The emergency_stop scenario configuration file");
// DEFINE_string(scenario_stop_sign_unprotected_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/stop_sign_unprotected_config.pb.txt",
//               "stop_sign_unprotected scenario configuration file");
// DEFINE_string(scenario_traffic_light_protected_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/traffic_light_protected_config.pb.txt",
//               "traffic_light_protected scenario config file");
// DEFINE_string(scenario_traffic_light_unprotected_left_turn_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/traffic_light_unprotected_left_turn_config.pb.txt",
//               "traffic_light_unprotected_left_turn scenario config file");
// DEFINE_string(scenario_traffic_light_unprotected_right_turn_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/traffic_light_unprotected_right_turn_config.pb.txt",
//               "traffic_light_unprotected_right_turn scenario config file");
// DEFINE_string(scenario_valet_parking_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/valet_parking_config.pb.txt",
//               "valet_parking scenario config file");
// DEFINE_string(scenario_yield_sign_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/yield_sign_config.pb.txt",
//               "yield_sign scenario config file");
// DEFINE_string(scenario_test_learning_model_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/test_learning_model_config.pb.txt",
//               "test_learning_model scenario config file");
// DEFINE_string(scenario_sweep_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/sweep_config.pb.txt",
//               "sweep scenario config file");
// DEFINE_string(scenario_sweep_narrow_u_turn_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "scenario/sweep_narrow_u_turn_config.pb.txt",
//               "sweep narrow u turn scenario config file");

// DEFINE_bool(enable_scenario_bare_intersection, true,
//             "enable bare_intersection scenarios in planning");

// DEFINE_bool(enable_scenario_park_and_go, true,
//             "enable park-and-go scenario in planning");

// DEFINE_bool(enable_scenario_pull_over, true,
//             "enable pull-over scenario in planning");

// DEFINE_bool(enable_scenario_emergency_pull_over, true,
//             "enable emergency-pull-over scenario in planning");

// DEFINE_bool(enable_scenario_emergency_stop, true,
//             "enable emergency-stop scenario in planning");

// DEFINE_bool(enable_scenario_side_pass_multiple_parked_obstacles, true,
//             "enable ADC to side-pass multiple parked obstacles without"
//             "worrying if the obstacles are blocked by others.");

// DEFINE_bool(enable_scenario_stop_sign, true,
//             "enable stop_sign scenarios in planning");

// DEFINE_bool(enable_scenario_test_learning_model, false,
//             "enable test learning model scenarios in planning");

// DEFINE_bool(enable_scenario_traffic_light, true,
//             "enable traffic_light scenarios in planning");

// DEFINE_bool(enable_scenario_yield_sign, true,
//             "enable yield_sign scenarios in planning");

// DEFINE_bool(enable_force_pull_over_open_space_parking_test, false,
//             "enable force_pull_over_open_space_parking_test");

// DEFINE_string(
//     traffic_rule_config_filename,
//     "/zhito/planning/middle_ware/cyber/conf/traffic_rule_config.pb.txt",
//     "Traffic rule config filename");

// DEFINE_string(
//     smoother_config_filename,
//     "/zhito/planning/middle_ware/cyber/conf/qp_spline_smoother_config.pb.txt",
//     "The configuration file for qp_spline smoother");

// DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
//               "Loop rate for planning node");

// DEFINE_uint64(rtk_trajectory_forward, 800,
//               "The number of points to be included in RTK trajectory "
//               "after the matched point");

// DEFINE_double(rtk_trajectory_resolution, 0.01,
//               "The time resolution of output trajectory for rtk planner.");

// DEFINE_bool(publish_estop, false, "publish estop decision in planning");
// DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory");

// DEFINE_double(lane_turn_cruise_speed, 2.78,
//               "The default speed when adc in turn");
// DEFINE_double(creep_cruise_speed, 3.0, "The default speed when creep");
// DEFINE_double(pull_over_stage_approach_cruise_speed, 3.0,
//               "The default cruise speed when at approach stage in pull over");

// DEFINE_bool(enable_reference_line_stitching, true,
//             "Enable stitching reference line, which can reducing computing "
//             "time and improve stability");
// DEFINE_double(look_forward_extend_distance, 50,
//               "The step size when extending reference line.");
// DEFINE_double(reference_line_stitch_overlap_distance, 20,
//               "The overlap distance with the existing reference line when "
//               "stitching the existing reference line");

// DEFINE_bool(enable_smooth_reference_line, true,
//             "enable smooth the map reference line");

// DEFINE_bool(prioritize_change_lane, false,
//             "change lane strategy has higher priority, always use a valid "
//             "change lane path if such path exists");
// DEFINE_double(change_lane_min_length, 30.0,
//               "meters. If the change lane target has longer length than this "
//               "threshold, it can shortcut the default lane.");

// DEFINE_double(replan_lateral_distance_threshold, 0.5,
//               "The lateral distance threshold of replan");
// DEFINE_double(replan_longitudinal_distance_threshold, 2.5,
//               "The longitudinal distance threshold of replan");

// DEFINE_bool(enable_reference_line_provider_thread, true,
//             "Enable reference line provider thread.");

// DEFINE_double(default_reference_line_width, 4.0,
//               "Default reference line width");

// DEFINE_double(smoothed_reference_line_max_diff, 5.0,
//               "Maximum position difference between the smoothed and the raw "
//               "reference lines.");

// DEFINE_double(planning_upper_speed_limit, 31.3,
//               "Maximum speed (m/s) in planning.");

// DEFINE_double(trajectory_time_length, 7.0, "Trajectory time length");

// // planning trajectory output time density control
// DEFINE_double(
//     trajectory_time_min_interval, 0.02,
//     "(seconds) Trajectory time interval when publish. The is the min value.");
// DEFINE_double(
//     trajectory_time_max_interval, 0.1,
//     "(seconds) Trajectory time interval when publish. The is the max value.");
// DEFINE_double(
//     trajectory_time_high_density_period, 1.0,
//     "(seconds) Keep high density in the next this amount of seconds. ");

// DEFINE_double(
//     max_stop_time_for_destination, 20.0,
//     "(seconds) Max time the adc should look ahead to stop for destination");

// DEFINE_bool(enable_trajectory_check, false,
//             "Enable sanity check for planning trajectory.");

// DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
// DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");
// DEFINE_double(
//     highway_acceptable_overspeed_ratio, 1.1,
//     "The highest acceptable ratio of highway overspeed to speed limit");
// DEFINE_double(speed_limit_wrt_kappa_ratio, 1.0,
//               "The ratio between the speed limit w.r.t. kappa deduced from "
//               "stardard road structure and the real restriction of adc in end "
//               "condition sampling and calculating reference speed");
// DEFINE_double(curise_speed_look_forward_time, 8.0,
//               "The look forward time for acc or dec in advance when cruise "
//               "speed changes in map. ");
// DEFINE_double(longitudinal_acceleration_lower_bound, -4.5,
//               "The lowest longitudinal acceleration allowed.");
// DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
//               "The highest longitudinal acceleration allowed.");
// DEFINE_double(lateral_acceleration_bound, 4.0,
//               "Bound of lateral acceleration; symmetric for left and right");

// DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
//               "The lower bound of longitudinal jerk.");
// DEFINE_double(longitudinal_jerk_upper_bound, 2.0,
//               "The upper bound of longitudinal jerk.");
// DEFINE_double(lateral_jerk_bound, 4.0,
//               "Bound of lateral jerk; symmetric for left and right");

// DEFINE_double(kappa_bound, 0.1979, "The bound for trajectory curvature");

// DEFINE_bool(use_acc_limit_wrt_v, false,
//             "Use acc limits with respect to speed chart as acc valid check");
// DEFINE_bool(use_jerk_limit_wrt_v, false,
//             "Use jerk limits with respect to speed chart as jerk valid check");
// DEFINE_bool(
//     use_speed_limit_wrt_kappa, false,
//     "Use speed limits with respect to kappa chart as velocity valid check");
// DEFINE_bool(use_strict_acc_jerk_limit, false,
//             "Use strict(otherwise, loose) acc and jerk limit with respect to "
//             "speed chart");

// // ST Boundary
// DEFINE_double(st_max_s, 100, "the maximum s of st boundary");
// DEFINE_double(st_max_t, 8, "the maximum t of st boundary");

// // Decision Part
// DEFINE_bool(enable_nudge_slowdown, true,
//             "True to slow down when nudge obstacles.");

// DEFINE_double(static_obstacle_nudge_l_buffer, 0.3,
//               "minimum l-distance to nudge a static obstacle (meters)");
// DEFINE_double(nonstatic_obstacle_nudge_l_buffer, 0.4,
//               "minimum l-distance to nudge a non-static obstacle (meters)");
// DEFINE_double(lane_change_obstacle_nudge_l_buffer, 0.3,
//               "minimum l-distance to nudge when changing lane (meters)");
// DEFINE_double(borrow_lane_nudge_l_buffer, 0.1,
//               "minimum l-distance to nudge when borrowing lane (meters)");
// DEFINE_double(no_borrow_lane_nudge_l_buffer, 0.1,
//               "minimum l-distance to nudge when not borrowing lane (meters)");
// DEFINE_double(lateral_ignore_buffer, 3.0,
//               "If an obstacle's lateral distance is further away than this "
//               "distance, ignore it");
// DEFINE_double(max_stop_distance_obstacle, 10.0,
//               "max stop distance from in-lane obstacle (meters)");
// DEFINE_double(min_stop_distance_obstacle, 6.0,
//               "min stop distance from in-lane obstacle (meters)");
// DEFINE_double(follow_min_distance, 3.0,
//               "min follow distance for vehicles/bicycles/moving objects");
// DEFINE_double(overtake_min_distance, 3.0,
//               "min follow distance for vehicles/bicycles/moving objects");
// DEFINE_double(follow_min_obs_lateral_distance, 2.5,
//               "obstacle min lateral distance to follow");
// DEFINE_double(
//     min_cos_value_consider_aligned, 0.707,
//     "when the heading_diff between the adc and obstacle heading is smaller"
//     "than 45 degree, they are considered to be aligned");
// DEFINE_double(yield_distance, 5.0,
//               "min yield distance for vehicles/moving objects "
//               "other than pedestrians/bicycles");
// DEFINE_double(follow_time_buffer, 2.5,
//               "time buffer in second to calculate the following distance.");
// DEFINE_double(overtake_time_buffer, 3.0,
//               "time buffer in second to calculate the overtake distance.");
// DEFINE_double(follow_min_time_sec, 2.0,
//               "min follow time in st region before considering a valid follow,"
//               " this is to differentiate a moving obstacle cross adc's"
//               " current lane and move to a different direction");
// DEFINE_double(signal_expire_time_sec, 5.0,
//               "traffic light signal info read expire time in sec");
// DEFINE_string(destination_obstacle_id, "DEST",
//               "obstacle id for converting destination to an obstacle");
// DEFINE_double(destination_check_distance, 5.0,
//               "if the distance between destination and ADC is less than this,"
//               " it is considered to reach destination");
// DEFINE_string(iov_stop_work_obstacle_id, "IOV_REMOTE_STOP_WORK",
//               "obstacle id for converting iov stop work cmd to an obstacle");
// DEFINE_double(
//     iov_stop_work_check_distance, 5.0,
//     "if the distance between iov_stop_work_fence and ADC is less than this,"
//     " it is considered to reach the fence");
// DEFINE_string(emergency_pull_over_obstacle_id, "EMERGENCY_PULL_OVER",
//               "virtual obstalce id for emergency pull over");
// DEFINE_string(sweep_working_destination_id, "SWEEP_WORKING_DESTINATION",
//               "virtual obstalce id for sweep-working");
// DEFINE_string(pre_add_water_stop_obstacle_id, "PRE_ADD_WATER_STOP",
//               "virtual obstalce id for pre-add water");
// DEFINE_string(stop_for_adding_water_obstacle_id, "STOP_FOR_ADDING_WATER",
//               "virtual obstalce id for stopping for adding water");
// DEFINE_string(finished_add_water_obstacle_id, "FINISHED_ADD_WATER",
//               "virtual obstalce id for finished add water");
// DEFINE_string(pre_dump_obstacle_id, "PRE_DUMP_STOP",
//               "virtual obstalce id for pre-add dump");
// DEFINE_string(stop_for_dumping_obstacle_id, "STOP_FOR_DUMPING",
//               "virtual obstalce id for stopping");
// DEFINE_string(finished_dump_obstacle_id, "FINISHED_DUMP",
//               "virtual obstalce id for pre-add water");

// DEFINE_double(virtual_stop_wall_length, 0.1,
//               "virtual stop wall length (meters)");
// DEFINE_double(virtual_stop_wall_height, 2.0,
//               "virtual stop wall height (meters)");
// DEFINE_double(park_and_go_cruise_l_buffer, 0.5,
//               "l buffer for checking cruise completed");
// DEFINE_double(park_and_go_cruise_heading_buffer, 0.1,
//               "heading buffer for checking cruise completed");
// // Lane Change
// DEFINE_double(safe_time_on_same_direct, 3.0,
//               "safe time on same direction (second)");
// DEFINE_double(safe_time_on_opp_direct, 5.0,
//               "safe time on opposite direction (second)");
// DEFINE_double(forward_min_safe_dist_on_same_direct, 10.0,
//               "forward min safe distance on same direction (meters)");
// DEFINE_double(backward_min_safe_dist_on_same_direct, 10.0,
//               "backward min safe distance on same direction (meters)");
// DEFINE_double(forward_min_safe_dist_on_opp_direct, 50.0,
//               "forward min safe distance on opposite direction (meters)");
// DEFINE_double(backward_min_safe_dist_on_opp_direct, 1.0,
//               "backward min safe distance on opposite direction (meters)");
// DEFINE_double(dist_buff, 0.5, "distance buffer (meters)");
// DEFINE_bool(enable_lane_change_judgement, false,
//             "enable  lane change judgement ");

// // Path Deciders
// DEFINE_bool(enable_skip_path_tasks, false,
//             "skip all path tasks and use trimmed previous path");

// DEFINE_double(obstacle_lat_buffer, 0.4,
//               "obstacle lateral buffer (meters) for deciding path boundaries");
// DEFINE_double(obstacle_lon_start_buffer, 3.0,
//               "obstacle longitudinal start buffer (meters) for deciding "
//               "path boundaries");
// DEFINE_double(obstacle_lon_end_buffer, 2.0,
//               "obstacle longitudinal end buffer (meters) for deciding "
//               "path boundaries");
// DEFINE_double(static_obstacle_speed_threshold, 0.5,
//               "The speed threshold to decide whether an obstacle is static "
//               "or not.");
// DEFINE_double(lane_borrow_max_speed, 5.0,
//               "The speed threshold for lane-borrow");
// DEFINE_int32(long_term_blocking_obstacle_cycle_threshold, 3,
//              "The cycle threshold for long-term blocking obstacle.");

// // Prediction Part
// DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
// DEFINE_bool(align_prediction_time, false,
//             "enable align prediction data based planning time");
// DEFINE_bool(align_localization_time, true,
//             "enable localization prediction data based planning time");

// // Trajectory

// // according to DMV's rule, turn signal should be on within 200 ft from
// // intersection.
// DEFINE_double(
//     turn_signal_distance, 100.00,
//     "In meters. If there is a turn within this distance, use turn signal");

// // planning config file
// DEFINE_string(planning_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/planning_config.pb.txt",
//               "planning config file");

// DEFINE_int32(trajectory_point_num_for_debug, 10,
//              "number of output trajectory points for debugging");

// DEFINE_double(lane_change_prepare_length, 80.0,
//               "The distance of lane-change preparation on current lane.");

// DEFINE_double(min_lane_change_prepare_length, 10.0,
//               "The minimal distance needed of lane-change on current lane.");

// DEFINE_double(allowed_lane_change_failure_time, 2.0,
//               "The time allowed for lane-change failure before updating"
//               "preparation distance.");

// DEFINE_bool(enable_smarter_lane_change, false,
//             "enable smarter lane change with longer preparation distance.");

// // QpSt optimizer
// DEFINE_double(slowdown_profile_deceleration, -4.0,
//               "The deceleration to generate slowdown profile. unit: m/s^2.");

// // SQP solver
// DEFINE_bool(enable_sqp_solver, true, "True to enable SQP solver.");

// /// thread pool
// DEFINE_bool(use_multi_thread_to_add_obstacles, false,
//             "use multiple thread to add obstacles.");
// DEFINE_bool(enable_multi_thread_in_dp_st_graph, false,
//             "Enable multiple thread to calculation curve cost in dp_st_graph.");

// /// Lattice Planner
// DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in lattice planner.");
// DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");
// DEFINE_double(trajectory_time_resolution, 0.1,
//               "Trajectory time resolution in planning");
// DEFINE_double(trajectory_space_resolution, 1.0,
//               "Trajectory space resolution in planning");
// DEFINE_double(speed_lon_decision_horizon, 200.0,
//               "Longitudinal horizon for speed decision making (meter)");
// DEFINE_uint64(num_velocity_sample, 6,
//               "The number of velocity samples in end condition sampler.");
// DEFINE_bool(enable_backup_trajectory, true,
//             "If generate backup trajectory when planning fail");
// DEFINE_double(backup_trajectory_cost, 1000.0,
//               "Default cost of backup trajectory");
// DEFINE_double(min_velocity_sample_gap, 1.0,
//               "Minimal sampling gap for velocity");
// DEFINE_double(lon_collision_buffer, 2.0,
//               "The longitudinal buffer to keep distance to other vehicles");
// DEFINE_double(lat_collision_buffer, 0.1,
//               "The lateral buffer to keep distance to other vehicles");
// DEFINE_double(lon_time_to_collision, 1e-3,
//               "The longitudinal time buffer to avoid collision caused by "
//               "sudden deacceleration");
// DEFINE_uint64(num_sample_follow_per_timestamp, 3,
//               "The number of sample points for each timestamp to follow");

// // Lattice Evaluate Parameters
// DEFINE_double(weight_lon_objective, 10.0, "Weight of longitudinal travel cost");
// DEFINE_double(weight_lon_jerk, 1.0, "Weight of longitudinal jerk cost");
// DEFINE_double(weight_lon_collision, 5.0,
//               "Weight of longitudinal collision cost");
// DEFINE_double(weight_lat_offset, 2.0, "Weight of lateral offset cost");
// DEFINE_double(weight_lat_comfort, 10.0, "Weight of lateral comfort cost");
// DEFINE_double(weight_centripetal_acceleration, 1.5,
//               "Weight of centripetal acceleration");
// DEFINE_double(cost_non_priority_reference_line, 5.0,
//               "The cost of planning on non-priority reference line.");
// DEFINE_double(weight_same_side_offset, 1.0,
//               "Weight of same side lateral offset cost");
// DEFINE_double(weight_opposite_side_offset, 10.0,
//               "Weight of opposite side lateral offset cost");
// DEFINE_double(weight_dist_travelled, 10.0, "Weight of travelled distance cost");
// DEFINE_double(weight_target_speed, 1.0, "Weight of target speed cost");
// DEFINE_double(lat_offset_bound, 3.0, "The bound of lateral offset");
// DEFINE_double(lon_collision_yield_buffer, 1.0,
//               "Longitudinal collision buffer for yield");
// DEFINE_double(lon_collision_overtake_buffer, 5.0,
//               "Longitudinal collision buffer for overtake");
// DEFINE_double(lon_collision_cost_std, 0.5,
//               "The standard deviation of longitudinal collision cost function");
// DEFINE_double(default_lon_buffer, 5.0,
//               "Default longitudinal buffer to sample path-time points.");
// DEFINE_double(time_min_density, 1.0,
//               "Minimal time density to search sample points.");
// DEFINE_double(comfort_acceleration_factor, 0.5,
//               "Factor for comfort acceleration.");
// DEFINE_double(comfort_deceleration_factor, 0.5,
//               "Factor for comfort acceleration.");
// DEFINE_double(polynomial_minimal_param, 0.01,
//               "Minimal time parameter in polynomials.");
// DEFINE_double(lattice_stop_buffer, 0.02,
//               "The buffer before the stop s to check trajectories.");

// DEFINE_bool(lateral_optimization, true,
//             "whether using optimization for lateral trajectory generation");
// DEFINE_double(weight_lateral_offset, 1.0,
//               "weight for lateral offset "
//               "in lateral trajectory optimization");
// DEFINE_double(weight_lateral_derivative, 500.0,
//               "weight for lateral derivative "
//               "in lateral trajectory optimization");
// DEFINE_double(weight_lateral_second_order_derivative, 1000.0,
//               "weight for lateral second order derivative "
//               "in lateral trajectory optimization");
// DEFINE_double(weight_lateral_third_order_derivative, 1000.0,
//               "weight for lateral third order derivative "
//               "in lateral trajectory optimization");
// DEFINE_double(
//     weight_lateral_obstacle_distance, 0.0,
//     "weight for lateral obstacle distance in lateral trajectory optimization");
// DEFINE_double(lateral_third_order_derivative_max, 0.1,
//               "the maximal allowance for lateral third order derivative");
// DEFINE_double(lateral_derivative_bound_default, 2.0,
//               "the default value for lateral derivative bound.");
// DEFINE_double(max_s_lateral_optimization, 60.0,
//               "The maximal s for lateral optimization.");
// DEFINE_double(default_delta_s_lateral_optimization, 1.0,
//               "The default delta s for lateral optimization.");
// DEFINE_double(bound_buffer, 0.1, "buffer to boundary for lateral optimization");
// DEFINE_double(nudge_buffer, 0.3, "buffer to nudge for lateral optimization");

// DEFINE_double(fallback_total_time, 3.0, "total fallback trajectory time");
// DEFINE_double(fallback_time_unit, 0.1,
//               "fallback trajectory unit time in seconds");

// DEFINE_double(speed_bump_speed_limit, 4.4704,
//               "the speed limit when passing a speed bump, m/s. The default "
//               "speed limit is 10 mph.");
// DEFINE_double(default_city_road_speed_limit, 15.67,
//               "default speed limit (m/s) for city road. 35 mph.");
// DEFINE_double(default_highway_speed_limit, 29.06,
//               "default speed limit (m/s) for highway. 65 mph.");

// // navigation mode
// DEFINE_bool(enable_planning_pad_msg, false,
//             "To control whether to enable planning pad message.");

// // story related
// DEFINE_bool(enable_storytelling, true,
//             "To control whether to enable storytelling message.");
// DEFINE_string(story_cruise_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "stories/story_cruise_config.pb.txt",
//               "The cruise story configuration file");
// DEFINE_string(story_close_to_ramp_merge_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "stories/story_close_to_ramp_merge_config.pb.txt",
//               "The close_to_ramp_merge story configuration file");
// DEFINE_string(story_close_to_ramp_split_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "stories/story_close_to_ramp_split_config.pb.txt",
//               "The close_to_ramp_split story configuration file");
// DEFINE_string(story_close_to_lane_change_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "stories/story_close_to_lane_change_config.pb.txt",
//               "The close_to_lane_change story configuration file");
// DEFINE_string(story_close_to_speed_limit_change_config_file,
//               "/zhito/planning/middle_ware/cyber/conf/"
//               "stories/story_close_to_speed_limit_change_config.pb.txt",
//               "The close_to_speed_limit_change story configuration file");

// DEFINE_bool(enable_story_close_to_ramp_merge, false,
//             "To control whether to enable close_to_ramp_merge.");
// DEFINE_bool(enable_story_close_to_ramp_split, false,
//             "To control whether to enable close_to_ramp_split.");
// DEFINE_bool(enable_story_close_to_lane_change, false,
//             "To control whether to enable close_to_lane_change.");
// DEFINE_bool(enable_story_close_to_speed_limit_change, false,
//             "To control whether to enable close_to_speed_limit_change.");

// // TODO(all): open space planner, merge with planning conf
// DEFINE_string(planner_open_space_config_filename,
//               "/zhito/modules/planning/conf/planner_open_space_config.pb.txt",
//               "The open space planner configuration file");

// DEFINE_double(open_space_planning_period, 4.0,
//               "estimated time for open space planner planning period");

// DEFINE_double(open_space_prediction_time_horizon, 2.0,
//               "the time in second we use from the trajectory of obstacles "
//               "given by prediction");

// DEFINE_bool(enable_perception_obstacles, true,
//             "enable the open space planner to take perception obstacles into "
//             "consideration");

// DEFINE_bool(enable_open_space_planner_thread, true,
//             "Enable thread in open space planner for trajectory publish.");

// DEFINE_bool(use_dual_variable_warm_start, true,
//             "whether or not enable dual variable warm start ");

// DEFINE_bool(use_gear_shift_trajectory, false,
//             "allow some time for the vehicle to shift gear");

// DEFINE_uint64(open_space_trajectory_stitching_preserved_length,
//               std::numeric_limits<uint32_t>::infinity(),
//               "preserved points number in trajectory stitching for open space "
//               "trajectory");
// DEFINE_bool(
//     enable_smoother_failsafe, false,
//     "whether to use warm start result as final output when smoother fails");

// DEFINE_bool(use_s_curve_speed_smooth, false,
//             "Whether use s-curve (piecewise_jerk) for smoothing Hybrid Astar "
//             "speed/acceleration.");

// DEFINE_bool(
//     use_iterative_anchoring_smoother, false,
//     "Whether use iterative_anchoring_smoother for open space planning ");

// DEFINE_bool(
//     enable_parallel_trajectory_smoothing, false,
//     "Whether to partition the trajectory first and do smoothing in parallel");

// DEFINE_bool(use_osqp_optimizer_for_reference_line, true,
//             "Use OSQP optimizer for reference line optimization.");
// DEFINE_bool(enable_osqp_debug, false,
//             "True to turn on OSQP verbose debug output in log.");

// DEFINE_bool(export_chart, false, "export chart in planning");
// DEFINE_bool(enable_record_debug, true,
//             "True to enable record debug info in chart format");

// DEFINE_double(
//     default_front_clear_distance, 300.0,
//     "default front clear distance value in case there is no obstacle around.");

// DEFINE_double(max_trajectory_len, 1000.0,
//               "(unit: meter) max possible trajectory length.");
// DEFINE_bool(enable_rss_fallback, false, "trigger rss fallback");
// DEFINE_bool(enable_rss_info, true, "enable rss_info in trajectory_pb");
// DEFINE_double(rss_max_front_obstacle_distance, 3000.0,
//               "(unit: meter) for max front obstacle distance.");

// DEFINE_bool(
//     enable_planning_smoother, false,
//     "True to enable planning smoother among different planning cycles.");
// DEFINE_double(smoother_stop_distance, 10.0,
//               "(unit: meter) for ADC stop, if it is close to the stop point "
//               "within this threshold, current planning will be smoothed.");

// DEFINE_bool(enable_parallel_hybrid_a, false,
//             "True to enable hybrid a* parallel implementation.");

// DEFINE_double(open_space_standstill_acceleration, 0.0,
//               "(unit: meter/sec^2) for open space stand still at destination");

// DEFINE_bool(enable_dp_reference_speed, true,
//             "True to penalize dp result towards default cruise speed");

// DEFINE_double(message_latency_threshold, 0.02, "Threshold for message delay");
// DEFINE_bool(enable_lane_change_urgency_checking, false,
//             "True to check the urgency of lane changing");
// DEFINE_double(short_path_length_threshold, 20.0,
//               "Threshold for too short path length");

// DEFINE_uint64(trajectory_stitching_preserved_length, 20,
//               "preserved points number in trajectory stitching");

// DEFINE_double(side_pass_driving_width_l_buffer, 0.1,
//               "(unit: meter) for side pass driving width l buffer");

// DEFINE_bool(use_st_drivable_boundary, false,
//             "True to use st_drivable boundary in speed planning");

// DEFINE_bool(enable_reuse_path_in_lane_follow, false,
//             "True to enable reuse path in lane follow");
// DEFINE_bool(
//     use_smoothed_dp_guide_line, false,
//     "True to penalize speed optimization result to be close to dp guide line");

// DEFINE_bool(use_soft_bound_in_nonlinear_speed_opt, true,
//             "False to disallow soft bound in nonlinear speed opt");

// DEFINE_bool(use_front_axe_center_in_path_planning, false,
//             "If using front axe center in path planning, the path can be "
//             "more agile.");

// DEFINE_bool(use_road_boundary_from_map, false, "get road boundary from HD map");

// DEFINE_double(
//     vehicle_heading_diff_threshold, 0.05,
//     "threshold to determine whether vehicle is aligned to the reference line");

// DEFINE_double(deceleration_ration_of_approaching_crosswalk, 0.6,
//               "start decelerating when ADC is approaching crosswalk");

// DEFINE_double(maximum_debouncing_duration_time, 3.0,
//               "debouncing duration time when path is blocking frequently");

// DEFINE_bool(enable_debounce_path_blocking, false,
//             "enable debounce path blocking in path bounds decider of planning");

// DEFINE_int32(max_bounce_count, 3,
//              "trigger debouncing when the number of path blocking jittering "
//              "reachs max_bounce_count");

// // spiral_smoother_util

// DEFINE_string(input_file, "", "input file with format x,y per line");
// DEFINE_string(output_file, "", "output file with format x,y per line");
// DEFINE_double(smooth_length, 200.0, "Smooth this amount of length ");
// DEFINE_double(minimum_point_spacing, 5.0,
//               "The minimum distance for input points.");

// DEFINE_string(planning_data_dir, "/zhito/planning/data/",
//               "Prefix of files to store learning_data_frame data");
// DEFINE_int32(planning_freq, 10, "frequence of planning message");
// DEFINE_int32(learning_data_frame_num_per_file, 100,
//              "number of learning_data_frame to write out in one data file.");
// DEFINE_int32(learning_data_obstacle_history_point_cnt, 20,
//              "number of history trajectory points for a obstacle");
// DEFINE_bool(enable_binary_learning_data, true,
//             "True to generate protobuf binary data file.");
// DEFINE_bool(enable_overlap_tag, true,
//             "True to add overlap tag to planning_tag");

// DEFINE_string(planning_source_dirs, "",
//               "a list of source files or directories for offline mode. "
//               "The items need to be separated by colon ':'. ");

// DEFINE_double(trajectory_delta_t, 0.2,
//               "delta time(sec) between trajectory points");
// DEFINE_bool(enable_evaluate_obstacle_trajectory, true,
//             "enable obstacle_trajectory evaluation by time");
// DEFINE_bool(enable_manual_mode_trajectory_stitching, true,
//             "enable trajectory stitching when vehicle is no in complete auto "
//             "drive mode");

// DEFINE_bool(use_aggressive_strategy_on_traffic_lights, false,
//             "use more aggressive strategy when encoutering traffic lights");
// DEFINE_double(yellow_light_duration_threshold, 5.0,
//               "duration threshold for ignoring the yellow light");
// DEFINE_int32(green_red_count_threshold, 6,
//              "max count threshold for wrong detection of traffic light color");

// DEFINE_string(planning_not_ready_reason_due_to_localization,
//               "localization not ready", "");
// DEFINE_string(planning_not_ready_reason_due_to_chassis, "chassis not ready",
//               "");
// DEFINE_string(planning_not_ready_reason_due_to_map, "map not ready", "");
// DEFINE_string(planning_not_ready_reason_due_to_relative_map,
//               "relative map not ready", "");
// DEFINE_string(planning_not_ready_reason_due_to_routing, "routing not ready",
//               "");
// DEFINE_string(planning_not_ready_reason_due_to_update_vehicleStateProvider,
//               "Update VehicleStateProvider failed", "");
// DEFINE_string(planning_not_ready_reason_due_to_init_frame,
//               "Failed to init frame", "");

// DEFINE_double(changelane_observe_speed, 5.55,
//               "change lane observe speed(m/s) default 20km/h");
// DEFINE_bool(use_speed_from_map, true, "use speed info from map");
// DEFINE_string(torch_ramp_model_cpu_file, "/zhito/planning/data/tamp.pt",
//               "tamp file");
// DEFINE_bool(use_rl_planner, false, "using RL planner or not");

// // Switches of features designed by Su Junsheng
// DEFINE_bool(use_new_lateral_offset_cost, false,
//             "use new lateral offset cost algorithm");

// DEFINE_double(sweep_safe_buffer, 0.1,
//               "safe boundary buffer for sweep along the road side.");
// DEFINE_double(sweep_offset_buffer, 0.1,
//               "offset reference buffer for sweep along the road side.");
// DEFINE_double(sweeping_state_traffic_light_start_distance, 15.0,
//               "set sweeping scenario change sweeping state to traffic light "
//               "start distance");
// DEFINE_double(sweeping_state_traffic_light_end_distance, 15.0,
//               "set sweeping scenario change sweeping state to traffic light "
//               "end distance");
// DEFINE_double(default_sweep_cruise_speed, 2.78, "default sweep cruise speed");
// DEFINE_double(sweep_u_turn_cruise_speed, 1.0, "sweep u turn cruise speed");
// DEFINE_bool(enable_iov, false, "connect to iov mdoule");
// DEFINE_bool(enable_pad_terminal, false, "connect to pad_terminal module");
// DEFINE_bool(use_debug_cost, false, "send lattice cost in topic debug part");
// DEFINE_bool(use_debug_acc_jerk, false,
//             "send lattice acc&jerk in topic debug part");

// DEFINE_double(react_time, 0.5, "default sweep cruise speed");

// // trailer related
// DEFINE_bool(use_perception_trailer_theta, false,
//             "use trailer theta detected by perception");
// DEFINE_double(
//     use_perception_trailer_theta_threshold, 0.0174,
//     "threshold(unit: rad) of using perception trailer theta, "
//     "if the diff between kinematics derivation theta and perception trailer "
//     "theta is less than the threshold, then use perception trailer theta.");
// DEFINE_bool(enable_planning_wrt_trailer, false,
//             "enable planning trajectory w.r.t trailer.");

// // active lane change related
// DEFINE_bool(active_lane_change_by_rerouting, true,
//             "if set as true, change lane based on rerouting, else "
//             "based on neighbor reference line.");
// DEFINE_double(min_length_active_lane_change, 200.0,
//               "minimum length(unit: m) that active lane change need.");
// DEFINE_bool(enable_left_reference_line, false,
//             "enable create left neighbor reference line");
// DEFINE_bool(enable_right_reference_line, false,
//             "enable create right neighbor reference line");
// DEFINE_double(min_valid_length_neighbor_reference_line, 10.0,
//               "minimum valid length of neighbor reference line");

// // used by updateOrderForward in pnc_map
// DEFINE_int32(order_forward_road_segment, 3,
//              "number of road_segments that used to update order_forward_ids_.");

// // adm related
// DEFINE_bool(enable_planning_adm, false,
//             "enable interface between adm and planning");
// // auto change lane
// DEFINE_double(change_lane_max_distance_to_destination, 250,
//               "distance to destination");
// DEFINE_double(long_term_blocking_obstacle_cycle_threhold, 0.8,
//               "long term blocking obstacle frequence per 10 planning cycle");
// DEFINE_bool(enable_right_direction_change_lane, false,
//             "enable right direction change lane");
// DEFINE_bool(enable_change_lane_comeback, false, "enable change lane come back");
// DEFINE_bool(enable_change_lane_start, false, "enable change lane start");
// DEFINE_double(
//     min_trigger_auto_lane_change_speed, 0.625,
//     "minimum speed in auto lane change need.default 0.625*speed_limit m/s");
// DEFINE_double(adjacent_lane_obstacle_min_distance, 80,
//               "minimum distance in auto lane change need.default m");
// DEFINE_double(adjacent_lane_obstacle_min_speed, 19.0,
//               "minimum speed in auto lane change need.default 13.9m/s");
// DEFINE_double(vechile_front_safe_distance, 80,
//               "change lane vechile front safe distance, m");
// DEFINE_double(vechile_rear_safe_distance, 150,
//               "change lane vechile front safe distance, m");
// DEFINE_double(check_boundary_type_lookforward_distance, 150,
//               "check boundary type lookforward distance, m");
// DEFINE_double(current_lane_obstacle_blocking_min_distance, 200,
//               "current lane obstacle blocking min distance, m");
// DEFINE_double(current_lane_obstacle_blocking_count, 5,
//               "current lane obstacle blocking min distance");
// DEFINE_bool(enable_side_lane_blocking_judge, false,
//             "enable side lane blocking judge");
// DEFINE_double(active_change_lane_done_delay_time, 5.0,
//               "active change lane done delay time");
// DEFINE_bool(enable_active_change_lane, false, "enable active change lane");
// DEFINE_double(follow_target_obstacle_low_speed_time, 5.0,
//               "Follow the target obstacle and keep driving at low speed time");
// DEFINE_double(check_blocking_delay_time, 0.5,
//               "check current lane is blocking delay time");
// DEFINE_double(feasible_region_rate, 1.2, "feasible region rate");
// DEFINE_bool(enable_feasible_region_rate, true, "enable feasible region rate");
// DEFINE_double(low_speed_obstacle_rate, 0.5, "low speed obstacle rate m/s");
// DEFINE_bool(enable_auto_change_lane, false, "enable auto change lane");
// DEFINE_double(calculate_ttc_time, 3, "calculate ttc time");
// DEFINE_double(target_lane_front_ttc_time, 4.4, "calculate front ttc time");
// DEFINE_double(target_lane_rear_ttc_time, 4.4, "calculate rear ttc time");

// DEFINE_bool(enable_planning_L3, false, "planning for L3 flag");
// DEFINE_bool(enable_planning_L3P, false, "planning for L3P flag");
// DEFINE_bool(enable_planning_L4, false, "planning for L4 flag");

// // cruising routing related
// DEFINE_double(min_cruising_routing_length, 100.0,
//               "min length of cruising routing");

// // turn signal in advance related
// DEFINE_double(time_for_turn_signal_in_advance, 3.0,
//               "time required for turning signal in advance");
// DEFINE_double(distance_for_lane_change_after_junction, 2.5,
//               "distance used to judge whether lane change is in junction or "
//               "after junction");

// DEFINE_bool(use_reverse_speed, false, "is negative speed allowed.");
