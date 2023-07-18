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

#pragma once

#include "gflags/gflags.h"

DECLARE_bool(planning_test_mode);

// DECLARE_string(planning_config_file);

// DECLARE_int32(history_max_record_num);
// DECLARE_int32(max_frame_history_num);

// // scenarios related
// DECLARE_string(scenario_bare_intersection_unprotected_config_file);
// DECLARE_string(scenario_emergency_pull_over_config_file);
// DECLARE_string(scenario_emergency_stop_config_file);
// DECLARE_string(scenario_lane_follow_config_file);
// DECLARE_string(scenario_narrow_street_u_turn_config_file);
// DECLARE_string(scenario_park_and_go_config_file);
// DECLARE_string(scenario_pull_over_config_file);
// DECLARE_string(scenario_stop_sign_unprotected_config_file);
// DECLARE_string(scenario_traffic_light_protected_config_file);
// DECLARE_string(scenario_traffic_light_unprotected_left_turn_config_file);
// DECLARE_string(scenario_traffic_light_unprotected_right_turn_config_file);
// DECLARE_string(scenario_traffic_light_unprotected_right_turn_config_file);
// DECLARE_string(scenario_valet_parking_config_file);
// DECLARE_string(scenario_yield_sign_config_file);
// DECLARE_string(scenario_test_learning_model_config_file);
// DECLARE_string(scenario_sweep_config_file);
// DECLARE_string(scenario_sweep_narrow_u_turn_config_file);

// DECLARE_bool(enable_scenario_bare_intersection);
// DECLARE_bool(enable_scenario_emergency_pull_over);
// DECLARE_bool(enable_scenario_emergency_stop);
// DECLARE_bool(enable_scenario_park_and_go);
// DECLARE_bool(enable_scenario_pull_over);
// DECLARE_bool(enable_scenario_stop_sign);
// DECLARE_bool(enable_scenario_test_learning_model);
// DECLARE_bool(enable_scenario_traffic_light);
// DECLARE_bool(enable_scenario_yield_sign);

// DECLARE_bool(enable_scenario_side_pass_multiple_parked_obstacles);
// DECLARE_bool(enable_force_pull_over_open_space_parking_test);

// DECLARE_string(traffic_rule_config_filename);
// DECLARE_string(smoother_config_filename);
// DECLARE_int32(planning_loop_rate);
// DECLARE_string(rtk_trajectory_filename);
// DECLARE_uint64(rtk_trajectory_forward);
// DECLARE_double(rtk_trajectory_resolution);
// DECLARE_bool(enable_reference_line_stitching);
// DECLARE_double(look_forward_extend_distance);
// DECLARE_double(reference_line_stitch_overlap_distance);

// DECLARE_bool(enable_smooth_reference_line);

// DECLARE_bool(prioritize_change_lane);
// DECLARE_double(change_lane_min_length);

// DECLARE_bool(publish_estop);
// DECLARE_bool(enable_trajectory_stitcher);

// DECLARE_double(lane_turn_cruise_speed);
// DECLARE_double(creep_cruise_speed);
// DECLARE_double(pull_over_stage_approach_cruise_speed);

// // parameters for trajectory stitching and reinit planning starting point.
// DECLARE_double(replan_lateral_distance_threshold);
// DECLARE_double(replan_longitudinal_distance_threshold);

// // parameter for reference line
// DECLARE_bool(enable_reference_line_provider_thread);
// DECLARE_double(default_reference_line_width);
// DECLARE_double(smoothed_reference_line_max_diff);

// // parameters for trajectory planning
// DECLARE_double(planning_upper_speed_limit);
// DECLARE_double(trajectory_time_length);
// DECLARE_double(trajectory_time_min_interval);
// DECLARE_double(trajectory_time_max_interval);
// DECLARE_double(trajectory_time_high_density_period);
// DECLARE_double(max_stop_time_for_destination);

// // parameters for trajectory sanity check
// DECLARE_bool(enable_trajectory_check);
// DECLARE_double(speed_lower_bound);
// DECLARE_double(speed_upper_bound);
// DECLARE_double(highway_acceptable_overspeed_ratio);
// DECLARE_double(speed_limit_wrt_kappa_ratio);
// DECLARE_double(curise_speed_look_forward_time);

// DECLARE_double(longitudinal_acceleration_lower_bound);
// DECLARE_double(longitudinal_acceleration_upper_bound);

// DECLARE_double(longitudinal_jerk_lower_bound);
// DECLARE_double(longitudinal_jerk_upper_bound);
// DECLARE_double(lateral_jerk_bound);

// DECLARE_double(kappa_bound);
// DECLARE_bool(use_acc_limit_wrt_v);
// DECLARE_bool(use_jerk_limit_wrt_v);
// DECLARE_bool(use_speed_limit_wrt_kappa);
// DECLARE_bool(use_strict_acc_jerk_limit);

// // STBoundary
// DECLARE_double(st_max_s);
// DECLARE_double(st_max_t);

// // Decision Part
// DECLARE_bool(enable_nudge_slowdown);
// DECLARE_double(static_obstacle_nudge_l_buffer);
// DECLARE_double(nonstatic_obstacle_nudge_l_buffer);
// DECLARE_double(lane_change_obstacle_nudge_l_buffer);
// DECLARE_double(borrow_lane_nudge_l_buffer);
// DECLARE_double(no_borrow_lane_nudge_l_buffer);
// DECLARE_double(lateral_ignore_buffer);
// DECLARE_double(min_stop_distance_obstacle);
// DECLARE_double(max_stop_distance_obstacle);
// DECLARE_double(follow_min_distance);
// DECLARE_double(overtake_min_distance);
// DECLARE_double(follow_min_obs_lateral_distance);
// DECLARE_double(min_cos_value_consider_aligned);
// DECLARE_double(yield_distance);
// DECLARE_double(follow_time_buffer);
// DECLARE_double(overtake_time_buffer);
// DECLARE_double(follow_min_time_sec);
// DECLARE_double(signal_expire_time_sec);
// DECLARE_double(park_and_go_cruise_l_buffer);
// DECLARE_double(park_and_go_cruise_heading_buffer);

// // Lane Change
// DECLARE_double(safe_time_on_same_direct);
// DECLARE_double(safe_time_on_opp_direct);
// DECLARE_double(forward_min_safe_dist_on_same_direct);
// DECLARE_double(backward_min_safe_dist_on_same_direct);
// DECLARE_double(forward_min_safe_dist_on_opp_direct);
// DECLARE_double(backward_min_safe_dist_on_opp_direct);
// DECLARE_double(dist_buff);
// DECLARE_bool(enable_lane_change_judgement);

// // Path Deciders
// DECLARE_bool(enable_skip_path_tasks);
// DECLARE_bool(enable_reuse_path_in_lane_follow);

// DECLARE_double(obstacle_lat_buffer);
// DECLARE_double(obstacle_lon_start_buffer);
// DECLARE_double(obstacle_lon_end_buffer);
// DECLARE_double(static_obstacle_speed_threshold);
// DECLARE_double(lane_borrow_max_speed);
// DECLARE_int32(long_term_blocking_obstacle_cycle_threshold);

// DECLARE_string(destination_obstacle_id);
// DECLARE_double(destination_check_distance);

// DECLARE_string(iov_stop_work_obstacle_id);
// DECLARE_double(iov_stop_work_check_distance);

// DECLARE_string(emergency_pull_over_obstacle_id);
// DECLARE_string(sweep_working_destination_id);
// DECLARE_string(pre_add_water_stop_obstacle_id);
// DECLARE_string(stop_for_adding_water_obstacle_id);
// DECLARE_string(finished_add_water_obstacle_id);
// DECLARE_string(pre_dump_obstacle_id);
// DECLARE_string(stop_for_dumping_obstacle_id);
// DECLARE_string(finished_dump_obstacle_id);

// DECLARE_double(virtual_stop_wall_length);
// DECLARE_double(virtual_stop_wall_height);

// DECLARE_double(prediction_total_time);
// DECLARE_bool(align_prediction_time);
// DECLARE_bool(align_localization_time);
// DECLARE_int32(trajectory_point_num_for_debug);
// DECLARE_double(lane_change_prepare_length);
// DECLARE_double(min_lane_change_prepare_length);
// DECLARE_double(allowed_lane_change_failure_time);
// DECLARE_bool(enable_smarter_lane_change);

// DECLARE_double(turn_signal_distance);

// DECLARE_double(sweep_safe_buffer);

// // QpSt optimizer
// DECLARE_double(slowdown_profile_deceleration);

// DECLARE_bool(enable_sqp_solver);

// /// thread pool
// DECLARE_bool(use_multi_thread_to_add_obstacles);
// DECLARE_bool(enable_multi_thread_in_dp_st_graph);

// DECLARE_double(numerical_epsilon);
// DECLARE_double(default_cruise_speed);

// DECLARE_double(trajectory_time_resolution);
// DECLARE_double(trajectory_space_resolution);
// DECLARE_double(lateral_acceleration_bound);
// DECLARE_double(speed_lon_decision_horizon);
// DECLARE_uint64(num_velocity_sample);
// DECLARE_bool(enable_backup_trajectory);
// DECLARE_double(backup_trajectory_cost);
// DECLARE_double(min_velocity_sample_gap);
// DECLARE_double(lon_collision_buffer);
// DECLARE_double(lat_collision_buffer);
// DECLARE_uint64(num_sample_follow_per_timestamp);

// DECLARE_bool(lateral_optimization);
// DECLARE_double(weight_lateral_offset);
// DECLARE_double(weight_lateral_derivative);
// DECLARE_double(weight_lateral_second_order_derivative);
// DECLARE_double(weight_lateral_third_order_derivative);
// DECLARE_double(weight_lateral_obstacle_distance);
// DECLARE_double(lateral_third_order_derivative_max);
// DECLARE_double(lateral_derivative_bound_default);

// // Lattice Evaluate Parameters
// DECLARE_double(weight_lon_objective);
// DECLARE_double(weight_lon_jerk);
// DECLARE_double(weight_lon_collision);
// DECLARE_double(weight_lat_offset);
// DECLARE_double(weight_lat_comfort);
// DECLARE_double(weight_centripetal_acceleration);
// DECLARE_double(cost_non_priority_reference_line);
// DECLARE_double(weight_same_side_offset);
// DECLARE_double(weight_opposite_side_offset);
// DECLARE_double(weight_dist_travelled);
// DECLARE_double(weight_target_speed);
// DECLARE_double(lat_offset_bound);
// DECLARE_double(lon_collision_yield_buffer);
// DECLARE_double(lon_collision_overtake_buffer);
// DECLARE_double(lon_collision_cost_std);
// DECLARE_double(default_lon_buffer);
// DECLARE_double(time_min_density);
// DECLARE_double(comfort_acceleration_factor);
// DECLARE_double(comfort_deceleration_factor);
// DECLARE_double(polynomial_minimal_param);
// DECLARE_double(lattice_stop_buffer);
// DECLARE_double(max_s_lateral_optimization);
// DECLARE_double(default_delta_s_lateral_optimization);
// DECLARE_double(bound_buffer);
// DECLARE_double(nudge_buffer);

// DECLARE_double(fallback_total_time);
// DECLARE_double(fallback_time_unit);

// DECLARE_double(speed_bump_speed_limit);
// DECLARE_double(default_city_road_speed_limit);
// DECLARE_double(default_highway_speed_limit);

// // navigation mode
// DECLARE_bool(enable_planning_pad_msg);
// //
// DECLARE_bool(enable_storytelling);
// DECLARE_string(story_cruise_config_file);
// DECLARE_string(story_close_to_ramp_merge_config_file);
// DECLARE_string(story_close_to_ramp_split_config_file);
// DECLARE_string(story_close_to_lane_change_config_file);
// DECLARE_string(story_close_to_speed_limit_change_config_file);
// DECLARE_bool(enable_story_close_to_ramp_merge);
// DECLARE_bool(enable_story_close_to_ramp_split);
// DECLARE_bool(enable_story_close_to_lane_change);
// DECLARE_bool(enable_story_close_to_speed_limit_change);

// // open space planner
// DECLARE_string(planner_open_space_config_filename);
// DECLARE_double(open_space_planning_period);
// DECLARE_double(open_space_prediction_time_horizon);
// DECLARE_bool(enable_perception_obstacles);
// DECLARE_bool(enable_open_space_planner_thread);
// DECLARE_bool(use_dual_variable_warm_start);
// DECLARE_bool(use_gear_shift_trajectory);
// DECLARE_uint64(open_space_trajectory_stitching_preserved_length);
// DECLARE_bool(enable_smoother_failsafe);
// DECLARE_bool(use_s_curve_speed_smooth);
// DECLARE_bool(use_iterative_anchoring_smoother);
// DECLARE_bool(enable_parallel_trajectory_smoothing);

// DECLARE_bool(use_osqp_optimizer_for_reference_line);
// DECLARE_bool(enable_osqp_debug);
// DECLARE_bool(export_chart);
// DECLARE_bool(enable_record_debug);

// DECLARE_double(default_front_clear_distance);

// DECLARE_double(max_trajectory_len);
// DECLARE_bool(enable_rss_fallback);
// DECLARE_bool(enable_rss_info);
// DECLARE_double(rss_max_front_obstacle_distance);

// DECLARE_bool(enable_planning_smoother);
// DECLARE_double(smoother_stop_distance);

// DECLARE_double(side_pass_driving_width_l_buffer);

// DECLARE_bool(enable_parallel_hybrid_a);

// DECLARE_double(open_space_standstill_acceleration);

// DECLARE_bool(enable_dp_reference_speed);

// DECLARE_double(message_latency_threshold);
// DECLARE_bool(enable_lane_change_urgency_checking);
// DECLARE_double(short_path_length_threshold);

// DECLARE_uint64(trajectory_stitching_preserved_length);

// DECLARE_bool(use_st_drivable_boundary);

// DECLARE_bool(use_smoothed_dp_guide_line);

// DECLARE_bool(use_soft_bound_in_nonlinear_speed_opt);

// DECLARE_bool(use_front_axe_center_in_path_planning);

// DECLARE_bool(use_road_boundary_from_map);

// DECLARE_double(vehicle_heading_diff_threshold);

// DECLARE_double(deceleration_ration_of_approaching_crosswalk);

// DECLARE_double(maximum_debouncing_duration_time);

// DECLARE_bool(enable_debounce_path_blocking);

// DECLARE_int32(max_bounce_count);

// DECLARE_string(input_file);
// DECLARE_string(output_file);
// DECLARE_double(smooth_length);
// DECLARE_double(minimum_point_spacing);

// DECLARE_double(backward_min_safe_dist_on_opp_direct);
// DECLARE_double(dist_buff);

// DECLARE_string(planning_data_dir);
// DECLARE_int32(planning_freq);
// DECLARE_int32(learning_data_frame_num_per_file);
// DECLARE_int32(learning_data_obstacle_history_point_cnt);
// DECLARE_bool(enable_binary_learning_data);
// DECLARE_bool(enable_overlap_tag);

// DECLARE_string(planning_source_dirs);
// DECLARE_double(trajectory_delta_t);
// DECLARE_bool(enable_evaluate_obstacle_trajectory);
// DECLARE_bool(enable_manual_mode_trajectory_stitching);
// DECLARE_bool(use_aggressive_strategy_on_traffic_lights);
// DECLARE_double(yellow_light_duration_threshold);
// DECLARE_int32(green_red_count_threshold);

// // the reason of plannning's MainNotReady
// DECLARE_string(planning_not_ready_reason_due_to_localization);
// DECLARE_string(planning_not_ready_reason_due_to_chassis);
// DECLARE_string(planning_not_ready_reason_due_to_map);
// DECLARE_string(planning_not_ready_reason_due_to_relative_map);
// DECLARE_string(planning_not_ready_reason_due_to_routing);
// DECLARE_string(planning_not_ready_reason_due_to_update_vehicleStateProvider);
// DECLARE_string(planning_not_ready_reason_due_to_init_frame);

// DECLARE_double(changelane_observe_speed);

// DECLARE_bool(use_speed_from_map);
// DECLARE_string(torch_ramp_model_cpu_file);
// DECLARE_bool(use_rl_planner);

// // Switches of features designed by Su Junsheng
// DECLARE_bool(use_new_lateral_offset_cost);

// // sweep
// DECLARE_double(sweep_safe_buffer);
// DECLARE_double(sweep_offset_buffer);
// DECLARE_double(sweeping_state_traffic_light_start_distance);
// DECLARE_double(sweeping_state_traffic_light_end_distance);
// DECLARE_double(default_sweep_cruise_speed);
// DECLARE_double(sweep_u_turn_cruise_speed);
// DECLARE_bool(enable_iov);
// DECLARE_bool(enable_pad_terminal);

// // use topic debug part print lattice cost
// DECLARE_bool(use_debug_cost);
// DECLARE_bool(use_debug_acc_jerk);

// DECLARE_double(react_time);

// // trailer related
// DECLARE_bool(use_perception_trailer_theta);
// DECLARE_double(use_perception_trailer_theta_threshold);
// DECLARE_bool(enable_planning_wrt_trailer);

// // active lane change related
// DECLARE_bool(active_lane_change_by_rerouting);
// DECLARE_double(min_length_active_lane_change);
// DECLARE_bool(enable_left_reference_line);
// DECLARE_bool(enable_right_reference_line);
// DECLARE_double(min_valid_length_neighbor_reference_line);

// // used by updateOrderForward in pnc_map
// DECLARE_int32(order_forward_road_segment);

// // adm related
// DECLARE_bool(enable_planning_adm);
// // auto change lane
// DECLARE_double(change_lane_max_distance_to_destination);
// DECLARE_double(long_term_blocking_obstacle_cycle_threhold);
// DECLARE_bool(enable_right_direction_change_lane);
// DECLARE_bool(enable_change_lane_comeback);
// DECLARE_bool(enable_change_lane_start);
// DECLARE_double(min_trigger_auto_lane_change_speed);
// DECLARE_double(adjacent_lane_obstacle_min_distance);
// DECLARE_double(adjacent_lane_obstacle_min_speed);
// DECLARE_double(vechile_front_safe_distance);
// DECLARE_double(vechile_rear_safe_distance);
// DECLARE_double(check_boundary_type_lookforward_distance);
// DECLARE_double(current_lane_obstacle_blocking_min_distance);
// DECLARE_double(current_lane_obstacle_blocking_count);
// DECLARE_bool(enable_side_lane_blocking_judge);
// DECLARE_double(active_change_lane_done_delay_time);
// DECLARE_bool(enable_active_change_lane);
// DECLARE_double(follow_target_obstacle_low_speed_time);
// DECLARE_double(check_blocking_delay_time);
// DECLARE_double(feasible_region_rate);
// DECLARE_bool(enable_feasible_region_rate);
// DECLARE_double(low_speed_obstacle_rate);
// DECLARE_bool(enable_auto_change_lane);
// DECLARE_double(calculate_ttc_time);
// DECLARE_double(target_lane_front_ttc_time);
// DECLARE_double(target_lane_rear_ttc_time);

// DECLARE_bool(enable_planning_L3);
// DECLARE_bool(enable_planning_L3P);
// DECLARE_bool(enable_planning_L4);

// // cruising routing related
// DECLARE_double(min_cruising_routing_length);

// DECLARE_double(time_for_turn_signal_in_advance);
// DECLARE_double(distance_for_lane_change_after_junction);

// DECLARE_bool(use_reverse_speed);
