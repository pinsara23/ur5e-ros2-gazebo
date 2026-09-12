from pathlib import Path
import time
import copy

from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy
from moveit.planning import MoveItPy, PlanRequestParameters


moveit_config = (
    MoveItConfigsBuilder(
        robot_name="ur",
        package_name="ur_moveit_config"
    )
    .robot_description_semantic(
        Path("srdf") / "ur.srdf.xacro",
        {"name": "ur5e"}
    )
    .planning_pipelines(
        pipelines=["ompl"],
        default_planning_pipeline="ompl"
    )
    .to_moveit_configs()
)


config_dict = moveit_config.to_dict()

config_dict["planning_pipelines"] = {
    "pipeline_names": ["ompl"]
}

# Gazebo uses simulated ROS time
config_dict["use_sim_time"] = True


moveit = MoveItPy(
    node_name="moveit_state_test",
    config_dict=config_dict
)


ur5e = moveit.get_planning_component("ur_manipulator")

time.sleep(1.0)

ur5e.set_start_state_to_current_state()

current_state = ur5e.get_start_state()

print("\n========== CURRENT MOVEIT START STATE ==========")
print(current_state)

joint_positions = current_state.get_joint_group_positions(
    "ur_manipulator"
)

print("\n========== CURRENT JOINT POSITIONS ==========")
print(joint_positions)



goal_positions = [
    0.3,
    -1.3,
    -0.3,
    -1.4,
    0.2,
    0.0
]

print("\n========== GOAL JOINT POSITIONS ==========")
print(goal_positions)

goal_state = copy.copy(current_state)

goal_state.set_joint_group_positions(
    "ur_manipulator",
    goal_positions
)

ur5e.set_goal_state(robot_state=goal_state)

print("\n========== PLANNING ==========")

plan_parameters = PlanRequestParameters(
    moveit,
    "plan_request_params"
)

plan_parameters.planning_pipeline = "ompl"
plan_parameters.planner_id = "RRTConnectkConfigDefault"
plan_parameters.planning_time = 5.0
plan_parameters.planning_attempts = 1
plan_parameters.max_velocity_scaling_factor = 0.5
plan_parameters.max_acceleration_scaling_factor = 0.5

plan_result = ur5e.plan(
    single_plan_parameters=plan_parameters
)

if plan_result:

    print("Planning succeeded!")

    trajectory = plan_result.trajectory

    print("\n========== TRAJECTORY INFORMATION ==========")
    print("Number of waypoints:", len(trajectory))
    print("Total trajectory duration:", trajectory.duration)

    durations = trajectory.get_waypoint_durations()
    print("\nWaypoint durations:")
    print(durations)

    print("\n========== TRAJECTORY WAYPOINTS ==========")

    for i in range(len(trajectory)):

        waypoint = trajectory[i]

        positions = waypoint.get_joint_group_positions(
            "ur_manipulator"
        )

        print(f"\nWaypoint {i}:")
        print(positions)

    print("\n========== EXECUTING TRAJECTORY ==========")

    execution_result = moveit.execute(
        trajectory,
        controllers=[]
    )

    print("\n========== EXECUTION RESULT ==========")
    print(execution_result)

else:
    print("Planning failed!")

print("\n========== GOAL STATE SET IN MOVEIT ==========")
print(
    goal_state.get_joint_group_positions(
        "ur_manipulator"
    )
)