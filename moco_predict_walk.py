import math
import opensim as osim
import numpy as np

def deg(d): return math.radians(d)

def moco_predict_walk(
    torque_model_path="models/armed_torque_model.osim",
    tracking_solution_path="solutions_guarded/predictive_solution_good_walk_361_5x.sto",
    output_solution_path="solutions_guarded/predictive_solution.sto", 
    mesh_intervals = 100
):
    
    # Decides when to stop
    conv_tolerance = 1e-3
    con_tolerance = 1e-3
    desired_speed = 1.2

    # Sets Moco problem
    study = osim.MocoStudy()
    problem = study.updProblem()
    problem.setModel(osim.Model(torque_model_path))
    problem.setTimeBounds(0, 10)

    # Sets the pelvis bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value", osim.MocoBounds(0.75, 1.0))
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tz/value", osim.MocoBounds(-0.3, 0.3))


    # Sets the effort goal so we optimize for less effort
    effort = osim.MocoControlGoal("effort", 1.0)
    effort.setExponent(2)
    effort.setDivideByDisplacement(True)
    problem.addGoal(effort)

    # Constrains to holding sword and shield out
    shield_pose = {
        "/jointset/shoulder_l/shoulder_flexion_l/value": deg(37),
        "/jointset/shoulder_l/shoulder_adduction_l/value": deg(-30),
        "/jointset/shoulder_l/shoulder_rotation_l/value": deg(30),
        "/jointset/elbow_l/elbow_flexion_l/value": deg(105),
    }
    sword_pose = {
        "/jointset/shoulder_r/shoulder_flexion_r/value": deg(-32),
        "/jointset/shoulder_r/shoulder_adduction_r/value": deg(-24),
        "/jointset/shoulder_r/shoulder_rotation_r/value": deg(8),
        "/jointset/elbow_r/elbow_flexion_r/value": deg(67),
    }
    pose_tolerance = 0.3

    # Sets the constraint
    all_poses = {**shield_pose, **sword_pose}
    for coord_path, target in all_poses.items():
        problem.setStateInfo(
            coord_path,
            osim.MocoBounds(target - pose_tolerance, target + pose_tolerance)
        )


    time_vec = np.linspace(0, 10, 50)
    coord_names = list(all_poses.keys())
    target_values = list(all_poses.values())

    table = osim.TimeSeriesTable()
    table.addTableMetaDataString("inDegrees", "no")
    table.setColumnLabels(coord_names)
    for t in time_vec:
        row = osim.RowVector(target_values)
        table.appendRow(t, row)

    hold_goal = osim.MocoStateTrackingGoal("weapon_hold", 1.0)
    hold_goal.setReference(osim.TableProcessor(table))
    hold_goal.setAllowUnusedReferences(True)

    weight_set = osim.MocoWeightSet()
    for coord in coord_names:
        weight_set.cloneAndAppend(osim.MocoWeight(coord, 1.0))
    hold_goal.setWeightSet(weight_set)

    problem.addGoal(hold_goal)

    # Constrains against leg crossing to prevent spippin with legs
    distance_constraint = osim.MocoFrameDistanceConstraint()
    distance_constraint.setName("no_leg_crossing")
    distance_constraint.addFramePair(
        osim.MocoFrameDistanceConstraintPair(
            "/bodyset/calcn_r",
            "/bodyset/calcn_l",
            0.05,
            math.inf
        )
    )
    problem.addPathConstraint(distance_constraint)

    # Sets the speed goal
    speed = osim.MocoAverageSpeedGoal("speed")
    speed.set_desired_average_speed(desired_speed)
    problem.addGoal(speed)

    # Sets the periodicity goal to ensure start and end are the same
    periodic = osim.MocoPeriodicityGoal("periodicity")
    periodic.setMode("endpoint_constraint")

    def add_self(path):
        periodic.addStatePair(osim.MocoPeriodicityGoalPair(path, path))

    # Adding more coords
    add_self("/jointset/groundPelvis/pelvis_tx/speed")
    add_self("/jointset/groundPelvis/pelvis_tz/value")
    add_self("/jointset/groundPelvis/pelvis_tz/speed")

    for coord in ["pelvis_ty", "pelvis_tilt", "pelvis_list", "pelvis_rotation"]:
        add_self(f"/jointset/groundPelvis/{coord}/value")
        add_self(f"/jointset/groundPelvis/{coord}/speed")

    for coord in [
        "/jointset/hip_r/hip_flexion_r",
        "/jointset/hip_r/hip_adduction_r",
        "/jointset/hip_r/hip_rotation_r",
        "/jointset/knee_r/knee_angle_r",
        "/jointset/ankle_r/ankle_angle_r",
        "/jointset/subtalar_r/subtalar_angle_r",
        "/jointset/mtp_r/mtp_angle_r",
    ]:
        add_self(f"{coord}/value")
        add_self(f"{coord}/speed")

    for coord in [
        "/jointset/hip_l/hip_flexion_l",
        "/jointset/hip_l/hip_adduction_l",
        "/jointset/hip_l/hip_rotation_l",
        "/jointset/knee_l/knee_angle_l",
        "/jointset/ankle_l/ankle_angle_l",
        "/jointset/subtalar_l/subtalar_angle_l",
        "/jointset/mtp_l/mtp_angle_l",
    ]:
        add_self(f"{coord}/value")
        add_self(f"{coord}/speed")

    for coord in ["lumbar_ext", "lumbar_bend", "lumbar_rota"]:
        add_self(f"/jointset/lumbar/{coord}/value")
        add_self(f"/jointset/lumbar/{coord}/speed")

    for coord in [
        "/jointset/shoulder_r/shoulder_flexion_r",
        "/jointset/shoulder_r/shoulder_adduction_r",
        "/jointset/shoulder_r/shoulder_rotation_r",
        "/jointset/elbow_r/elbow_flexion_r",
        "/jointset/shoulder_l/shoulder_flexion_l",
        "/jointset/shoulder_l/shoulder_adduction_l",
        "/jointset/shoulder_l/shoulder_rotation_l",
        "/jointset/elbow_l/elbow_flexion_l",
    ]:
        add_self(f"{coord}/value")
        add_self(f"{coord}/speed")

    problem.addGoal(periodic)

    # Sets up solver
    solver = study.initCasADiSolver()
    solver.set_multibody_dynamics_mode("implicit")
    solver.set_num_mesh_intervals(mesh_intervals)
    solver.set_optim_convergence_tolerance(conv_tolerance)
    solver.set_optim_constraint_tolerance(con_tolerance)

    # Warm start
    solver.resetProblem(study.updProblem())
    guess = solver.createGuess("bounds")

    tracking_traj = osim.MocoTrajectory(tracking_solution_path)
    tracking_traj.resampleWithNumTimes(guess.getNumTimes())

    tracking_state_names = set(str(n) for n in tracking_traj.getStateNames())
    copied = 0
    for state_name in guess.getStateNames():
        name = str(state_name)
        if name in tracking_state_names:
            state_vec = tracking_traj.getState(name)
            guess.setState(name, [state_vec[i] for i in range(state_vec.size())])
            copied += 1

    tracking_control_names = set(str(n) for n in tracking_traj.getControlNames())
    controls_copied = 0
    for control_name in guess.getControlNames():
        name = str(control_name)
        if name in tracking_control_names:
            ctrl_vec = tracking_traj.getControl(name)
            guess.setControl(name, [ctrl_vec[i] for i in range(ctrl_vec.size())])
            controls_copied += 1

    print(f"Warm start: {copied} states and {controls_copied} controls copied from tracking solution")

    solver.setGuess(guess)

    print("Solving predictive problem...")
    solution = study.solve()

    # Error handling
    if solution.success():
        solution.write(output_solution_path)
        print(f"Predictive solve complete.")
        print(f"Converged in {solution.getNumIterations()} iterations.")
        print(f"Final objective: {solution.getObjective():.6f}")
        print(f"Solution saved to: {output_solution_path}")
    else:
        solution.unseal()
        failed_path = output_solution_path.replace(".sto", "_FAILED.sto")
        solution.write(failed_path)
        print(f"DIDN'T CONVERGE")
        print(f"Final objective: {solution.getObjective():.6f}")
        print(f"Failed solution saved to: {failed_path}")

    return

def main():
    moco_predict_walk(
        "models/armed_torque_model.osim",
        "solutions_guarded/predictive_solution_good_walk_361_5x.sto",
        "solutions_guarded/predictive_solution.sto", 
        mesh_intervals = 100
    )

    return

if __name__ == "__main__":
    main()