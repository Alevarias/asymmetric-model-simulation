import opensim as osim

def moco_predict_tracking(
    torque_model_path="models/armed_torque_model.osim",
    muscle_solution_path="models/sim_S00_N50.sto",
    output_solution_path="solutions/tracking_solution.sto", 
    mesh_intervals = 20
):

    # Setting weight for tracking
    tracking_weight = 10.0

    # Decides when we stop
    conv_tolerance = 1e-2
    con_tolerance = 1e-2

    # Builds the tracking problem
    track = osim.MocoTrack()
    track.setName("armed_walking_track")

    track.setModel(osim.ModelProcessor(torque_model_path))

    track.setStatesReference(osim.TableProcessor(muscle_solution_path))
    track.set_states_global_tracking_weight(tracking_weight)
    track.set_allow_unused_references(True)

    track.set_track_reference_position_derivatives(False)

    # Sets up solver
    study = track.initialize()
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())

    solver.set_multibody_dynamics_mode("implicit")

    solver.set_num_mesh_intervals(mesh_intervals)
    solver.set_optim_convergence_tolerance(conv_tolerance)
    solver.set_optim_constraint_tolerance(con_tolerance)

    # Warm Start
    solver.resetProblem(study.updProblem())
    guess = solver.createGuess("bounds")

    muscle_traj = osim.MocoTrajectory(muscle_solution_path)
    muscle_traj.resampleWithNumTimes(guess.getNumTimes())

    muscle_state_names = set(str(n) for n in muscle_traj.getStateNames())

    copied = 0
    skipped = 0
    for state_name in guess.getStateNames():
        name = str(state_name)
        if name in muscle_state_names:
            state_vec = muscle_traj.getState(name)
            guess.setState(name, [state_vec[i] for i in range(state_vec.size())])
            copied += 1
        else:
            skipped += 1
    
    # print(f"Warm start: {copied} states copied from muscle solution, {skipped} initialised from bounds")

    solver.setGuess(guess)

    print("Solving tracking problem...")
    solution = study.solve()

    # Error Handling
    if solution.success():
        solution.write(output_solution_path)
        print("Simulation Finished")
        print(f"Converged in {solution.getNumIterations()} iterations.")
        print(f"Final objective: {solution.getObjective():.6f}")
        print(f"Solution saved to: {output_solution_path}")

    else:
        solution.unseal()
        failed_path = output_solution_path.replace(".sto", "_FAILED.sto")
        solution.write(failed_path)
        print("DIDN'T CONVERGE")
        print(f"Final objective: {solution.getObjective():.6f}")
        print(f"Failed solution saved to: {failed_path}")

    return

def main():
    moco_predict_tracking(
        "models/armed_torque_model.osim",
        "models/sim_S00_N50.sto",
        "solutions/tracking_solution.sto", 
        mesh_intervals=20
    )

    return

if __name__ == "__main__":
    main()
