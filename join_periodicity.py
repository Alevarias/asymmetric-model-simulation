import numpy as np
import opensim as osim

def join_periodicity(input_sim_path, output_sim_path, num_repeats=10):

    source_traj = osim.MocoTrajectory(input_sim_path)

    state_names = [str(n) for n in source_traj.getStateNames()]
    control_names = [str(n) for n in source_traj.getControlNames()]
    mult_names = [str(n) for n in source_traj.getMultiplierNames()]
    param_names = [str(n) for n in source_traj.getParameterNames()]

    time_vec = source_traj.getTime()
    times = np.array([time_vec[i] for i in range(time_vec.size())])
    cycle_duration = float(times[-1] - times[0])
    n_rows = len(times)

    state_data = np.zeros((n_rows, len(state_names)))
    for ci, name in enumerate(state_names):
        v = source_traj.getState(name)
        state_data[:, ci] = [v[i] for i in range(v.size())]

    control_data = np.zeros((n_rows, len(control_names)))
    for ci, name in enumerate(control_names):
        v = source_traj.getControl(name)
        control_data[:, ci] = [v[i] for i in range(v.size())]

    mult_data = np.zeros((n_rows, len(mult_names)))
    for ci, name in enumerate(mult_names):
        v = source_traj.getMultiplier(name)
        mult_data[:, ci] = [v[i] for i in range(v.size())]

    param_data = np.array([source_traj.getParameter(name) for name in param_names]) \
        if param_names else np.zeros(0)

    pelvis_tx_path = "/jointset/groundPelvis/pelvis_tx/value"
    tx_index = state_names.index(pelvis_tx_path) if pelvis_tx_path in state_names else None
    stride_length = float(state_data[-1, tx_index] - state_data[0, tx_index]) \
        if tx_index is not None else 0.0

    clip_times, clip_states, clip_controls, clip_mults = [], [], [], []
    for i in range(num_repeats):
        for j in range(0 if i == 0 else 1, n_rows):
            clip_times.append(float(times[j]) + i * cycle_duration)
            state_row = state_data[j].copy()
            if tx_index is not None:
                state_row[tx_index] += i * stride_length
            clip_states.append(state_row)
            clip_controls.append(control_data[j].copy())
            clip_mults.append(mult_data[j].copy())

    n_clip_rows = len(clip_times)

    time_simtk = osim.Vector(clip_times)

    states_mat = osim.Matrix(n_clip_rows, len(state_names))
    for ri in range(n_clip_rows):
        for ci in range(len(state_names)):
            states_mat.set(ri, ci, float(clip_states[ri][ci]))

    controls_mat = osim.Matrix(n_clip_rows, len(control_names))
    for ri in range(n_clip_rows):
        for ci in range(len(control_names)):
            controls_mat.set(ri, ci, float(clip_controls[ri][ci]))

    mults_mat = osim.Matrix(n_clip_rows, len(mult_names))
    for ri in range(n_clip_rows):
        for ci in range(len(mult_names)):
            mults_mat.set(ri, ci, float(clip_mults[ri][ci]))

    params_row_vec = osim.RowVector(param_data.tolist())

    clip_traj = osim.MocoTrajectory(
        time_simtk, state_names, control_names, mult_names, param_names,
        states_mat, controls_mat, mults_mat, params_row_vec
    )
    clip_traj.write(output_sim_path)

    total_duration = clip_times[-1] - clip_times[0]
    print(f"Generated {output_sim_path} ({total_duration:.2f}s)")

def main():
    join_periodicity(
        "solutions_guarded/predictive_solution_good_walk_361.sto",
        "solutions_guarded/predictive_solution_good_walk_361_5x.sto",
        num_repeats=5,
    )

    return

if __name__ == "__main__":
    main()
