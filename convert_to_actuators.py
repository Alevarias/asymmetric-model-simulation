import opensim as osim

def convert_to_actuators(input_model_path="models/armed_model.osim", output_model_path="models/armed_torque_model.osim"):

    # Default and general optimal forces
    default_optimal_force = 300
    pelvis_optimal_force = 1000
    weapon_optimal_force = 400

    print("Building the torque-driven model")

    # Builds the model, removing muscles, and adding actuators
    model_processor = osim.ModelProcessor(input_model_path)
    model_processor.append(osim.ModOpRemoveMuscles())
    model_processor.append(osim.ModOpAddReserves(default_optimal_force))

    model = model_processor.process()
    model.setName("armed_model_torque")
    model.initSystem()

    print("Adjusting the optimal forces")

    # Adds the forces for the joints
    actuator_set = model.updActuators()
    for i in range(actuator_set.getSize()):
        actu = actuator_set.get(i)
        name = actu.getName()
        coord_actu = osim.CoordinateActuator.safeDownCast(actu)
        if coord_actu is None:
            continue

        if any(sub in name for sub in ("pelvis_tx", "pelvis_ty", "pelvis_tz")):
            coord_actu.setOptimalForce(pelvis_optimal_force)
        elif any(sub in name for sub in ("shoulder", "elbow")):
            coord_actu.setOptimalForce(weapon_optimal_force)

    print(f"Final actuator count: {model.getActuators().getSize()}")
    print("Expected: 31 (33 coordinates minus 2 patellofemoral beta constrained coords)")
    print("Actuator list:")

    actuators = model.getActuators()
    for i in range(actuators.getSize()):
        actu = actuators.get(i)
        coord_actu = osim.CoordinateActuator.safeDownCast(actu)
        force_str = f"{coord_actu.getOptimalForce()}" if coord_actu else "N/A"
        print(f"{actu.getName():70s}, optimal_force = {force_str}")

    # Saves the model
    model.printToXML(output_model_path)
    print(f"Saved torque-driven model to: {output_model_path}")

    return

def main():
    convert_to_actuators(
        "models/armed_model.osim",
        "models/armed_torque_model.osim"
    )

    return

if __name__ == "__main__":
    main()