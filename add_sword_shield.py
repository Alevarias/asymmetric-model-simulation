import opensim as osim
import math

def deg(num):
    return math.radians(num)

def add_sword_shield(input_model_path = "models/model_S00.osim", output_model_path = "models/armed_model.osim"):

    model = osim.Model(input_model_path)

    # Sword values
    sword_mass = 1.1 # Avg arming sword is 1.0 to 1.3kg
    sword_length = 1.0 # Avg arming sword is 0.9 to 1m 
    sword_radius = 0.01

    # Sword inertia
    sword_ax_inertia = 0.5 * sword_mass * sword_radius**2
    sword_lat_inertia = (1/12) * sword_mass * sword_length**2

    # Combined sword inertia
    sword_inertia = osim.Inertia(
        sword_lat_inertia,
        sword_ax_inertia,
        sword_lat_inertia
    )

    # Actual physics body
    sword_body = osim.Body(
        "sword",
        sword_mass,
        osim.Vec3(0, 0, 0),
        sword_inertia
    )

    # Just visualization
    sword_geom = osim.Brick(osim.Vec3(
        0.01,
        sword_length / 2,
        0.01
    ))
    sword_body.attachGeometry(sword_geom)

    # Attaches sword to right hand
    model.addBody(sword_body)
    sword_offset = osim.Vec3(0.03, -0.07, -0.1)
    sword_joint = osim.WeldJoint(
        "sword_weld",
        model.getBodySet().get("hand_r"),
        sword_offset,
        osim.Vec3(0, 0, 0),
        sword_body,
        osim.Vec3(0, sword_length / 2, 0),
        osim.Vec3(deg(90), 0, 0)
    )
    model.addJoint(sword_joint)


    # Shield values
    shield_mass = 3.7
    shield_width = 0.55
    shield_height = 0.70
    shield_depth = 0.008

    # Shield intertia 
    shield_xx_inertia = (0.80 * (1/12) * shield_mass * shield_height**2) + ((1/12) * shield_mass * shield_depth**2)
    shield_yy_inertia = (0.65 * (1/12) * shield_mass * shield_width**2) + ((1/12) * shield_mass * shield_depth**2)
    shield_zz_inertia = (0.65 * (1/12) * shield_mass * shield_width**2) + (0.80 * (1/12) * shield_mass * shield_height**2)
    shield_inertia = osim.Inertia(
        shield_xx_inertia, 
        shield_yy_inertia, 
        shield_zz_inertia
    )

    # Creating and attaching shield
    com_offset_y = shield_height * 0.10
    shield_body = osim.Body(
        "shield",
        shield_mass,
        osim.Vec3(0, com_offset_y, 0),
        shield_inertia
    )

    # Makes shield visualization
    shield_geom = osim.Brick(osim.Vec3(
        shield_width / 2,
        shield_height / 2,
        shield_depth / 2
    ))
    shield_body.attachGeometry(shield_geom)

    # Attaches the shield to the left radius
    model.addBody(shield_body)
    shield_offset_in_forearm = osim.Vec3(-0.05, -0.15, 0)
    shield_joint = osim.WeldJoint(
        "shield_weld",
        model.getBodySet().get("radius_l"),
        shield_offset_in_forearm,
        osim.Vec3(0, 0, 0),
        shield_body,
        osim.Vec3(0, 0, 0),
        osim.Vec3(deg(90), deg(45), deg(90))
    )
    model.addJoint(shield_joint)

    # Saves the new model
    model.initSystem()
    model.printToXML(output_model_path)
    print(f"Saved: {output_model_path}")

    return

def main():
    add_sword_shield(
        "models/model_S00.osim", 
        "models/armed_model.osim"
    )

    return

if __name__ == "__main__":
    main()
