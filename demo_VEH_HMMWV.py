# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simone Benatti, Radu Serban
# =============================================================================
#
# Main driver function for the HMMWV full model (headless, no visualization).
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import errno
import os
import math as m


#// =============================================================================

def main():
    # Create systems

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain

    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat,
                             chrono.CSYSNORM,
                             terrainLength, terrainWidth)
    terrain.Initialize()

    # Create the driver system (inputs are set from the throttle/brake profile in the loop)

    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Initialize output

    try:
        os.makedirs(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
           print("Error creating output directory " )

    # Output file: simulation time and x y position of the vehicle reference frame
    out_file = open(out_dir + "output.txt", "w")
    out_file.write("time x y\n")

    # Log file: the periodic system information (markers, tire forces)
    log_file = open(out_dir + "log.txt", "w")

    # Simulation loop

    # Number of simulation steps between miscellaneous events
    debug_steps = m.ceil(debug_step_size / step_size)
    output_steps = round(output_step_size / step_size)

    # Initialize simulation frame counter
    step_number = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while True:
        time = hmmwv.GetSystem().GetChTime()

        #End simulation
        if (time >= t_end):
            break

        #Debug logging
        if (debug_output and step_number % debug_steps == 0) :
            print("\n\n============ System Information ============\n", file=log_file)
            print(f"Time = {time:.3f}\n", file=log_file)

            marker_driver = hmmwv.GetChassis().GetMarkers()[0].GetAbsCoordsys().pos
            marker_com = hmmwv.GetChassis().GetMarkers()[1].GetAbsCoordsys().pos
            print( "\nMarkers\n", file=log_file)
            print( "  Driver loc:      " , marker_driver.x , " " , marker_driver.y , " " , marker_driver.z, file=log_file)
            print( "  Chassis COM loc: " , marker_com.x, " ", marker_com.y, " ",marker_com.z, file=log_file)

            print("\nTire forces\n", file=log_file)
            tf_FL = hmmwv.GetVehicle().GetTire(0, veh.LEFT).ReportTireForce(terrain)
            tf_FR = hmmwv.GetVehicle().GetTire(0, veh.RIGHT).ReportTireForce(terrain)
            tf_RL = hmmwv.GetVehicle().GetTire(1, veh.LEFT).ReportTireForce(terrain)
            tf_RR = hmmwv.GetVehicle().GetTire(1, veh.RIGHT).ReportTireForce(terrain)
            print("   Front left:  ", tf_FL.force.x, " ", tf_FL.force.y, " ", tf_FL.force.z, file=log_file)
            print("   Front right: ", tf_FR.force.x, " ", tf_FR.force.y, " ", tf_FR.force.z, file=log_file)
            print("   Rear left:   ", tf_RL.force.x, " ", tf_RL.force.y, " ", tf_RL.force.z, file=log_file)
            print("   Rear right:  ", tf_RR.force.x, " ", tf_RR.force.y, " ", tf_RR.force.z, file=log_file)

        # Output vehicle position
        if (step_number % output_steps == 0) :
            pos = hmmwv.GetVehicle().GetPos()
            out_file.write(f"{time:.1f} {pos.x:.4f} {pos.y:.4f}\n")

        # Throttle/brake profile: settle, accelerate, then brake to a stop
        if (time < t_throttle_start):
            driver.SetThrottle(0)
            driver.SetBraking(0)
        elif (time < t_brake_start):
            driver.SetThrottle(throttle)
            driver.SetBraking(0)
        else:
            driver.SetThrottle(0)
            driver.SetBraking(braking)

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)

        # Increment frame number
        step_number += 1

    out_file.close()
    log_file.close()

    return 0


# Point Chrono at the data files shipped with the conda package
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of powertrain models (SHAFTS, SIMPLE)
engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineTypeWV_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height (FLAT terrain only)
terrainLength = 200.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

# Simulation end time
t_end = 10

# Driver profile: settle on the ground, full throttle, then brake to a stop
t_throttle_start = 1.0
t_brake_start = 7.0
throttle = 1.0
braking = 0.6

# Set output root directory
chrono.SetChronoOutputPath("DEMO_OUTPUT/")

# Output directories
out_dir = chrono.GetChronoOutputPath() + "HMMWV/"

# Debug logging
debug_output = True
debug_step_size = 1.0 / 1  # FPS = 1

# Time interval between two output.txt entries
output_step_size = 0.1

main()
