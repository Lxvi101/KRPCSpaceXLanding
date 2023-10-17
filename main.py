import krpc
from apiv6 import FlightPhase, magnitude
import apiv6
import time

INITIAL_PITCH = 95
INITIAL_OVERSHOOT = 2000
FINAL_OVERSHOOT = 400
ROCKET_HEIGHT_OFFSET = 14
LANDING_HEIGHT_SAFETY = 3

logger = apiv6.Logger()
flightPhase = FlightPhase.IDLE

connection = krpc.connect()
logger.log_strict("Rocket connected.")

#region basic functions
transform_position = connection.space_center.transform_position

vessel = connection.space_center.active_vessel
orbit = vessel.orbit
earth = vessel.orbit.body
control = vessel.control
#endregion basic functions

targetManager = apiv6.TargetManager(connection)
targetManager.target(-0.284387007317676, -70.7407433766901)
starting_ref_frame = targetManager.define_start_pad_ref_frame()
landing_ref_frame = targetManager.define_landing_pad_ref_frame()
pointing_ref_frame = targetManager.get_pointing_ref_frame()

impactManager = apiv6.ImpactManager(connection)

suicideBurnManager = apiv6.SuicideBurnManager(connection)

vessel.auto_pilot.reference_frame = landing_ref_frame
vessel.auto_pilot.auto_tune = True
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 0)

# initial_distance = magnitude(vessel.position(landing_ref_frame))
start_lat_lon = (vessel.flight().latitude, vessel.flight().longitude)
initial_distance = apiv6.haversine_distance(start_lat_lon[0], start_lat_lon[1], targetManager.target_lat_lon[0], targetManager.target_lat_lon[1], earth.equatorial_radius)
logger.log_strict(f"Telemetry set. Distance to target: {initial_distance}")

control.throttle = 1
control.activate_next_stage()
logger.log_strict("Lift-off.")
flightPhase = FlightPhase.START

time.sleep(.5)

control.gear = False
logger.log_strict("Gear retracted.")

time.sleep(1)

initial_heading = targetManager.calculate_required_heading()
initial_pitch = INITIAL_PITCH
vessel.auto_pilot.target_pitch_and_heading(initial_pitch, initial_heading)
logger.log_strict("Autopilot engaged.")
logger.log_strict(f"Initial target heading set to {initial_heading}.")

initial_distance += INITIAL_OVERSHOOT
with connection.stream(getattr, vessel.flight(), 'surface_altitude') as alt:
    while True:
        impact_position = impactManager.calculate_impact(pointing_ref_frame)
        target_position = targetManager.get_target_position(pointing_ref_frame)

        # difference = (-(target_position[0] - impact_position[0])) + INITIAL_OVERSHOOT
        difference = apiv6.haversine_distance(start_lat_lon[0], start_lat_lon[1], impactManager.impact_lat_lon[0], impactManager.impact_lat_lon[1], earth.equatorial_radius)
        logger.log(f"Altitude: {alt()}; Impact Difference: {difference}")

        progress_percent = (difference/initial_distance) * 100
        if progress_percent > 85 and control.throttle != 0.2:
            control.throttle = 0.2
        elif progress_percent > 75 and control.throttle != 0.5:
            control.throttle = 0.7

        if alt() > 40000:
            vessel.auto_pilot.target_pitch = INITIAL_PITCH + 15
        if alt() > 30000:
            vessel.auto_pilot.target_pitch = INITIAL_PITCH + 10
        elif alt() > 10000:
            vessel.auto_pilot.target_pitch = INITIAL_PITCH + 5

        if(0 > initial_distance - difference):
            control.throttle = 0
            flightPhase = FlightPhase.FLIGHT
            logger.log_strict("Cutting throtte. Entering Flightphase FLIGHT.")
            break

control.rcs = True

with connection.stream(getattr, vessel.flight(), 'surface_altitude') as alt:
    with connection.stream(getattr, vessel.flight(landing_ref_frame), 'velocity') as velVec:
        while True:
                vel_dir = apiv6.normalize(velVec())
                vel_dir_2d = (0, vel_dir[1], vel_dir[2])
                impact_position_error_3d = apiv6.vec_sub(impactManager.calculate_impact(landing_ref_frame), apiv6.scalar_mult(vel_dir_2d, FINAL_OVERSHOOT))
                impact_position_error = (0, impact_position_error_3d[1], impact_position_error_3d[2])

                required_pitch = 105
                calculated_heading = targetManager.calculate_error_correction_heading(impact_position_error)

                vessel.auto_pilot.target_pitch_and_heading(required_pitch, calculated_heading)

                match flightPhase:
                    case FlightPhase.FLIGHT:
                        if alt() / orbit.apoapsis_altitude > 0.98:
                            flightPhase = FlightPhase.BOOSTBACK
                            logger.log_strict(f"Calculated impact at with error {magnitude(impact_position_error)}")
                            logger.log_strict("Initiating Boostback Maneuver.")
                        continue

                    case FlightPhase.BOOSTBACK:
                        if alt() > 20000:
                            if magnitude(impact_position_error) > 20:

                                logger.log(f"Correcting. Error: {magnitude(impactManager.calculate_impact(landing_ref_frame))} ; Calculated Heading: {calculated_heading}")

                                if magnitude(impact_position_error) < 200:
                                    control.throttle = .1
                                elif magnitude(impact_position_error) < 1000:
                                    control.throttle = .15
                                else:
                                    control.throttle = .2
                            else:
                                control.throttle = 0
                                logger.log_strict("Boostback Maneuver finished.")

                                flightPhase = FlightPhase.RETURN
                                break
                        else:
                            control.throttle = 0
                            logger.log_strict("Boostback Maneuver failed.")

                            flightPhase = FlightPhase.RETURN
                            break
                        continue


control.brakes = True
vessel.auto_pilot.reference_frame = landing_ref_frame

with connection.stream(getattr, vessel.flight(), "surface_altitude") as altitude:
    with connection.stream(getattr, vessel.flight(landing_ref_frame), 'velocity') as velVec:
        while True:
            v = velVec()[0]
            h = altitude() - (ROCKET_HEIGHT_OFFSET + LANDING_HEIGHT_SAFETY)

            suicideBurnManager.update_calculations(h, v)

            impact_position_error_3d = impactManager.calculate_impact(landing_ref_frame)
            impact_position_error = (0, impact_position_error_3d[1], impact_position_error_3d[2])

            calculated_heading = targetManager.calculate_error_correction_heading(impact_position_error)

            match flightPhase:
                case FlightPhase.RETURN:
                    logger.log(f"Altitude: {h} ; Throttle: {suicideBurnManager.throttle_targeted} ; Impact Error: {magnitude(impact_position_error)}")

                    if h < 5000 and h > 1000 and magnitude(impact_position_error) > 10 :
                        required_pitch = 20
                        vessel.auto_pilot.target_pitch_and_heading(required_pitch, calculated_heading)
                        control.throttle = 0.2
                        control.rcs = True
                    else:
                        control.throttle = 0
                        control.rcs = False
                        required_pitch = 20
                        vessel.auto_pilot.target_pitch_and_heading(required_pitch, calculated_heading)

                    if suicideBurnManager.throttle_targeted > 0.85 and h < 4500:
                        flightPhase = FlightPhase.BURN 
                        print("Landing burn initiated.")
                        logger.log("Throttling for Suicide Burn.")
                    continue
                case FlightPhase.BURN:
                    if suicideBurnManager.throttle_targeted < 0.5:
                                control.throttle = 0
                    if suicideBurnManager.throttle_targeted > 0.8:
                            control.throttle = suicideBurnManager.throttle_targeted

                    logger.log(f"Altitude: {round(h)} ; Throttle: {suicideBurnManager.throttle_targeted} ; Impact Error: {magnitude(impact_position_error)}")
                        
                    if magnitude(impact_position_error) < 5:
                        required_pitch = 90
                    elif magnitude(impact_position_error) < 30:
                        required_pitch = 94
                    elif magnitude(impact_position_error) < 50:
                        required_pitch = 98
                    else:
                        required_pitch = 103

                    vessel.auto_pilot.target_pitch_and_heading(required_pitch, calculated_heading)
                    
                    if h < 200:
                        control.gear = True
                        flightPhase = FlightPhase.LANDING

                        logger.log_strict("Deploying landing gear.")
                    continue
                case FlightPhase.LANDING:
                    control.throttle = suicideBurnManager.throttle_targeted
                    
                    vel_trg = vessel.velocity(landing_ref_frame)
                    hor_vel = (0, vel_trg[1], vel_trg[2])
                    heading = apiv6.angle_between_vectors(hor_vel, orbit.body.direction(landing_ref_frame))

                    if magnitude(hor_vel) > 2:
                        if hor_vel[2] < 0:
                            heading = 360 - heading
                        vessel.auto_pilot.target_pitch_and_heading(100 , heading)
                    else:
                        vessel.auto_pilot.target_pitch = 90
                    if h < 30:
                        vessel.auto_pilot.target_pitch = 90

                    logger.log_timout(f"Balancing horizontal speed: {magnitude(hor_vel)}", 0.5)

                    if v > -1:
                        flightPhase = FlightPhase.LANDED
                        logger.log_strict(f"Flight Computer Registered Touchdown. Target distance: {impact_position_error[0]}, {impact_position_error[1]}, {impact_position_error[2]} ABSOLUTE: {magnitude((0, impact_position_error[1], impact_position_error[2]))}")
                        break
                case _:
                    continue

control.throttle = 0
print("Landed!")




