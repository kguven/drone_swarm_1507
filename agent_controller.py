#!/usr/bin/env python3

import asyncio
import sys
import socket
import json
import subprocess
import numpy as np

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)

from vehicle import Vehicle
from swarm_helpers import *

hardware_enabled = False
takeoff_start = False
controller_start = False
mission_started = False
pixhawk_serial_string = "/dev/ttyS1:57600"

agent = Vehicle()
master = Vehicle()

swarm_telemetry = dict()
formation = "LINE"

target_position_from_me = np.asarray([0.0, 0.0])


async def main(drone):
    global swarm_telemetry, formation, controller_start, takeoff_start

    Kp = 2
    Ki = 0
    Kd = 20
    error_rate = 0
    error_old = 0
    error_total = 0
    init_flag = True

    Kp_alt = 2
    Ki_alt = 0
    Kd_alt = 0
    error_rate_alt = 0
    error_old_alt = 0
    error_total_alt = 0

    critical_altitude = 2

    while True:
        if agent.id != 0:
            master_uav = "UAV0"
            if master_uav in swarm_telemetry and agent.is_armed is not None:
                target_uav = swarm_telemetry[master_uav]
                if "lat" in target_uav and "long" in target_uav and "mode" in target_uav:
                    master_lat = swarm_telemetry[master_uav]["lat"]
                    master_long = swarm_telemetry[master_uav]["long"]
                    master_alt = swarm_telemetry[master_uav]["rel_alt"]
                    master_heading = swarm_telemetry[master_uav]["heading"]
                    master_mode = swarm_telemetry[master_uav]["mode"]
                    print(master_mode)
                    if master_mode == "TAKEOFF":
                        if not agent.is_armed:
                            await drone.action.arm()
                        if not takeoff_start:
                            await drone.action.takeoff()
                    if master_mode == "LAND":
                        await drone.action.land()
                        controller_start = False
                        break
                    target_lat, target_long = get_agent_target_position_from_master(master_lat, master_long,
                                                                                    master_heading, 7.0, uav_id,
                                                                                    formation)
                    error = agent.get_local_position_from_global_positions(target_lat, target_long)
                    error_alt = master_alt - agent.position.relative_altitude_m
                    if init_flag and controller_start:
                        init_flag = False
                        error_old = error
                        error_old_alt = error_alt
                        if not agent.is_armed:
                            await drone.action.arm()
                        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                        try:
                            await drone.offboard.start()
                        except OffboardError as error:
                            await drone.action.disarm()
                            return
                    else:
                        error_rate = error - error_old
                        error_old = error

                        error_rate_alt = error_alt - error_old_alt
                        error_old_alt = error_alt

                    # if controller_start:
                    error_total += error
                    error_total_alt += error_alt
                    command = error * Kp + error_rate * Kd + error_total * Ki
                    command_altitude = error_alt * Kp_alt + error_rate_alt * Kd_alt + error_total_alt * Ki_alt
                    command_altitude = command_altitude * -1

                    if abs(agent.position.relative_altitude_m) < critical_altitude:
                        north_speed = 0
                        east_speed = 0
                        up_speed = command_altitude
                    else:
                        north_speed = command[0]
                        east_speed = command[1]
                        up_speed = command_altitude
                    await drone.offboard.set_velocity_ned(
                        VelocityNedYaw(north_speed, east_speed, up_speed, master_heading))

        await asyncio.sleep(0.05)


async def communication():
    global swarm_telemetry, mission_started, formation
    UDP_IP = "127.0.0.1"
    UDP_PORT = 12345
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.1)

    print("Communication started!")

    while True:
        # print(agent.position)
        telemetry_message = agent.get_telemetry_as_json()
        # print(telemetry_message)
        sock.sendto(telemetry_message.encode("utf-8"), (UDP_IP, UDP_PORT))
        try:
            data, addr = sock.recvfrom(1024)
            swarm_telemetry = json.loads(data.decode("utf-8"))
            # print(swarm_telemetry)
            if "Command" in swarm_telemetry:
                mission_started = True
                formation = swarm_telemetry["Command"]
        except:
            print("Timed out")
        await asyncio.sleep(0.05)


async def run(uav_id):
    mavsdk_server_port = 50040 + uav_id
    uav_udp_port = 14540 + uav_id
    mav_host_ip = ""
    uav_udp_port_str = "udp://" + mav_host_ip + ":" + str(uav_udp_port)
    uav_serial_port_str = "serial://" + pixhawk_serial_string
    if hardware_enabled:
        subprocess.Popen(["/usr/src/mavsdk_server_linux-armv7", "-p", str(mavsdk_server_port), uav_serial_port_str],
                         stdout=subprocess.PIPE)
    else:
        subprocess.Popen(["mavsdk_server", "-p", str(mavsdk_server_port), uav_udp_port_str], stdout=subprocess.PIPE)

    drone = System(mavsdk_server_address="localhost", port=mavsdk_server_port)
    await drone.connect()
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    await drone.param.set_param_float("MPC_XY_CRUISE", float(1.0))
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", float(1.0))
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", float(2.0))
    # Start the tasks
    asyncio.ensure_future(get_position(drone))
    asyncio.ensure_future(get_attitude(drone))
    asyncio.ensure_future(is_armed(drone))
    asyncio.ensure_future(get_flight_mode(drone))
    asyncio.ensure_future(main(drone))
    asyncio.ensure_future(communication())


async def get_position(drone):
    async for position in drone.telemetry.position():
        agent.position = position


async def get_attitude(drone):
    async for attitude in drone.telemetry.attitude_euler():
        agent.attitude = attitude


async def is_armed(drone):
    async for result in drone.telemetry.armed():
        agent.is_armed = result


async def get_flight_mode(drone):
    global takeoff_start, controller_start
    previous_flight_mode = None
    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode is not previous_flight_mode:
            previous_flight_mode = flight_mode
            flight_mode = str(flight_mode)
            agent.flight_mode = flight_mode
            if flight_mode == "TAKEOFF":
                takeoff_start = True
            if flight_mode == "HOLD":
                if takeoff_start:
                    controller_start = True
                    print("Controller started!")


if __name__ == "__main__":
    uav_id = int(sys.argv[1])
    agent.id = uav_id
    if uav_id == 0:
        agent.master = 1
    asyncio.ensure_future(run(uav_id))
    asyncio.get_event_loop().run_forever()
