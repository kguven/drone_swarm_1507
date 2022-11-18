import numpy as np


def get_agent_target_position_from_master(lat, lon, heading, offset, id, formation="LINE"):
    earth_radius = 6371000.0  # Radius of "spherical" earth
    angle = 0
    if formation == "ARROW":
        reverse_heading = np.mod(heading + 180, 360)
        if id == 1:
            angle = reverse_heading - 45
        if id == 2:
            angle = reverse_heading + 45
    if formation == "LINE":
        reverse_heading = np.mod(heading + 180, 360)
        if id == 1:
            angle = reverse_heading - 90
        if id == 2:
            angle = reverse_heading + 90
    dNorth = np.cos(angle / 180 * np.pi) * offset
    dEast = np.sin(angle / 180 * np.pi) * offset
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * np.cos(np.pi * lat / 180))

    # New position in decimal degrees
    newlat = lat + (dLat * 180 / np.pi)
    newlon = lon + (dLon * 180 / np.pi)
    return newlat, newlon
