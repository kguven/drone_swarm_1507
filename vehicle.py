import json
import numpy as np


class Vehicle:
    id = None
    rc_channels = None
    position = None
    position_velocity_ned = None
    attitude = None
    heading = None
    absolute_altitude = None
    fixed_wing_metrics = None
    attitude_angular_velocity_body = None
    flight_mode = None

    is_armed = None

    master = 0

    def get_telemetry_as_dict(self):
        telemetry_dict = dict()
        telemetry_dict["id"] = self.id
        telemetry_dict["master"] = self.master
        if self.position is not None:
            telemetry_dict["lat"] = self.position.latitude_deg
            telemetry_dict["long"] = self.position.longitude_deg
            telemetry_dict["rel_alt"] = self.position.relative_altitude_m
            telemetry_dict["abs_alt"] = self.position.absolute_altitude_m
            telemetry_dict["heading"] = self.attitude.yaw_deg
            telemetry_dict["mode"] = self.flight_mode
        return telemetry_dict

    def get_telemetry_as_json(self):
        telemetry_dict = self.get_telemetry_as_dict()
        return json.dumps(telemetry_dict)

    def get_local_position_from_global_positions(self, dest_lat, dest_lon):
        if self.position is None:
            return None
        earthRadiusKm = 6371
        dest_lat = float(dest_lat) * np.pi / 180
        dest_lon = float(dest_lon) * np.pi / 180
        base_lat = float(self.position.latitude_deg) * np.pi / 180
        base_lon = float( self.position.longitude_deg) * np.pi / 180
        dLat = dest_lat - base_lat
        dLon = dest_lon - base_lon
        x = np.cos(dest_lat) * np.sin(dest_lon - base_lon)
        y = np.cos(base_lat) * np.sin(dest_lat) - np.sin(base_lat) * np.cos(dest_lat) * np.cos(dLon)
        bearing = np.arctan2(x, y) * 180 / np.pi
        a = np.sin(dLat / 2) * np.sin(dLat / 2) + np.sin(dLon / 2) * np.sin(dLon / 2) * np.cos(base_lat) * np.cos(
            dest_lat)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        distance_xy = earthRadiusKm * c * 1000
        relative_pos_x = distance_xy * np.sin(bearing * np.pi / 180)
        relative_pos_y = distance_xy * np.cos(bearing * np.pi / 180)
        return np.asarray([relative_pos_y, relative_pos_x])





