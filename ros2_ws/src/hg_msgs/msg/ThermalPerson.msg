# One person seen by the thermal camera (element of ThermalDetections.persons).

sensor_msgs/RegionOfInterest bbox  # Pixels: x_offset = left col, y_offset = top row, width, height.
float32 peak_c                     # Warmest pixel of the person, deg C.
float32 delta_c                    # peak_c minus the room temperature.
float32 bearing_deg                # Bearing of the bounding box's center column.
