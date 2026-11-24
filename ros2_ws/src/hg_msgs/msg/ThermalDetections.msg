# Person and fire detections from one thermal image (hg_perception thermal_detector).
# Pixel coordinates refer to the 32 x 24 image on /thermal/image: column 0 at the image's left
# (the robot's left), row 0 at the top. Bearings in degrees: 0 = straight ahead, positive = the
# robot's left, 110 deg across the 32 columns.

std_msgs/Header header          # Copied from the thermal image.
float32 room_c                  # Room temperature: median of all pixels, deg C.

ThermalPerson[] persons         # Confirmed persons only (seen in person_min_frames frames).

bool fire                       # True after fire_min_frames frames with a pixel >= fire_min_c.
float32 fire_max_c              # Warmest pixel of this frame, deg C (also when fire is false).
uint16 fire_col                 # Column of the warmest pixel.
uint16 fire_row                 # Row of the warmest pixel.
float32 fire_bearing_deg        # Bearing of fire_col.
