Header header

# possible traffic light states (constants)
uint8 GREEN      = 0
uint8 YELLOW     = 1
uint8 RED        = 2
uint8 RED_YELLOW = 3

# invalid float64 value for dist_m and dphi_rad
float64 INVALID = 10000

# traffic light state (REEN, YELLOW, RED, or RED_YELLOW)
uint8 state

# distance of center of front bumper to stop line of traffic light (in m).
# positive distance means we are in front of the stop line, negative
# distances indicate that we passed the stop line.
float64 dist_m

# orientation difference (in rad, -pi..pi) between vehicle's orientation and
# the normal vector of the stop line (in driving direction of the lane
# associated with the stop line).
# For an illustration of dphi_rad, see "stopline_msg.png" in the
# "car_demo/doc" directory.
float64 dphi_rad
