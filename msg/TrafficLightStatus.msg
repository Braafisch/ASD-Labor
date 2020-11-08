Header header

# possible traffic light states (constants)
uint8 GREEN      = 0
uint8 YELLOW     = 1
uint8 RED        = 2
uint8 RED_YELLOW = 3

# traffic light state (REEN, YELLOW, RED, or RED_YELLOW)
uint8 state

# distance of front bumper to stop line of traffic light (in m)
float64 dist_m

# orientation towards closest point on stop line from front bumper 
# (in rad, -pi..pi, 0 means "in front of vehicle")
float64 heading_rad



