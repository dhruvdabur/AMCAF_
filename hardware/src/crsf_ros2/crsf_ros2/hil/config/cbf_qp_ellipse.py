from obstacle import Obstacle

# CBF parameters
R_SAFE = 1.0      # safety radius around obstacle center [m]
GAMMA1 = 2.0 #1.5      # CBF gain on velocity term
GAMMA2 = 1.0 #0.1      # CBF gain on barrier term
GAMMA3 = 1.0 

DT = 0.05
V_TARGET = 2.0
LANE_WIDTH = 2.0
LOOKAHEAD = 12.0
LOOKAHEAD_CBF = 5.0
LOWEST_CAR_SPEED = 0.01

kp_v, kd_v     = 1.2, 0.1
kp_y, kd_y     = 3.0, 0.5
kp_psi, kd_psi = 2.2, 0.3

CAR_L = 1.8
CAR_W = 0.8

# CBF-QP ellipse parameters (edit here; used in cbf_qp_ellipse + visualisation)
ELL_CAR_HALF_L = CAR_L / 2
ELL_CAR_HALF_W = CAR_W / 2
ELL_R_OBS      = 0.0   # assumed obstacle bounding radius
ELL_BUFFER_L   = 1.2   # longitudinal buffer
ELL_BUFFER_LAT = 0.25   # lateral buffer
A_ELL = ELL_CAR_HALF_L + ELL_R_OBS + ELL_BUFFER_L    # semi-major (longitudinal)
B_ELL = ELL_CAR_HALF_W + ELL_R_OBS + ELL_BUFFER_LAT  # semi-minor (lateral)

obstacles = sorted([
        Obstacle(x= 10.0, y= -1.0),
        Obstacle(x= 20.0, y= -1.0),
        Obstacle(x= 30.0, y= -1.5),
        Obstacle(x= 50.0, y=  0.0),
        Obstacle(x= 60.0, y=  1.0),
        Obstacle(x= 70.0, y=  1.0),
        Obstacle(x= 75.0, y=  1.5),
    ], key=lambda o: o.x) 

counter  = 0
cbf_infeasible = False
cbf_h_min = float("nan")

VIEW_X = 30
VIEW_Y = 4.5
