"""Named controller modes for hil ArUco followers."""

PID = 'pid'
PID_CBF = 'pid_cbf'
PID_VELOCITY = 'pid_velocity'
PID_VELOCITY_CBF = 'pid_velocity_cbf'
PID_VELOCITY_CBF_QP_ELLIPSE = 'pid_velocity_cbf_qp_ellipse'
PID_VELOCITY_DCLF_DCBF = 'pid_velocity_dclf_dcbf'
MPC_CBF = 'mpc_cbf'

CONTROLLER_MODES = (
    PID,
    PID_CBF,
    PID_VELOCITY,
    PID_VELOCITY_CBF,
    PID_VELOCITY_CBF_QP_ELLIPSE,
    PID_VELOCITY_DCLF_DCBF,
    MPC_CBF,
)
