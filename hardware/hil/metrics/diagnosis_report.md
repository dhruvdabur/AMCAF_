# Robot Controller Auto-Diagnosis Report
**Trial Duration**: 25.84 s | **Total Samples**: 143
**Controller Score**: 8.6 / 100

## Telemetry Evidence Summary
* **Lateral Tracking RMSE**: 34.22 px
* **Max Lateral Deviation**: 131.07 px
* **Heading Tracking RMSE**: 46.14 deg
* **Minimum Safety Margin h(x)**: -0.317
* **Steering Saturation**: 51.7%
* **Steering Reversal Frequency**: 2.44 Hz

## Diagnostic Findings
### 1. High tracking error (🟡 WARN)
**Evidence**:
- lateral RMSE = 0.342 m
- max lateral error = 1.311 m
**Likely Causes**:
- controller gain too low
- lookahead too large
- reference path too aggressive
- localization error
- vehicle model mismatch
**Recommended Fixes**:
- plot lateral error against curvature
- reduce lookahead or increase tracking gain
- check odom/map transform consistency
- validate wheelbase and steering model

### 2. Steering oscillation / chattering (🟡 WARN)
**Evidence**:
- steering reversal rate = 2.44 Hz
**Likely Causes**:
- controller gain too high
- lookahead too small
- yaw estimate noisy
- actuator delay
- CBF/QP command switching
**Recommended Fixes**:
- reduce steering gain
- increase lookahead distance
- add steering-rate penalty
- low-pass filter yaw-rate input
- compare u_nominal and u_safe

### 3. Steering saturation (🔴 ERROR)
**Evidence**:
- steering saturation = 51.7% of run
**Likely Causes**:
- speed too high for path curvature
- path curvature exceeds vehicle capability
- controller demands infeasible steering
- incorrect wheelbase or steering limit
**Recommended Fixes**:
- reduce speed on high-curvature sections
- add curvature-aware velocity planning
- check steering angle limits
- verify Ackermann model parameters

### 4. CBF safety violation (🔴 ERROR)
**Evidence**:
- minimum CBF h(x) = -0.317
**Likely Causes**:
- CBF constraint too weak
- obstacle estimate delayed
- actuator bounds not included in QP
- sampling time too large
- gamma parameter too aggressive
**Recommended Fixes**:
- include actuator limits directly in the QP
- increase safety margin
- reduce controller timestep
- check obstacle timestamp latency
- plot h(x), u_nominal, and u_safe together

### 5. CLF/DCLF stability violation (🟡 WARN)
**Evidence**:
- CLF violation percentage = 35.0%
**Likely Causes**:
- CLF constraint relaxed too often
- CBF constraint dominates CLF objective
- goal temporarily unreachable
- bad Lyapunov function design
- insufficient control authority
**Recommended Fixes**:
- inspect CLF slack values
- increase CLF weight if safe
- add a slack hierarchy between CBF and CLF
- check whether reference trajectory is dynamically feasible

### 6. QP solver latency spike (🟡 WARN)
**Evidence**:
- max QP solve time = 32.40 ms
**Likely Causes**:
- too many active constraints
- ill-conditioned QP
- solver warm-start missing
- CPU overload
- control loop deadline too tight
**Recommended Fixes**:
- enable solver warm start
- reduce unnecessary constraints
- profile callback timing
- log active constraint count
- compare solve time against control period
