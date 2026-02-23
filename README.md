# Cortex MPC Infrastructure Architecture

## How Everything Fits Together

---

## System Overview

Cortex is three systems that share a physics model but serve different purposes:

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  1. PRODUCTION CONTROL SYSTEM                                       │
│     Real hardware, real sensors, real actuators.                     │
│     Runs on Pi 5 in the grow room. 1 Hz closed-loop.               │
│     This is the thing that actually controls the environment.        │
│                                                                     │
│  2. PHYSICS SIMULATION ENVIRONMENT                                  │
│     Software model of the grow room. No hardware.                   │
│     Runs on dev laptop or CI server.                                │
│     Used for: development, testing, tuning, replay, what-if.        │
│                                                                     │
│  3. BACKGROUND CALIBRATION SYSTEM                                   │
│     Runs alongside production on the Pi 5.                          │
│     Compares model predictions to real sensor data.                 │
│     Tunes model parameters so the simulation matches reality.       │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

All three share the same core: **a set of 7 coupled ODEs that describe the thermodynamics of a grow room**. The difference is what drives the inputs and what consumes the outputs.

---

## The Core Physics Model

This is the single source of truth. Every other component is either feeding data into it or consuming predictions from it.

```
┌─────────────────────────────────────────────────────────────────┐
│                    GrowRoomDynamics                              │
│                                                                 │
│  STATE VECTOR x (7 states):                                     │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ x[0] T_a    Air temperature              °C             │    │
│  │ x[1] w_a    Absolute humidity             g/kg          │    │
│  │ x[2] C      CO2 concentration             ppm           │    │
│  │ x[3] T_l    Leaf temperature              °C            │    │
│  │ x[4] θ      Substrate water content       m³/m³         │    │
│  │ x[5] T_s    Supply air temperature        °C            │    │
│  │ x[6] T_w    Wall/thermal mass temp        °C            │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  CONTROL VECTOR u (6-7 inputs):                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ Integrated HVACD:  [T_sp, RH_sp, u_hum, u_co2,         │    │
│  │                      u_fan, u_irr]                       │    │
│  │ Direct control:    [u_cool, u_heat, u_hum, u_co2,       │    │
│  │                      u_fan, u_irr]                       │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  DISTURBANCE VECTOR d (5 known/forecast):                       │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ Q_light   Lighting heat load              W             │    │
│  │ T_out     Outdoor temperature             °C            │    │
│  │ w_out     Outdoor humidity                g/kg          │    │
│  │ N_plants  Plant count/canopy density      —             │    │
│  │ phase     Growth phase                    enum          │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  DERIVED OUTPUTS (computed from x, not states):                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ VPD           = f(T_l, w_a)               kPa           │    │
│  │ RH            = f(T_a, w_a)               %             │    │
│  │ dryback_rate  = dθ/dt                     m³/m³/hr      │    │
│  │ transpiration = f(VPD, PAR, θ, LAI)       g/m²/hr       │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  PARAMETERS p (tuned by SLSQP, not measured):                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ UA_wall    Wall thermal conductance       W/K           │    │
│  │ UA_env     Envelope conductance           W/K           │    │
│  │ τ_hvac     HVAC time constant             s             │    │
│  │ g_s_max    Max stomatal conductance       mol/m²/s      │    │
│  │ LAI        Leaf area index                m²/m²         │    │
│  │ K_drain    Drainage coefficient           1/hr          │    │
│  │ C_wall     Thermal mass capacitance       J/K           │    │
│  │ η_hvac     HVAC efficiency                —             │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  TWO INTERFACES:                                                │
│                                                                 │
│  nonlinear_step(x, u, d, p, dt) → x_next                       │
│    Full nonlinear ODE integration (RK4).                        │
│    Used by: simulation environment, SLSQP calibrator.           │
│    Cost: ~0.1ms per step.                                       │
│                                                                 │
│  linearize(x0, u0, d0, p) → A_d, B_d, B_dd                    │
│    Analytical Jacobians + Padé discretization.                  │
│    Used by: OSQP solver (every MPC cycle), EKF.                 │
│    Cost: ~0.5ms.                                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### What Lives Inside the Physics Model

```python
class GrowRoomDynamics:
    """
    THE core model. Everything else wraps or consumes this.
    
    This class owns:
    - The 7 coupled ODEs (continuous-time dynamics)
    - The analytical Jacobian computation (for linearization)
    - The discretization (continuous → discrete for OSQP)
    - The HVACD sub-model (compressor/reheat/coil dynamics)
    - The transpiration sub-model (Penman-Monteith with stomatal conductance)
    - The derived output calculations (VPD, RH, dryback rate)
    
    This class does NOT own:
    - Sensor data (that's the EKF's job)
    - Optimization (that's OSQP's job)
    - Parameter tuning (that's SLSQP's job)
    - Actuation (that's the control loop's job)
    """
    
    def __init__(self, room_config, equipment_config, params):
        # Room geometry (constant)
        self.V = room_config['volume_m3']
        self.A_floor = room_config['floor_area_m2']
        self.A_canopy = room_config['canopy_area_m2']
        
        # Equipment models (initialized from config)
        self.hvacd = HVACDModel(equipment_config['hvacd'])
        # OR
        self.minisplit = MiniSplitModel(equipment_config['minisplit'])
        
        # Tunable parameters (updated by SLSQP calibrator)
        self.params = params  # SharedParams reference
    
    def nonlinear_step(self, x, u, d, params, dt):
        """
        Integrate ODEs forward by dt seconds using RK4.
        
        This is the GROUND TRUTH model. It includes all nonlinearities:
        - Saturation vapor pressure (exponential)
        - Transpiration (product of VPD × stomatal conductance)
        - Radiation (T⁴ term, though linearized in practice)
        - HVACD mode switching and thermal lag
        
        Used by:
        - Simulation environment (as the "plant" being controlled)
        - SLSQP calibrator (replay actual data through model)
        - EKF prediction step (propagate state estimate)
        """
        # RK4 integration
        k1 = self._derivatives(x, u, d, params)
        k2 = self._derivatives(x + 0.5*dt*k1, u, d, params)
        k3 = self._derivatives(x + 0.5*dt*k2, u, d, params)
        k4 = self._derivatives(x + dt*k3, u, d, params)
        
        x_next = x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return x_next
    
    def linearize(self, x0, u0, d0, params):
        """
        Compute linearized discrete-time model around operating point.
        
        Returns A_d, B_d, B_dd such that:
            x[k+1] ≈ A_d @ x[k] + B_d @ u[k] + B_dd @ d[k]
        
        Used by:
        - OSQP solver (build QP matrices)
        - EKF (covariance propagation)
        """
        A_c, B_c, Bd_c = self._analytical_jacobians(x0, u0, d0, params)
        A_d, B_d, Bd_d = self._discretize_pade2(A_c, B_c, Bd_c, dt=60.0)
        return A_d, B_d, Bd_d
    
    def derived_outputs(self, x):
        """Compute VPD, RH, dryback rate from state vector."""
        T_a, w_a, C, T_l, theta, T_s, T_w = x
        SVP_l = 0.6108 * np.exp(17.27 * T_l / (T_l + 237.3))
        VPD = SVP_l - (w_a / (622 + w_a)) * 101.325
        w_sat = 622 * SVP_l / (101.325 - SVP_l)
        RH = (w_a / w_sat) * 100
        return {'VPD': VPD, 'RH': RH, 'T_leaf': T_l, 'theta': theta}
    
    def _derivatives(self, x, u, d, params):
        """The 7 coupled ODEs. Heart of the entire system."""
        # ... (see cortex-mpc-implementation.md, ODEs 1-7)
        pass
    
    def _analytical_jacobians(self, x0, u0, d0, params):
        """49 partial derivatives for A, 49 for B, computed analytically."""
        # ... (Numba JIT compiled, ~0.05ms)
        pass
```

---

## System 1: Production Control (Real Hardware)

This is what runs in the grow room on the Pi 5. Every component has a specific job, a specific frequency, and specific data interfaces.

### Component Map

```
┌─────────────────────────────────────────────────────────────────────────┐
│  RASPBERRY PI 5                                                         │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  PROCESS 1: Control Loop (Python asyncio, pinned to core 0)    │    │
│  │                                                                 │    │
│  │  ┌──────────┐   ┌─────┐   ┌──────┐   ┌──────────┐             │    │
│  │  │  Sensor   │──▶│ EKF │──▶│ OSQP │──▶│ Actuator │             │    │
│  │  │  Reader   │   │     │   │      │   │  Writer  │             │    │
│  │  └──────────┘   └─────┘   └──────┘   └──────────┘             │    │
│  │       │            │          │            │                    │    │
│  │       │            │          │            │                    │    │
│  │       ▼            ▼          ▼            ▼                    │    │
│  │  ┌──────────────────────────────────────────────┐              │    │
│  │  │         Telemetry Writer (async)              │              │    │
│  │  │  → Redis (hot: last 4 hrs, ring buffer)       │              │    │
│  │  │  → SQLite (cold: full history)                │              │    │
│  │  └──────────────────────────────────────────────┘              │    │
│  │                                                                 │    │
│  │  ┌──────────────────────────────────────────────┐              │    │
│  │  │         Compliance Tracker (async)            │              │    │
│  │  │  → Rolling 1-hr window, reports every minute  │              │    │
│  │  └──────────────────────────────────────────────┘              │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  PROCESS 2: MQTT Bridge (separate process, core 1)              │    │
│  │                                                                 │    │
│  │  ┌────────────┐        ┌────────────┐                          │    │
│  │  │ MQTT Sub   │───────▶│ Redis Pub  │  (sensor data in)        │    │
│  │  │ (sensors)  │        │            │                          │    │
│  │  └────────────┘        └────────────┘                          │    │
│  │  ┌────────────┐        ┌────────────┐                          │    │
│  │  │ Redis Sub  │───────▶│ MQTT Pub   │  (actuator commands out) │    │
│  │  │            │        │ (commands) │                          │    │
│  │  └────────────┘        └────────────┘                          │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  PROCESS 3: SLSQP Calibrator (background thread, core 2)       │    │
│  │                                                                 │    │
│  │  Every 5-30 min:                                                │    │
│  │  1. Read last 1-4 hrs of telemetry from Redis                   │    │
│  │  2. Replay through nonlinear model with candidate params        │    │
│  │  3. SLSQP minimizes prediction error                           │    │
│  │  4. Atomic swap of updated params → SharedParams                │    │
│  │                                                                 │    │
│  │  Trigger: adaptive (EKF innovation threshold, phase change,     │    │
│  │           lights transition, irrigation event)                  │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │  PROCESS 4: API Server (FastAPI, core 3)                        │    │
│  │                                                                 │    │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │    │
│  │  │Dashboard │  │ Config   │  │ Phase    │  │ Override │       │    │
│  │  │WebSocket │  │ REST API │  │ Manager  │  │ Handler  │       │    │
│  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌──────────┐                                                          │
│  │  Redis   │  In-memory store. ~100KB for 4 hrs of 7-state data.      │
│  └──────────┘                                                          │
│  ┌──────────┐                                                          │
│  │  SQLite  │  On-disk archive. Full telemetry history.                │
│  └──────────┘                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Data Flow (One Control Cycle — 1 second)

```
  PHYSICAL WORLD                    CORTEX (Pi 5)                     PHYSICAL WORLD
  ─────────────                    ──────────────                     ─────────────
                                        
  ┌──────────┐    MQTT     ┌──────────────────┐                           
  │ ESP32 #1 │────────────▶│                  │                           
  │ SHT31    │  T,RH @1Hz  │   MQTT Bridge    │                           
  │ SCD30    │  CO2 @1Hz   │   (Process 2)    │                           
  └──────────┘             │                  │                           
  ┌──────────┐    MQTT     │   Writes to      │                           
  │ ESP32 #2 │────────────▶│   Redis:         │                           
  │ MLX90614 │  T_leaf     │   sensors:esp32: │                           
  │ Supply T │  T_supply   │    room1:latest  │                           
  └──────────┘             └────────┬─────────┘                           
  ┌──────────┐    MQTT              │                                     
  │ ESP32 #3 │────────────▶         │  (also: AROYA API poller            
  │ TEROS ONE│  θ, EC              │   writes sensors:aroya:room1:latest  
  └──────────┘                      │   every 3 min)                      
                                    │                                     
                                    ▼                                     
                           ┌──────────────────┐                           
                      ┌────│  Redis            │                           
                      │    │  sensors:*:latest │                           
                      │    └──────────────────┘                           
                      │                                                   
                      ▼                                                   
  ╔══════════════════════════════════════════════════════════════╗         
  ║  CONTROL LOOP (Process 1) — runs every 1 second             ║         
  ║                                                              ║         
  ║  STEP 1: Read Sensors (2ms)                                  ║         
  ║  ┌─────────────────────────────────────────────────────┐     ║         
  ║  │ z_raw = redis.hgetall('sensors:esp32:room1:latest') │     ║         
  ║  │ z_aroya = redis.hgetall('sensors:aroya:room1:latest')│    ║         
  ║  │ Check staleness (ESP32 > 5s? AROYA > 10min?)        │     ║         
  ║  │ Build measurement vector z_meas (6 elements)        │     ║         
  ║  └──────────────────────────┬──────────────────────────┘     ║         
  ║                              │                                ║         
  ║                              ▼                                ║         
  ║  STEP 2: State Estimation — EKF (3ms)                        ║         
  ║  ┌─────────────────────────────────────────────────────┐     ║         
  ║  │ PREDICT:                                            │     ║         
  ║  │   x_pred = x_hat + f(x_hat, u_last, d) × dt       │     ║         
  ║  │   P_pred = F @ P @ F' + Q                          │     ║         
  ║  │                                                     │     ║         
  ║  │ UPDATE:                                             │     ║         
  ║  │   innovation = z_meas - H @ x_pred                 │     ║         
  ║  │   K = P_pred @ H' @ inv(H @ P_pred @ H' + R)      │     ║         
  ║  │   x_hat = x_pred + K @ innovation                  │     ║         
  ║  │   P = (I - K @ H) @ P_pred                         │     ║         
  ║  │                                                     │     ║         
  ║  │ OUTPUT: x_hat (7), P (7×7), innovation (6)         │     ║         
  ║  │                                                     │     ║         
  ║  │ USES: GrowRoomDynamics._derivatives() for predict   │     ║         
  ║  │       GrowRoomDynamics._analytical_jacobians() → F  │     ║         
  ║  └──────────────────────────┬──────────────────────────┘     ║         
  ║                              │                                ║         
  ║                              ▼                                ║         
  ║  STEP 3: MPC Solve — OSQP (2-15ms)                          ║         
  ║  ┌─────────────────────────────────────────────────────┐     ║         
  ║  │ 3a. LINEARIZE (0.5ms)                               │     ║         
  ║  │   A_d, B_d, Bd_d = dynamics.linearize(x_hat, u, d) │     ║         
  ║  │                                                     │     ║         
  ║  │ 3b. BUILD QP (1.5ms)                                │     ║         
  ║  │   Propagate A_d, B_d into equality constraints      │     ║         
  ║  │   Update cost with current reference trajectory     │     ║         
  ║  │   Update bounds from current constraints            │     ║         
  ║  │   (only overwrite .data arrays — structure fixed)   │     ║         
  ║  │                                                     │     ║         
  ║  │ 3c. WARM START                                      │     ║         
  ║  │   Shift previous solution forward by one step       │     ║         
  ║  │   u_warm = [u[1],...,u[Nc-1],u[Nc-1]]             │     ║         
  ║  │   x_warm = [x[1],...,x[Np-1],x[Np-1]]             │     ║         
  ║  │                                                     │     ║         
  ║  │ 3d. SOLVE (1-10ms)                                  │     ║         
  ║  │   osqp.update(Px, q, Ax, l, u)                     │     ║         
  ║  │   osqp.warm_start(x_warm, y_warm)                   │     ║         
  ║  │   result = osqp.solve()                             │     ║         
  ║  │                                                     │     ║         
  ║  │ 3e. EXTRACT                                         │     ║         
  ║  │   u_opt = result.x[0:nu]  (first control move)     │     ║         
  ║  │   trajectory = result.x[nu:]  (predicted states)    │     ║         
  ║  │                                                     │     ║         
  ║  │ QP STRUCTURE:                                       │     ║         
  ║  │   Decision vars: 539 (105 control + 420 state + 14) │     ║         
  ║  │   Constraints: ~1100 (equality + inequality)        │     ║         
  ║  │                                                     │     ║         
  ║  │ USES: GrowRoomDynamics.linearize()                  │     ║         
  ║  │       SharedParams.current() for model parameters   │     ║         
  ║  │       ReferenceGenerator for target trajectory       │     ║         
  ║  │       DisturbanceForecast for d over horizon        │     ║         
  ║  └──────────────────────────┬──────────────────────────┘     ║         
  ║                              │                                ║         
  ║                              ▼                                ║         
  ║  STEP 4: Apply Control (2ms)                                 ║         
  ║  ┌─────────────────────────────────────────────────────┐     ║         
  ║  │ Convert u_opt → actuator commands:                  │     ║         
  ║  │                                                     │     ║         
  ║  │ Option A (setpoint mode):                           │     ║         
  ║  │   T_sp, RH_sp → Modbus write to HVACD registers    │     ║         
  ║  │   fan_pct → 0-10V via DAC or Modbus                │     ║         
  ║  │   u_co2 → relay via MQTT → ESP32                   │     ║         
  ║  │   u_hum → relay via MQTT → ESP32                   │     ║         
  ║  │   u_irr → relay via MQTT → ESP32 (or AROYA API)    │     ║         
  ║  │                                                     │     ║         
  ║  │ Option B (direct mode):                             │     ║         
  ║  │   cool_pct → Modbus write to HVACD                 │     ║         
  ║  │   heat_pct → Modbus write to HVACD                 │     ║         
  ║  │   fan_pct → Modbus write to HVACD                  │     ║         
  ║  │                                                     │     ║         
  ║  │ Publish via Redis → MQTT Bridge → ESP32/Modbus     │     ║         
  ║  └──────────────────────────┬──────────────────────────┘     ║         
  ║                              │                                ║         
  ║  STEP 5: Log + Comply (async, non-blocking)                  ║         
  ║  ┌─────────────────────────────────────────────────────┐     ║         
  ║  │ Write to Redis ring buffer:                         │     ║         
  ║  │   x_hat, u_opt, d, z_raw, innovation, solve_info   │     ║         
  ║  │ Write to SQLite (batched, every 10s):               │     ║         
  ║  │   Full telemetry record                             │     ║         
  ║  │ Update compliance tracker:                          │     ║         
  ║  │   VPD, T, RH, CO2, θ, dryback_rate                │     ║         
  ║  │ Push to WebSocket (dashboard):                      │     ║         
  ║  │   Current state + predicted trajectory              │     ║         
  ║  │ Check model accuracy:                               │     ║         
  ║  │   If innovation > threshold for 30s → trigger SLSQP │     ║         
  ║  └─────────────────────────────────────────────────────┘     ║         
  ╚══════════════════════════════════════════════════════════════╝         
                      │                                                   
                      ▼                                                   
             ┌──────────────────┐                                         
             │  Redis            │                                         
             │  commands:room1   │                                         
             └────────┬─────────┘                                         
                      │                                                   
                      ▼                                                   
             ┌──────────────────┐     MQTT      ┌───────────────┐         
             │  MQTT Bridge     │──────────────▶│ ESP32 Relays  │         
             │  (Process 2)     │               │ CO2 solenoid  │         
             └──────────────────┘               │ Humidifier    │         
                      │                         │ Irrigation    │         
                      │  Modbus (direct)        └───────────────┘         
                      ▼                                                   
             ┌───────────────┐                                            
             │ HVACD Unit    │                                            
             │ (Desert Aire) │                                            
             │ T_sp, RH_sp,  │                                            
             │ fan_pct       │                                            
             └───────────────┘                                            
```

### Timing Budget (1-second cycle)

```
0ms          2ms    5ms                    7-20ms        22ms       23ms
 │            │      │                       │            │          │
 ▼            ▼      ▼                       ▼            ▼          ▼
┌────────────┬──────┬────────────────────────┬────────────┬──────────┐
│ Read       │ EKF  │ OSQP Solve             │ Actuate    │ Log/     │
│ Sensors    │      │ (linearize+build+solve) │            │ Comply   │
└────────────┴──────┴────────────────────────┴────────────┴──────────┘
│◀─────── synchronous, blocking (~23ms) ──────────────▶│◀── async──▶│
│                                                                    │
│◀─────────────────── 1000ms total cycle ──────────────────────────▶│
│                                                        977ms idle  │
```

---

## System 2: Physics Simulation Environment

This is where you develop and test without hardware. The simulation replaces the real grow room with a software model, but everything else — EKF, OSQP, control loop logic — runs identically.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  SIMULATION ENVIRONMENT (dev laptop or CI)                          │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  SimulatedRoom                                               │   │
│  │                                                               │   │
│  │  This IS the grow room. It uses the SAME GrowRoomDynamics    │   │
│  │  class as production, but instead of reading sensors and      │   │
│  │  writing actuators, it:                                       │   │
│  │                                                               │   │
│  │  1. Maintains its own "true" state x_true (7 values)         │   │
│  │  2. Receives control commands u from the controller           │   │
│  │  3. Advances x_true via nonlinear_step(x_true, u, d, p, dt) │   │
│  │  4. Generates synthetic sensor readings with realistic noise  │   │
│  │  5. Returns noisy z_meas to the controller (mimics sensors)  │   │
│  │                                                               │   │
│  │  The controller doesn't know it's talking to a simulation.   │   │
│  │  Same code runs on both.                                      │   │
│  └──────────────────────────────────┬───────────────────────────┘   │
│                                      │                               │
│              ┌───────────────────────┼────────────────────┐          │
│              │                       │                    │          │
│              ▼                       ▼                    ▼          │
│  ┌───────────────┐    ┌──────────────────┐    ┌──────────────────┐  │
│  │ Disturbance   │    │ Noise Generator  │    │ Fault Injector   │  │
│  │ Scheduler     │    │                  │    │                  │  │
│  │               │    │ Adds sensor noise│    │ Simulates:       │  │
│  │ Generates:    │    │ matching real    │    │ - Sensor dropout │  │
│  │ - Light       │    │ hardware specs:  │    │ - HVAC failure   │  │
│  │   schedule    │    │ - SHT31 ±0.3°C  │    │ - Stuck actuator │  │
│  │ - Outdoor     │    │ - SCD30 ±30ppm  │    │ - Power loss     │  │
│  │   weather     │    │ - TEROS ±0.01   │    │ - Network outage │  │
│  │ - Irrigation  │    │ - Bias drift     │    │ - Param drift    │  │
│  │   events      │    │ - Quantization   │    │                  │  │
│  │ - Phase       │    │                  │    │                  │  │
│  │   transitions │    │                  │    │                  │  │
│  └───────────────┘    └──────────────────┘    └──────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  SAME controller code as production:                         │   │
│  │                                                               │   │
│  │  EKF → OSQP → Control Logic → (writes to SimulatedRoom)     │   │
│  │                                                               │   │
│  │  The only difference: instead of Redis/MQTT, the controller  │   │
│  │  calls sim.get_measurements() and sim.apply_control(u)       │   │
│  │  via a SimulatedIO adapter that matches the real IO interface.│   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  Test Harness / Scenario Runner                              │   │
│  │                                                               │   │
│  │  Runs scenarios like:                                         │   │
│  │  - "24 hours of mid-flower, lights 12/12, outdoor 30°C"     │   │
│  │  - "Lights-on transition with 50kW heat spike"               │   │
│  │  - "HVAC compressor fails at hour 3, recovers at hour 5"    │   │
│  │  - "Phase transition from veg to flower"                     │   │
│  │  - "Replay actual telemetry from facility X"                 │   │
│  │                                                               │   │
│  │  Collects: compliance metrics, solve times, energy usage,    │   │
│  │  trajectory tracking error, actuator cycling stats           │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### The SimulatedRoom Class

```python
class SimulatedRoom:
    """
    Software replacement for the physical grow room.
    
    Uses the SAME GrowRoomDynamics as production, but:
    - Maintains "true" state that the controller can't see directly
    - Adds realistic sensor noise to measurements
    - Accepts control commands and propagates physics forward
    - Generates disturbances on a schedule (lights, weather, etc.)
    """
    
    def __init__(self, room_config, equipment_config, initial_state, sim_params=None):
        self.dynamics = GrowRoomDynamics(room_config, equipment_config, sim_params or default_params)
        
        # The "true" state of the room (controller never sees this directly)
        self.x_true = np.array(initial_state)
        
        # Disturbance schedule
        self.disturbance_schedule = DisturbanceScheduler(room_config)
        
        # Sensor noise model (matches real hardware specs)
        self.noise = SensorNoiseModel({
            'T_air':    {'std': 0.3, 'bias_drift': 0.01, 'quantization': 0.01},
            'w_a':      {'std': 0.2, 'bias_drift': 0.005},
            'CO2':      {'std': 30.0, 'bias_drift': 1.0, 'quantization': 1.0},
            'T_leaf':   {'std': 0.5, 'bias_drift': 0.02},
            'theta':    {'std': 0.01, 'bias_drift': 0.001},
            'T_supply': {'std': 0.3, 'bias_drift': 0.01},
        })
        
        self.time = 0.0  # simulation clock (seconds)
        self.history = []  # full state history for analysis
    
    def step(self, u, dt=1.0):
        """
        Advance the simulated room by dt seconds.
        
        Args:
            u: control vector from the controller
            dt: timestep (default 1s to match control loop)
            
        Returns:
            z_meas: noisy sensor measurements (what the controller sees)
            d: current disturbances (what the controller knows about)
        """
        # Get current disturbances
        d = self.disturbance_schedule.get(self.time)
        
        # Propagate TRUE physics (full nonlinear, no approximations)
        self.x_true = self.dynamics.nonlinear_step(
            self.x_true, u, d, self.dynamics.params, dt
        )
        
        # Generate noisy measurements (what sensors would report)
        z_meas = self.noise.corrupt(self.x_true, self.time)
        
        # Record history
        self.history.append({
            'time': self.time,
            'x_true': self.x_true.copy(),
            'z_meas': z_meas.copy(),
            'u': u.copy(),
            'd': d.copy(),
        })
        
        self.time += dt
        return z_meas, d
    
    def get_true_state(self):
        """For post-hoc analysis only. Controller never calls this."""
        return self.x_true.copy()


class DisturbanceScheduler:
    """
    Generates realistic disturbance profiles for simulation.
    
    Models the things that happen TO the room that the controller
    must respond to.
    """
    
    def __init__(self, config):
        self.light_schedule = config['light_schedule']  # e.g., on at 6am, off at 6pm
        self.light_power_W = config['light_power_W']    # e.g., 50000 W
        self.par_efficiency = config.get('par_efficiency', 0.4)  # LED: 40% PAR, 60% heat
        
        # Outdoor weather (can be constant, sinusoidal, or loaded from real data)
        self.outdoor_profile = config.get('outdoor_profile', 'sinusoidal')
        self.T_out_mean = config.get('T_out_mean_C', 28.0)
        self.T_out_amplitude = config.get('T_out_amplitude_C', 5.0)
        self.w_out = config.get('w_out_gkg', 12.0)
        
        # Plant parameters
        self.N_plants = config.get('N_plants', 100)
        self.phase = config.get('phase', 'mid_flower')
    
    def get(self, time_s):
        """Return disturbance vector d at simulation time."""
        hour = (time_s / 3600) % 24
        
        # Light schedule (deterministic — known exactly)
        lights_on = self.light_schedule['on_hour'] <= hour < self.light_schedule['off_hour']
        Q_light = self.light_power_W * (1 - self.par_efficiency) if lights_on else 0.0
        
        # Outdoor temperature (sinusoidal daily cycle)
        T_out = self.T_out_mean + self.T_out_amplitude * np.sin(
            2 * np.pi * (hour - 14) / 24  # peak at 2pm
        )
        
        return np.array([Q_light, T_out, self.w_out, self.N_plants, 
                         self._phase_to_int(self.phase)])
```

### IO Adapter Pattern (Same Controller, Two Backends)

```python
class IOInterface:
    """Abstract interface. Both RealIO and SimulatedIO implement this."""
    
    def read_sensors(self) -> np.ndarray:
        """Return measurement vector z (6 elements)."""
        raise NotImplementedError
    
    def write_actuators(self, u: np.ndarray):
        """Apply control vector u (6-7 elements)."""
        raise NotImplementedError
    
    def get_disturbance_forecast(self) -> np.ndarray:
        """Return disturbance forecast over prediction horizon."""
        raise NotImplementedError


class RealIO(IOInterface):
    """Production: reads from Redis (sensors via MQTT), writes via MQTT/Modbus."""
    
    def __init__(self, redis_client, mqtt_client, modbus_client):
        self.redis = redis_client
        self.mqtt = mqtt_client
        self.modbus = modbus_client
    
    def read_sensors(self):
        esp32 = self.redis.hgetall('sensors:esp32:room1:latest')
        aroya = self.redis.hgetall('sensors:aroya:room1:latest')
        return self._build_measurement_vector(esp32, aroya)
    
    def write_actuators(self, u):
        # HVACD via Modbus
        self.modbus.write_register(HVACD_TEMP_SP_REG, u[0])
        self.modbus.write_register(HVACD_RH_SP_REG, u[1])
        self.modbus.write_register(HVACD_FAN_REG, u[4])
        # Relays via MQTT → ESP32
        self.mqtt.publish('cortex/room1/co2', u[3])
        self.mqtt.publish('cortex/room1/humidifier', u[2])
        self.mqtt.publish('cortex/room1/irrigation', u[5])
    
    def get_disturbance_forecast(self):
        # Light schedule is known, weather from API or outdoor sensor
        return self._build_forecast()


class SimulatedIO(IOInterface):
    """Simulation: reads/writes directly to SimulatedRoom."""
    
    def __init__(self, sim_room):
        self.room = sim_room
        self.last_z = None
        self.last_d = None
    
    def read_sensors(self):
        return self.last_z  # Set by simulation loop after room.step()
    
    def write_actuators(self, u):
        # Apply to simulated room and advance physics
        self.last_z, self.last_d = self.room.step(u, dt=1.0)
    
    def get_disturbance_forecast(self):
        # In simulation, we know the full schedule perfectly
        return self.room.disturbance_schedule.forecast(
            self.room.time, horizon_s=3600
        )


class ControlLoop:
    """
    SAME class for production and simulation.
    The only difference is which IOInterface it's given.
    """
    
    def __init__(self, io: IOInterface, dynamics, params):
        self.io = io
        self.ekf = StateEstimator()
        self.mpc = MPCSolver(dynamics, params)
        self.compliance = ComplianceTracker()
        self.reference = ReferenceGenerator()
        # ... same as production
    
    def tick(self):
        """One control cycle. Called at 1 Hz."""
        z_meas = self.io.read_sensors()
        x_hat, P = self.ekf.update(z_meas, self.last_u, ...)
        u_opt, trajectory, info = self.mpc.solve(x_hat, ...)
        self.io.write_actuators(u_opt)
        self.compliance.update(x_hat, ...)
        self.last_u = u_opt
        return x_hat, u_opt, trajectory, info
```

### Simulation Modes

```
MODE 1: REAL-TIME SIMULATION (development)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Runs at wall-clock speed (1 Hz).
  Dashboard connects and shows live data.
  Used for: interactive development, demo to growers.
  
  sim = SimulatedRoom(config, initial_state)
  io = SimulatedIO(sim)
  loop = ControlLoop(io, dynamics, params)
  
  while True:
      loop.tick()
      time.sleep(1.0)

MODE 2: FAST-FORWARD SIMULATION (testing, tuning)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Runs as fast as possible. 24 hours in ~30 seconds.
  No sleep between cycles — just compute.
  Used for: unit tests, CI pipeline, parameter sweeps.
  
  sim = SimulatedRoom(config, initial_state)
  io = SimulatedIO(sim)
  loop = ControlLoop(io, dynamics, params)
  
  for step in range(86400):  # 24 hours at 1 Hz
      loop.tick()
  
  report = loop.compliance.report()
  assert report['VPD']['compliance_pct'] > 95.0

MODE 3: REPLAY SIMULATION (calibration, post-mortem)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Replays actual recorded data through the model.
  Compares model prediction to what actually happened.
  Used for: SLSQP calibration, incident analysis.
  
  history = load_telemetry('room1', '2026-02-20', '2026-02-21')
  
  x_sim = history.x[0]
  for k in range(len(history)):
      x_sim = dynamics.nonlinear_step(
          x_sim, history.u[k], history.d[k], params, dt=1.0
      )
      prediction_error[k] = x_sim - history.x[k+1]
  
  # This IS the SLSQP objective function

MODE 4: MONTE CARLO (robustness analysis)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Run Mode 2 hundreds of times with:
  - Randomized initial parameters (±30% from nominal)
  - Randomized outdoor weather profiles
  - Randomized sensor noise realizations
  - Random fault injection (HVAC trips, sensor dropout)
  
  Produces: distribution of compliance metrics.
  Answers: "How robust is the controller to model uncertainty?"
```

---

## System 3: Background Calibration (SLSQP)

### How It Connects

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  PRODUCTION CONTROL (real-time, every 1s)                           │
│                                                                     │
│  Sensors → EKF → OSQP(A_d, B_d from params) → Actuators            │
│                         ↑                                           │
│                    A_d, B_d depend on                                │
│                    model parameters p                                │
│                         ↑                                           │
│                  ┌──────┴───────┐                                   │
│                  │ SharedParams │ ← atomic read by OSQP (lock-free) │
│                  │              │ ← atomic write by SLSQP           │
│                  └──────┬───────┘                                   │
│                         ↑                                           │
│                    new params                                        │
│                    (only if cost improved)                           │
│                         ↑                                           │
│  ┌──────────────────────┴──────────────────────────────────┐        │
│  │  SLSQP CALIBRATOR (background, every 5-30 min)          │        │
│  │                                                          │        │
│  │  INPUT:                                                  │        │
│  │  ┌────────────────────────────────────────────────────┐  │        │
│  │  │ From Redis ring buffer (last 1-4 hours):           │  │        │
│  │  │   x_actual[k]  — EKF state estimates at each step  │  │        │
│  │  │   u_applied[k] — control actions that were sent     │  │        │
│  │  │   d_observed[k] — disturbances that were measured   │  │        │
│  │  │                                                     │  │        │
│  │  │ This is what ACTUALLY HAPPENED in the room.         │  │        │
│  │  └────────────────────────────────────────────────────┘  │        │
│  │                                                          │        │
│  │  PROCESS:                                                │        │
│  │  ┌────────────────────────────────────────────────────┐  │        │
│  │  │ For candidate parameter vector p_candidate:        │  │        │
│  │  │                                                     │  │        │
│  │  │   x_sim = x_actual[0]  (start from real data)      │  │        │
│  │  │                                                     │  │        │
│  │  │   for k in range(N):                                │  │        │
│  │  │     x_sim = nonlinear_step(                         │  │        │
│  │  │       x_sim, u_applied[k], d_observed[k],           │  │        │
│  │  │       p_candidate, dt=1.0                           │  │        │
│  │  │     )                                                │  │        │
│  │  │     error += weight[k] * ||x_sim - x_actual[k+1]|| │  │        │
│  │  │                                                     │  │        │
│  │  │ SLSQP minimizes error over p_candidate              │  │        │
│  │  │ Subject to: physical bounds on each parameter       │  │        │
│  │  └────────────────────────────────────────────────────┘  │        │
│  │                                                          │        │
│  │  OUTPUT:                                                 │        │
│  │  ┌────────────────────────────────────────────────────┐  │        │
│  │  │ p_new — updated model parameters                   │  │        │
│  │  │                                                     │  │        │
│  │  │ Only applied if:                                    │  │        │
│  │  │   cost(p_new) < cost(p_current) × 1.01             │  │        │
│  │  │   (must actually improve fit, not just converge)    │  │        │
│  │  │                                                     │  │        │
│  │  │ Written to SharedParams via atomic swap.            │  │        │
│  │  │ Next OSQP cycle picks up new params automatically.  │  │        │
│  │  └────────────────────────────────────────────────────┘  │        │
│  └──────────────────────────────────────────────────────────┘        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### What SLSQP Tunes vs What It Doesn't

```
SLSQP TUNES (parameters — the physics of the room):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  UA_wall     → How fast heat moves between air and walls
  UA_env      → How fast heat leaks to outdoors
  τ_hvac      → How fast the HVAC unit responds
  g_s_max     → How much the canopy transpires at peak
  LAI         → How big the canopy is (leaf area)
  K_drain     → How fast water drains from substrate
  C_wall      → How much heat the building stores
  η_hvac      → Actual HVAC efficiency (vs rated)

  These feed into A_d, B_d matrices that OSQP uses.
  Getting them right = accurate predictions = good control.

SLSQP DOES NOT TUNE (these are set by the grower/operator):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  VPD target          → Grower's cultivation strategy
  Temperature target  → Phase-dependent setpoint
  CO2 target          → Phase-dependent setpoint
  Dryback targets     → Irrigation strategy (veg vs generative)
  Q, R, S weights     → Control priority/energy tradeoff
                         (future: nightly meta-optimization)

SLSQP DOES NOT TUNE (these are measured directly by sensors):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  T_air, w_a, CO2     → ESP32 SHT31, SCD30
  T_leaf              → ESP32 MLX90614
  θ (substrate VWC)   → TEROS ONE or AROYA
  T_supply            → ESP32 thermocouple
  T_outdoor           → ESP32 or weather API
  Q_light             → Light schedule (deterministic)
```

---

## Component Dependency Graph

This shows which component depends on which, and what data flows between them.

```
                    ┌──────────────────┐
                    │   room.yaml      │ Room geometry, construction
                    │   equipment.yaml │ HVAC specs, actuator limits
                    │   cultivar.yaml  │ Per-strain growth profiles
                    │   phases/*.yaml  │ Phase-specific targets
                    └────────┬─────────┘
                             │ loaded at startup
                             ▼
              ┌──────────────────────────────┐
              │    GrowRoomDynamics           │
              │    (core physics model)       │
              │                              │
              │ Methods:                     │
              │  .nonlinear_step()           │
              │  .linearize()               │
              │  .derived_outputs()          │
              │  ._derivatives()            │
              │  ._analytical_jacobians()   │
              │                              │
              │ Sub-models:                  │
              │  .hvacd (HVACDModel)        │
              │  .transpiration (PM model)   │
              │  .photosynthesis (MM model)  │
              └──────┬────────┬────────┬────┘
                     │        │        │
          ┌──────────┘        │        └──────────┐
          │                   │                    │
          ▼                   ▼                    ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ StateEstimator  │ │ MPCSolver       │ │ ModelCalibrator  │
│ (EKF)           │ │ (OSQP)          │ │ (SLSQP)          │
│                 │ │                 │ │                  │
│ Uses:           │ │ Uses:           │ │ Uses:            │
│ .nonlinear_step │ │ .linearize()   │ │ .nonlinear_step  │
│  (predict step) │ │  (A_d, B_d)    │ │  (replay data)   │
│ ._jacobians()   │ │                 │ │                  │
│  (F matrix)     │ │ Also needs:     │ │ Also needs:      │
│                 │ │ - SharedParams  │ │ - Telemetry data │
│ Also needs:     │ │ - Reference     │ │ - SharedParams   │
│ - Sensor data   │ │ - Forecast      │ │   (read & write) │
│ - Last u        │ │ - Constraints   │ │                  │
│                 │ │                 │ │ Runs:            │
│ Runs:           │ │ Runs:           │ │  every 5-30 min  │
│  every 1s       │ │  every 1s       │ │  in background   │
│  in control loop│ │  in control loop│ │  thread          │
│                 │ │                 │ │                  │
│ Output:         │ │ Output:         │ │ Output:          │
│  x_hat (7)      │ │  u_opt (6-7)    │ │  params_new      │
│  P (7×7)        │ │  trajectory     │ │                  │
│  innovation     │ │  solve_info     │ │                  │
└────────┬────────┘ └────────┬────────┘ └────────┬─────────┘
         │                   │                    │
         └─────────┬─────────┘                    │
                   │                              │
                   ▼                              │
         ┌─────────────────┐                      │
         │ ControlLoop     │                      │
         │                 │                      │
         │ Orchestrates:   │◀─────────────────────┘
         │  1. io.read()   │  (triggers fast calibration
         │  2. ekf.update()│   when innovation too high)
         │  3. mpc.solve() │
         │  4. io.write()  │
         │  5. log/comply  │
         │                 │
         │ Also owns:      │
         │ - ComplianceTracker
         │ - FailSafeManager
         │ - TelemetryWriter
         └────────┬────────┘
                  │
         ┌───────┴────────┐
         │                │
         ▼                ▼
┌──────────────┐  ┌──────────────┐
│ RealIO       │  │ SimulatedIO  │
│ (production) │  │ (testing)    │
│              │  │              │
│ Redis/MQTT/  │  │ SimulatedRoom│
│ Modbus       │  │              │
└──────────────┘  └──────────────┘
```

---

## Auxiliary Components

### ReferenceGenerator

```python
class ReferenceGenerator:
    """
    Produces the target trajectory r[k] over the prediction horizon.
    
    This is WHAT the MPC is trying to achieve. Changes when:
    - Growth phase changes (veg → flower → dry)
    - Light state changes (day targets vs night targets)
    - Cultivar profile loaded
    - Grower overrides a target
    
    Does NOT change every MPC cycle (only when the above happen).
    Pre-computed and cached.
    """
    
    def __init__(self, phase_configs, cultivar_config):
        self.phases = phase_configs      # from phases/*.yaml
        self.cultivar = cultivar_config  # from cultivar/*.yaml
    
    def generate(self, current_phase, current_time, light_schedule, horizon_steps=60):
        """
        Returns: reference trajectory (horizon_steps × n_outputs)
        
        Each step contains targets for:
          [VPD_target, T_target, CO2_target, dryback_rate_target, theta_target]
        
        Accounts for:
        - Day/night setpoint transitions within the horizon
        - Ramp rates (don't step-change targets — ramp over 10-15 min)
        - Phase-specific targets from config
        - Cultivar-specific VPD preferences
        """
        ref = np.zeros((horizon_steps, self.n_outputs))
        
        for k in range(horizon_steps):
            future_time = current_time + k * 60  # 60s per prediction step
            lights_on = light_schedule.is_on(future_time)
            
            targets = self.phases[current_phase]
            ref[k] = targets['day'] if lights_on else targets['night']
            
            # Apply cultivar-specific overrides
            if self.cultivar.get('vpd_offset'):
                ref[k, 0] += self.cultivar['vpd_offset']
        
        return ref
```

### DisturbanceForecast

```python
class DisturbanceForecast:
    """
    Predicts disturbances over the prediction horizon.
    
    Some disturbances are known exactly (light schedule).
    Some are forecasted (outdoor weather).
    Some are effectively constant (plant count, phase).
    """
    
    def generate(self, current_time, current_d, horizon_steps=60):
        """
        Returns: disturbance forecast (horizon_steps × n_disturbances)
        """
        forecast = np.zeros((horizon_steps, self.n_d))
        
        for k in range(horizon_steps):
            future_time = current_time + k * 60
            
            # Lighting: KNOWN EXACTLY from schedule
            forecast[k, 0] = self.light_schedule.heat_load(future_time)
            
            # Outdoor weather: forecasted (simple persistence or API)
            # For now: hold current outdoor conditions constant
            # Future: weather API forecast
            forecast[k, 1] = current_d[1]  # T_out
            forecast[k, 2] = current_d[2]  # w_out
            
            # Plant/phase: constant over horizon
            forecast[k, 3] = current_d[3]  # N_plants
            forecast[k, 4] = current_d[4]  # phase
        
        return forecast
```

### FailSafeManager

```python
class FailSafeManager:
    """
    Monitors system health and manages graceful degradation.
    
    Five priority levels (see implementation spec):
    
    P1: MPC normal → full optimization
    P2: MPC fails  → hold last valid u for 30s
    P3: MPC offline → ESP32 local PID on cached setpoints
    P4: ESP32 offline → TrolMaster static setpoints (if present)
    P5: Total loss → HVAC built-in safety limits + normally-closed solenoids
    
    This class manages P1 → P2 transitions.
    P3-P5 are handled by firmware and external equipment.
    """
    
    def __init__(self):
        self.consecutive_failures = 0
        self.last_valid_u = None
        self.last_valid_time = None
        self.max_hold_seconds = 30
    
    def check_solve(self, result_status, u_opt, solve_info):
        """Called after every MPC solve."""
        
        if result_status in ('solved', 'solved_inaccurate'):
            self.consecutive_failures = 0
            self.last_valid_u = u_opt
            self.last_valid_time = time.time()
            return u_opt, 'P1_NORMAL'
        
        self.consecutive_failures += 1
        
        if self.consecutive_failures <= 30:
            # P2: Hold last valid control action
            return self.last_valid_u, 'P2_HOLD_LAST'
        
        # P2 exhausted: fall to P3
        # Push last-known setpoints to ESP32 flash via MQTT
        # ESP32 firmware switches to local PID mode
        self._push_failsafe_setpoints()
        return None, 'P3_ESP32_PID'
```

---

## Data Stores

### Redis (Hot Store)

```
KEY                              TYPE       TTL       PURPOSE
─────────────────────────────────────────────────────────────────
sensors:esp32:room1:latest       HASH       10s       Latest ESP32 readings
sensors:aroya:room1:latest       HASH       600s      Latest AROYA readings
commands:room1:latest            HASH       5s        Current actuator commands

telemetry:room1:x_hat            LIST       4h        EKF state estimates (ring buffer)
telemetry:room1:u_applied        LIST       4h        Applied controls (ring buffer)
telemetry:room1:d_observed       LIST       4h        Measured disturbances (ring buffer)
telemetry:room1:z_raw            LIST       4h        Raw sensor readings (ring buffer)
telemetry:room1:innovation       LIST       4h        EKF prediction error (ring buffer)
telemetry:room1:solve_info       LIST       4h        OSQP solve timing (ring buffer)

params:room1:current             HASH       —         Current model parameters
params:room1:history             LIST       24h       Parameter calibration history

compliance:room1:latest          HASH       —         Latest compliance report
compliance:room1:history         LIST       24h       Compliance report history

mpc:room1:trajectory             STRING     2s        Latest predicted trajectory (for dashboard)
mpc:room1:reference              STRING     60s       Current reference trajectory
```

**Memory footprint:** 7 states × 4 bytes × 14400 samples (4 hrs at 1 Hz) × 6 lists ≈ **2.4 MB**. Negligible for 8 GB Pi 5.

### SQLite (Cold Store)

```sql
-- Full telemetry archive (one row per second)
CREATE TABLE telemetry (
    timestamp   REAL PRIMARY KEY,    -- unix epoch
    room_id     TEXT NOT NULL,
    
    -- State estimate (from EKF)
    T_air       REAL, w_a       REAL, CO2       REAL,
    T_leaf      REAL, theta     REAL, T_supply  REAL,  T_wall  REAL,
    
    -- Raw sensor readings
    z_T_air     REAL, z_RH      REAL, z_CO2     REAL,
    z_T_leaf    REAL, z_theta   REAL, z_T_supply REAL,
    
    -- Applied control
    u_cool      REAL, u_heat    REAL, u_hum     REAL,
    u_co2       REAL, u_fan     REAL, u_irr     REAL,
    
    -- Disturbances
    Q_light     REAL, T_out     REAL, w_out     REAL,
    
    -- Derived
    VPD         REAL, RH        REAL, dryback   REAL,
    
    -- MPC diagnostics
    solve_ms    REAL, iterations INTEGER, status TEXT,
    
    -- Calibration state
    params_version INTEGER
);

-- Index for SLSQP time-range queries
CREATE INDEX idx_telemetry_time ON telemetry(timestamp);

-- Calibration events
CREATE TABLE calibration (
    timestamp       REAL PRIMARY KEY,
    cost_before     REAL,
    cost_after      REAL,
    params          TEXT,  -- JSON blob
    iterations      INTEGER,
    solve_time_s    REAL,
    trigger_reason  TEXT
);

-- Phase transitions
CREATE TABLE phases (
    timestamp   REAL PRIMARY KEY,
    room_id     TEXT,
    phase       TEXT,
    cultivar    TEXT,
    notes       TEXT
);
```

---

## Complete File Structure

```
cortex/
├── core/
│   ├── __init__.py
│   ├── dynamics.py             # GrowRoomDynamics — THE core model
│   │                           #   .nonlinear_step()
│   │                           #   .linearize()
│   │                           #   .derived_outputs()
│   │                           #   ._derivatives() (7 ODEs)
│   │                           #   ._analytical_jacobians() (Numba JIT)
│   │                           #   ._discretize_pade2()
│   ├── hvacd.py                # HVACDModel — multi-mode HVAC dynamics
│   │                           #   .step() → Q_sensible, Q_latent, moisture, T_supply
│   │                           #   ._step_setpoint_mode()
│   │                           #   ._step_direct_mode()
│   │                           #   ._apply_dynamics() (coil, compressor, reheat lags)
│   ├── minisplit.py            # MiniSplitModel — simpler single-setpoint HVAC
│   ├── transpiration.py        # Penman-Monteith with stomatal conductance
│   │                           #   .rate(VPD, PAR, theta, CO2, LAI) → g/m²/hr
│   │                           #   .stomatal_conductance(PAR, VPD, theta, CO2)
│   └── thermodynamics.py       # Shared utility functions
│                               #   saturation_humidity(T)
│                               #   dew_point_from_w(w)
│                               #   saturation_vapor_pressure(T)
│
├── mpc/
│   ├── __init__.py
│   ├── solver.py               # MPCSolver — OSQP wrapper
│   │                           #   .solve(x_hat, d_forecast, reference, params)
│   │                           #   ._build_qp() — sparse matrix construction
│   │                           #   ._shift_warmstart() — warm-starting from prev
│   │                           #   ._extract_trajectory()
│   ├── reference.py            # ReferenceGenerator — target trajectories
│   │                           #   .generate(phase, time, light_schedule)
│   ├── constraints.py          # ConstraintBuilder — safety/comfort/equipment
│   │                           #   .build(x_hat, reference, equipment_limits)
│   ├── forecast.py             # DisturbanceForecast — d over horizon
│   │                           #   .generate(current_time, current_d)
│   └── generated/              # OSQP code-generated C solver
│       ├── osqp_solver.c       #   (compiled once, called via ctypes)
│       ├── osqp_solver.h
│       ├── Makefile
│       └── build.sh
│
├── estimation/
│   ├── __init__.py
│   ├── ekf.py                  # StateEstimator — Extended Kalman Filter
│   │                           #   .update(z_meas, u, d, dt, params)
│   │                           #   .build_measurement(esp32_data, aroya_data)
│   │                           #   ._observation_matrix(available_sensors)
│   └── fusion.py               # Sensor fusion utilities
│                               #   Staleness detection
│                               #   Multi-sensor inverse-variance weighting
│
├── calibration/
│   ├── __init__.py
│   ├── calibrator.py           # ModelCalibrator — SLSQP background optimizer
│   │                           #   .start() / .stop()
│   │                           #   ._run_loop() (background thread)
│   │                           #   ._calibrate() → (params, cost, info)
│   │                           #   .trigger_fast_calibration()
│   ├── params.py               # SharedParams — thread-safe param store
│   │                           #   .current() — lock-free read
│   │                           #   .update() — atomic swap
│   │                           # CalibrationParams dataclass
│   └── bounds.py               # PARAM_BOUNDS — physical limits per parameter
│
├── control/
│   ├── __init__.py
│   ├── loop.py                 # ControlLoop — main 1 Hz orchestrator
│   │                           #   .tick() — one complete control cycle
│   │                           #   .run() — async loop
│   ├── io.py                   # IOInterface (abstract)
│   │                           # RealIO (Redis/MQTT/Modbus)
│   │                           # SimulatedIO (SimulatedRoom)
│   ├── actuators.py            # Low-level actuator drivers
│   │                           #   Modbus register writes (HVACD)
│   │                           #   MQTT publishes (ESP32 relays)
│   │                           #   0-10V DAC output
│   ├── sensors.py              # Low-level sensor readers
│   │                           #   Redis subscription
│   │                           #   AROYA API polling
│   │                           #   Staleness detection
│   ├── failsafe.py             # FailSafeManager — degradation logic
│   └── compliance.py           # ComplianceTracker — rolling metrics
│
├── simulation/
│   ├── __init__.py
│   ├── room.py                 # SimulatedRoom — software grow room
│   │                           #   .step(u, dt) → z_meas, d
│   │                           #   .get_true_state() (analysis only)
│   ├── disturbance.py          # DisturbanceScheduler — light/weather/phase
│   ├── noise.py                # SensorNoiseModel — realistic measurement noise
│   ├── faults.py               # FaultInjector — simulate failures
│   └── scenarios.py            # Scenario definitions for testing
│                               #   24h_mid_flower, lights_transition,
│                               #   hvac_failure, phase_change, etc.
│
├── api/
│   ├── __init__.py
│   ├── server.py               # FastAPI application
│   ├── routes/
│   │   ├── dashboard.py        # WebSocket: live state + trajectory
│   │   ├── config.py           # REST: room/equipment/cultivar config
│   │   ├── override.py         # REST: manual overrides + setpoint changes
│   │   ├── compliance.py       # REST: compliance reports + history
│   │   └── calibration.py      # REST: calibration status + param history
│   └── ws.py                   # WebSocket manager
│
├── config/
│   ├── room.yaml               # V, A_floor, UA_wall (initial estimates), etc.
│   ├── equipment.yaml          # HVAC specs, Modbus addresses, actuator limits
│   ├── cultivar/
│   │   ├── default.yaml        # Generic cannabis profile
│   │   └── white_runtz.yaml    # Strain-specific VPD/temp preferences
│   └── phases/
│       ├── clone.yaml
│       ├── veg.yaml
│       ├── early_flower.yaml
│       ├── mid_flower.yaml
│       ├── late_flower.yaml
│       └── dry.yaml
│
├── tests/
│   ├── test_dynamics.py        # Unit: ODE correctness, Jacobian accuracy
│   ├── test_hvacd.py           # Unit: HVACD mode switching, thermal lags
│   ├── test_solver_speed.py    # Benchmark: assert solve_time < 100ms on target HW
│   ├── test_ekf.py             # Unit: EKF convergence with noisy sensors
│   ├── test_calibrator.py      # Unit: SLSQP converges to known params
│   ├── test_compliance.py      # Integration: 24h sim, check compliance targets
│   ├── test_failsafe.py        # Integration: fault injection, check degradation
│   ├── test_io_adapter.py      # Unit: SimulatedIO matches RealIO interface
│   └── test_full_loop.py       # Integration: full sim with all components
│
├── scripts/
│   ├── run_simulation.py       # Run a scenario, plot results
│   ├── benchmark_solver.py     # OSQP timing benchmarks on current hardware
│   ├── calibrate_offline.py    # Run SLSQP against recorded telemetry
│   ├── generate_osqp_code.py   # Generate C solver from QP structure
│   └── replay_telemetry.py     # Replay recorded data through model
│
└── main.py                     # Entry point: load config, start all processes
```

---

## Startup Sequence

```python
# main.py — production startup

def main():
    # 1. Load configuration
    room_config = load_yaml('config/room.yaml')
    equipment_config = load_yaml('config/equipment.yaml')
    cultivar_config = load_yaml('config/cultivar/white_runtz.yaml')
    phase_config = load_yaml('config/phases/mid_flower.yaml')
    
    # 2. Initialize core physics model
    initial_params = CalibrationParams()  # defaults from config
    shared_params = SharedParams(initial_params)
    dynamics = GrowRoomDynamics(room_config, equipment_config, shared_params)
    
    # 3. Initialize MPC solver
    reference_gen = ReferenceGenerator(phase_config, cultivar_config)
    forecast_gen = DisturbanceForecast(room_config)
    mpc = MPCSolver(dynamics, shared_params)
    
    # 4. Initialize state estimator
    ekf = StateEstimator(dynamics)
    
    # 5. Initialize IO (production)
    redis = Redis()
    mqtt = MQTTClient()
    modbus = ModbusClient(equipment_config['hvacd']['modbus_address'])
    io = RealIO(redis, mqtt, modbus)
    
    # 6. Initialize compliance + failsafe
    compliance = ComplianceTracker(phase_config)
    failsafe = FailSafeManager()
    
    # 7. Build control loop
    loop = ControlLoop(
        io=io,
        dynamics=dynamics,
        ekf=ekf,
        mpc=mpc,
        reference=reference_gen,
        forecast=forecast_gen,
        compliance=compliance,
        failsafe=failsafe,
        shared_params=shared_params,
    )
    
    # 8. Start background calibrator
    calibrator = ModelCalibrator(dynamics, redis, shared_params)
    calibrator.start()  # background thread
    
    # 9. Start API server
    api = create_api(loop, compliance, shared_params, calibrator)
    api_thread = threading.Thread(target=uvicorn.run, args=(api,), kwargs={'port': 8000})
    api_thread.start()
    
    # 10. Start MQTT bridge (separate process)
    mqtt_bridge = MQTTBridge(redis, mqtt)
    mqtt_bridge.start()
    
    # 11. Run control loop (blocks — this is the main loop)
    asyncio.run(loop.run())
```

---

## Key Invariants

These must always be true for the system to function correctly:

```
1. GrowRoomDynamics is the SINGLE SOURCE OF TRUTH for room physics.
   EKF, OSQP, SLSQP, and SimulatedRoom all use the same model code.
   There is never a second, divergent copy of the ODEs.

2. OSQP never sees the nonlinear model directly.
   It only sees linearized A_d, B_d matrices.
   Linearization happens every MPC cycle (1 Hz).

3. SLSQP never runs in the control loop.
   It runs in a background thread on a separate core.
   It writes params via atomic swap. Control loop reads lock-free.

4. The EKF predict step uses the SAME nonlinear model as the simulation.
   The EKF update step uses real sensor data to correct prediction errors.
   The EKF is what makes the linear OSQP model work despite nonlinearities.

5. SharedParams is the ONLY bridge between calibration and control.
   SLSQP writes to it. OSQP reads from it. No other shared state.

6. The IO adapter pattern means the control loop code is IDENTICAL
   for production and simulation. Only the IO backend changes.

7. Fail-safe degradation is layered and independent.
   Each layer works without the layers above it.
   Removing MPC degrades to PID. Removing PID degrades to static setpoints.
   Removing setpoints degrades to hardware safety limits.

8. All sensor data enters through Redis. All commands exit through Redis.
   The control loop never directly touches MQTT or Modbus.
   The MQTT Bridge process handles all protocol translation.

9. The 100ms solve budget is for the ENTIRE MPC step:
   linearize + build QP + solve + extract.
   Not just the OSQP solve call.

10. The simulation environment uses the physics model with DIFFERENT
    parameters than the controller, to test robustness to model error.
    In production, SLSQP minimizes this gap. In simulation, we 
    intentionally maintain it to test controller robustness.
```

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 0.1 | 2026-02-22 | Initial architecture — component map, data flow, simulation environment, IO adapter pattern, file structure, startup sequence, invariants |
