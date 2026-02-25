"""IO adapter pattern — same controller, two backends.

IOInterface is the abstract base. Both RealIO (production) and
SimulatedIO (testing) implement it. The SolverProcess doesn't know
which backend it's talking to.
"""

import numpy as np
from abc import ABC, abstractmethod


class IOInterface(ABC):
    """Abstract IO interface for the control loop."""

    @abstractmethod
    def read_sensors(self) -> np.ndarray:
        """Return measurement vector z (6 elements)."""
        ...

    @abstractmethod
    def write_actuators(self, u: np.ndarray):
        """Apply control vector u (6 elements)."""
        ...

    @abstractmethod
    def get_disturbance(self) -> np.ndarray:
        """Return current disturbance vector d (5 elements)."""
        ...

    @abstractmethod
    def get_time(self) -> float:
        """Return current time in seconds."""
        ...


class SimulatedIO(IOInterface):
    """Simulation backend: reads/writes directly to SimulatedRoom."""

    def __init__(self, sim_room):
        self.room = sim_room
        self.last_z = None
        self.last_d = None
        self._initialized = False

    def read_sensors(self) -> np.ndarray:
        if self.last_z is None:
            # First read: generate initial measurements from true state
            self.last_z = self.room.noise.corrupt(self.room.x_true, self.room.time)
            self.last_d = self.room.disturbance_schedule.get(self.room.time)
        return self.last_z.copy()

    def write_actuators(self, u: np.ndarray):
        # Apply control to simulated room and advance physics
        self.last_z, self.last_d = self.room.step(u, dt=1.0)

    def get_disturbance(self) -> np.ndarray:
        if self.last_d is None:
            return self.room.disturbance_schedule.get(self.room.time)
        return self.last_d.copy()

    def get_time(self) -> float:
        return self.room.time


class RealIO(IOInterface):
    """Production backend: Redis for sensors, MQTT/Modbus for actuators.

    Requires redis, paho-mqtt, and pymodbus at runtime.
    """

    def __init__(self, redis_client, registry, mqtt_client=None,
                 modbus_client=None, room_id='room1'):
        self.redis = redis_client
        self.registry = registry
        self.mqtt = mqtt_client
        self.modbus = modbus_client
        self.room_id = room_id
        self._start_time = None

    def read_sensors(self) -> np.ndarray:
        import time
        if self._start_time is None:
            self._start_time = time.time()

        # Read from Redis
        esp32_key = f'sensors:esp32:{self.room_id}:latest'
        aroya_key = f'sensors:aroya:{self.room_id}:latest'

        esp32 = self.redis.hgetall(esp32_key)
        aroya = self.redis.hgetall(aroya_key)

        return self._build_measurement_vector(esp32, aroya)

    def write_actuators(self, u: np.ndarray):
        # Write to Redis for MQTT bridge to pick up
        cmd_key = f'commands:{self.room_id}:latest'
        mapping = {}
        for i, io_map in enumerate(self.registry.all_io_mappings()):
            mapping[io_map.io_field] = float(u[i])
        self.redis.hset(cmd_key, mapping=mapping)

    def get_disturbance(self) -> np.ndarray:
        # Light schedule is deterministic, outdoor from sensor or API
        # Simplified: read from Redis if available
        d_key = f'disturbance:{self.room_id}:latest'
        data = self.redis.hgetall(d_key)
        if data:
            return np.array([
                float(data.get(b'Q_light', 0)),
                float(data.get(b'T_out', 28)),
                float(data.get(b'w_out', 12)),
                float(data.get(b'N_plants', 100)),
                float(data.get(b'phase', 3)),
            ])
        # Fallback: typical mid-flower day disturbance
        return np.array([30000.0, 28.0, 12.0, 100.0, 3.0])

    def get_time(self) -> float:
        import time
        if self._start_time is None:
            self._start_time = time.time()
        return time.time() - self._start_time

    def _build_measurement_vector(self, esp32, aroya):
        """Convert Redis hash data to measurement vector."""
        z = np.full(6, np.nan)

        if esp32:
            z[0] = float(esp32.get(b'T_air', esp32.get('T_air', np.nan)))
            # Convert RH to absolute humidity if needed
            T = z[0]
            RH = float(esp32.get(b'RH', esp32.get('RH', np.nan)))
            if not np.isnan(T) and not np.isnan(RH):
                from core.thermodynamics import absolute_humidity_from_rh
                z[1] = absolute_humidity_from_rh(T, RH)
            z[2] = float(esp32.get(b'CO2', esp32.get('CO2', np.nan)))
            z[3] = float(esp32.get(b'T_leaf', esp32.get('T_leaf', np.nan)))
            z[5] = float(esp32.get(b'T_supply', esp32.get('T_supply', np.nan)))

        if aroya:
            z[4] = float(aroya.get(b'theta', aroya.get('theta', np.nan)))

        return z
