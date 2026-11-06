"""
flightstack/vehicle_model.py
============================
First-principles multi-rotor mass, aerodynamic, and battery energy model.

Key distinctions:
  - Models physical mass breakdown: empty airframe, battery pack, and payload.
  - Enforces Maximum Takeoff Weight (MTOW) constraint:
        m_total = m_empty + m_battery + m_cargo <= MTOW
  - Payload mass-dependent power scaling: based on momentum theory for rotary-wing
    aircraft, hover/induced power scales with mass^(1.5):
        P_induced ~ sqrt( (m * g)^3 / (2 * rho * A) )
    As deliveries occur, vehicle mass decreases, reducing energy consumption
    on subsequent legs and the return-to-base flight.
  - Distinguishes theoretical/calibrated vehicle model predictions from
    observed autopilot simulator telemetry.

Pure Python standard library (math, dataclasses) for fast verification and CI.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

GRAVITY = 9.80665              # m/s^2
AIR_DENSITY_SEA_LEVEL = 1.225  # kg/m^3


@dataclass
class BatterySpec:
    """Battery pack electrical specifications."""
    nominal_voltage_v: float = 22.2    # 6S LiPo nominal (3.7V/cell)
    capacity_mah: float = 16000.0      # 16 Ah high-capacity medical logistics pack
    reserve_fraction: float = 0.20     # Minimum 20% state-of-charge reserve at landing
    cell_count: int = 6

    @property
    def total_energy_wh(self) -> float:
        return (self.nominal_voltage_v * self.capacity_mah) / 1000.0

    @property
    def usable_energy_wh(self) -> float:
        return self.total_energy_wh * (1.0 - self.reserve_fraction)


@dataclass
class VehicleProfile:
    """Physical multicopter mass, propulsion, and aerodynamic properties."""
    name: str = "MedDrone-Hexa-16"
    empty_mass_kg: float = 5.5          # Airframe, motors, ESCs, avionics, landing gear
    battery_mass_kg: float = 3.2        # Battery pack mass
    max_payload_kg: float = 8.0         # Maximum cargo payload bay capacity
    mtow_kg: float = 17.5               # Maximum Take-Off Weight structural/regulatory limit
    num_rotors: int = 6                 # Hexacopter configuration
    rotor_diameter_m: float = 0.55      # ~21.6 inch carbon fiber propellers
    power_transfer_efficiency: float = 0.72  # Figure of merit * motor/ESC electrical efficiency
    parasitic_drag_area_m2: float = 0.12     # Flat-plate equivalent parasite area (Cd * A)
    avionics_power_w: float = 25.0           # Flight controller, companion computer, telemetry

    def __post_init__(self):
        base_mass = self.empty_mass_kg + self.battery_mass_kg
        if base_mass > self.mtow_kg:
            raise ValueError(f"Base empty + battery mass ({base_mass:.1f}kg) exceeds MTOW ({self.mtow_kg:.1f}kg)")
        if base_mass + self.max_payload_kg > self.mtow_kg:
            raise ValueError(
                f"Full payload ({base_mass + self.max_payload_kg:.1f}kg) exceeds MTOW ({self.mtow_kg:.1f}kg)"
            )

    @property
    def total_rotor_disk_area_m2(self) -> float:
        """Total actuator disk area swept by all rotors."""
        radius = self.rotor_diameter_m / 2.0
        return self.num_rotors * math.pi * (radius ** 2)

    def total_mass(self, payload_kg: float) -> float:
        """Calculate total operating mass and enforce structural payload / MTOW limits."""
        if payload_kg < 0:
            raise ValueError("Payload mass cannot be negative")
        if payload_kg > self.max_payload_kg + 1e-6:
            raise ValueError(
                f"Payload {payload_kg:.2f}kg exceeds vehicle cargo bay capacity {self.max_payload_kg:.2f}kg"
            )
        total = self.empty_mass_kg + self.battery_mass_kg + payload_kg
        if total > self.mtow_kg + 1e-6:
            raise ValueError(
                f"Total takeoff weight {total:.2f}kg exceeds certified MTOW {self.mtow_kg:.2f}kg"
            )
        return total

    def hover_power_watts(self, payload_kg: float, air_density: float = AIR_DENSITY_SEA_LEVEL) -> float:
        """Ideal actuator disk induced power + profile power in hover.
        
        P_ideal = sqrt( Thrust^3 / (2 * rho * A) )
        """
        mass = self.total_mass(payload_kg)
        thrust_newtons = mass * GRAVITY
        disk_area = self.total_rotor_disk_area_m2
        
        # Momentum theory induced power
        induced_power = math.sqrt((thrust_newtons ** 3) / (2.0 * air_density * disk_area))
        
        # Aerodynamic profile power + mechanical/electrical losses
        shaft_power = induced_power / self.power_transfer_efficiency
        return shaft_power + self.avionics_power_w

    def cruise_power_watts(
        self,
        payload_kg: float,
        airspeed_ms: float,
        air_density: float = AIR_DENSITY_SEA_LEVEL,
    ) -> float:
        """Total electrical power consumption at cruise velocity.
        
        Combines mass-dependent induced power (reduced in forward flight),
        rotor profile drag, and airframe parasite drag (scaling with v^3).
        """
        if airspeed_ms < 0:
            raise ValueError("Airspeed cannot be negative")
        if airspeed_ms == 0:
            return self.hover_power_watts(payload_kg, air_density)

        mass = self.total_mass(payload_kg)
        thrust_newtons = mass * GRAVITY
        disk_area = self.total_rotor_disk_area_m2

        # Glauert forward-flight induced power approximation:
        # For forward flight, induced velocity v_i satisfies: v_i = T / (2 * rho * A * V')
        # where V' ~= sqrt(V_inf^2 + v_i^2). In cruise, V_inf >> v_i, so v_i ~= T / (2 * rho * A * V_inf)
        induced_power = (thrust_newtons ** 2) / (2.0 * air_density * disk_area * airspeed_ms)
        
        # Clamp induced power so it doesn't exceed hover power at very low speeds
        hover_p = self.hover_power_watts(payload_kg, air_density) - self.avionics_power_w
        induced_power = min(induced_power, hover_p)

        # Parasitic body drag power: P_par = 0.5 * rho * v^3 * (Cd * A)
        parasite_power = 0.5 * air_density * (airspeed_ms ** 3) * self.parasitic_drag_area_m2

        shaft_power = (induced_power + parasite_power) / self.power_transfer_efficiency
        return shaft_power + self.avionics_power_w


    def total_mass_kg(self, payload_kg: float) -> float:
        return self.total_mass(payload_kg)

    @property
    def usable_battery_wh(self) -> float:
        return (22.2 * 16000.0 / 1000.0) * (1.0 - 0.20)


@dataclass
class MulticopterModel(VehicleProfile):
    """Convenience subclass / alias for vehicle profiles with explicit usable battery Wh."""
    battery_usable_wh: float = 284.16

    @property
    def usable_battery_wh(self) -> float:
        return self.battery_usable_wh


@dataclass
class LegEnergyResult:
    from_index: int
    to_index: int
    distance_km: float
    payload_carried_kg: float
    airspeed_ms: float
    flight_time_s: float
    dwell_time_s: float
    cruise_power_w: float
    hover_power_w: float
    energy_wh: float


@dataclass
class RouteEnergyReport:
    is_feasible: bool
    total_energy_wh: float
    usable_energy_wh: float
    total_energy_pct: float
    reserve_margin_pct: float
    initial_cargo_kg: float
    mtow_ok: bool
    legs: List[LegEnergyResult]
    failure_reason: Optional[str] = None


class EnergyEstimator:
    """Evaluates mission route energy feasibility under mass changes."""

    def __init__(self, vehicle: VehicleProfile, battery: Optional[BatterySpec] = None):
        self.vehicle = vehicle
        self.battery = battery or BatterySpec()

    def evaluate_route(
        self,
        route_indices: Any,
        leg_distances_km: Any = None,
        demands_kg: Any = None,
        cruise_speed_ms: float = 11.11,
        dwell_time_s: float = 120.0,
        hover_time_s: Optional[float] = None,
    ) -> RouteEnergyReport:
        """Compute leg-by-leg energy consumption as cargo is delivered.

        Supports both index-based route evaluation and aggregate distance evaluation:
        1. evaluate_route(route_indices, leg_distances_km, demands_kg, ...)
        2. evaluate_route(planned_km, stops_payload, cruise_speed_ms, hover_time_s)
        """
        # Aggregate distance overload: evaluate_route(planned_km, stops_payload, ...)
        if isinstance(route_indices, (int, float)):
            planned_km = float(route_indices)
            stops_payload = leg_distances_km if isinstance(leg_distances_km, list) else []
            h_time = hover_time_s if hover_time_s is not None else dwell_time_s
            
            # Cargo carried for cruise approximation
            rem_cargo = stops_payload[0][1] if stops_payload else 0.0
            p_cruise = self.vehicle.cruise_power_watts(rem_cargo, cruise_speed_ms)
            flight_time_s = (planned_km * 1000.0) / max(cruise_speed_ms, 0.1)
            e_cruise_wh = (p_cruise * flight_time_s) / 3600.0
            
            e_hover_wh = sum(
                (self.vehicle.hover_power_watts(c) * h_time) / 3600.0
                for _, c in stops_payload
            )
            total_wh = e_cruise_wh + e_hover_wh
            usable_wh = getattr(self.vehicle, "usable_battery_wh", self.battery.usable_energy_wh)
            feasible = total_wh <= usable_wh
            return RouteEnergyReport(
                is_feasible=feasible,
                total_energy_wh=total_wh,
                usable_energy_wh=usable_wh,
                total_energy_pct=(total_wh / max(usable_wh, 1.0)) * 100.0,
                reserve_margin_pct=100.0 - (total_wh / max(usable_wh, 1.0)) * 100.0,
                initial_cargo_kg=rem_cargo,
                mtow_ok=True,
                legs=[],
                failure_reason=None if feasible else f"Required {total_wh:.1f}Wh exceeds {usable_wh:.1f}Wh",
            )

        if len(route_indices) < 2:
            return RouteEnergyReport(
                is_feasible=True, total_energy_wh=0.0,
                usable_energy_wh=self.battery.usable_energy_wh,
                total_energy_pct=0.0, reserve_margin_pct=100.0,
                initial_cargo_kg=0.0, mtow_ok=True, legs=[],
            )

        # Initial cargo mass loaded at depot
        initial_cargo = sum(demands_kg[idx] for idx in route_indices if idx != 0)
        
        # Verify MTOW and payload limits
        try:
            self.vehicle.total_mass(initial_cargo)
        except ValueError as e:
            return RouteEnergyReport(
                is_feasible=False, total_energy_wh=0.0,
                usable_energy_wh=self.battery.usable_energy_wh,
                total_energy_pct=0.0, reserve_margin_pct=0.0,
                initial_cargo_kg=initial_cargo, mtow_ok=False,
                legs=[], failure_reason=str(e),
            )

        current_cargo = initial_cargo
        total_energy_wh = 0.0
        legs_result = []

        for leg_idx in range(len(route_indices) - 1):
            u = route_indices[leg_idx]
            v = route_indices[leg_idx + 1]
            dist_km = leg_distances_km[leg_idx]
            dist_m = dist_km * 1000.0

            flight_time_s = dist_m / cruise_speed_ms if cruise_speed_ms > 0 else 0.0
            # Dwell occurs at delivery stop (v != 0); return to depot (v == 0) has 0 delivery dwell
            leg_dwell_s = dwell_time_s if v != 0 else 0.0

            p_cruise = self.vehicle.cruise_power_watts(current_cargo, cruise_speed_ms)
            p_hover = self.vehicle.hover_power_watts(current_cargo)

            # Energy in Watt-hours = (Power * time_seconds) / 3600
            e_cruise_wh = (p_cruise * flight_time_s) / 3600.0
            e_hover_wh = (p_hover * leg_dwell_s) / 3600.0
            leg_energy_wh = e_cruise_wh + e_hover_wh

            total_energy_wh += leg_energy_wh

            legs_result.append(LegEnergyResult(
                from_index=u, to_index=v, distance_km=dist_km,
                payload_carried_kg=current_cargo, airspeed_ms=cruise_speed_ms,
                flight_time_s=flight_time_s, dwell_time_s=leg_dwell_s,
                cruise_power_w=p_cruise, hover_power_w=p_hover,
                energy_wh=leg_energy_wh,
            ))

            # Cargo is dropped at stop v
            if v != 0:
                current_cargo = max(0.0, current_cargo - demands_kg[v])

        usable_wh = self.battery.usable_energy_wh
        total_batt_wh = self.battery.total_energy_wh
        consumed_pct = (total_energy_wh / total_batt_wh) * 100.0
        reserve_pct = 100.0 - consumed_pct

        is_feasible = total_energy_wh <= usable_wh
        reason = None if is_feasible else (
            f"Required energy ({total_energy_wh:.1f} Wh) exceeds usable battery budget "
            f"({usable_wh:.1f} Wh, {self.battery.reserve_fraction*100:.0f}% reserve)"
        )

        return RouteEnergyReport(
            is_feasible=is_feasible,
            total_energy_wh=total_energy_wh,
            usable_energy_wh=usable_wh,
            total_energy_pct=consumed_pct,
            reserve_margin_pct=reserve_pct,
            initial_cargo_kg=initial_cargo,
            mtow_ok=True,
            legs=legs_result,
            failure_reason=reason,
        )
