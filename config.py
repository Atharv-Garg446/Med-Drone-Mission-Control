"""
config.py
=========
Central configuration for the drone route optimizer.

Defines the data structures (CityConfig, DroneConfig, Location) used to model
different delivery scenarios. All algorithms in this project consume these 
objects rather than hardcoding coordinates, making it easy to swap scenarios.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class Location:
    """A single point the drone must either start from (the depot) or visit."""
    id: int
    name: str
    lat: float
    lon: float
    demand_kg: float = 0.0        # 0.0 for the depot; kg of medical supplies needed otherwise
    urgency: str = "routine"      # "critical" | "urgent" | "routine" -- consumed by time_windows.py
    window_minutes: Optional[float] = None  # must arrive within this many minutes of trip departure
    landing_zone_image: Optional[str] = None  # path to a sample image for the YOLO safety check

    # --- Cold-chain constraint ---
    # Maximum CUMULATIVE minutes this item can spend out of refrigeration
    # across the ENTIRE multi-stop trip (not just the final delivery leg).
    # None means no cold-chain requirement (room-temperature supplies).
    # Examples: vaccines (~30 min), blood products (~45 min), insulin (~60 min).
    cold_chain_limit_minutes: Optional[float] = None

    # Time in minutes from mission start (T+0) when this delivery request was created.
    # For initial pre-planned missions this is 0.0; for dynamic emergency events it is tick/60.
    request_time_min: float = 0.0

    # Optional delivery-site suitability descriptor (flightstack.payload.SiteSuitability
    # or compatible dict/object) specifying ground surface condition, slope, and clearance.
    site_suitability: Optional[object] = None


@dataclass
class DroneConfig:
    """Vehicle specifications and operational parameters for the drone fleet.
    
    Defines physical payload capacity, flight range budget, operational speeds,
    fleet sizing, and safety margins for multi-stop delivery sorties.
    """
    capacity_kg: float
    max_range_km: float
    cruise_speed_kmh: float = 40.0   # used only to turn distance into ETA (time windows, animation)
    fleet_size: int = 5              # Hard limit on number of physical drones available
    turnaround_min: float = 10.0     # Time required at depot to reload/recharge before next sortie
    dwell_min: float = 2.0           # Time spent at each delivery location (handoff/landing)
    reserve_fraction: float = 0.10   # Reserve range buffer (e.g. 0.10 means we can only plan to use 90% of max_range)
    safety_margin_m: float = 50.0    # Distance in meters to keep clear of all No-Fly Zones
    energy_estimator: Optional[object] = None

    def enable_energy_model(self, vehicle_profile=None, battery_spec=None):
        """Attach a first-principles physics and battery energy estimator to this vehicle config."""
        from flightstack.vehicle_model import VehicleProfile, BatterySpec, EnergyEstimator
        v = vehicle_profile or VehicleProfile(max_payload_kg=max(self.capacity_kg, 8.0))
        b = battery_spec or BatterySpec(reserve_fraction=self.reserve_fraction)
        self.energy_estimator = EnergyEstimator(vehicle=v, battery=b)
        return self.energy_estimator


@dataclass
class CityConfig:
    """Everything specific to one city/region/disaster scenario. This is the
    only object that changes when you swap one deployment for another."""
    name: str
    depot: Location
    deliveries: List[Location]
    # Each no-fly zone is a polygon: a list of (lat, lon) vertices, in order,
    # implicitly closed (last vertex connects back to the first).
    no_fly_zones: List[List[Tuple[float, float]]] = field(default_factory=list)
    drone: DroneConfig = None
    osmnx_network_type: str = "drive"   # passed straight through to osmnx.graph_from_bbox

    # Human-readable description of this scenario, printed at startup and
    # shown in the dashboard. Explains WHY this deployment exists.
    scenario_description: str = ""

    @property
    def all_locations(self) -> List[Location]:
        """Depot first (index 0), then every delivery, in a fixed order.
        Every other module in this project refers to locations by their
        position in this list, so this ordering is the single source of truth."""
        return [self.depot] + self.deliveries


# ===========================================================================
# SCENARIO 1: Jaipur Flood Disaster Response
# ===========================================================================
# Monsoon flooding along the Dravyavati River has cut off several low-lying 
# neighborhoods. A medical drone operating from SMS Hospital is used to 
# deliver emergency medical supplies to makeshift relief camps.
# ===========================================================================

JAIPUR_DISASTER_DEPOT = Location(
    id=0,
    name="SMS Hospital (Depot) — High Ground",
    lat=26.9127,
    lon=75.8010,
    demand_kg=0.0,
)

JAIPUR_DISASTER_DELIVERIES = [
    # Relief camps at schools/community centers on elevated ground,
    # makeshift medical posts at flood-affected areas along the
    # Dravyavati River and Amanishah Nala floodplain.
    Location(1, "Jagatpura Relief Camp (School)", 26.8460, 75.8280,
             demand_kg=8.0, urgency="critical", window_minutes=20,
             cold_chain_limit_minutes=30),  # vaccines for flood-displaced children
    Location(2, "Sanganer Flood Post (Community Hall)", 26.8080, 75.8010,
             demand_kg=6.0, urgency="critical", window_minutes=25,
             cold_chain_limit_minutes=45,  # blood products for trauma cases
             landing_zone_image="sanganer_clinic_landing.jpg"),
    Location(3, "Pratap Nagar Relief Camp", 26.8280, 75.8460,
             demand_kg=7.5, urgency="urgent", window_minutes=35),
    Location(4, "Malviya Nagar Medical Post", 26.8567, 75.8089,
             demand_kg=5.0, urgency="urgent", window_minutes=40,
             cold_chain_limit_minutes=60),  # insulin for diabetic evacuees
    Location(5, "Mansarovar Flood Shelter", 26.8750, 75.7600,
             demand_kg=4.0, urgency="urgent", window_minutes=45),
    Location(6, "Dravyavati Riverbank Emergency Post", 26.8650, 75.8200,
             demand_kg=3.5, urgency="critical", window_minutes=15,
             cold_chain_limit_minutes=30),  # snakebite antivenom (cold-chain)
    Location(7, "Vaishali Nagar Shelter (School)", 26.9260, 75.7180,
             demand_kg=5.5, urgency="routine"),
    Location(8, "Gopalpura Medical Camp", 26.8850, 75.7930,
             demand_kg=3.0, urgency="routine"),
]

# Two real restricted-airspace zones:
#   1) Jaipur International Airport (VIJP/JAI) — built from the airport's
#      published ARP coordinates and runway threshold coordinates
#      (26.8242N 75.8122E, runway 08/26 + 15/33 thresholds). Indian
#      drone rules (DGCA Drone Rules 2021 "Red Zones") restrict flight
#      near operational airports; this box is a simplified stand-in.
#   2) City Palace / Hawa Mahal heritage & security zone.
JAIPUR_AIRPORT_ZONE = [
    (26.8130, 75.7930),
    (26.8130, 75.8300),
    (26.8380, 75.8300),
    (26.8380, 75.7930),
]

CITY_PALACE_ZONE = [
    (26.9230, 75.8210),
    (26.9230, 75.8265),
    (26.9280, 75.8265),
    (26.9280, 75.8210),
]

JAIPUR_DISASTER_DRONE = DroneConfig(
    capacity_kg=10.0,
    max_range_km=40.0,
    cruise_speed_kmh=45.0,
)

JAIPUR_DISASTER = CityConfig(
    name="Jaipur Flood Disaster Response",
    depot=JAIPUR_DISASTER_DEPOT,
    deliveries=JAIPUR_DISASTER_DELIVERIES,
    no_fly_zones=[JAIPUR_AIRPORT_ZONE, CITY_PALACE_ZONE],
    drone=JAIPUR_DISASTER_DRONE,
    scenario_description=(
        "Monsoon flooding along the Dravyavati River has cut off several "
        "low-lying Jaipur neighborhoods from road access. SMS Hospital "
        "(on high ground) operates as the medical supply depot. Drone "
        "delivery is the only viable path to reach relief camps and "
        "flood-stranded medical posts with vaccines, blood products, "
        "antivenom, and emergency supplies."
    ),
)


# ===========================================================================
# SCENARIO 2: Jaipur Routine Hospital Network
# ===========================================================================
# Same city, normal conditions. Demonstrates routine hospital-to-hospital 
# medical logistics.
# ===========================================================================

JAIPUR_HOSPITAL_DEPOT = Location(
    id=0,
    name="SMS Hospital (Depot)",
    lat=26.9127,
    lon=75.8010,
    demand_kg=0.0,
)

JAIPUR_HOSPITAL_DELIVERIES = [
    Location(1, "Fortis Escorts Hospital, Malviya Nagar", 26.8567, 75.8089,
             demand_kg=8.0, urgency="critical", window_minutes=25,
             cold_chain_limit_minutes=45),
    Location(2, "Manipal Hospital, Vidyadhar Nagar", 26.9700, 75.7650,
             demand_kg=5.5, urgency="routine"),
    Location(3, "Narayana Hospital, Pratap Nagar", 26.8280, 75.8460,
             demand_kg=6.0, urgency="urgent", window_minutes=45),
    Location(4, "Rukmani Birla Hospital, Gopalpura", 26.8850, 75.7930,
             demand_kg=4.0, urgency="routine",
             cold_chain_limit_minutes=60),  # routine insulin restock
    Location(5, "Eternal Hospital, Jagatpura Road", 26.8460, 75.8280,
             demand_kg=7.5, urgency="urgent", window_minutes=50),
    Location(6, "Apex Hospital, Malviya Nagar", 26.8580, 75.8130,
             demand_kg=3.0, urgency="routine"),
    Location(7, "Shalby Hospital, Vaishali Nagar", 26.9260, 75.7180,
             demand_kg=5.0, urgency="routine"),
    Location(8, "Rungta Hospital, Jhalana Gram", 26.8890, 75.8180,
             demand_kg=4.5, urgency="critical", window_minutes=20,
             cold_chain_limit_minutes=30),
    Location(9, "Sanganer Clinic (near airport)", 26.8080, 75.8010,
             demand_kg=3.5, urgency="routine",
             landing_zone_image="sanganer_clinic_landing.jpg"),
]

JAIPUR_HOSPITAL_DRONE = DroneConfig(
    capacity_kg=10.0,
    max_range_km=40.0,
    cruise_speed_kmh=45.0,
)

JAIPUR_HOSPITAL = CityConfig(
    name="Jaipur Hospital Network (Routine)",
    depot=JAIPUR_HOSPITAL_DEPOT,
    deliveries=JAIPUR_HOSPITAL_DELIVERIES,
    no_fly_zones=[JAIPUR_AIRPORT_ZONE, CITY_PALACE_ZONE],
    drone=JAIPUR_HOSPITAL_DRONE,
    scenario_description=(
        "Routine daily medical supply delivery across Jaipur's hospital "
        "network. Same routing engine as the disaster scenario, proving "
        "the system handles everyday operations too — not just emergencies."
    ),
)

# Keep backward-compatible alias
JAIPUR_DEMO = JAIPUR_HOSPITAL


# ===========================================================================
# SCENARIO 3: Chennai Cyclone Flood Response
# ===========================================================================
# Modeled after the 2015 South Indian floods. Large parts of the city south 
# of the Adyar River are submerged, requiring drone delivery from RGGGH.
# ===========================================================================

CHENNAI_DEPOT = Location(
    id=0,
    name="Rajiv Gandhi Government General Hospital (RGGGH) (Depot)",
    lat=13.0827,
    lon=80.2707,
    demand_kg=0.0,
)

CHENNAI_DELIVERIES = [
    Location(1, "Adyar Flood Relief Camp (School)", 13.0067, 80.2565,
             demand_kg=7.0, urgency="critical", window_minutes=20,
             cold_chain_limit_minutes=30),  # vaccines
    Location(2, "Velachery Emergency Medical Post", 12.9815, 80.2180,
             demand_kg=6.5, urgency="critical", window_minutes=25,
             cold_chain_limit_minutes=45),  # blood products
    Location(3, "Tambaram Flood Shelter", 12.9500, 80.2000,
             demand_kg=5.0, urgency="urgent", window_minutes=40),
    Location(4, "Pallavaram Relief Camp", 12.9675, 80.1800,
             demand_kg=4.5, urgency="urgent", window_minutes=45,
             cold_chain_limit_minutes=60),
    Location(5, "Chrompet Medical Post", 12.9520, 80.1450,
             demand_kg=8.0, urgency="critical", window_minutes=30,
             cold_chain_limit_minutes=30),
    Location(6, "T. Nagar Medical Camp", 13.0418, 80.2341,
             demand_kg=3.0, urgency="routine"),
    Location(7, "Mylapore Shelter (Temple Complex)", 13.0368, 80.2676,
             demand_kg=4.0, urgency="routine"),
]

# Chennai Airport (MAA/VOMM) restricted zone — simplified bounding box
# around the runway complex at Meenambakkam
CHENNAI_AIRPORT_ZONE = [
    (12.9800, 80.1500),
    (12.9800, 80.1850),
    (13.0050, 80.1850),
    (13.0050, 80.1500),
]

# Guindy National Park — drone-restricted wildlife area
GUINDY_PARK_ZONE = [
    (13.0040, 80.2360),
    (13.0040, 80.2470),
    (13.0120, 80.2470),
    (13.0120, 80.2360),
]

CHENNAI_DRONE = DroneConfig(
    capacity_kg=10.0,
    max_range_km=55.0,     # higher range for Chennai's spread-out flood zones
    cruise_speed_kmh=45.0,
)

CHENNAI_FLOOD = CityConfig(
    name="Chennai Cyclone Flood Response",
    depot=CHENNAI_DEPOT,
    deliveries=CHENNAI_DELIVERIES,
    no_fly_zones=[CHENNAI_AIRPORT_ZONE, GUINDY_PARK_ZONE],
    drone=CHENNAI_DRONE,
    scenario_description=(
        "Monsoon and storm-driven flooding has submerged low-lying Chennai "
        "neighborhoods south of the Adyar River (similar to the historic 2015 "
        "Chennai floods). Rajiv Gandhi Government General Hospital (RGGGH), on higher "
        "ground in the city center, serves as the medical supply depot. "
        "Road access to Velachery, Tambaram, and surrounding areas is "
        "cut off — drone delivery is the only viable route for emergency "
        "medical aid."
    ),
)


# ===========================================================================
# SCENARIO 4: Jaipur Single-UAV Reference Autonomy Sortie
# ===========================================================================
# Grounded in flightstack/vehicle_model.py (VehicleProfile "MedDrone-Hexa-16"
# and BatterySpec):
#   - Empty airframe: 5.5 kg, 6S 16 Ah LiPo battery: 3.2 kg (355.2 Wh total,
#     284.16 Wh usable with 20% reserve_fraction).
#   - Structural cargo limit: max_payload_kg = 8.0 kg (MTOW = 17.5 kg).
#   - Cruise speed: 10.0 m/s (36.0 km/h), matching ArduPilot WPNAV_SPEED=1000 cm/s.
#   - Provisional range assumption: at 10.0 m/s with 3.5 kg initial payload
#     (676 W outbound cruise, 417 W empty return, ~546 W mean), 284.16 Wh usable
#     energy yields ~18.0 km safe nominal round-trip range with hover dwell and
#     20% battery reserve.
# ===========================================================================

JAIPUR_SINGLE_UAV_DELIVERIES = [
    Location(
        1,
        "Gopalpura Flooded Relief Camp",
        26.8850,
        75.7930,
        demand_kg=2.0,
        urgency="critical",
        window_minutes=20,
        cold_chain_limit_minutes=30,
        site_suitability={
            "is_safe_for_landing": False,
            "surface_condition": "flooded",
            "slope_deg": 4.0,
            "obstacle_clearance_m": 12.0,
            "recommended_mode": "hover",
            "notes": "Inundated school courtyard; hover rope/gripper release required",
        },
    ),
    Location(
        2,
        "Jhalana Elevated Medical Post",
        26.8890,
        75.8180,
        demand_kg=1.5,
        urgency="urgent",
        window_minutes=35,
        cold_chain_limit_minutes=45,
        landing_zone_image="sanganer_clinic_landing.jpg",
        site_suitability={
            "is_safe_for_landing": True,
            "surface_condition": "clear",
            "slope_deg": 2.0,
            "obstacle_clearance_m": 18.0,
            "recommended_mode": "land",
            "notes": "Elevated dry courtyard suitable for either hover or land handoff",
        },
    ),
]

JAIPUR_SINGLE_UAV_DRONE = DroneConfig(
    capacity_kg=8.0,          # Matches VehicleProfile.max_payload_kg (5.5+3.2+8.0 = 16.7 kg <= 17.5 kg MTOW)
    max_range_km=18.0,        # Grounded in 284.16 Wh usable battery budget at 10 m/s (36 km/h)
    cruise_speed_kmh=36.0,    # Matches SITL WPNAV_SPEED = 1000 cm/s (10.0 m/s)
    fleet_size=1,             # Dedicated single-UAV reference configuration
    reserve_fraction=0.20,    # Matches BatterySpec.reserve_fraction (20% SoC floor)
    safety_margin_m=50.0,
)

JAIPUR_SINGLE_UAV = CityConfig(
    name="Jaipur Single-UAV Medical Autonomy Reference",
    depot=JAIPUR_DISASTER_DEPOT,
    deliveries=JAIPUR_SINGLE_UAV_DELIVERIES,
    no_fly_zones=[JAIPUR_AIRPORT_ZONE, CITY_PALACE_ZONE],
    drone=JAIPUR_SINGLE_UAV_DRONE,
    scenario_description=(
        "Single-UAV autonomous medical delivery reference scenario grounded in "
        " the MedDrone-Hexa-16 physical model (8.0 kg max payload, 284.16 Wh "
        "usable energy, 20% reserve, 10 m/s cruise). Visits a flooded relief camp "
        "(requiring hover release) and an elevated medical post (supporting "
        "hover or land handoff) while enforcing 4D airspace restrictions, "
        "cold-chain limits, and proximity obstacle avoidance."
    ),
)


# ===========================================================================
# CITY_CONFIGS REGISTRY
# ===========================================================================
# Maps CLI-friendly names to CityConfig objects.
# ===========================================================================

CITY_CONFIGS = {
    "jaipur_single_uav": JAIPUR_SINGLE_UAV,
    "jaipur_disaster": JAIPUR_DISASTER,
    "jaipur_hospital": JAIPUR_HOSPITAL,
    "jaipur": JAIPUR_HOSPITAL,       # backward-compatible alias
    "chennai_flood": CHENNAI_FLOOD,
}
