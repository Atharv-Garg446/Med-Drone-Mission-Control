#!/usr/bin/env python3
"""
flightstack/payload.py
======================
Explicit medical cargo, site suitability assessment, and delivery state model.

Tracks:
  - Package identity, assigned customer stop, and cargo mass (kg).
  - Explicit delivery lifecycle states:
      LOADED -> ARRIVED -> SITE_ASSESSED -> METHOD_SELECTED ->
      RELEASE_COMMANDED -> RELEASE_ACKNOWLEDGED -> CUSTODY_CONFIRMED ->
      DELIVERY_COMPLETE (or FAILED).
  - Site suitability assessment in disaster response (debris, floodwater, slope, clearance).
  - Actuator feedback source: SITL SERVO9 / Gripper STATUSTEXT feedback vs
    simulated actuator vs ground-crew custody handoff.
  - Remaining vehicle payload mass for dynamic energy calculations.
  - Safety interlocks: duplicate release rejection, stuck-gripper detection,
    wrong-stop prevention, unladen release warnings, and delivery site validation.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class DeliveryState(str, Enum):
    LOADED = "LOADED"
    ARRIVED = "ARRIVED"
    SITE_ASSESSED = "SITE_ASSESSED"
    METHOD_SELECTED = "METHOD_SELECTED"
    RELEASE_COMMANDED = "RELEASE_COMMANDED"
    RELEASE_ACKNOWLEDGED = "RELEASE_ACKNOWLEDGED"
    CUSTODY_CONFIRMED = "CUSTODY_CONFIRMED"
    DELIVERY_COMPLETE = "DELIVERY_COMPLETE"
    FAILED = "FAILED"


class PackageStatus(str, Enum):
    LOADED = "LOADED"
    RELEASE_COMMANDED = "RELEASE_COMMANDED"
    DELIVERED = "DELIVERED"
    FAILED = "FAILED"
    DROPPED = "DROPPED"


class ReleaseSource(str, Enum):
    SIM_GRIPPER_FEEDBACK = "SIM_GRIPPER_FEEDBACK"          # Autopilot STATUSTEXT "Gripper Released" / "Gripper load released"
    SIMULATED_ACTUATOR = "SIMULATED_ACTUATOR"              # Explicit simulated servo feedback in unit tests
    SIM_GRIPPER_HOVER_RELEASE = "SIM_GRIPPER_HOVER_RELEASE"
    SIM_GRIPPER_GROUND_RELEASE = "SIM_GRIPPER_GROUND_RELEASE"
    GROUND_CONFIRMED = "GROUND_CONFIRMED"                  # Retained for explicit external custody tests
    HOVER_ROPE_RELEASE = "HOVER_ROPE_RELEASE"              # Legacy alias for simulated hover gripper release
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"                    # Operator intervention


@dataclass
class SiteSuitability:
    """Assessment of the delivery site in a disaster zone.

    Distinguishes static scenario fixtures from live sensor or operator updates via
    ``provenance`` and ``uncertainty`` ('known' | 'uncertain' | 'missing').
    """
    is_safe_for_landing: bool = True
    surface_condition: str = "clear"     # "clear" | "flooded" | "debris" | "mud" | "submerged" | "collapsed" | "unknown"
    slope_deg: float = 0.0
    obstacle_clearance_m: float = 15.0
    recommended_mode: str = "hover"      # "hover" | "land" | "abort"
    rejection_reason: Optional[str] = None
    uncertainty: str = "known"           # "known" | "uncertain" | "missing"
    provenance: str = "SYNTHETIC_FIXTURE"
    notes: str = ""

    @classmethod
    def from_any(cls, obj) -> "SiteSuitability":
        """Normalize a SiteSuitability instance, dict, or None into a SiteSuitability."""
        if isinstance(obj, cls):
            return obj
        if isinstance(obj, dict):
            surf = str(obj.get("surface_condition", "clear")).strip().lower()
            default_unc = "uncertain" if surf in ("unknown", "uncertain", "") else "known"
            return cls(
                is_safe_for_landing=bool(obj.get("is_safe_for_landing", True)),
                surface_condition=surf or "unknown",
                slope_deg=float(obj.get("slope_deg", 0.0)),
                obstacle_clearance_m=float(obj.get("obstacle_clearance_m", 15.0)),
                recommended_mode=str(obj.get("recommended_mode", "hover")).strip().lower(),
                rejection_reason=obj.get("rejection_reason"),
                uncertainty=str(obj.get("uncertainty", default_unc)).strip().lower(),
                provenance=str(obj.get("provenance", "SYNTHETIC_FIXTURE")),
                notes=str(obj.get("notes", "")),
            )
        return cls(
            is_safe_for_landing=True,
            surface_condition="clear",
            slope_deg=0.0,
            obstacle_clearance_m=15.0,
            recommended_mode="hover",
            uncertainty="missing",
            provenance="MISSING_SITE_DATA",
            notes="No site_suitability provided; auto mode defaults to conservative hover",
        )

    def to_dict(self) -> dict:
        return {
            "is_safe_for_landing": self.is_safe_for_landing,
            "surface_condition": self.surface_condition,
            "slope_deg": self.slope_deg,
            "obstacle_clearance_m": self.obstacle_clearance_m,
            "recommended_mode": self.recommended_mode,
            "rejection_reason": self.rejection_reason,
            "uncertainty": self.uncertainty,
            "provenance": self.provenance,
            "notes": self.notes,
        }

    def assess_method(self, requested_mode: str = "auto") -> Tuple[bool, str, str]:
        """Deterministic delivery-method selection and safety validation.

        Semantics:
          1. Supports 'auto' (uses site recommended_mode when site data is known),
             or explicit 'hover' / 'land' preferences.
          2. Hard abort constraints apply to ALL modes: if recommended_mode == 'abort',
             rejection_reason is set, or obstacle_clearance_m < 10.0m, neither hover
             nor landing is safe -> returns (False, 'abort', reason).
          3. Explicit 'land' preference NEVER overrides hard landing-safety constraints
             (flooded/debris/mud/submerged/collapsed surface, slope > 7.0 deg,
             is_safe_for_landing=False, or uncertain/unknown surface).
          4. Missing or uncertain site data defaults to 'hover' in 'auto' mode and
             downgrades 'land' to 'hover' when uncertainty is 'uncertain' or surface is 'unknown'.
        """
        req = (requested_mode or "auto").strip().lower()
        surf = (self.surface_condition or "unknown").strip().lower()
        rec = (self.recommended_mode or "hover").strip().lower()

        # 1. Hard rejection / abort checks (unsafe for BOTH hover and landing)
        if req == "abort" or rec == "abort" or self.rejection_reason:
            why = self.rejection_reason or f"Site unsuitable: recommended_mode={rec}, surface={surf}"
            return False, "abort", why

        if self.obstacle_clearance_m < 10.0:
            return (
                False,
                "abort",
                f"Site unsuitable: ground is {surf} and clearance ({self.obstacle_clearance_m:.1f}m) < 10m minimum",
            )

        # 2. Resolve candidate mode from requested_mode ('auto' | 'hover' | 'land')
        if req == "auto":
            if self.uncertainty in ("missing", "uncertain") or surf in ("unknown", "uncertain"):
                desired = "hover"
                auto_reason = (
                    f"Auto-selected hover: site suitability is {self.uncertainty} "
                    f"(surface={surf}); ground touchdown not assumed safe"
                )
            elif rec in ("hover", "land"):
                desired = rec
                auto_reason = (
                    f"Auto-selected {desired} from site recommended_mode "
                    f"(surface={surf}, slope={self.slope_deg:.1f} deg, clearance={self.obstacle_clearance_m:.1f}m, "
                    f"provenance={self.provenance})"
                )
            else:
                desired = "hover"
                auto_reason = f"Auto-selected hover: unrecognized recommended_mode={rec!r}"
        elif req in ("hover", "land"):
            desired = req
            auto_reason = None
        else:
            return False, "abort", f"Unsupported requested delivery mode: {requested_mode!r}"

        # 3. Validate candidate against hard landing-safety constraints
        if desired == "land":
            if surf in ("flooded", "debris", "mud", "submerged", "collapsed"):
                return True, "hover", f"Degraded from land to hover: ground surface is {surf}"
            if self.slope_deg > 7.0:
                return True, "hover", f"Degraded from land to hover: ground slope is {self.slope_deg:.1f} deg"
            if not self.is_safe_for_landing:
                return True, "hover", "Degraded from land to hover: site unsuitable for touchdown"
            if surf in ("unknown", "uncertain") or self.uncertainty == "uncertain":
                return (
                    True,
                    "hover",
                    f"Degraded from land to hover: site condition is uncertain/unverified (surface={surf})",
                )
            return True, "land", auto_reason or "Site assessed suitable"

        # 4. Hover mode (already verified obstacle_clearance_m >= 10.0m above)
        if not self.is_safe_for_landing and surf in ("flooded", "debris", "mud", "submerged", "collapsed"):
            return (
                True,
                "hover",
                auto_reason or f"Site assessed suitable for hover only (ground surface is {surf})",
            )
        return True, "hover", auto_reason or "Site assessed suitable"


def resolve_stop_delivery_mode(
    stop: object,
    requested_mode: str = "auto",
    stop_modes: Optional[Dict[int, str]] = None,
) -> Tuple[bool, str, str, SiteSuitability]:
    """Single deterministic source of truth for per-stop delivery method selection.

    Used identically by mission generation (``build_delivery_mission``), payload
    initialization (``PayloadManager.from_stops``), and runtime site revalidation
    (``MissionRunner``). Never returns ``ok=True`` with ``'abort'``.
    """
    sid = getattr(stop, "id", None)
    eff_req = (stop_modes or {}).get(sid, requested_mode) if (stop_modes and sid in stop_modes) else requested_mode
    site = SiteSuitability.from_any(getattr(stop, "site_suitability", None))
    ok, selected_mode, reason = site.assess_method(eff_req)
    if not ok or selected_mode not in ("hover", "land"):
        return False, "abort", reason, site
    return True, selected_mode, reason, site


@dataclass
class MedicalPackage:
    package_id: str
    assigned_stop: int
    mass_kg: float
    description: str = "Medical Supplies"
    status: PackageStatus = PackageStatus.LOADED
    delivery_state: DeliveryState = DeliveryState.LOADED
    release_mode: str = "hover"                         # "hover" | "land"
    planned_mode: str = "hover"                         # Mode resolved at mission planning
    site_suitability: Optional[SiteSuitability] = None
    assessment_reason: Optional[str] = None
    arrived_time_s: Optional[float] = None
    site_assessed_time_s: Optional[float] = None
    commanded_time_s: Optional[float] = None
    acknowledged_time_s: Optional[float] = None
    dwell_completed_time_s: Optional[float] = None
    confirmed_time_s: Optional[float] = None
    delivered_time_s: Optional[float] = None
    confirmation_source: Optional[ReleaseSource] = None
    error_reason: Optional[str] = None
    actuator_mechanism: str = "SIM_Gripper_Servo (MAV_CMD_DO_GRIPPER=211)"
    simulated_release_acknowledged: bool = False
    stop_completion_recorded: bool = False
    recipient_handover_verified: bool = False
    duplicate_feedback_count: int = 0
    release_provenance: Optional[str] = None
    handover_classification: str = "SIMULATED_PACKAGE_RELEASE_NOT_PHYSICAL_RECIPIENT_HANDOVER"

    def to_dict(self) -> dict:
        return {
            "package_id": self.package_id,
            "assigned_stop": self.assigned_stop,
            "mass_kg": self.mass_kg,
            "description": self.description,
            "status": self.status.value,
            "delivery_state": self.delivery_state.value,
            "release_mode": self.release_mode,
            "planned_mode": self.planned_mode,
            "assessment_reason": self.assessment_reason,
            "arrived_time_s": self.arrived_time_s,
            "site_assessed_time_s": self.site_assessed_time_s,
            "commanded_time_s": self.commanded_time_s,
            "acknowledged_time_s": self.acknowledged_time_s,
            "dwell_completed_time_s": self.dwell_completed_time_s,
            "confirmed_time_s": self.confirmed_time_s,
            "delivered_time_s": self.delivered_time_s,
            "confirmation_source": self.confirmation_source.value if self.confirmation_source else None,
            "error_reason": self.error_reason,
            "actuator_mechanism": self.actuator_mechanism,
            "simulated_release_acknowledged": self.simulated_release_acknowledged,
            "stop_completion_recorded": self.stop_completion_recorded,
            "recipient_handover_verified": self.recipient_handover_verified,
            "duplicate_feedback_count": self.duplicate_feedback_count,
            "release_provenance": self.release_provenance,
            "handover_classification": self.handover_classification,
            "site_suitability": self.site_suitability.to_dict() if self.site_suitability else None,
        }


class PayloadManager:
    """Manages the full cargo inventory of a delivery aircraft."""

    def __init__(self, packages: Optional[List[MedicalPackage]] = None):
        self.packages: Dict[int, MedicalPackage] = {}
        self.duplicate_feedback_count: int = 0
        if packages:
            for p in packages:
                self.packages[p.assigned_stop] = p

    @classmethod
    def from_stops(
        cls,
        stops,
        default_mass_kg: float = 2.0,
        mode: str = "auto",
        stop_modes: Optional[Dict[int, str]] = None,
    ) -> "PayloadManager":
        """Construct inventory from a list of delivery stop objects using deterministic mode resolution."""
        pkgs = []
        for s in stops:
            sid = getattr(s, "id", None)
            if sid is None or sid == 0:  # Skip depot
                continue
            mass = getattr(s, "demand_kg", getattr(s, "cargo_kg", default_mass_kg))
            name = getattr(s, "name", f"Delivery Stop {sid}")
            ok, resolved_mode, reason, site = resolve_stop_delivery_mode(
                s, requested_mode=mode, stop_modes=stop_modes
            )
            pkg = MedicalPackage(
                package_id=f"PKG-STOP-{sid:02d}",
                assigned_stop=sid,
                mass_kg=float(mass),
                description=f"Supply package for {name}",
                release_mode=resolved_mode if ok else "abort",
                planned_mode=resolved_mode if ok else "abort",
                site_suitability=site,
                assessment_reason=reason,
            )
            if not ok or resolved_mode == "abort":
                pkg.status = PackageStatus.FAILED
                pkg.delivery_state = DeliveryState.FAILED
                pkg.error_reason = reason
            pkgs.append(pkg)
        return cls(pkgs)

    def remaining_cargo_kg(self) -> float:
        """Sum of masses of all packages still aboard (not fully DELIVERED)."""
        return sum(p.mass_kg for p in self.packages.values()
                   if p.status != PackageStatus.DELIVERED)

    def delivered_count(self) -> int:
        return sum(1 for p in self.packages.values() if p.status == PackageStatus.DELIVERED)

    def pending_count(self) -> int:
        return sum(1 for p in self.packages.values() if p.status == PackageStatus.LOADED)

    def get_package(self, stop_id: int) -> Optional[MedicalPackage]:
        return self.packages.get(stop_id)

    def record_arrival(self, stop_id: int, t_rel_s: float) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Record that aircraft has arrived at delivery waypoint overhead."""
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        if pkg.status == PackageStatus.DELIVERED:
            return False, pkg, "ALREADY_DELIVERED"
        pkg.arrived_time_s = round(t_rel_s, 2)
        pkg.delivery_state = DeliveryState.ARRIVED
        return True, pkg, "ARRIVED_AT_STOP"

    def assess_site(
        self,
        stop_id: int,
        site: Optional[SiteSuitability],
        requested_mode: str,
        t_rel_s: float
    ) -> Tuple[bool, str, str]:
        """Assess landing/hover site suitability and determine actual delivery method."""
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, requested_mode, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        site = SiteSuitability.from_any(site) if site is not None else (pkg.site_suitability or SiteSuitability.from_any(None))
        pkg.site_suitability = site
        pkg.site_assessed_time_s = round(t_rel_s, 2)
        pkg.delivery_state = DeliveryState.SITE_ASSESSED

        ok, selected_mode, reason = site.assess_method(requested_mode)
        if not ok or selected_mode not in ("hover", "land"):
            selected_mode = "abort"
            ok = False
        pkg.release_mode = selected_mode
        pkg.assessment_reason = reason
        if ok:
            pkg.delivery_state = DeliveryState.METHOD_SELECTED
        else:
            pkg.delivery_state = DeliveryState.FAILED
            pkg.status = PackageStatus.FAILED
            pkg.error_reason = reason
        return ok, selected_mode, reason

    def command_release(
        self,
        stop_id: int,
        mode: str,
        t_rel_s: float,
        simulate_stuck: bool = False
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Issue release command for the package assigned to stop_id.

        Does NOT automatically acknowledge the actuator; acknowledgement must
        come from causal actuator feedback (e.g. ArduPilot SIM_Gripper_Servo STATUSTEXT).
        """
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"

        if pkg.status == PackageStatus.DELIVERED:
            return False, pkg, f"DUPLICATE_RELEASE_ATTEMPT: Package {pkg.package_id} already delivered"

        if pkg.status == PackageStatus.FAILED:
            return False, pkg, f"PACKAGE_ALREADY_FAILED: {pkg.error_reason}"

        if pkg.delivery_state in (DeliveryState.RELEASE_COMMANDED, DeliveryState.RELEASE_ACKNOWLEDGED):
            return False, pkg, f"DUPLICATE_RELEASE_ATTEMPT: Package {pkg.package_id} release already in progress"

        pkg.commanded_time_s = round(t_rel_s, 2)
        pkg.release_mode = mode

        if simulate_stuck:
            pkg.status = PackageStatus.FAILED
            pkg.delivery_state = DeliveryState.FAILED
            pkg.error_reason = "GRIPPER_MECHANICAL_JAM"
            return False, pkg, "ACTUATOR_FAULT: Gripper mechanism jammed or failed to open"

        pkg.status = PackageStatus.RELEASE_COMMANDED
        pkg.delivery_state = DeliveryState.RELEASE_COMMANDED
        return True, pkg, "RELEASE_COMMANDED"

    def acknowledge_actuator(
        self,
        stop_id: int,
        t_rel_s: float,
        source: ReleaseSource = ReleaseSource.SIM_GRIPPER_FEEDBACK
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Record causal simulated-gripper actuator acknowledgement (e.g. 'Gripper Released').

        Strictly records simulated package release only; does NOT mark the mission
        stop complete and does NOT claim physical recipient handover.
        """
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        if pkg.status == PackageStatus.FAILED or pkg.delivery_state == DeliveryState.FAILED:
            return False, pkg, f"PACKAGE_ALREADY_FAILED: {pkg.error_reason}"
        if pkg.simulated_release_acknowledged or pkg.delivery_state in (
            DeliveryState.RELEASE_ACKNOWLEDGED,
            DeliveryState.CUSTODY_CONFIRMED,
            DeliveryState.DELIVERY_COMPLETE,
        ):
            self.duplicate_feedback_count += 1
            pkg.duplicate_feedback_count += 1
            return False, pkg, f"DUPLICATE_ACTUATOR_FEEDBACK: Package {pkg.package_id} already acknowledged"
        if pkg.delivery_state != DeliveryState.RELEASE_COMMANDED:
            return False, pkg, f"INVALID_STATE: Current state {pkg.delivery_state.value}, expected RELEASE_COMMANDED"

        pkg.acknowledged_time_s = round(t_rel_s, 2)
        pkg.confirmation_source = source
        pkg.simulated_release_acknowledged = True
        pkg.delivery_state = DeliveryState.RELEASE_ACKNOWLEDGED
        pkg.release_provenance = (
            "simulated_ground_gripper_release_after_dwell"
            if (pkg.release_mode == "land" and pkg.dwell_completed_time_s is not None)
            else ("simulated_ground_gripper_release" if pkg.release_mode == "land" else "simulated_hover_gripper_release")
        )
        return True, pkg, "ACTUATOR_ACKNOWLEDGED"

    def fail_release(
        self,
        stop_id: int,
        t_rel_s: float,
        reason: str = "MISSING_ACTUATOR_FEEDBACK",
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Mark a package release as FAILED due to missing or faulted actuator feedback."""
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        if pkg.status == PackageStatus.DELIVERED:
            return False, pkg, "ALREADY_DELIVERED"
        pkg.status = PackageStatus.FAILED
        pkg.delivery_state = DeliveryState.FAILED
        pkg.error_reason = reason
        return True, pkg, reason

    def record_dwell_completed(
        self,
        stop_id: int,
        t_rel_s: float,
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Record that the configured ground dwell (MAV_CMD_NAV_DELAY) has finished."""
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        if pkg.status == PackageStatus.FAILED or pkg.delivery_state == DeliveryState.FAILED:
            return False, pkg, f"PACKAGE_FAILED: {pkg.error_reason}"
        pkg.dwell_completed_time_s = round(t_rel_s, 2)
        if pkg.release_mode == "land":
            pkg.release_provenance = "simulated_ground_gripper_release_after_dwell"
        return True, pkg, "GROUND_DWELL_COMPLETED"

    def confirm_custody(
        self,
        stop_id: int,
        t_rel_s: float,
        source: ReleaseSource = ReleaseSource.GROUND_CONFIRMED,
        verified_recipient_handover: bool = False,
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Explicit custody state transition (retained for state-machine tests).

        Never sets ``recipient_handover_verified = True`` unless ``verified_recipient_handover=True``
        (independent physical handover verification, which does not exist in SITL simulation).
        """
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        if pkg.delivery_state not in (DeliveryState.RELEASE_COMMANDED, DeliveryState.RELEASE_ACKNOWLEDGED):
            return False, pkg, f"INVALID_STATE: Current state {pkg.delivery_state.value}"

        pkg.confirmed_time_s = round(t_rel_s, 2)
        pkg.confirmation_source = source
        pkg.delivery_state = DeliveryState.CUSTODY_CONFIRMED
        pkg.recipient_handover_verified = bool(verified_recipient_handover)
        return True, pkg, "CUSTODY_CONFIRMED"

    def complete_delivery(
        self,
        stop_id: int,
        t_rel_s: float,
        release_provenance: Optional[str] = None,
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Complete mission-stop delivery sequence after causal actuator acknowledgement.

        Refuses to complete if actuator acknowledgement is missing or if the release failed.
        For ground ('land') deliveries, requires ``dwell_completed_time_s`` to be recorded
        when enforced by the mission runner.
        """
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"
        if pkg.status == PackageStatus.FAILED or pkg.delivery_state == DeliveryState.FAILED:
            return False, pkg, f"CANNOT_COMPLETE_FAILED_RELEASE: {pkg.error_reason}"
        if pkg.status == PackageStatus.DELIVERED or pkg.delivery_state == DeliveryState.DELIVERY_COMPLETE:
            return False, pkg, f"ALREADY_COMPLETED: Package {pkg.package_id} already delivered"
        if not pkg.simulated_release_acknowledged and pkg.delivery_state not in (
            DeliveryState.RELEASE_ACKNOWLEDGED,
            DeliveryState.CUSTODY_CONFIRMED,
        ):
            return (
                False,
                pkg,
                f"MISSING_ACTUATOR_ACKNOWLEDGEMENT: Current state is {pkg.delivery_state.value}; "
                "cannot mark delivery complete without actuator feedback",
            )

        pkg.status = PackageStatus.DELIVERED
        pkg.delivery_state = DeliveryState.DELIVERY_COMPLETE
        pkg.delivered_time_s = round(t_rel_s, 2)
        pkg.stop_completion_recorded = True
        if release_provenance is not None:
            pkg.release_provenance = release_provenance
        elif pkg.release_provenance is None:
            pkg.release_provenance = (
                "simulated_ground_gripper_release_after_dwell"
                if pkg.release_mode == "land"
                else "simulated_hover_gripper_release"
            )
        return True, pkg, "DELIVERY_COMPLETE"

    def confirm_release(
        self,
        stop_id: int,
        t_rel_s: float,
        source: ReleaseSource = ReleaseSource.SIM_GRIPPER_FEEDBACK
    ) -> Tuple[bool, Optional[MedicalPackage], str]:
        """Convenience method acknowledging simulated release and completing stop delivery.

        Does NOT call ``confirm_custody`` or claim physical recipient handover.
        """
        pkg = self.packages.get(stop_id)
        if pkg is None:
            return False, None, f"NO_PAYLOAD_FOR_STOP_{stop_id}"

        if pkg.status == PackageStatus.DELIVERED:
            return False, pkg, "ALREADY_CONFIRMED"

        if pkg.status != PackageStatus.RELEASE_COMMANDED:
            return False, pkg, f"CANNOT_CONFIRM: Status is {pkg.status.value}, expected RELEASE_COMMANDED"

        ok_ack, _, why_ack = self.acknowledge_actuator(stop_id, t_rel_s, source)
        if not ok_ack:
            return False, pkg, why_ack
        return self.complete_delivery(stop_id, t_rel_s)

    def record_actuator_text(
        self,
        text: str,
        t_rel_s: float,
        stop_id: Optional[int] = None,
    ) -> Optional[Tuple[int, MedicalPackage]]:
        """Inspect autopilot STATUSTEXT for causal ArduPilot SIM_Gripper_Servo feedback.

        - On 'Gripper Released' or 'Gripper load released': transitions the active
          RELEASE_COMMANDED package to RELEASE_ACKNOWLEDGED (does NOT complete the
          stop or claim recipient custody).
        - On 'Gripper failed' / 'Gripper fault' / 'Gripper jam': transitions the active
          package to FAILED.
        - Duplicate feedback for an already-acknowledged package increments
          ``duplicate_feedback_count`` and returns None.
        """
        text_lower = (text or "").strip().lower()
        target_pkgs = (
            [self.packages[stop_id]]
            if (stop_id is not None and stop_id in self.packages)
            else list(self.packages.values())
        )

        if any(err in text_lower for err in ("gripper failed", "gripper fault", "gripper jam", "servo fault")):
            for pkg in target_pkgs:
                if pkg.delivery_state == DeliveryState.RELEASE_COMMANDED:
                    self.fail_release(
                        pkg.assigned_stop,
                        t_rel_s,
                        reason=f"ACTUATOR_FEEDBACK_FAULT: {text}",
                    )
                    return pkg.assigned_stop, pkg
            return None

        if "gripper released" in text_lower or "gripper load released" in text_lower:
            for pkg in target_pkgs:
                if pkg.delivery_state == DeliveryState.RELEASE_COMMANDED:
                    ok, _, _ = self.acknowledge_actuator(
                        pkg.assigned_stop, t_rel_s, ReleaseSource.SIM_GRIPPER_FEEDBACK
                    )
                    if ok:
                        return pkg.assigned_stop, pkg
            # Check if this was duplicate feedback for an already acknowledged package
            acked = [p for p in target_pkgs if p.simulated_release_acknowledged]
            if acked:
                self.duplicate_feedback_count += 1
                acked[-1].duplicate_feedback_count += 1
            return None
        return None

    def snapshot(self) -> dict:
        return {
            "total_packages": len(self.packages),
            "delivered": self.delivered_count(),
            "pending": self.pending_count(),
            "remaining_cargo_kg": round(self.remaining_cargo_kg(), 2),
            "duplicate_feedback_count": self.duplicate_feedback_count,
            "packages": {sid: p.to_dict() for sid, p in self.packages.items()}
        }
