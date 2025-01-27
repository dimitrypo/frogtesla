#!/usr/bin/env python3
import json
import numpy as np

from sortedcontainers import SortedDict

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX

from openpilot.selfdrive.frogpilot.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME, params

CURVATURE_THRESHOLD = 1e-10
ROUNDING_PRECISION = 10

class SmartTurnSpeedController:
  def __init__(self, FrogPilotVCruise):
    self.frogpilot_vcruise = FrogPilotVCruise

    self.data = SortedDict({entry["speed"]: [np.array([curve["curvature"], curve["lateral_accel"]]) for curve in entry.get("curvatures", [])] for entry in json.loads(params.get("UserCurvature") or "[]")})

    self.manual_long_timer = 0

  def update_curvature_data(self, v_ego):
    road_curvature = round(self.frogpilot_vcruise.frogpilot_planner.road_curvature, ROUNDING_PRECISION)
    lateral_accel = round(v_ego**2 * road_curvature, ROUNDING_PRECISION)

    if abs(road_curvature) < CURVATURE_THRESHOLD or abs(lateral_accel) < 1:
      return

    entries = self.data.setdefault(v_ego, [])
    for i, entry in enumerate(entries):
      if abs(entry[0] - road_curvature) < CURVATURE_THRESHOLD:
        entries[i][1] = (entry[1] + lateral_accel) / 2
        return

    entries.append(np.array([road_curvature, lateral_accel]))

  def update(self, carControl, v_cruise, v_ego):
    if not carControl.longActive and V_CRUISE_MAX * CV.KPH_TO_MS >= v_ego > CRUISING_SPEED and not self.frogpilot_vcruise.frogpilot_planner.tracking_lead:
      if self.manual_long_timer >= PLANNER_TIME:
        self.update_curvature_data(v_ego)
      self.manual_long_timer += DT_MDL

    elif self.manual_long_timer >= PLANNER_TIME:
      params.put_nonblocking("UserCurvature", json.dumps([
        {"speed": speed, "curvatures": [{"curvature": entry[0], "lateral_accel": entry[1]} for entry in entries]}
        for speed, entries in self.data.items()
      ]))
      self.manual_long_timer = 0

    else:
      self.manual_long_timer = 0
