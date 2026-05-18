#pragma once

#define INVALID_VALUE (-123456)

enum SemanticLabel {
  UNKNOWN      = 0,
  ROAD_BRIDGE  = 1,
  ROAD_SURFACE = 2,
  GROUND       = 3,
  OBSTACLE     = 4,
  HANGING      = 5,
  FUSIONOB     = 6
};