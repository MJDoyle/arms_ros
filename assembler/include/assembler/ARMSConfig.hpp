#ifndef ARMS_CONFIG_HPP
#define ARMS_CONFIG_HPP

#include "assembler/MeshFunctions.hpp"

const extern double PARTS_BED_HEIGHT;

const extern std::vector<std::vector<gp_Pnt>> PARTS_BAY_POSITIONS;

const extern std::vector<int> BAY_SIZES;

const extern double PRINT_BED_CENTER[2];

const extern double PRINT_BED_BOTTOM_LEFT[2];

const extern double PRINT_BED_TOP_RIGHT[2];

const extern double PRINT_MIN_SPACING;

const extern double PRINT_BED_HEIGHT;

#endif