#include "assembler/ARMSConfig.hpp"

const double PARTS_BED_HEIGHT = 0;

const std::vector<std::vector<gp_Pnt>> PARTS_BAY_POSITIONS = {{gp_Pnt(576.4, 245.8, 0), 
                                                    gp_Pnt(576.4, 195.8, 0), 
                                                    gp_Pnt(576.4, 145.8, 0), 
                                                    gp_Pnt(576.4, 95.8, 0),
                                                    gp_Pnt(576.4, 45.8, 0)},
                                                
                                                 {gp_Pnt(517.4, 50.8, 0),
                                                  gp_Pnt(517.4, 120.8, 0)}};

                                                 //[(576.4, 245.8), (576.4, 195.8), (576.4, 145.8), (576.4, 95.8)]
                                                 
//const std::vector<gp_Pnt> 60_PARTS_BAY_POSITIONS = {gp_Pnt(0, 0, 0),
                                                    //gp_Pnt(0, 0, 0)};

const std::vector<int> BAY_SIZES = {40, 60};

const double PRINT_BED_CENTER[2] = {0, 0};

const double PRINT_BED_BOTTOM_LEFT[2] = {50, 100};

const double PRINT_BED_TOP_RIGHT[2] = {250, 250};

const double PRINT_MIN_SPACING = 20;

const double PRINT_BED_HEIGHT = 0;

