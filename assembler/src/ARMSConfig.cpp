#include "assembler/ARMSConfig.hpp"

const double PARTS_BED_HEIGHT = 0;

// const std::vector<std::vector<gp_Pnt>> PARTS_BAY_POSITIONS = {{gp_Pnt(576.4, 245.8, 0), 
//                                                     gp_Pnt(576.4, 195.8, 0), 
//                                                     gp_Pnt(576.4, 145.8, 0), 
//                                                     gp_Pnt(576.4, 95.8, 0),
//                                                     gp_Pnt(576.4, 45.8, 0)},
                                                
//                                                  {gp_Pnt(517.4, 50.8, 0),
//                                                   gp_Pnt(517.4, 120.8, 0)}};

const std::vector<std::vector<gp_Pnt>> PARTS_BAY_POSITIONS = {{gp_Pnt(331.13, 119.0, 0),
                                                               gp_Pnt(376.21, 119.26, 0), 
                                                               gp_Pnt(421.29, 119.52, 0), 
                                                               gp_Pnt(511.45, 120.04, 0),
                                                               gp_Pnt(556.53, 120.3, 0)},

                                                              {gp_Pnt(341.33, 64.5, 0),
                                                               gp_Pnt(406.36, 64.86, 0),
                                                               gp_Pnt(471.39, 65.23, 0),
                                                               gp_Pnt(536.43, 65.6, 0)}};

                                                 //[(576.4, 245.8), (576.4, 195.8), (576.4, 145.8), (576.4, 95.8)]
                                                 
//const std::vector<gp_Pnt> 60_PARTS_BAY_POSITIONS = {gp_Pnt(0, 0, 0),
                                                    //gp_Pnt(0, 0, 0)};

const std::vector<int> BAY_SIZES = {40, 60};

const double PRINT_BED_CENTER[2] = {0, 0};

const double PRINT_BED_BOTTOM_LEFT[2] = {50, 120};

const double PRINT_BED_TOP_RIGHT[2] = {230, 250};

const double PRINT_MIN_SPACING = 20;

const double PRINT_BED_HEIGHT = 0;

const double JIG_HEIGHT = 10;

const double JIG_CENTER_Z = 1.1;

