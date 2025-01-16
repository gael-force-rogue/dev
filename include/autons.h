#pragma once

#include <vector>
#include <string>

// RED AUTONS MUST ALWAYS BE ODD (or 0)
// MUST BE IN ORDER FOR AUTON SELECTOR (red blue red blue is a good guide)
enum Auton {
    SKILLS = 0,
    RED_AWP = 1,
    BLUE_AWP = 2,
    RED_MOGORUSH = 3,
    BLUE_MOGORUSH = 4,
    RED_ELIMS = 5,
    BLUE_ELIMS = 6
};

void blue_awp();
void blue_elims();
void blue_mogorush();
void red_awp();
void red_elims();
void red_mogorush();
void skills();

const std::vector<std::string> AUTON_NAMES = {"SKILLS", "RED_AWP", "BLUE_AWP", "RED_MOGORUSH", "BLUE_MOGORUSH", "RED_ELIMS", "BLUE_ELIMS"};

#define AUTON RED_AWP
#define IS_ALLIANCE_RED AUTON % 2 != 0

#define TO_STRING(x) #x
#define STRINGIFY_AUTON(x) TO_STRING(x)