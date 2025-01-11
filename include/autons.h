#pragma once

enum AllianceColor {
    RED = 0,
    BLUE = 1
};

// RED AUTONS MUST ALWAYS BE EVEN (or 0)
enum Auton {
    RED_AWP = 0,
    BLUE_AWP = 1,
    SKILLS = 2
};

void blue_awp();
void red_awp();
void skills();

#define AUTON SKILLS
#define IS_ALLIANCE_RED AUTON % 2 == 0
