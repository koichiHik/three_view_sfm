
#ifndef PNP_POSE_H
#define PNP_POSE_H

// Original
#include <common.h>
#include <database.h>

namespace three_view_sfm {

void StructureFromMotionViaPNP(Database &database);
}

#endif