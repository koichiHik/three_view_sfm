
#ifndef TWO_VIEW_POSE_H
#define TWO_VIEW_POSE_H

// STL
#include <vector>

// Original
#include <common.h>
#include <database.h>

namespace three_view_sfm {

void StructureFromMotionViaTwoViewGeometry(Database &database);

}  // namespace three_view_sfm

#endif