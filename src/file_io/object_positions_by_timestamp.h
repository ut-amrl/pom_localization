//
// Created by amanda on 6/13/21.
//

#ifndef AUTODIFF_GP_OBJECT_POSITIONS_BY_TIMESTAMP_H
#define AUTODIFF_GP_OBJECT_POSITIONS_BY_TIMESTAMP_H

#include <cstdint>

namespace file_io {

    struct ObjectPositionByTimestamp {
        std::string label_;
        uint64_t identifier_;
        uint32_t seconds_;
        uint32_t nano_seconds_;
        double transl_x_;
        double transl_y_;
        double theta_;
    };
}
#endif //AUTODIFF_GP_OBJECT_POSITIONS_BY_TIMESTAMP_H
