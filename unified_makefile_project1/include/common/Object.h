//
// Created by 20174019 on 14/06/2021.
//

#ifndef UNIFIED_MAKEFILE_PROJECT1_OBJECT_H
#define UNIFIED_MAKEFILE_PROJECT1_OBJECT_H

#include <vector>
#include "gfx/vec2.h"

class Object {

public:
    virtual void draw_object();
    virtual std::vector<Vec2f> get_points();

};


#endif //UNIFIED_MAKEFILE_PROJECT1_OBJECT_H
