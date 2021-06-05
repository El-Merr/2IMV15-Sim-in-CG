#pragma once
#include <vector>
#include <stdlib.h>
#include <stdio.h>

class FixedObject {
    public:
        FixedObject(std::vector<float> pointsVector);
      //  virtual ~FixedObject(void);
        void DrawFixedObject();
    private:
        std::vector<float> points;
};


