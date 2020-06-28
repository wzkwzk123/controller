//
// Created by cjh on 5/14/20.
//

#ifndef CONTROLLER_MATH_UTILS_H
#define CONTROLLER_MATH_UTILS_H

namespace control{

    template <typename T>
    T Clamp(const T value, T bound1, T bound2){
        if(bound1 > bound2){
            std::swap(bound1, bound2);
        }

        if(value < bound1){
            return bound1;
        } else if(value > bound2){
            return bound2;
        }
        return value;
    }
}


#endif //CONTROLLER_MATH_UTILS_H
