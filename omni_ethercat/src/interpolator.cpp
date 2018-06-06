//
// Created by amaldo on 6/6/18.
//

#include "omni_ethercat/interpolator.hpp"
#include <iostream>

ReflexxesInterpolator::ReflexxesInterpolator() {
    std::cout << "ReflexxesInterpolator(): starting up." << std::endl;
}

ReflexxesInterpolator::~ReflexxesInterpolator() {
    std::cout << "~ReflexxesInterpolator(): closing shop." << std::endl;
}
