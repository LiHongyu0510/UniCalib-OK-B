// Cereal polymorphism registration for veta camera types.
// Include this ONCE in exactly one compilation unit (e.g. calib_param_manager.cpp).
#pragma once
#ifndef VETA_STUB_CEREAL_REGISTRATION_H
#define VETA_STUB_CEREAL_REGISTRATION_H

#include "veta/camera/pinhole.h"
#include "veta/camera/pinhole_fisheye.h"
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>

// Register veta camera types for cereal polymorphism
CEREAL_REGISTER_TYPE(ns_veta::PinholeIntrinsic)
CEREAL_REGISTER_TYPE(ns_veta::PinholeIntrinsicBrownT2)
CEREAL_REGISTER_TYPE(ns_veta::PinholeIntrinsicFisheye)

CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::PinholeIntrinsic, ns_veta::PinholeIntrinsicBrownT2)
CEREAL_REGISTER_POLYMORPHIC_RELATION(ns_veta::PinholeIntrinsic, ns_veta::PinholeIntrinsicFisheye)

#endif // VETA_STUB_CEREAL_REGISTRATION_H
