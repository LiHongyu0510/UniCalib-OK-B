#ifndef VETA_STUB_VETA_H
#define VETA_STUB_VETA_H

#include "veta/type_def.hpp"
#include "veta/landmark.h"
#include <memory>
#include <map>
#include <string>

namespace ns_veta {

struct Veta {
  using Ptr = std::shared_ptr<Veta>;
  std::map<IndexT, Landmark> landmarks;
  std::map<IndexT, Posed> poses;
  std::map<std::string, IndexT> viewIdToName;
  static Ptr Create() { return std::make_shared<Veta>(); }
};

using VetaPtr = Veta::Ptr;

}  // namespace ns_veta

#endif
