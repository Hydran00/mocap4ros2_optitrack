// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <stdexcept>

#include "mocap4r2_optitrack_driver/natnet_backend.hpp"
#include "mocap4r2_optitrack_driver/natnet_client.hpp"

namespace mocap4r2_optitrack_driver
{

#ifdef MOCAP4R2_HAVE_NATNET_SDK
std::unique_ptr<NatNetBackend> make_sdk_backend(const NatNetConfig & config);
#endif

bool
sdk_backend_available()
{
#ifdef MOCAP4R2_HAVE_NATNET_SDK
  return true;
#else
  return false;
#endif
}

std::unique_ptr<NatNetBackend>
make_backend(BackendKind kind, const NatNetConfig & config)
{
  if (kind == BackendKind::SDK && !sdk_backend_available()) {
    throw std::invalid_argument(
            "backend 'sdk' requested, but this build has no NatNet SDK "
            "(run install_natnet_sdk.sh and rebuild, or use backend 'open')");
  }
#ifdef MOCAP4R2_HAVE_NATNET_SDK
  if (kind != BackendKind::OPEN) {
    return make_sdk_backend(config);
  }
#endif
  return std::make_unique<NatNetClient>(config);
}

}  // namespace mocap4r2_optitrack_driver
