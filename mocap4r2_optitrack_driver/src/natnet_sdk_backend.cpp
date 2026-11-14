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

// Backend on the official OptiTrack NatNet SDK. Only built when the SDK is
// installed (see install_natnet_sdk.sh); the SDK itself is not distributed here.

#include <arpa/inet.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "NatNetCAPI.h"
#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "mocap4r2_optitrack_driver/natnet_backend.hpp"

namespace mocap4r2_optitrack_driver
{

namespace
{

natnet::RigidBodyDescription to_rb_description(const sRigidBodyDescription & d)
{
  natnet::RigidBodyDescription r;
  r.name = d.szName;
  r.id = d.ID;
  r.parent_id = d.parentID;
  r.offset = {d.offsetx, d.offsety, d.offsetz};
  r.orientation = {d.offsetqx, d.offsetqy, d.offsetqz, d.offsetqw};
  for (int32_t i = 0; i < d.nMarkers; i++) {
    r.marker_names.push_back(d.szMarkerNames && d.szMarkerNames[i] ? d.szMarkerNames[i] : "");
  }
  return r;
}

natnet::RigidBodyData to_rb_data(const sRigidBodyData & d)
{
  natnet::RigidBodyData r;
  r.id = d.ID;
  r.position = {d.x, d.y, d.z};
  r.orientation = {d.qx, d.qy, d.qz, d.qw};
  r.mean_error = d.MeanError;
  r.params = d.params;
  return r;
}

class SdkBackend : public NatNetBackend
{
public:
  explicit SdkBackend(const NatNetConfig & config)
  : config_(config)
  {
    in_addr a{};
    if (inet_pton(AF_INET, config_.server_address.c_str(), &a) != 1) {
      throw std::invalid_argument("invalid server_address '" + config_.server_address + "'");
    }
    if (inet_pton(AF_INET, config_.local_address.c_str(), &a) != 1) {
      throw std::invalid_argument("invalid local_address '" + config_.local_address + "'");
    }
    worker_ = std::thread(&SdkBackend::command_loop, this);
    run([this] {
        client_ = std::make_unique<::NatNetClient>();
        client_->SetFrameReceivedCallback(&SdkBackend::frame_callback, this);
      });
  }

  ~SdkBackend() override
  {
    disconnect();
    run([this] {client_.reset();});
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      quit_ = true;
    }
    queue_cv_.notify_one();
    worker_.join();
  }

  natnet::ServerInfo connect() override
  {
    return run([this] {return do_connect();});
  }

  natnet::DataDescriptions request_descriptions() override
  {
    return run([this] {return do_descriptions();});
  }

private:
  // The SDK requires every command from the thread that created the client
  // (it asserts otherwise): all calls go through this queue.
  template<typename F>
  auto run(F && f) -> decltype(f())
  {
    using R = decltype(f());
    auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
    auto result = task->get_future();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.push_back([task] {(*task)();});
    }
    queue_cv_.notify_one();
    return result.get();  // rethrows exceptions of the task
  }

  void command_loop()
  {
    while (true) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] {return quit_ || !queue_.empty();});
        if (queue_.empty()) {
          return;
        }
        job = std::move(queue_.front());
        queue_.pop_front();
      }
      job();
    }
  }

  natnet::ServerInfo do_connect()
  {
    sNatNetClientConnectParams p;
    // The SDK needs an explicit type; Motive streams multicast by default.
    multicast_ = config_.connection != NatNetConfig::Connection::UNICAST;
    p.connectionType = multicast_ ? ConnectionType_Multicast : ConnectionType_Unicast;
    p.serverAddress = config_.server_address.c_str();
    p.localAddress = config_.local_address.c_str();
    p.serverCommandPort = config_.command_port;
    p.serverDataPort = config_.data_port ? config_.data_port : 1511;
    p.multicastAddress = config_.multicast_address.empty() ? nullptr :
      config_.multicast_address.c_str();
    const ErrorCode rc = client_->Connect(p);
    if (rc != ErrorCode_OK) {
      throw std::runtime_error(
              "NatNet SDK cannot connect to " + config_.server_address + " (error " +
              std::to_string(static_cast<int>(rc)) + ")");
    }
    connected_ = true;
    sServerDescription sd;
    std::memset(&sd, 0, sizeof(sd));
    if (client_->GetServerDescription(&sd) != ErrorCode_OK || !sd.HostPresent) {
      throw std::runtime_error("NatNet SDK: no answer from " + config_.server_address);
    }
    natnet::ServerInfo info;
    info.app_name = sd.szHostApp;
    std::memcpy(info.app_version.data(), sd.HostAppVersion, 4);
    std::memcpy(info.natnet_version.data(), sd.NatNetVersion, 4);
    info.clock_frequency = sd.HighResClockFrequency;
    info.has_connection_info = sd.bConnectionInfoValid;
    info.data_port = sd.ConnectionDataPort;
    info.multicast = sd.ConnectionMulticast;
    std::memcpy(info.multicast_address.data(), sd.ConnectionMulticastAddress, 4);
    return info;
  }

  natnet::DataDescriptions do_descriptions()
  {
    sDataDescriptions * dd = nullptr;
    if (client_->GetDataDescriptionList(&dd) != ErrorCode_OK || !dd) {
      throw std::runtime_error("NatNet SDK: cannot get data descriptions");
    }
    natnet::DataDescriptions out;
    for (int i = 0; i < dd->nDataDescriptions; i++) {
      const auto & e = dd->arrDataDescriptions[i];
      if (e.type == Descriptor_MarkerSet && e.Data.MarkerSetDescription) {
        const auto * m = e.Data.MarkerSetDescription;
        natnet::MarkerSetDescription ms;
        ms.name = m->szName;
        for (int32_t k = 0; k < m->nMarkers; k++) {
          ms.marker_names.push_back(m->szMarkerNames[k]);
        }
        out.marker_sets.push_back(ms);
      } else if (e.type == Descriptor_RigidBody && e.Data.RigidBodyDescription) {
        out.rigid_bodies.push_back(to_rb_description(*e.Data.RigidBodyDescription));
      } else if (e.type == Descriptor_Skeleton && e.Data.SkeletonDescription) {
        const auto * s = e.Data.SkeletonDescription;
        natnet::SkeletonDescription sk;
        sk.name = s->szName;
        sk.id = s->skeletonID;
        for (int32_t k = 0; k < s->nRigidBodies; k++) {
          sk.bones.push_back(to_rb_description(s->RigidBodies[k]));
        }
        out.skeletons.push_back(sk);
      }
    }
    NatNet_FreeDescriptions(dd);
    return out;
  }

public:
  void start(FrameCallback on_frame, ErrorCallback on_error) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_frame_ = std::move(on_frame);
    on_error_ = std::move(on_error);
    running_ = true;
  }

  void stop() override
  {
    // The SDK keeps receiving; frames are dropped until start() again.
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);  // wait for a callback in progress
  }

  void disconnect() override
  {
    stop();
    run([this] {
        if (connected_ && client_) {
          client_->Disconnect();
          connected_ = false;
        }
      });
  }

  bool multicast() const override {return multicast_;}
  std::string name() const override {return "sdk";}

private:
  static void NATNET_CALLCONV frame_callback(sFrameOfMocapData * data, void * self)
  {
    static_cast<SdkBackend *>(self)->on_sdk_frame(*data);
  }

  void on_sdk_frame(const sFrameOfMocapData & d)
  {
    if (!running_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !on_frame_) {
      return;
    }
    natnet::FrameData f;
    f.frame_number = d.iFrame;
    for (int32_t i = 0; i < d.nOtherMarkers && d.OtherMarkers; i++) {
      f.unlabeled_markers.push_back(
        {d.OtherMarkers[i][0], d.OtherMarkers[i][1], d.OtherMarkers[i][2]});
    }
    for (int32_t i = 0; i < d.nRigidBodies; i++) {
      f.rigid_bodies.push_back(to_rb_data(d.RigidBodies[i]));
    }
    for (int32_t i = 0; i < d.nSkeletons; i++) {
      natnet::SkeletonData sk;
      sk.id = d.Skeletons[i].skeletonID;
      for (int32_t k = 0; k < d.Skeletons[i].nRigidBodies; k++) {
        sk.bones.push_back(to_rb_data(d.Skeletons[i].RigidBodyData[k]));
      }
      f.skeletons.push_back(sk);
    }
    for (int32_t i = 0; i < d.nLabeledMarkers; i++) {
      const auto & m = d.LabeledMarkers[i];
      natnet::LabeledMarker lm;
      lm.id = m.ID;
      lm.position = {m.x, m.y, m.z};
      lm.size = m.size;
      lm.params = m.params;
      lm.residual = m.residual;
      f.labeled_markers.push_back(lm);
    }
    f.timecode = d.Timecode;
    f.timecode_sub = d.TimecodeSubframe;
    f.timestamp = d.fTimestamp;
    f.camera_mid_exposure = d.CameraMidExposureTimestamp;
    f.camera_data_received = d.CameraDataReceivedTimestamp;
    f.transmit = d.TransmitTimestamp;
    f.params = d.params;
    try {
      on_frame_(std::move(f));
    } catch (const std::exception & e) {
      if (on_error_) {
        on_error_(e.what());
      }
    }
  }

  NatNetConfig config_;
  std::unique_ptr<::NatNetClient> client_;  // official SDK client, owned by worker_
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::function<void()>> queue_;
  bool quit_{false};
  std::mutex mutex_;
  std::atomic<bool> running_{false};
  bool connected_{false};
  bool multicast_{true};
  FrameCallback on_frame_;
  ErrorCallback on_error_;
};

}  // namespace

std::unique_ptr<NatNetBackend>
make_sdk_backend(const NatNetConfig & config)
{
  return std::make_unique<SdkBackend>(config);
}

}  // namespace mocap4r2_optitrack_driver
