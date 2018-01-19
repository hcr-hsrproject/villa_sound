/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_INCLUDE_MODULE_COMMON_TYPES_H_
#define WEBRTC_MODULES_INCLUDE_MODULE_COMMON_TYPES_H_

#include <assert.h>
#include <string.h>  // memcpy

#include <algorithm>
#include <limits>

#include "base/constructormagic.h"
#include "base/deprecation.h"
#include "base/optional.h"
#include "base/safe_conversions.h"
// #include "common_types.h"
#include "beamformer/typedefs.h"

namespace webrtc {

/* This class holds up to 60 ms of super-wideband (32 kHz) stereo audio. It
 * allows for adding and subtracting frames while keeping track of the resulting
 * states.
 *
 * Notes
 * - The total number of samples is samples_per_channel_ * num_channels_
 * - Stereo data is interleaved starting with the left channel.
 */
class AudioFrame {
 public:
  // Using constexpr here causes linker errors unless the variable also has an
  // out-of-class definition, which is impractical in this header-only class.
  // (This makes no sense because it compiles as an enum value, which we most
  // certainly cannot take the address of, just fine.) C++17 introduces inline
  // variables which should allow us to switch to constexpr and keep this a
  // header-only class.
  enum : size_t {
    // Stereo, 32 kHz, 60 ms (2 * 32 * 60)
    kMaxDataSizeSamples = 3840,
    kMaxDataSizeBytes = kMaxDataSizeSamples * sizeof(int16_t),
  };

  enum VADActivity {
    kVadActive = 0,
    kVadPassive = 1,
    kVadUnknown = 2
  };
  enum SpeechType {
    kNormalSpeech = 0,
    kPLC = 1,
    kCNG = 2,
    kPLCCNG = 3,
    kUndefined = 4
  };

  AudioFrame();

  // Resets all members to their default state.
  void Reset();

  void UpdateFrame(int id, uint32_t timestamp, const int16_t* data,
                   size_t samples_per_channel, int sample_rate_hz,
                   SpeechType speech_type, VADActivity vad_activity,
                   size_t num_channels = 1);

  void CopyFrom(const AudioFrame& src);

  // data() returns a zeroed static buffer if the frame is muted.
  // mutable_frame() always returns a non-static buffer; the first call to
  // mutable_frame() zeros the non-static buffer and marks the frame unmuted.
  const int16_t* data() const;
  int16_t* mutable_data();

  // Prefer to mute frames using AudioFrameOperations::Mute.
  void Mute();
  // Frame is muted by default.
  bool muted() const;

  // These methods are deprecated. Use the functions in
  // webrtc/audio/utility instead. These methods will exists for a
  // short period of time until webrtc clients have updated. See
  // webrtc:6548 for details.
  RTC_DEPRECATED AudioFrame& operator>>=(const int rhs);
  RTC_DEPRECATED AudioFrame& operator+=(const AudioFrame& rhs);

  int id_;
  // RTP timestamp of the first sample in the AudioFrame.
  uint32_t timestamp_ = 0;
  // Time since the first frame in milliseconds.
  // -1 represents an uninitialized value.
  int64_t elapsed_time_ms_ = -1;
  // NTP time of the estimated capture time in local timebase in milliseconds.
  // -1 represents an uninitialized value.
  int64_t ntp_time_ms_ = -1;
  size_t samples_per_channel_ = 0;
  int sample_rate_hz_ = 0;
  size_t num_channels_ = 0;
  SpeechType speech_type_ = kUndefined;
  VADActivity vad_activity_ = kVadUnknown;

 private:
  // A permamently zeroed out buffer to represent muted frames. This is a
  // header-only class, so the only way to avoid creating a separate empty
  // buffer per translation unit is to wrap a static in an inline function.
  static const int16_t* empty_data() {
    static const int16_t kEmptyData[kMaxDataSizeSamples] = {0};
    static_assert(sizeof(kEmptyData) == kMaxDataSizeBytes, "kMaxDataSizeBytes");
    return kEmptyData;
  }

  int16_t data_[kMaxDataSizeSamples];
  bool muted_ = true;

  RTC_DISALLOW_COPY_AND_ASSIGN(AudioFrame);
};

inline AudioFrame::AudioFrame() {
  // Visual Studio doesn't like this in the class definition.
  static_assert(sizeof(data_) == kMaxDataSizeBytes, "kMaxDataSizeBytes");
}

inline void AudioFrame::Reset() {
  id_ = -1;
  // TODO(wu): Zero is a valid value for |timestamp_|. We should initialize
  // to an invalid value, or add a new member to indicate invalidity.
  timestamp_ = 0;
  elapsed_time_ms_ = -1;
  ntp_time_ms_ = -1;
  muted_ = true;
  samples_per_channel_ = 0;
  sample_rate_hz_ = 0;
  num_channels_ = 0;
  speech_type_ = kUndefined;
  vad_activity_ = kVadUnknown;
}

inline void AudioFrame::UpdateFrame(int id,
                                    uint32_t timestamp,
                                    const int16_t* data,
                                    size_t samples_per_channel,
                                    int sample_rate_hz,
                                    SpeechType speech_type,
                                    VADActivity vad_activity,
                                    size_t num_channels) {
  id_ = id;
  timestamp_ = timestamp;
  samples_per_channel_ = samples_per_channel;
  sample_rate_hz_ = sample_rate_hz;
  speech_type_ = speech_type;
  vad_activity_ = vad_activity;
  num_channels_ = num_channels;

  const size_t length = samples_per_channel * num_channels;
  assert(length <= kMaxDataSizeSamples);
  if (data != nullptr) {
    memcpy(data_, data, sizeof(int16_t) * length);
    muted_ = false;
  } else {
    muted_ = true;
  }
}

inline void AudioFrame::CopyFrom(const AudioFrame& src) {
  if (this == &src) return;

  id_ = src.id_;
  timestamp_ = src.timestamp_;
  elapsed_time_ms_ = src.elapsed_time_ms_;
  ntp_time_ms_ = src.ntp_time_ms_;
  muted_ = src.muted();
  samples_per_channel_ = src.samples_per_channel_;
  sample_rate_hz_ = src.sample_rate_hz_;
  speech_type_ = src.speech_type_;
  vad_activity_ = src.vad_activity_;
  num_channels_ = src.num_channels_;

  const size_t length = samples_per_channel_ * num_channels_;
  assert(length <= kMaxDataSizeSamples);
  if (!src.muted()) {
    memcpy(data_, src.data(), sizeof(int16_t) * length);
    muted_ = false;
  }
}

inline const int16_t* AudioFrame::data() const {
  return muted_ ? empty_data() : data_;
}

// TODO(henrik.lundin) Can we skip zeroing the buffer?
// See https://bugs.chromium.org/p/webrtc/issues/detail?id=5647.
inline int16_t* AudioFrame::mutable_data() {
  if (muted_) {
    memset(data_, 0, kMaxDataSizeBytes);
    muted_ = false;
  }
  return data_;
}

inline void AudioFrame::Mute() {
  muted_ = true;
}

inline bool AudioFrame::muted() const { return muted_; }

inline AudioFrame& AudioFrame::operator>>=(const int rhs) {
  assert((num_channels_ > 0) && (num_channels_ < 3));
  if ((num_channels_ > 2) || (num_channels_ < 1)) return *this;
  if (muted_) return *this;

  for (size_t i = 0; i < samples_per_channel_ * num_channels_; i++) {
    data_[i] = static_cast<int16_t>(data_[i] >> rhs);
  }
  return *this;
}

inline AudioFrame& AudioFrame::operator+=(const AudioFrame& rhs) {
  // Sanity check
  assert((num_channels_ > 0) && (num_channels_ < 3));
  if ((num_channels_ > 2) || (num_channels_ < 1)) return *this;
  if (num_channels_ != rhs.num_channels_) return *this;

  bool noPrevData = muted_;
  if (samples_per_channel_ != rhs.samples_per_channel_) {
    if (samples_per_channel_ == 0) {
      // special case we have no data to start with
      samples_per_channel_ = rhs.samples_per_channel_;
      noPrevData = true;
    } else {
      return *this;
    }
  }

  if ((vad_activity_ == kVadActive) || rhs.vad_activity_ == kVadActive) {
    vad_activity_ = kVadActive;
  } else if (vad_activity_ == kVadUnknown || rhs.vad_activity_ == kVadUnknown) {
    vad_activity_ = kVadUnknown;
  }

  if (speech_type_ != rhs.speech_type_) speech_type_ = kUndefined;

  if (!rhs.muted()) {
    muted_ = false;
    if (noPrevData) {
      memcpy(data_, rhs.data(),
             sizeof(int16_t) * rhs.samples_per_channel_ * num_channels_);
    } else {
      // IMPROVEMENT this can be done very fast in assembly
      for (size_t i = 0; i < samples_per_channel_ * num_channels_; i++) {
        int32_t wrap_guard =
            static_cast<int32_t>(data_[i]) + static_cast<int32_t>(rhs.data_[i]);
        data_[i] = rtc::saturated_cast<int16_t>(wrap_guard);
      }
    }
  }

  return *this;
}

template <typename U>
inline bool IsNewer(U value, U prev_value) {
  static_assert(!std::numeric_limits<U>::is_signed, "U must be unsigned");
  // kBreakpoint is the half-way mark for the type U. For instance, for a
  // uint16_t it will be 0x8000, and for a uint32_t, it will be 0x8000000.
  constexpr U kBreakpoint = (std::numeric_limits<U>::max() >> 1) + 1;
  // Distinguish between elements that are exactly kBreakpoint apart.
  // If t1>t2 and |t1-t2| = kBreakpoint: IsNewer(t1,t2)=true,
  // IsNewer(t2,t1)=false
  // rather than having IsNewer(t1,t2) = IsNewer(t2,t1) = false.
  if (value - prev_value == kBreakpoint) {
    return value > prev_value;
  }
  return value != prev_value &&
         static_cast<U>(value - prev_value) < kBreakpoint;
}

inline bool IsNewerSequenceNumber(uint16_t sequence_number,
                                  uint16_t prev_sequence_number) {
  return IsNewer(sequence_number, prev_sequence_number);
}

inline bool IsNewerTimestamp(uint32_t timestamp, uint32_t prev_timestamp) {
  return IsNewer(timestamp, prev_timestamp);
}

inline uint16_t LatestSequenceNumber(uint16_t sequence_number1,
                                     uint16_t sequence_number2) {
  return IsNewerSequenceNumber(sequence_number1, sequence_number2)
             ? sequence_number1
             : sequence_number2;
}

inline uint32_t LatestTimestamp(uint32_t timestamp1, uint32_t timestamp2) {
  return IsNewerTimestamp(timestamp1, timestamp2) ? timestamp1 : timestamp2;
}

// Utility class to unwrap a number to a larger type. The numbers will never be
// unwrapped to a negative value.
template <typename U>
class Unwrapper {
  static_assert(!std::numeric_limits<U>::is_signed, "U must be unsigned");
  static_assert(std::numeric_limits<U>::max() <=
                    std::numeric_limits<uint32_t>::max(),
                "U must not be wider than 32 bits");

 public:
  // Get the unwrapped value, but don't update the internal state.
  int64_t UnwrapWithoutUpdate(U value) {
    if (!last_value_)
      return value;

    constexpr int64_t kMaxPlusOne =
        static_cast<int64_t>(std::numeric_limits<U>::max()) + 1;

    U cropped_last = static_cast<U>(*last_value_);
    int64_t delta = value - cropped_last;
    if (IsNewer(value, cropped_last)) {
      if (delta < 0)
        delta += kMaxPlusOne;  // Wrap forwards.
    } else if (delta > 0 && (*last_value_ + delta - kMaxPlusOne) >= 0) {
      // If value is older but delta is positive, this is a backwards
      // wrap-around. However, don't wrap backwards past 0 (unwrapped).
      delta -= kMaxPlusOne;
    }

    return *last_value_ + delta;
  }

  // Only update the internal state to the specified last (unwrapped) value.
  void UpdateLast(int64_t last_value) {
    last_value_ = rtc::Optional<int64_t>(last_value);
  }

  // Unwrap the value and update the internal state.
  int64_t Unwrap(U value) {
    int64_t unwrapped = UnwrapWithoutUpdate(value);
    UpdateLast(unwrapped);
    return unwrapped;
  }

 private:
  rtc::Optional<int64_t> last_value_;
};

using SequenceNumberUnwrapper = Unwrapper<uint16_t>;
using TimestampUnwrapper = Unwrapper<uint32_t>;

struct PacedPacketInfo {
  PacedPacketInfo() {}
  PacedPacketInfo(int probe_cluster_id,
                  int probe_cluster_min_probes,
                  int probe_cluster_min_bytes)
      : probe_cluster_id(probe_cluster_id),
        probe_cluster_min_probes(probe_cluster_min_probes),
        probe_cluster_min_bytes(probe_cluster_min_bytes) {}

  bool operator==(const PacedPacketInfo& rhs) const {
    return send_bitrate_bps == rhs.send_bitrate_bps &&
           probe_cluster_id == rhs.probe_cluster_id &&
           probe_cluster_min_probes == rhs.probe_cluster_min_probes &&
           probe_cluster_min_bytes == rhs.probe_cluster_min_bytes;
  }

  static constexpr int kNotAProbe = -1;
  int send_bitrate_bps = -1;
  int probe_cluster_id = kNotAProbe;
  int probe_cluster_min_probes = -1;
  int probe_cluster_min_bytes = -1;
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_INCLUDE_MODULE_COMMON_TYPES_H_
