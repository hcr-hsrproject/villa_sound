#ifndef VILLA_AUDIO_SHARED_MICROPHONE_STREAM_H
#define VILLA_AUDIO_SHARED_MICROPHONE_STREAM_H

#include <memory>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include "beamformer/nonlinear_beamformer.h"

#define MIC_CHANNELS 4
#define MIC_FRAME_LENGTH 512

namespace villa_audio {

// Shared memory data structure
struct shared_memory_log {
  // Sequence number of data (used by reader to tell if data is fresh)
  long seq;

  // Frame shift (see the following for why this is useful)
  // https://dsp.stackexchange.com/a/17301
  int advance;

  // Items to fill
  float data[MIC_CHANNELS*MIC_FRAME_LENGTH];
};

class SharedMicrophoneStream {
  std::shared_ptr<boost::interprocess::mapped_region> shared_region;
  std::shared_ptr<boost::interprocess::named_mutex> shared_region_lock;
  std::shared_ptr<boost::interprocess::named_condition> shared_region_cond;

  std::shared_ptr<webrtc::NonlinearBeamformer> bf;

  long seq = 0;
  int advance = 0;

  public:
    SharedMicrophoneStream();
    std::vector<float> read(bool beamform, bool allchannels);

  private:
    std::vector<float> readRaw();
};

}

#endif
