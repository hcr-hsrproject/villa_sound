#include <sstream>
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <memory>
#include <unistd.h>

#include <sys/time.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <math.h>
#include <vector>
#include <numeric>

#include <villa_audio/shared_microphone_stream.h>

// Beamformer
#include "beamformer/array_util.h"
#include "beamformer/test_utils.h"
#include "common_audio/audio_util.h"
#include "common_audio/channel_buffer.h"

using namespace std;
using namespace webrtc;
using namespace villa_audio;
using namespace boost;
using namespace boost::interprocess;


SharedMicrophoneStream::SharedMicrophoneStream() {
  // Construct shared memory
  bool retry = false; // Retry if failed to connect
  do {
    try {
      shared_memory_object shm(open_only, "VillaSharedMicrophone", read_write);

      // Allocate shared memory
      shared_region = make_shared<mapped_region>(shm, read_only);

      retry = false;
    } catch (interprocess_exception e) {
      // Retry if shared memory does not yet exist
      retry = true;
      usleep(100000L); // Sleep for 100 ms
    }
  } while(retry);

  shared_region_lock = make_shared<named_mutex>(open_only, "VillaSharedMicrophone_lock");
  shared_region_cond = make_shared<named_condition>(open_only, "VillaSharedMicrophone_cond");

  cout << "Region info:" << endl;
  cout << shared_region->get_address() << endl;
  cout << shared_region->get_size() << endl;

  // Set up beamformer
  string geom = "0.00 -0.03 0.00 0.00 -0.01 0.00 0.00 0.01 0.00 0.00 0.03 0.00";
  vector<Point> array_geometry = ParseArrayGeometry(geom, MIC_CHANNELS);
  bf = make_shared<NonlinearBeamformer>(array_geometry, array_geometry.size());
  bf->Initialize(10, 16000);
}

/**
 * Reads audio and postprocesses
 * @param beamform: whether to beamform or not
 */
vector<float> SharedMicrophoneStream::read(bool beamform, bool allchannels) {
  vector<float> data = this->readRaw();
  vector<float> res(advance);

  // Beamforming
  if (beamform) {
    ChannelBuffer<float> buf(advance, MIC_CHANNELS);
    FloatS16ToFloat(&data[0], data.size(), &data[0]);
    // Copy from channel buffer to data vector
    float* const* channels = buf.channels();
    for (int c=0; c < MIC_CHANNELS; c++) {
        memcpy(channels[c], &data[c*advance], advance*sizeof(float));
    }

    // Perform beamforming
    bf->AnalyzeChunk(buf);
    bf->PostFilter(&buf);

    // Copy from channel buffer to data vector
    channels = buf.channels();
    for (int c=0; c < MIC_CHANNELS; c++) {
        memcpy(&data[c*advance], channels[c], advance*sizeof(float));
    }
    FloatToFloatS16(&data[0], data.size(), &data[0]);
  }

  if (allchannels) {
    vector<float> interleaved(data.size());

    // Interleave data
    for (int c = 0; c < MIC_CHANNELS; c++) {
      int interleaved_idx = c;
      for (int j = 0; j < advance; j++) {
        interleaved[interleaved_idx] = data[advance*c + j];
        interleaved_idx += MIC_CHANNELS;
      }
    }

    return interleaved;
  }

  // Sum up all channels
  for (int c = 0; c < MIC_CHANNELS; c++) {
    for (int t = 0; t < advance; t++) {
      res[t] += data[advance*c + t];
    }
  }

  return res;
}

/**
 * Reads audio from shared memory and
 * compensates for frame shift
 */
vector<float> SharedMicrophoneStream::readRaw() {
  shared_memory_log *data = new (shared_region->get_address()) shared_memory_log;
  scoped_lock<named_mutex> lock(*shared_region_lock);

  while (data->seq <= seq) {
    shared_region_cond->wait(lock);
  }

  // Read new data (i.e. frame shift portion)
  advance = data->advance;
  vector<float> res(advance * MIC_CHANNELS);

  for (int c = 0; c < MIC_CHANNELS; c++) {
    for (int t = 0; t < advance; t++) {
      res[advance*c + t] = data->data[MIC_FRAME_LENGTH*c + (MIC_FRAME_LENGTH-advance+t)];
    }
  }

  // Update to latest seq id
  seq = data->seq;

  return res;
}

