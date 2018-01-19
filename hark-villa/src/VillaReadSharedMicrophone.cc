#include "BufferedNode.h"
#include "Buffer.h"
#include "Vector.h"
#include "Map.h"
#include "Source.h"
#include "Matrix.h"
#include <sstream>
#include <iomanip>
#include <assert.h>
#include <memory>
#include <unistd.h>

#include <sys/time.h>

#include <boost/shared_ptr.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <../config.h>


#include <math.h>
#include <vector>

using namespace std;
using namespace boost;
using namespace boost::interprocess;
using namespace FD;

class VillaReadSharedMicrophone;

DECLARE_NODE(VillaReadSharedMicrophone)
/*Node
 *
 * @name VillaReadSharedMicrophone
 * @category Villa
 * @description Reads microphone data from shared memory. Compiled with CHANNELS and LENGTH at compile time
 *
 * @parameter_name ADVANCE
 * @parameter_type int
 * @parameter_value 160
 * @parameter_description Frame shift.
 *
 * @parameter_name ENABLE_DEBUG
 * @parameter_type bool
 * @parameter_value false
 * @parameter_list true:false
 * @parameter_description print debug message of this module in case of true.
 *
 * @output_name AUDIO
 * @output_type Matrix<float>
 * @output_description Windowed multi-channel sound data. A row index is a channel, and a column index is time.
 *
 * @output_name NOT_EOF
 * @output_type bool
 * @output_description Always true, this is for "condition" setting
 *
 *
 END*/

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

class VillaReadSharedMicrophone : public BufferedNode {
  bool enable_debug;     // flag whether print debug message or not.
  int advance;

  int audioID;
  int eofID;

  std::shared_ptr<mapped_region> shared_region;
  std::shared_ptr<named_mutex> shared_region_lock;
  std::shared_ptr<named_condition> shared_region_cond;

  long seq = 0;

public:
  VillaReadSharedMicrophone(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params) {
    advance = dereference_cast<int>(parameters.get("ADVANCE"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));

    audioID = addOutput("AUDIO");
    eofID = addOutput("NOT_EOF");

    inOrder = true;
    cout << getName() << " constructor end..." << endl;
  }

  virtual void initialize() {
    cout << getName() << " initialized..." << endl;

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

    this->BufferedNode::initialize();
  }


  // process per one iteration
  void calculate(int output_id, int count, Buffer &out) {

    Buffer &audioBuffer = *(outputs[audioID].buffer);

    // Always True
    Buffer &eofBuffer = *(outputs[eofID].buffer);
    eofBuffer[count] = TrueObject;

    // Matrix to write to
    RCPtr<Matrix<float> > outputp(new Matrix<float>(MIC_CHANNELS, MIC_FRAME_LENGTH));
    audioBuffer[count] = outputp;
    Matrix<float>& output = *outputp;

    // This scope will cause mutex to unlock
    {
      // Read microphone data from shared memory
      shared_memory_log *data = new (shared_region->get_address()) shared_memory_log;
      scoped_lock<named_mutex> lock(*shared_region_lock);

      while (data->seq <= seq) {
        shared_region_cond->wait(lock);
      }

      for (int c = 0; c < MIC_CHANNELS; c++) {
        for (int t = 0; t < MIC_FRAME_LENGTH; t++) {
          output(c, t) = data->data[MIC_FRAME_LENGTH*c + t];
        }
      }

      seq = data->seq;
    }


  }

  IN_ORDER_NODE_SPEEDUP(VillaReadSharedMicrophone)

};
