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

class VillaSharedMicrophone;

DECLARE_NODE(VillaSharedMicrophone)
/*Node
 *
 * @name VillaSharedMicrophone
 * @category Villa
 * @description Publishes microphone data to shared memory. Compiled with CHANNELS and LENGTH at compile time
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
 * @input_name MIC_WAVE
 * @input_type Matrix<float>
 * @input_description Microphone input signals. 
 *
 * @output_name OUTPUT
 * @output_type ObjectRef
 * @output_description This is a dummy output, and it has no mean. Only for an activation of this module.
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

class VillaSharedMicrophone : public BufferedNode {
  bool enable_debug;     // flag whether print debug message or not.
  int advance;

  int mic_waveID;
  int outputID;

  std::shared_ptr<mapped_region> shared_region;
  std::shared_ptr<named_mutex> shared_region_lock;
  std::shared_ptr<named_condition> shared_region_cond;

  long seq = 0;

public:
  VillaSharedMicrophone(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params),
      mic_waveID(-1) {
    advance = dereference_cast<int>(parameters.get("ADVANCE"));
    enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));

    outputID = addOutput("OUTPUT");

    inOrder = true;
    cout << getName() << " constructor end..." << endl;
  }

  virtual void initialize() {
    cout << getName() << " initialized..." << endl;

    // Construct shared memory
    shared_memory_object shm(open_or_create, "VillaSharedMicrophone", read_write);
    shm.truncate(sizeof(shared_memory_log));

    // Allocate shared memory
    shared_region = make_shared<mapped_region>(shm, read_write);

    shared_region_lock = make_shared<named_mutex>(open_or_create, "VillaSharedMicrophone_lock");
    shared_region_cond = make_shared<named_condition>(open_or_create, "VillaSharedMicrophone_cond");

    cout << "Region info:" << endl;
    cout << shared_region->get_address() << endl;
    cout << shared_region->get_size() << endl;

    this->BufferedNode::initialize();
  }


  // dynamic input-port translation
  virtual int translateInput(string inputName) {
    //if (inputName == "STRING") {
    //    return strID = addInput(inputName);
    //}
    //else
    // check whether the input ports have arc from other module.
    if (inputName == "MIC_WAVE") {
      return mic_waveID = addInput(inputName);
    } else {
      throw new NodeException(this, inputName
			      + " is not supported.", __FILE__, __LINE__);
    }
  }

  // process per one iteration
  void calculate(int output_id, int count, Buffer &out) {

    int bytes;
    RCPtr<Matrix<float> > mic_wave_ptr;

    // bind objects of input-port to local variable
    if (mic_waveID != -1){
      mic_wave_ptr = getInput(mic_waveID, count);
      out[count] = mic_wave_ptr;
    }

    //----- for microphone array input signals
    //////////////////////////////////////////////////////////////
    if (!mic_wave_ptr.isNil()) {

      // Assert channels and length are consistent
      assert(MIC_CHANNELS == mic_wave_ptr->nrows());
      assert(MIC_FRAME_LENGTH == mic_wave_ptr->ncols());

      if (enable_debug) {
        printf("MIC_WAVE: %d %d\n", MIC_CHANNELS, MIC_FRAME_LENGTH);
      }


      // This scope will cause mutex to unlock
      {
        // Write microphone data to shared memory
        shared_memory_log *data = new (shared_region->get_address()) shared_memory_log;
        scoped_lock<named_mutex> lock(*shared_region_lock);

        for (int c = 0; c < MIC_CHANNELS; c++) {
          for (int t = 0; t < MIC_FRAME_LENGTH; t++) {
            data->data[MIC_FRAME_LENGTH*c + t] = (float)(*mic_wave_ptr)(c, t);
          }
        }

        data->advance = advance;
        data->seq = ++seq;
        shared_region_cond->notify_all();
      }

    }

    /**
     * Do NOT remove this line as the gods of synchronization have
     * deemed that the code will not work without it. May the
     * blessed soul of Edsger Dijkstra cometh forward to shed some light
     * on wtf is going on
     */
    usleep(0);

  }

  ~VillaSharedMicrophone() {
    shared_memory_object::remove("VillaSharedMicrophone");
    named_mutex::remove("VillaSharedMicrophone_lock");
    named_condition::remove("VillaSharedMicrophone_cond");
  }

  IN_ORDER_NODE_SPEEDUP(VillaSharedMicrophone)

};
