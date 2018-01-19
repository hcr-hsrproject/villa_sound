#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <villa_audio/shared_microphone_stream.h>

typedef std::vector<float> AudioData;

class SharedMicrophoneStreamWrapper : public villa_audio::SharedMicrophoneStream {
  public:
    SharedMicrophoneStreamWrapper() : villa_audio::SharedMicrophoneStream() {}
};

BOOST_PYTHON_MODULE(_shared_microphone_stream_wrapper_cpp)
{
    using namespace boost::python;

    class_<AudioData>("AudioData")
        .def(vector_indexing_suite<AudioData>());

    class_<SharedMicrophoneStreamWrapper>("SharedMicrophoneStreamWrapper", boost::python::init<>())
        .def("read", &SharedMicrophoneStreamWrapper::read)
    ;
}
