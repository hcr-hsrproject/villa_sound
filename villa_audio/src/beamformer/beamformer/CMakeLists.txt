set(libbeam_beamformer_sources
      test_utils.h test_utils.cc
      audio_processing.h audio_processing.cc
      array_util.h array_util.cc
      nonlinear_beamformer.h nonlinear_beamformer.cc
      covariance_matrix_generator.h covariance_matrix_generator.cc
)

add_library(libbeam_beamformer ${libbeam_beamformer_sources})

target_link_libraries(libbeam_beamformer libbeam_base libbeam_commonaudio libbeam_systemwrappers)

# Don't duplicate 'lib' prefix
set_target_properties(libbeam_beamformer PROPERTIES
  PREFIX ""
)
