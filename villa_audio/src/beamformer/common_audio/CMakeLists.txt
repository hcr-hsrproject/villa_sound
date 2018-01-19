set(libbeam_commonaudio_sources
      audio_util.h audio_util.cc
      lapped_transform.h lapped_transform.cc
      real_fourier.h real_fourier.cc
      real_fourier_ooura.h real_fourier_ooura.cc
      window_generator.h window_generator.cc
      fft4g.h fft4g.c
      blocker.h blocker.cc
      ring_buffer.h ring_buffer.c
      audio_ring_buffer.h audio_ring_buffer.cc
      wav_file.h wav_file.cc
      wav_header.h wav_header.cc
)

add_library(libbeam_commonaudio ${libbeam_commonaudio_sources})

target_link_libraries(libbeam_commonaudio)

# Don't duplicate 'lib' prefix
set_target_properties(libbeam_commonaudio PROPERTIES
  PREFIX ""
)
