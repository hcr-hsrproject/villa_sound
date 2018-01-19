Villa Sound [![Build Status](https://travis-ci.com/AustinVillaatHome/villa_sound.svg?token=1o9Avy4mGixFuRg9knBP&branch=master)](https://travis-ci.com/AustinVillaatHome/villa_sound)
===========
Contains sound functionality which includes microphone streaming, microphone array signal processing (sound localization, beamforming), and speech recognition.

Architecture
------------
Digital audio from the microphone is published to shared memory for subscribers to concurrently use. These subscribers are sound localization, noise calibration, beamforming, and speech recognition, of which we will now give an overview of.

*Sound localization* is done using GEVD-MUSIC [1], which improves upon MUSIC by using a pre-measured noise correlation matrix that our *noise calibration* module generates. The result of this is used directly (e.g. in the speech recongition task) and is also passed on to the *beamformer*. The beamformer's role is to improve the signal to noise ratio for significantly better speech recognition which it accomplishes by using a form of spatial filtering where the microphone array is digitally "steered" towards the speech source and the sensitivity of the array is decreased at all directions away from the speaker. Specifically, we use [2]. The end result of this is passed onto Google Speech for our *speech recognition* to generate speech transcripts on the fly.

Installation
------------
First install required dependencies:

    sudo bash -c 'echo -e "deb http://archive.hark.jp/harkrepos $(lsb_release -cs) non-free\ndeb-src http://archive.hark.jp/harkrepos $(lsb_release -cs) non-free" > /etc/apt/sources.list.d/hark.list'
    wget -q -O - http://archive.hark.jp/harkrepos/public.gpg | sudo apt-key add -
    curl -sL https://deb.nodesource.com/setup_6.x | sudo -E bash -
    sudo apt-get install -y nodejs
    sudo apt-get update
    sudo apt-get install harkfd hark-designer
    sudo apt-get install julius-4.2.3-hark-plugin
    sudo apt-get install harktool4
    sudo apt-get install harktool5
    sudo apt-get install kaldidecoder-hark
    sudo apt-get install harkfd hark-ros-indigo hark-ros-stacks-indigo
    sudo apt-get install autoconf

Next, run `villa_sound_localization/build_custom_nodes.sh`. Note that this needs a superuser, so if doing this on the robot, the austin or zilker account must be used.

Finally, run `catkin build villa_sound_localization villa_audio`.

**Caveat (local installation):** If you are installing this locally, you probably do not have a 4-microphone array. In that case, do the following. Edit `build_custom_nodes.sh` by changing MIC_CHANNELS to 1. Do the same with `villa_audio/include/villa_audio/shared_microphone_stream.h` and `villa_sound_localization/share/network/audiostream_sharedmem.n`.

Usage
-----
**Important:** The noise correlation matrix described above must be generated when the environment changes (i.e. as much as possible). To do so, run `rosrun villa_sound_localization calibrate_noise`.

The main entry point is the sound localization node, which can be started by running `rosrun villa_sound_localization sound_localization`. This will also start up the microphone streaming. To read from the microphone, use the *SharedMicrophoneStream* class in villa_audio.

The MUSIC spectrum (crucial for debugging) can be viewed by running `rosrun villa_sound_localization visualize`. Run this *locally* in hsrb_mode.

![spectrum](https://user-images.githubusercontent.com/2482629/27843577-d6d258e8-60da-11e7-83af-dd97d06f36f9.png)

Speech recognition can be run with `rosrun villa_audio google_speech.py`. This will publish transcripts to the rostopic */villa/speech_transcripts*.

**Parameter Tuning**
(To be written)

References
----------
[1] *Nakamura, Keisuke, et al. "Intelligent sound source localization for dynamic environments." Intelligent Robots and Systems, 2009. IROS 2009. IEEE/RSJ International Conference on. IEEE, 2009.*

[2]  *Kleijn, Willem Bastiaan. Methods and Systems for Robust Beamforming. Google Inc, assignee. Patent US9502021. 22 Nov. 2016. Print.*
