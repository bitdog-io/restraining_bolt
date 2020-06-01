// 
// 
// 

#include "AudioPlayer.h"
#include <SD.h>
#include <Audio.h>

AudioPlaySdWav           _playSdWav1;
AudioMixer4              _mixer1;
AudioOutputAnalog        _dac1;
AudioConnection _patchCord1( _playSdWav1, 0, _mixer1, 0 );
AudioConnection _patchCord2( _playSdWav1, 1, _mixer1, 3 );
AudioConnection _patchCord3( _mixer1, _dac1 );

AudioPlayer::AudioPlayer()
{
	

		_playSdWav1.play( "test.wav" );

}
