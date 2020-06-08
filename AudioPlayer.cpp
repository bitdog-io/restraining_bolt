// 
// 
// 

#include "AudioPlayer.h"



AudioPlayer::AudioPlayer()
{

	_patchCord = new AudioConnection( _playSdWav1, 0, _mixer1, 0 );
	_patchCord2 = new AudioConnection( _playSdWav1, 1, _mixer1, 3 );
	_patchCord3 = new AudioConnection( _mixer1, 0, _mqs1, 0 );
	_patchCord4 = new AudioConnection( _mixer1, 0, _mqs1, 1 );


}

AudioPlayer::~AudioPlayer()
{
	delete _patchCord;
	delete _patchCord2;
	delete _patchCord3;
	delete _patchCord4;

}

void AudioPlayer::play( const char* filepath )
{
	int position = _playQueue.size();

	if ( position < FILE_QUEUE_SIZE )
	{
		Log.trace( "scheduling sound file for playback: %s", filepath );
		_playQueue.enqueue( filepath );

	}

}

void AudioPlayer::tick()
{
	if ( (!_playSdWav1.isPlaying()) && _playQueue.size() > 0 )
	{
		const char* filepath = _playQueue.dequeue();

		 if ( filepath != nullptr )
		 {
			//
			 if ( !_playSdWav1.play( filepath ) )
			 {
				 Log.trace( "Could not play sound file: %s", filepath );
			 }
			 else
			 {
				 delay( 1000 );

				 while ( _playSdWav1.isPlaying() )
				 {
					 // stop processing to allow the sound to play. Unfortunately haven't found a way to allow the audio interrupts to work without it.
					 delay( 1000 );
				 }
			 }
		 }


	}
}






