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

	_charPtrArray = new char* [FILE_QUEUE_SIZE];

	for ( int i = 0; i < FILE_QUEUE_SIZE; i++ )
	{
		_charPtrArray[i] = new char[MAX_FILEPATH_SIZE];
	}

	//AudioInterrupts();
	//AudioNoInterrupts();


}

AudioPlayer::~AudioPlayer()
{
	delete _patchCord;
	delete _patchCord2;
	delete _patchCord3;
	delete _patchCord4;
}

void AudioPlayer::play( const char* filename )
{
	int position = _playQueue.size();

	if ( position < FILE_QUEUE_SIZE && strlen( SOUND_DIRECTORY ) + strlen( filename ) + 1 < MAX_FILEPATH_SIZE )
	{
		_charPtrArray[position][0] = '\0';
		char* fullpath = _charPtrArray[position];
		strcpy( fullpath, SOUND_DIRECTORY );
		strcat( fullpath, filename );

		Log.trace( "scheduling sound file for playback: %s", fullpath );
		_playQueue.enqueue( fullpath );

	}

}

void AudioPlayer::tick()
{
	if ( (!_playSdWav1.isPlaying()) && _playQueue.size() > 0 )
	{
		 char* filepath = _playQueue.dequeue();

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






