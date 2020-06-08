// AudioPlayer.h

#ifndef _AUDIOPLAYER_h
#define _AUDIOPLAYER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include <Audio.h>
#include "Queue.h"

constexpr auto READY_SOUND = "sounds/ready.wav";
constexpr auto MAVLINK_GOOD_SOUND = "sounds/mlgood.wav";
constexpr auto MOTORS_ARMED_SOUND = "sounds/armed.wav";
constexpr auto AUTO_MODE_SOUND = "sounds/auto.wav";
constexpr auto EMERGENCY_STOP_SOUND = "sounds/estop.wav";
constexpr auto MAVLINK_BAD_SOUND = "sounds/mlbad.wav";
constexpr auto PROGRESS_STOPPED_SOUND = "sounds/stopped.wav";
constexpr auto WRONG_DIRECTION_SOUND = "sounds/wrongdir.wav";
constexpr auto NOT_AUTO_SOUND = "sounds/autostop.wav";
constexpr auto MISSION_END_SOUND = "sounds/ended.wav";

constexpr int FILE_QUEUE_SIZE = 20;
constexpr int MAX_FILEPATH_SIZE = 255;

/**
 * @brief AudioPlayer plays WAV files from SD card. It will queue in FIFO order until done.
 *
*/
class AudioPlayer
{
public:

	AudioPlayer();
	~AudioPlayer();

	/**
	 * @brief Plays the WAV file at the given file path on  SD card.
	 *
	 * @param filepath File path to configuration file on SD card.
	*/
	void play(const char* filepath );

	/**
	 * @brief Used by the scheduling system to give player exeecution time
	*/
	void tick();


private:


	Queue _playQueue;
	AudioPlaySdWav  _playSdWav1;
	AudioMixer4 _mixer1;
	AudioOutputMQS  _mqs1;
	char** _charPtrArray;

	AudioConnection* _patchCord;
	AudioConnection* _patchCord2;
	AudioConnection* _patchCord3;
	AudioConnection* _patchCord4;

};
#endif

