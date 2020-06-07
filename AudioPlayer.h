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

constexpr auto READY_SOUND = "ready.wav";
constexpr auto MAVLINK_GOOD_SOUND = "mlgood.wav";
constexpr auto MOTORS_ARMED_SOUND = "armed.wav";
constexpr auto AUTO_MODE_SOUND = "auto.wav";
constexpr auto EMERGENCY_STOP_SOUND = "estop.wav";
constexpr auto MAVLINK_BAD_SOUND = "mlbad.wav";
constexpr auto PROGRESS_STOPPED_SOUND = "stopped.wav";
constexpr auto WRONG_DIRECTION_SOUND = "wrongdir.wav";
constexpr auto NOT_AUTO_SOUND = "autostop.wav";
constexpr auto MISSION_END_SOUND = "ended.wav";

constexpr const char* SOUND_DIRECTORY = "sounds/";
constexpr int FILE_QUEUE_SIZE = 20;
constexpr int MAX_FILEPATH_SIZE = 255;

class AudioPlayer
{
public:
	AudioPlayer();
	~AudioPlayer();
	void play(const char* filename );
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

