// 
// 
// 

#include "Configuration.h"
constexpr unsigned int str2int( const char* str, int h = 0 )
{
    return !str[h] ? 5381 : (str2int( str, h + 1 ) * 33) ^ str[h];
}

Configuration::Configuration()
{

}

bool Configuration::init( const char* configurationFilePath )
{
    const uint8_t CONFIG_LINE_LENGTH = 127;

    // The open configuration file.
    SDConfigFile configFile;

    if ( !configFile.begin( configurationFilePath, CONFIG_LINE_LENGTH ) )
    {
        return false;
    }

    while ( configFile.readNextSetting() )
    {
        const char* name = configFile.getName();

        switch ( str2int(name) )
        {
            case str2int("test"):
                _testValue = configFile.getBooleanValue();
                break;
            case str2int("testFileName"):
                _testFileName = configFile.getValue();
                break;
            case str2int( "fileSpeedMilliseonds" ):
                _testValue = configFile.getBooleanValue();
                break;
            case str2int( "LEDPin" ):
                _testFileName = configFile.getValue();
                break;
        }
    }
    configFile.end();

    return true;
}

bool Configuration::getTestValue()
{
    return _testValue;;
}

const char* Configuration::getTestFileName()
{
    return _testFileName;;
}

uint8_t Configuration::getFileSpeedMilliseconds()
{
    return _fileSpeedMilliseconds;

}

uint8_t Configuration::getLEDPin()
{
    return _ledPin;
}
