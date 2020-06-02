// 
// 
// 

#include "Configuration.h"


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
        if ( configFile.nameIs( "test" ) )
        {
            _testValue = configFile.getBooleanValue();
        }
    }
    configFile.end();

    return true;
}

bool Configuration::getTestValue()
{
    return _testValue;;
}
