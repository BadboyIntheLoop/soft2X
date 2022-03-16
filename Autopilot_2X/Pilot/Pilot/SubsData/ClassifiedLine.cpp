/**
*                                                                  
* @class ClassifiedLine                                                    
*                                                                   
* Class represents classified line shared by the subsystem based on the LineDispatcher class.                        
* This class is an interface between subsystems.             
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include "PilotIncludes.h"

/**
* Constructor 
*/
ClassifiedLine::ClassifiedLine (void)
{
    reset();
}

/**
* variables initialization
*/
void ClassifiedLine::reset (void)
{
    _line[0] = 0;
    _cmdId[0] = 0;
    _lineOffset = 0;
    _crc[0] = 0;
    _answerLines = 0;
    _sender = NULL;
}


/**
* Returns pointer to the begining of the line.
* Pointer could be moved relative to the beginning of buffer, when sended command consists CRC or/and command ID.
*/
const char* ClassifiedLine::getLine (void) const
{
    return (_line + _lineOffset);
}

/**
* Sends answer to the communication chanel from which line was received.
* To the answer ID of received command and CRC sum is added.
*/
bool ClassifiedLine::answer (const char* lineToSend, bool withWaiting, bool summary)
{
    char buf[LINESIZE];
    char* b = buf;

    // Add ID
    if (_cmdId[0] != 0)
    {
        *(b++) = '{';
        b = static_cast<char*>(MEMCCPY (b, _cmdId, 0, LINESIZE - (b-buf)));
        if (b == NULL)
            return false;
        b--;
        *(b++) = '}';
        *(b++) = ' ';
    }

    //  Add lines number
    if (summary)
    {
        int n = SNPRINTF (b, LINESIZE - (b-buf), "<%i> ", _answerLines);
        if (n < 0)
            return false;
        b += n;
    }

    //  Add text of the line
    MEMCCPY (b, lineToSend, 0, LINESIZE - (b-buf));
    //  Line end
    buf[LINESIZE-1] = 0;
    
    //  Incerement line sended counter
    _answerLines++;

    //  Send to communication chanel.
    //  If during receiving was CRC, CRC flag is being set.
    if (_sender != NULL)
        return _sender->sendLine (buf, false, withWaiting, _crc[0] != 0);
    else
    {
        Log.errorPrintf ("ClassifiedLine_answer_1");
        return false;
    }
}

bool ClassifiedLine::extractCrc (void)
{
    //  Protection from overflowing buffer
    if (_lineOffset < 0 || _lineOffset + CRC_BUF_LEN + 1 > LINESIZE)
        return false;

    // When line do not start with CRC prefix ignore checing
    if (_line[_lineOffset] != Crc::crcPrefix)
        return true;

    // CRC separation
    for (int i=0; i<CRC_BUF_LEN-1; i++)
    {
        _crc[i] = _line[_lineOffset];
        // Break when string ends earlier.
        if (_crc[i] == 0)
            break;
        _lineOffset++;
    }
    _crc[CRC_BUF_LEN-1] = 0;

    //  _lineOffset points first sign after CRC or 0 at the end.
    //  skip space
    while (_lineOffset < LINESIZE-1 && _line[_lineOffset] == ' ')
        _lineOffset++;

    //  compute CRC
    char tmpCrc[20];
    Crc::compute (getLine(), tmpCrc);
    
    //  Compare computed CRC with received one (first 5 signs could be spaces).
    if (STRNICMP(_crc, tmpCrc, 5) == 0)
        return true;

    return false;
}


/**
* Function set _cmdId and sets _lineBeg pointer to the beginning of the correct line text (skip command ID)
* Private function used by LineDispatcher (friend).
* Return false when command ID is incorrect.
*/
bool ClassifiedLine::extractCmdId (void)
{
    if (_line[_lineOffset] == '{')
    {
        //  Line has command ID at the beginning ( {xx})
        if (isalnum(_line[_lineOffset+1]) && 
            isalnum(_line[_lineOffset+2]) &&
            _line[_lineOffset+3] == '}')
        {
            //  Correct format
            _cmdId[0] = _line[_lineOffset+1];
            _cmdId[1] = _line[_lineOffset+2];
            _cmdId[2] = 0;

            //  skip space
            _lineOffset += 4;
            while (_line[_lineOffset] == ' ')
                _lineOffset++;
        }
        else
            return false;
    }

    return true;
}
