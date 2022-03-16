/**
*                                                                  
* @class ClassifiedLine                                                    
*                                                                   
* Class represents classified line shared by the subsystem based on the LineDispatcher class.                        
* This class is an interface between subsystems.             
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef CLASSIFIEDLINE_H
#define CLASSIFIEDLINE_H

class SerialDeviceBase;

class ClassifiedLine
{
public:
    friend class LineDispatcher; ///<  lineDispatcher class has access to private elements

    ClassifiedLine (void);
    void reset (void);
    const char* getLine (void) const;
    // By default number of lines is returned always (summary) besides a multiline response
    bool answer (const char* lineToSend, bool withWaiting = false, bool summary = true);
    bool extractCrc (void);

    // public fields

private:
    static const int CRC_BUF_LEN = 6;

    char _line[LINESIZE];           ///< Lines text
    int  _lineOffset;               ///< Offset to the beggining of text in the line, after bypassing command id and crc
    char _cmdId[3];                 ///< Command ID (2-bajt string)
    SerialDeviceBase* _sender;      ///< Pointer to the sender
    char _crc[CRC_BUF_LEN];         ///< CRC text with prefix and 0 ending
    int  _answerLines;              ///< Counter of sended answer lines

    bool extractCmdId (void);
};

#endif  //CLASSIFIEDLINE_H
