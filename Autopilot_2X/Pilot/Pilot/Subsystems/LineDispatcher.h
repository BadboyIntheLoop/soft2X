#ifndef LINEDISPATCHER_H
#define LINEDISPATCHER_H

/** \file
* \brief Declaration of communication channels message dicpatcher class
*/

/** Class is responsible for reading text lines from observed subsystems (Gps, Tty0, Tty1, etc.) and recognizing kind of
* an input data by its source and content. LineDispatcher holds the last recived text lines, one per each type of source. After 
* detecting the type of a text line class sends a notification to observers associated with such type of line. LineDispatcher can
* interprets some of external commands. Successfully recognized external commnads are interpreted and executed without notifications
* and storing the content of its commands.
*/
/// Class implements dispatcher for messages from communication channel
class LineDispatcher: public ODTObserver, public ODTSubject
{
public:
    LineDispatcher(void);
    void task(const void* pdata);   ///< System task handler method.
    void linkObserver(void);        ///< Observed objects linking method.

	/** \name Methods performes classified lines acquisition (by copying an obiect)
	* \{
	*/
    bool getSimLine(ClassifiedLine &cl);
    bool getNmeaLine(ClassifiedLine &cl);
    bool getAuxNmeaLine(ClassifiedLine &cl);
    bool getCmdLine(ClassifiedLine &cl);
    bool getServoLine(ClassifiedLine &cl);
#if MAGNETOMETER_TYPE == USE_HMR
    bool getHmrLine(ClassifiedLine &cl);
#endif
	///\}

protected:
	/// Destructor is disabled (object should never be destroyed)
    virtual ~LineDispatcher(void){};

private:
    /// Copy constructor is disabled
    LineDispatcher(LineDispatcher&);
	/// Copy operator is disabled
    LineDispatcher& operator=(const LineDispatcher&);

    ClassifiedLine clArr[7];
    ClassifiedLine *nmeaLine, *cmdLine, *simLine, *tmpLine, *auxNmeaLine, *hmrLine;
#if MAGNETOMETER_TYPE == USE_HMR
    int GpsTag, Tty0Tag, Tty0ChTag, HmrTag; ///< Tags received during registration of a observed object
#else
    int GpsTag, Tty0Tag, Tty0ChTag; ///< Tags received during registration of a observed object
#endif
    int Tty1Tag, Tty2Tag;           ///< Tags received during registration of a observed object
    Semaphore _vSem;                ///< Semaphore for controlling access to the variables
    FParser prs;                    ///< Parser

    void classify (void);
    bool swapBuffers (ClassifiedLine** cl);
    bool localCmd (void);
    bool subsystemCmd (void);
    bool getXLine(ClassifiedLine &clTarget, const ClassifiedLine* clSource);
    void logInput (const ClassifiedLine* inp) const;
    void inputAuxData (void);
};

#endif  //LINEDISPATCHER
