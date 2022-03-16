/**
*                                                                  
* @ class WindResolver                                                      
*                                                                   
* Class calculates wind speed and direction based on the differences between airspeed and groundspeed at different angles of the track.
* Object of this class is binnary saved in the memory. It can not store any pointers or virtual functions.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef WINDRESOLVER_H
#define WINDRESOLVER_H

class PStateData;
class PStateHealth;

class WindResolver
{
public:
    WindResolver (void) :
      _lastSampleTime100(0), _isWindValid(false), _a2SampleCnt(0), _a2StartTime100(0)
    {};

    int  putData (const PStateData& psd, const PStateHealth& psh);  ///<  Passing new set data needet to wind calculations.
    bool  getWind (float &from, float &speed) const;
    void  reset (void);

private:
    class InputData
    {
    public:
        int   time100;
        float track;
        float groundspeed;
        float tas;
        bool  isValid;

        InputData (void) :
            time100(0), track(0.0f), groundspeed(0.0f), tas(0.0f), isValid(false)
        {};
    };

    class WindVector
    {
    public:
        WindVector (void):  _x(0.0f), _y(0.0f) {};
        void add (const WindVector &w);

        float _x,_y;
    };

    /**
	* Wind averager.
    * Averaging take place during new element adding.(putWind)
	*/
    class WindAverager
    {
    public:
         static const int NITEMS = 6;    // Maximum number of averaging elements.

        WindAverager (void) { reset(); };
        void putWind (const WindVector &w);
        bool getAvgWind (float &from, float &speed) const;
        void reset (void);

    private:
        WindVector  _items[NITEMS];      //  Queue of averaging elements.
        int         _usedItems;          //  Cuurent number of elements in queue
        int         _newPos;             //  index of the new position
        float       _avgSpeed;           //  Average wind speed [kph]
        float       _avgFrom;            //  Average wind direction [stopnie]
    };

    InputData dataOld;
    InputData dataNew;
    int _lastSampleTime100;
    bool      _isWindValid;
    WindAverager _wavg;
    //  variables for algorithm 2
    int         _a2SampleCnt;
    int         _a2StartTime100;
    GpsPosition _a2GpsStart;
    Vector2f    _a2Trace;

    bool compute (void);
    bool a2Compute (const PStateData& psd);
};

#endif  // WINDRESOLVER_H
