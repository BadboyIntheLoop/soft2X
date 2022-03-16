/**
*                                                                   
* @class ParameterNames                                                    
*                                                                   
* @brief Class to link name of subsystems parameters with their adrresses.
* Used to interpret command setting and getting parameters.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#ifndef PARAMETERNAMES_H
#define PARAMETERNAMES_H

class ClassifiedLine;
class PidParams;
struct ServoConf;

class ParameterNames
{
public:
    enum ERRCODE
    {
        ERR_OK = 0,
        ERR_READONLY,
        ERR_OTHER
    };

    ParameterNames (const char* prefix, int maxParameters, const char* parentPrefix=NULL); ///< Parameterized constructor, no default one

    ERRCODE setParams (const char* nameValueItems, unsigned int* userActionFlags=NULL);
    bool getParam (const char* name, ClassifiedLine& cl) const;
    bool getParam (const char* name, char* buf, int bufsize) const;

    //  Function that defines parameter (overloaded for different parameters)
    bool insert (const char* name, float* ptr, float vmin, float vmax, int decimal=3, bool readOnly=false,
        unsigned int userActionFlag=0);
    bool insert (const char* name, double* ptr, double vmin, double vmax, int decimal=6, bool readOnly=false);
    bool insert (const char* name, int* ptr, int vmin, int vmax, bool readOnly=false, unsigned int userActionFlag=0);
    bool insert (const char* name, bool* ptr, bool readOnly=false, unsigned int userActionFlag=0);
    bool insert (const char* name, GpsPosition* gpsPtr, GpsPosition::setDblFun setFun,
       GpsPosition::getDblFun getFun, double vmin, double vmax, int decimal=6, bool readOnly=false,
       unsigned int userActionFlag=0);

    //  Functions defining parameters being specilized containers
    bool insert (const char* name, FPRealData::ControllerProperties* ptr);
    bool insert (const char* name, PidParams* ptr);
	bool insert (const char* name, ServoConf* s);
	bool insert(const char* name, L1Params* s);
    bool insert(const char* name, TECSParams* s);
    // Function defining general container
    bool insertContainer (const char* name, int maxItems, ParameterNames*& innerFields);

	bool isParsValid (void);

private:
    //  Base class for parameters different types.
    class ParBase
    {
    public:
        const char*  _name;           ///< Name of parameter
        bool         _isContainer;    ///< If parameter is a container for other parameters
        bool         _isReadOnly;     ///< Is it read only
        int          _nameLength;     ///< Names length (only for _isContainer==true)
        unsigned int _userActionFlag; ///< Any number > 0 (stored and returned after parameters setting)
        ParameterNames* _fields;      ///< Internal container's fields

        virtual ERRCODE setValue (const char* fullName, const char* value);
        virtual ERRCODE setContainerValue (const char* fullName, const char* value, unsigned int* userActionFlags);
        virtual bool getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const;
        virtual bool getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const;

    protected:
        ParBase(const char* name, bool isContainer, bool readOnly, unsigned int userActionFlag=0); ///< Parameterized constructor
        virtual ~ParBase(void){}; ///< destructor lock - object could not be destroyed

    private:
        // Locking of the copy constructor and assigment operator
        ParBase(ParBase&);
        ParBase& operator=(const ParBase&);
    };


    //  Class for parameters type float
    class ParFloat: public ParBase
    {
    public:
        //  Parameterized constructor inline
        ParFloat(const char* name, float *fptr, float vmin, float vmax, int decimal, bool readOnly,
            unsigned int userActionFlag)
        :   ParBase(name, false, readOnly, userActionFlag), _fptr(fptr), _vmin(vmin), _vmax(vmax), _decimal(decimal)
        {};
        float* _fptr;      ///< Pointer to parameters value
        float  _vmin;      ///< Minimal value checked by the setParam function
        float  _vmax;      ///< Maximum value checked by the setParam function
        int    _decimal;   ///< Decimal point of displayed value

        ERRCODE setValue (const char* fullName, const char* value);
        bool getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const;
        bool getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const;
    };


    //  Class for parameters type double
    class ParDouble: public ParBase
    {
    public:
        //  Parameterized constructor inline
        ParDouble(const char* name, double *dptr, double vmin, double vmax, int decimal, bool readOnly)
        :   ParBase(name, false, readOnly), _dptr(dptr), _vmin(vmin), _vmax(vmax), _decimal(decimal)
        {};
        double* _dptr;      ///< Pointer to parameters value
        double  _vmin;      ///< Minimal value checked by the setParam function
        double  _vmax;      ///< Maximum value checked by the setParam function
        int     _decimal;   ///< Decimal point of displayed value

        ERRCODE setValue (const char* fullName, const char* value);
        bool getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const;
        bool getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const;
    };


    //  Class for parameteres type int
    class ParInt: public ParBase
    {
    public:
        //  Parameterized constructor inline
        ParInt(const char* name, int *iptr, int vmin, int vmax, bool readOnly, unsigned int userActionFlag)
        :   ParBase(name, false, readOnly, userActionFlag), _iptr(iptr), _vmin(vmin), _vmax(vmax)
        {};
        int* _iptr;      ///< Pointer to parameters value
        int  _vmin;      ///< Minimal value checked by the setParam function
        int  _vmax;      ///< Maximum value checked by the setParam function

        ERRCODE setValue (const char* fullName, const char* value);
        bool getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const;
        bool getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const;
    };


    //  Class for parameteres type bool
    class ParBool: public ParBase
    {
    public:
        //  Parameterized constructor inline
        ParBool(const char* name, bool *bptr, bool readOnly, unsigned int userActionFlag)
        :   ParBase(name, false, readOnly, userActionFlag), _bptr(bptr)
        {};
        bool* _bptr;      ///< Pointer to parameters value

        ERRCODE setValue (const char* fullName, const char* value);
        bool getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const;
        bool getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const;
    };


    /**
	* Class for parameters type double available by the set.. and get.. function from GpsPosition class
    *  Interface of the class is type double
	*/
    class ParGpsDoubleFun: public ParBase
    {
    public:
        //  Parameterized constructor inline
        ParGpsDoubleFun(const char* name, GpsPosition* gpsPtr, GpsPosition::setDblFun setFun,
            GpsPosition::getDblFun getFun, double vmin, double vmax, int decimal, bool readOnly, unsigned int userActionFlag)
            :   ParBase(name, false, readOnly, userActionFlag), _gpsPtr(gpsPtr), 
                _setFun(setFun), _getFun(getFun), _vmin(vmin), _vmax(vmax), _decimal(decimal)
        {};
        GpsPosition* _gpsPtr;            ///< Pointer to object type GpsPosition
        GpsPosition::setDblFun _setFun;  ///< Relative pointer to the function setting parameter type double
        GpsPosition::getDblFun _getFun;  ///< Relative pointer to the function getting parameter type double
        double  _vmin;      ///< Minimal value checked by the setParam function
        double  _vmax;      ///< Maximum value checked by the setParam function
        int     _decimal;   ///< Decimal point of displayed value

        ERRCODE setValue (const char* fullName, const char* value);
        bool getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const;
        bool getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const;
    };


    /**
	* Class for parameters type FPRealData::ControllerProperties
    * It is a container including external parameters of controllers.
	*/
    class ParCProp: public ParBase
    {
    public:
        //  Parameterized constructor 
        ParCProp(const char* name, FPRealData::ControllerProperties *cptr, const char* parentPrefix);

        FPRealData::ControllerProperties* _cptr;    // pointer to the parameters value
    };

  
	/**
	* Class for PidParams type parameters
    * It is a container including internal controllers parameters.
	*/
    class ParPidParams: public ParBase
    {
    public:
        //  Parameterized constructor
        ParPidParams(const char* name, PidParams *pp, const char* parentPrefix);

        PidParams* _pp;							    // pointer to the parameters value
    };

	
	/**
	* Class for servos parameters
	*/
	class ParServman: public ParBase
	{
	public:
		ParServman(const char* name, ServoConf *s, const char* parentPrefix);

		ServoConf* _s;
	};

	/*
	* Class for L1 parameters
	*/
    class ParL1Control : public ParBase
	{
	public:
		ParL1Control(const char* name, L1Params* s, const char* parentPrefix);
		L1Params* _s;
	};

    /*
	* Class for TECS parameters
	*/
    class ParTECSControl : public ParBase
	{
	public:
		ParTECSControl(const char* name, TECSParams* s, const char* parentPrefix);
		TECSParams* _s;
	};

    /**
	* General containers for parameters class.
    * Storage container component fields is accomplished by nesting inside the whole class.
	*/
    class ParContainer: public ParBase
    {
    public:
        //  Parameterized constructor 
        ParContainer(const char* name, int maxItems, const char* parentPrefix);
    };


	
	// Private fields
    typedef ParBase* ParBasePtr;
    ParBasePtr* _parTab;        ///< Descriptions of parameters array
    int         _nPars;         ///< Current number of elements in array
    int         _maxPars;       ///< Maximum number of elements in array
    const char* _prefix;        ///< Prefix with subsystems name displayed with getParam command
    const char* _parentPrefix;  ///< Optional prefix valid in parent container.

    // Lock copy constructor and assigment operator
    ParameterNames(ParameterNames&);
    ParameterNames& operator=(const ParameterNames&);

    // Private functions
    ERRCODE setParam (const char* name, const char* value, unsigned int* userActionFlags);

    // Lock destructor 
    ~ParameterNames(void);
};
#endif  //PARAMETERNAMES_H
