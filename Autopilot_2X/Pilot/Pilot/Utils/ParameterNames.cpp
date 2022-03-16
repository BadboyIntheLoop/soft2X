/**
*                                                                   
* @class ParameterNames                                                    
*                                                                   
* @brief Class to link name of subsystems parameters with their adrresses.
* Used to interpret command setting and getting parameters.
*                                                                   
* 2008 Witold Kruczek @ Flytronic                                   
*/

#include "PilotIncludes.h"


/** 
* Parameterized constructor
* /param prefix - name of subsystem displayed by function getParameter (NOTE: specified text must be static!)
* /param maxParameters - maximum number of handled parameters in subsystem
*/
ParameterNames::ParameterNames (const char* prefix, int maxParameters, const char* parentPrefix)
{
    _nPars = 0; _maxPars = maxParameters;
    _prefix = prefix;
    // Set pointer at empty string or parameter
    _parentPrefix = parentPrefix==NULL ? "" : parentPrefix;

    //  Dynamic create array of pointers to description of parameters.
    _parTab = new ParBasePtr[maxParameters];
    if (_parTab == NULL)
        Log.abort ("Critical Error: ParameterNames_1.");
}


/**
* Set value of parameter.
* /param name - name of parameter
* /param value - new value of parameter
*/
ParameterNames::ERRCODE ParameterNames::setParam (const char* name, const char* value, unsigned int* userActionFlags)
{
    for (int i=0; i<_nPars; i++)
    {
        //  non containers
        if (!_parTab[i]->_isContainer && STRICMP (_parTab[i]->_name, name) == 0)
            //  not set when readOnly
            if (!_parTab[i]->_isReadOnly)
            {
                //  set flag assiociated with parameter
                if (userActionFlags != NULL)
                    *userActionFlags |= _parTab[i]->_userActionFlag;
                return (_parTab[i]->setValue(name, value));
            }
            else
                return ERR_READONLY;

        //  containers
        if ((_parTab[i]->_isContainer && STRNICMP (_parTab[i]->_name, name, _parTab[i]->_nameLength) == 0))
            return (_parTab[i]->setContainerValue(name, value, userActionFlags));
    }

    return ERR_OTHER;
}

/**
* Set values of may parameters
* /param nameValueItems  - series of pairs <name> <value>
* /param userActionFlags - pointer to variable in which bit flag will be set 
* Function preserve previous values of parameters in case of an error
*  userActionFlags is not defined in case of an error
*/
ParameterNames::ERRCODE ParameterNames::setParams (const char* nameValueItems, unsigned int* userActionFlags)
{
    static const int MAX_VALUE_LENGTH = 30;
    static const int MAX_PAIRS = 10;

    char prevVals[MAX_PAIRS][MAX_VALUE_LENGTH];
    ERRCODE errc = ParameterNames::ERR_OTHER;

    //  Parse pairsr <name> <value>
    FParser parser;
    if (!parser.loadLine (nameValueItems))
    {
        Log.errorPrintf("ParameterNames_setParams_1");
        return ERR_OTHER;
    }

    //  Check number of tokens
    int nt = parser.count();
    if (nt <= 0 || nt > MAX_PAIRS*2 || (nt % 2) > 0)
        return ERR_OTHER;

    int lastSet = -1;       //  last set parameter 
    bool rollback = false;  //  rollback flag in case of an error

    //  reset returned flags
    if (userActionFlags != NULL)
        *userActionFlags = 0;

    // Set next parameters
    for (int i=0; i<nt; i+=2)
    {
        //  Read previous value
        if (!getParam (parser.getToken(i), prevVals[i], MAX_VALUE_LENGTH))
        {
            Log.msgPrintf ("%sSet parameter: %s = %s [error - not found]",
                _prefix, parser.getToken(i), parser.getToken(i+1));
            //  Error in reading rollback values
            rollback = true;
            break;
        }

        const char* m2 = "system error";
        errc = setParam (parser.getToken(i), parser.getToken(i+1), userActionFlags);

        if (errc == ERR_OK)
        {
            m2 = "ok";
            lastSet = i;
        }
        else if (errc == ERR_READONLY)
            m2 = "read only - not set";
        else if (errc == ERR_OTHER)
            m2 = "error - not set";
        else
        {
            Log.errorPrintf("ParameterNames_setParams_3 [name=%s, value=%s, prefix=%s]",
                parser.getToken(i), parser.getToken(i+1), _prefix);
        }

        Log.msgPrintf ("%sSet parameter: %s = %s (from %s) [%s]",
            _prefix, parser.getToken(i), parser.getToken(i+1), prevVals[i], m2);

        if (errc != ERR_OK)
        {
            // Error in saving
            rollback = true;
            break;
        }
    }

    //  rollback values when error
    if (rollback && (lastSet >= 0))
    {
        for (int i=0; i<=lastSet; i+=2)
        {
            if (setParam (parser.getToken(i), prevVals[i], NULL) != ERR_OK)
            {
                Log.errorPrintf("ParameterNames_setParams_2 [%s = %s]", parser.getToken(i), prevVals[i]);
            }
        }
        Log.msgPrintf ("%sSet parameter error - all changes rolled back", _prefix);
    }
  
    if (!rollback)
        return ERR_OK;

    return errc;
}


/**
* Send value of parameter to the communication chanel.
* /param name - name of parameter or container
* When name is * then display all parameters.
*/
bool ParameterNames::getParam (const char* name, ClassifiedLine& cl) const
{
    bool ret = false;
    char pfxBuf[LINESIZE];
    char buf2[LINESIZE];

    for (int i=0; i<_nPars; i++)
    {
        //  fold prefix
        SNPRINTF (pfxBuf, sizeof(pfxBuf), "%s%s", _parentPrefix, _prefix);

        //  for containers check only beginning of name 
        if ((!_parTab[i]->_isContainer && *name == '*') ||
            (!_parTab[i]->_isContainer && STRICMP (_parTab[i]->_name, name) == 0) ||
            (_parTab[i]->_isContainer && STRNICMP (_parTab[i]->_name, name, _parTab[i]->_nameLength) == 0))
        {
            ret = _parTab[i]->getValue(name, pfxBuf, cl);

            //  when name is not  * then break
            if (*name != '*')
                break;
        }

        else if (_parTab[i]->_isContainer && *name == '*')
        {
            // Display name of container without number of lines
            SNPRINTF (buf2, sizeof(buf2), "%s[%s]", pfxBuf, _parTab[i]->_name);
            cl.answer (buf2, false, false);
            ret = true;
        }
    }

    return ret;
}


/**
* Get value of parameter as text
* /param name - name of parameter(could not include *)
*/
bool ParameterNames::getParam (const char* name, char* buf, int bufsize) const
{
    bool ret = false;
    char pfxBuf[LINESIZE];

    for (int i=0; i<_nPars; i++)
    {
        //  fold prefix
        SNPRINTF (pfxBuf, sizeof(pfxBuf), "%s%s", _parentPrefix, _prefix);

        if ((!_parTab[i]->_isContainer && STRICMP (_parTab[i]->_name, name) == 0) ||
            (_parTab[i]->_isContainer && STRNICMP (_parTab[i]->_name, name, _parTab[i]->_nameLength) == 0))
        {
            ret = _parTab[i]->getValue(name, pfxBuf, buf, bufsize);
            break;
        }
    }

    return ret;
}


/**
* Define type float parameter
* /param name - name of parameter(when it is an container must be eneded with dot .)
* /param ptr - pointer to value of the parameter
* /param vmin, vmax - limitations for value checked in setParam function
* /param userActionFlag - bit flag returned after set specified parameter
*/
bool ParameterNames::insert (const char* name, float* ptr, float vmin, float vmax, int decimal, bool readOnly,
                             unsigned int userActionFlag)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (float)");
        return false;
    }

    ParFloat* pf = new ParFloat(name, ptr, vmin, vmax, decimal, readOnly, userActionFlag);
    if (pf == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (float)");
        return false;
    }

    _parTab[_nPars++] = pf;

    return true;
}


/**
* Define type double parameter
* /param name - name of parameter(when it is an container must be eneded with dot .)
* /param ptr - pointer to value of the parameter
* /param vmin, vmax - limitations for value checked in setParam function
* /param userActionFlag - bit flag returned after set specified parameter
*/
bool ParameterNames::insert (const char* name, double* ptr, double vmin, double vmax, int decimal, bool readOnly)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (double)");
        return false;
    }

    ParDouble* pd = new ParDouble(name, ptr, vmin, vmax, decimal, readOnly);
    if (pd == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (double)");
        return false;
    }

    _parTab[_nPars++] = pd;

    return true;
}


/** 
* Define type int parameter.
* /param name - name of parameter(when it is an container must be eneded with dot .)
* /param ptr - pointer to value of the parameter
* /param vmin, vmax - limitations for value checked in setParam function
* /param readOnly 
* /param userActionFlag - bit flag returned after set specified parameter
*/
bool ParameterNames::insert (const char* name, int* ptr, int vmin, int vmax, bool readOnly, unsigned int userActionFlag)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (int)");
        return false;
    }

    ParInt* pi = new ParInt(name, ptr, vmin, vmax, readOnly, userActionFlag);
    if (pi == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (int)");
        return false;
    }

    _parTab[_nPars++] = pi;

    return true;
}


/** 
* Define type bool parameter.
* /param name - name of parameter(when it is an container must be eneded with dot .)
* /param ptr - pointer to value of the parameter
* /param vmin, vmax - limitations for value checked in setParam function
* /param readOnly 
* /param userActionFlag - bit flag returned after set specified parameter
*/
bool ParameterNames::insert (const char* name, bool* ptr, bool readOnly, unsigned int userActionFlag)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (bool)");
        return false;
    }

    ParBool* pb = new ParBool(name, ptr, readOnly, userActionFlag);
    if (pb == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (bool)");
        return false;
    }

    _parTab[_nPars++] = pb;

    return true;
}

/** 
* Define type double parameter available by function set.. and get..
* /param name - name of parameter(when it is an container must be eneded with dot .)
* /param gpsPtr - pointer to GpsPosition object
* /param setFun - realtive pointer to the function setting parameter value type double
* /param getFun - realtive pointer to the function getting parameter value type double
* /param vmin, vmax - limitations for value checked in setParam function
* /param readOnly 
* /param userActionFlag - bit flag returned after set specified parameter
*/
bool ParameterNames::insert (const char* name, GpsPosition* gpsPtr, GpsPosition::setDblFun setFun, GpsPosition::getDblFun getFun,
                             double vmin, double vmax, int decimal, bool readOnly, unsigned int userActionFlag)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (GpsPosition (function ptr))");
        return false;
    }

    ParGpsDoubleFun* pdf = new ParGpsDoubleFun(name, gpsPtr, setFun, getFun, vmin, vmax, decimal, readOnly, userActionFlag);
    if (pdf == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (GpsPosition (function ptr))");
        return false;
    }

    _parTab[_nPars++] = pdf;

    return true;
}


/**
* Define parameter type FPRealData::ControllerProperties
* /param name - name of parameter(when it is an container must be eneded with dot .)
* /param ptr - pointer to value of the parameter
*/
bool ParameterNames::insert (const char* name, FPRealData::ControllerProperties* ptr)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (ControllerProperties)");
        return false;
    }

    ParCProp* cp = new ParCProp(name, ptr, _prefix);
    if (cp == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (ControllerProperties)");
        return false;
    }

    _parTab[_nPars++] = cp;

    return true;
}

/**
* Define parameter type PidParams
*/
bool ParameterNames::insert (const char* name, PidParams* ptr)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (PidParams)");
        return false;
    }

    ParPidParams* pp = new ParPidParams(name, ptr, _prefix);
    if (pp == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (PidParams)");
        return false;
    }

    _parTab[_nPars++] = pp;

    return true;
}

/**
* Define parameter type ServoConf
*/
bool ParameterNames::insert (const char* name, ServoConf* s)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (ServMan)");
        return false;
    }

	ParServman* psm = new ParServman(name, s, _prefix);
    if (psm == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (ServMan)");
        return false;
    }

    _parTab[_nPars++] = psm;

    return true;
}

/*
* Define parameter type L1 Controller
*/
bool ParameterNames::insert(const char *name, L1Params *s)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (L1)");
        return false;
    }

    ParL1Control *psm = new ParL1Control(name, s, _prefix);
    if (psm == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (L1)");
        return false;
    }
    _parTab[_nPars++] = psm;
    return true;
}

/*
* Define parameter type TECS Controller
*/
bool ParameterNames::insert(const char *name, TECSParams *s)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insert_1 (TECS)");
        return false;
    }

    ParTECSControl *psm = new ParTECSControl(name, s, _prefix);
    if (psm == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insert_2 (TECS)");
        return false;
    }
    _parTab[_nPars++] = psm;
    return true;
}

/**
* Define parameter being general container
* /param name - name of container
* /param maxParameters - maximum number of parameters in the container
* /param innerPN - reference to variable in which adrress of internal ParameterNames object will be written.
*/
bool ParameterNames::insertContainer (const char* name, int maxItems, ParameterNames*& innerFields)
{
    if (!isParsValid())
    {
        Log.abort("Critical Error: ParameterNames_insertContainer_1.");
        return false;
    }

    ParContainer* pc = new ParContainer (name, maxItems, _prefix);
    if (pc == NULL)
    {
        Log.abort("Critical Error: ParameterNames_insertContainer_2.");
        return false;
    }

    _parTab[_nPars++] = pc;
    innerFields = pc->_fields;

    return true;
}


//***************************************************************************
//  ParBase Class
//***************************************************************************

/**
* Constructor
*/
ParameterNames::ParBase::ParBase(const char* name, bool isContainer, bool isReadOnly, unsigned int userActionFlag)
    : _name(name), _isContainer(isContainer), _isReadOnly(isReadOnly), _userActionFlag(userActionFlag),
    _fields(NULL)
{
    // when parameter is not an container name length is no needed
    if (_isContainer) _nameLength = strlen(_name);
    else              _nameLength = 0;
};

/** 
* Fake function to override the derived class
* /param fullName - name of parameter
* /param value - value as text
* For containers function is not used.
*/
ParameterNames::ERRCODE ParameterNames::ParBase::setValue (const char* fullName, const char* value)
{
    Log.errorPrintf("ParameterNames_ParBase_setValue_1 [fullName=%s, _name=%s]", fullName, _name);
    return ERR_OTHER;
}


/**
* Set elements value of current parameter when it is an container
* /param fullName - name of parameter
* /param value - value as text
*/
ParameterNames::ERRCODE ParameterNames::ParBase::setContainerValue (const char* fullName, const char* value, unsigned int* userActionFlags)
{
    if (_isContainer)
    {
        // Find space in whole name where start name of field
        const char* fieldName = fullName + strlen(_name);
        return _fields->setParam(fieldName, value, userActionFlags);
    }
    else
    {
        //  parameter is not a container - error
        Log.errorPrintf("ParameterNames_ParBase_setContainerValue_1 [fullName=%s, _name=%s]", fullName, _name);
        return ERR_OTHER;
    }
}


/**
* Dispalys content of element current parameter when it is a container
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param cl - communication chanel in whoch sent result
*/
bool ParameterNames::ParBase::getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const
{
    if (_isContainer)
    {
        // Find space in whole name where start name of field
        const char* fieldName = fullName + strlen(_name);
        return _fields->getParam(fieldName, cl);
    }
    else
    {
        //  Parameter is not a container - ovveride this function in derivative class
        Log.errorPrintf("ParameterNames_ParBase_getValue_1 [prefix=%s, fullName=%s, _name=%s]", prefix, fullName, _name);
        return false;
    }
}


/**
* Dispalys content of element current parameter when it is a container
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param buf - buffer to which send data
* /param bufsize - buffer size
*/
bool ParameterNames::ParBase::getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const
{
    if (_isContainer)
    {
        // Find space in whole name where start name of field
        const char* fieldName = fullName + strlen(_name);
        return _fields->getParam(fieldName, buf, bufsize);
    }
    else
    {
        //  Parameter is not a container - ovveride this function in derivative class
        Log.errorPrintf("ParameterNames_ParBase_getValue_2 [prefix=%s, fullName=%s, _name=%s]", prefix, fullName, _name);
        return false;
    }
}


//***************************************************************************
//  ParFloat class
//***************************************************************************

/**
* Sets value of current parameter
* /param fullName - name of parameter
* /param value - value as text
*/
ParameterNames::ERRCODE ParameterNames::ParFloat::setValue (const char* fullName, const char* value)
{
    float fval;

    if (TypeParser::toFloat (value, fval))
    {
        if (fval >= _vmin && fval <= _vmax)
        {
            *_fptr = fval;
            return ERR_OK;
        }
    }

    return ERR_OTHER;
}


/**
* Dispalys value of current parameter
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param cl - communication chanel to which send result
*/
bool ParameterNames::ParFloat::getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const
{
    char buf[LINESIZE];
    char format[30];

    SNPRINTF (format, sizeof(format), "%%s%%-21s = %%.%df", _decimal);

    if (SNPRINTF (buf, sizeof(buf), format, prefix, _name, *_fptr) < 0)
        return false;
    //  Display without nmber of lines
    cl.answer(buf, false, false);

    return true;
}


/**
* Read value of current parameter to the specified buffer
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param buf - bufer to whoch read data
* /param bufsize - buffer size
*/
bool ParameterNames::ParFloat::getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const
{
    char format[30];

    SNPRINTF (format, sizeof(format), "%%.%df", _decimal);

    if (SNPRINTF (buf, bufsize, format, *_fptr) < 0)
        return false;
    // macro guarante 0 at the end

    return true;
}


//***************************************************************************
//  ParDouble class
//***************************************************************************

/**
* Dispalys value of current parameter
* /param fullName - name of parameter
* /param value - value as text
*/
ParameterNames::ERRCODE ParameterNames::ParDouble::setValue (const char* fullName, const char* value)
{
    double dval;

    if (TypeParser::toDouble (value, dval))
    {
        if (dval >= _vmin && dval <= _vmax)
        {
            *_dptr = dval;
            return ERR_OK;
        }
    }

    return ERR_OTHER;
}


/**
* Dispalys value of current parameter
* /param fullName - name of parameter
* /param value - value as text
* /param cl - communication chanel to which send result
*/
bool ParameterNames::ParDouble::getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const
{
    char buf[LINESIZE];
    char format[30];

    SNPRINTF (format, sizeof(format), "%%s%%-21s = %%.%df", _decimal);

    if (SNPRINTF (buf, sizeof(buf), format, prefix, _name, *_dptr) < 0)
        return false;
    // macro guarante 0 at the end
    // display without number of lines
    cl.answer(buf, false, false);

    return true;
}

/**
* Read value of current parameter to the specified buffer
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param buf - bufer to whoch read data
* /param bufsize - buffer size
*/
bool ParameterNames::ParDouble::getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const
{
    char format[30];

    SNPRINTF (format, sizeof(format), "%%.%df", _decimal);

    if (SNPRINTF (buf, bufsize, format, *_dptr) < 0)
        return false;
    // macro guarante 0 at the end

    return true;
}


//***************************************************************************
//  ParGpsDoubleFun class
//***************************************************************************

/**
* Sets value of specified parameter using function with parameter type double.
* /param fullName - name of parameter
* /param value - value as text
*/
 
ParameterNames::ERRCODE ParameterNames::ParGpsDoubleFun::setValue (const char* fullName, const char* value)
{
    double dval;

    if (TypeParser::toDouble (value, dval))
    {
        if (dval >= _vmin && dval <= _vmax)
        {
            // Call function via pointer
            (_gpsPtr->*_setFun)(dval);
            return ERR_OK;
        }
    }

    return ERR_OTHER;
}


/**
* Dispalys value of current parameter
* /param fullName - name of parameter
* /param value - value as text
* /param cl - communication chanel to which send result
*/
bool ParameterNames::ParGpsDoubleFun::getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const
{
    char buf[LINESIZE];
    char format[30];

    SNPRINTF (format, sizeof(format), "%%s%%-21s = %%.%df", _decimal);

    double d = 0.0;
    (_gpsPtr->*_getFun)(d);     //  Wywo�anie funkcji poprzez wska�nik, w "d" wynik

    if (SNPRINTF (buf, sizeof(buf), format, prefix, _name, d) < 0)
        return false;
    // macro guarante 0 at the end
    // display without number of lines
    cl.answer(buf, false, false);

    return true;
}


/**
* Read value of current parameter to the specified buffer
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param buf - bufer to whoch read data
* /param bufsize - buffer size
*/
bool ParameterNames::ParGpsDoubleFun::getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const
{
    char format[30];

    SNPRINTF (format, sizeof(format), "%%.%df", _decimal);

    double d = 0.0;
    (_gpsPtr->*_getFun)(d);     // Call function via pointerin "d" is a result

    if (SNPRINTF (buf, bufsize, format, d) < 0)
        return false;
    // macro guarante 0 at the end

    return true;
}


//***************************************************************************
//  ParInt class
//***************************************************************************

/**
* Set current parameter values
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
*/
ParameterNames::ERRCODE ParameterNames::ParInt::setValue (const char* fullName, const char* value)
{
    int ival;

    if (TypeParser::toInt (value, ival))
    {
        if (ival >= _vmin && ival <= _vmax)
        {
            *_iptr = ival;
            return ERR_OK;
        }
    }

    return ERR_OTHER;
}

/**
* Dispalys value of current parameter
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param cl - communication chanel to which send result
*/
bool ParameterNames::ParInt::getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const
{
    char buf[LINESIZE];

    if (SNPRINTF (buf, sizeof(buf), "%s%-21s = %d", prefix, _name, *_iptr) < 0)
        return false;
    // macro guarante 0 at the end
    // display without number of lines
    cl.answer(buf, false, false);

    return true;
}


/**
* Read value of current parameter to the specified buffer
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param buf - bufer to whoch read data
* /param bufsize - buffer size
*/
bool ParameterNames::ParInt::getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const
{
    if (SNPRINTF (buf, bufsize, "%d", *_iptr) < 0)
        return false;
    // macro guarante 0 at the end

    return true;
}


//***************************************************************************
//  ParBool class
//***************************************************************************

/**
* Set current parameter values
* /param fullName - name of parameter
* /param value - as text
*/
ParameterNames::ERRCODE ParameterNames::ParBool::setValue (const char* fullName, const char* value)
{
    if (*value == '1')
    {
        *_bptr = true;
        return ERR_OK;
    }
    else if (*value == '0')
    {
        *_bptr = false;
        return ERR_OK;
    }
    return ERR_OTHER;
}

/**
* Dispalys value of current parameter
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param cl - communication chanel to which send result
*/
bool ParameterNames::ParBool::getValue (const char* fullName, const char* prefix, ClassifiedLine& cl) const
{
    char buf[LINESIZE];

    if (SNPRINTF (buf, sizeof(buf), "%s%-21s = %d", prefix, _name, static_cast<int>(*_bptr)) < 0)
        return false;
    // macro guarante 0 at the end
    // display without number of lines
    cl.answer(buf, false, false);

    return true;
}

/**
* Read value of current parameter to the specified buffer
* /param fullName - name of parameter
* /param prefix - name of subsystem to display
* /param buf - bufer to whoch read data
* /param bufsize - buffer size
*/
bool ParameterNames::ParBool::getValue (const char* fullName, const char* prefix, char* buf, int bufsize) const
{
    if (SNPRINTF (buf, bufsize, "%d", static_cast<int>(*_bptr)) < 0)
        return false;
    // macro guarante 0 at the end


    return true;
}


//***************************************************************************
//  ParCProp class
//***************************************************************************

/**
* Parameterized constructor
*/
ParameterNames::ParCProp::ParCProp(const char* name, FPRealData::ControllerProperties *cptr, const char* parentPrefix)
:   ParBase(name, true, false), _cptr(cptr)
{
	_fields = new ParameterNames(name, 10, parentPrefix);
    _fields->insert("enable", &_cptr->enable);
    _fields->insert("minValue", &_cptr->minValue, -10000.0f, 10000.0f);
    _fields->insert("maxValue", &_cptr->maxValue, -10000.0f, 10000.0f);
    _fields->insert("marginLow", &_cptr->marginLow, -10000.0f, 10000.0f);
    _fields->insert("marginHigh", &_cptr->marginHigh, -10000.0f, 10000.0f);
    _fields->insert("invMargins", &_cptr->invMargins);
    _fields->insert("bank", &_cptr->bank, 0, 1);
};

//***************************************************************************
//  ParPidParams class
//***************************************************************************

/**
* Parameterized constructor
*/
ParameterNames::ParPidParams::ParPidParams(const char* name, PidParams *pp, const char* parentPrefix)
:   ParBase(name, true, false), _pp(pp)
{
	_fields = new ParameterNames(name, 32, parentPrefix);

	_fields->insert("Kp[0]", &_pp->bank[0].Kp, -10000.0f, 10000.0f);
	_fields->insert("Ti[0]", &_pp->bank[0].Ti, 0.0f, 100.0f);
	_fields->insert("Tt[0]", &_pp->bank[0].Tt, 0.0f, 100.0f);
	_fields->insert("Td[0]", &_pp->bank[0].Td, 0.0f, 1000.0f);
	_fields->insert("N[0]",  &_pp->bank[0].N,  0.1f, 1000.0f);
	_fields->insert("wP[0]", &_pp->bank[0].wP, 0.0f, 2.0f);   
	_fields->insert("wD[0]", &_pp->bank[0].wD, 0.0f, 2.0f);
	_fields->insert("wX[0]", &_pp->bank[0].wX, 0.0f, 2.0f);
	_fields->insert("sRef[0]", &_pp->bank[0].sRef, 0.0f, 1000.0f);
	_fields->insert("sOut[0]", &_pp->bank[0].sOut, 0.0f, 1000.0f);
	_fields->insert("Imin[0]", &_pp->bank[0].Imin, -10000.0f, 10000.0f);
	_fields->insert("Imax[0]", &_pp->bank[0].Imax, -10000.0f, 10000.0f);

	_fields->insert("Kp[1]", &_pp->bank[1].Kp, -10000.0f, 10000.0f);
	_fields->insert("Ti[1]", &_pp->bank[1].Ti, 0.0f, 100.0f);
	_fields->insert("Tt[1]", &_pp->bank[1].Tt, 0.0f, 100.0f);
	_fields->insert("Td[1]", &_pp->bank[1].Td, 0.0f, 1000.0f);
	_fields->insert("N[1]",  &_pp->bank[1].N,  0.1f, 1000.0f);
	_fields->insert("wP[1]", &_pp->bank[1].wP, 0.0f, 1.0f);
	_fields->insert("wD[1]", &_pp->bank[1].wD, 0.0f, 1.0f);
	_fields->insert("wX[1]", &_pp->bank[1].wX, 0.0f, 2.0f);
	_fields->insert("sRef[1]", &_pp->bank[1].sRef, 0.0f, 1000.0f);
	_fields->insert("sOut[1]", &_pp->bank[1].sOut, 0.0f, 1000.0f);
	_fields->insert("Imin[1]", &_pp->bank[1].Imin, -10000.0f, 10000.0f);
	_fields->insert("Imax[1]", &_pp->bank[1].Imax, -10000.0f, 10000.0f);

	_fields->insert("maxKas", &_pp->maxKas, -10000.0f, 10000.0f);
	_fields->insert("tlmEnable", &_pp->tlmEnable);
	_fields->insert("logEnable", &_pp->logEnable);
};


//***************************************************************************
//  ParServman class
//***************************************************************************

/**
* Parameterized constructor
*/
ParameterNames::ParServman::ParServman(const char* name, ServoConf *s, const char* parentPrefix)
:   ParBase(name, true, false), _s(s)
{
	_fields = new ParameterNames(name, 33, parentPrefix);

	// general parameters (2)
	_fields->insert("address", &_s->address, 0, 0xFFFF);
	_fields->insert("gain", &_s->gain, -10000.0f, 10000.0f);
	_fields->insert("zero", &_s->trim, 0, 4096);
	_fields->insert("minVal", &_s->minValue, 0, 4096);	//timer PWM is 12bits
	_fields->insert("maxVal", &_s->maxValue, 0, 4096);

	
	// parameters for servo position calculations (28)
	_fields->insert("ailerons_p", &_s->coeffs_p[ServoManager::InAilerons], -10000.0f, 10000.0f);
	_fields->insert("ailerons_n", &_s->coeffs_n[ServoManager::InAilerons], -10000.0f, 10000.0f);

	_fields->insert("elevator_p", &_s->coeffs_p[ServoManager::InElevator], -10000.0f, 10000.0f);
	_fields->insert("elevator_n", &_s->coeffs_n[ServoManager::InElevator], -10000.0f, 10000.0f);
	
	_fields->insert("rudder_p", &_s->coeffs_p[ServoManager::InRudder], -10000.0f, 10000.0f);
	_fields->insert("rudder_n", &_s->coeffs_n[ServoManager::InRudder], -10000.0f, 10000.0f);

	_fields->insert("throttle_p", &_s->coeffs_p[ServoManager::InThrottle], -10000.0f, 10000.0f);
	_fields->insert("throttle_n", &_s->coeffs_n[ServoManager::InThrottle], -10000.0f, 10000.0f);

	_fields->insert("flaps_p", &_s->coeffs_p[ServoManager::InFlaps], -10000.0f, 10000.0f);
	_fields->insert("flaps_n", &_s->coeffs_n[ServoManager::InFlaps], -10000.0f, 10000.0f);

	_fields->insert("airbrakes_p", &_s->coeffs_p[ServoManager::InAirbreakes], -10000.0f, 10000.0f);
	_fields->insert("airbrakes_n", &_s->coeffs_n[ServoManager::InAirbreakes], -10000.0f, 10000.0f);

	_fields->insert("containerDrop_p", &_s->coeffs_p[ServoManager::InContainerDrop], -10000.0f, 10000.0f);
	_fields->insert("containerDrop_n", &_s->coeffs_n[ServoManager::InContainerDrop], -10000.0f, 10000.0f);

	_fields->insert("butterfly_p", &_s->coeffs_p[ServoManager::InButterfly], -10000.0f, 10000.0f);
	_fields->insert("butterfly_n", &_s->coeffs_n[ServoManager::InButterfly], -10000.0f, 10000.0f);

	_fields->insert("flapsAsAilerons_p", &_s->coeffs_p[ServoManager::InFlapsAsAilerons], -10000.0f, 10000.0f);
	_fields->insert("flapsAsAilerons_n", &_s->coeffs_n[ServoManager::InFlapsAsAilerons], -10000.0f, 10000.0f);

	_fields->insert("parachute_p", &_s->coeffs_p[ServoManager::InParachute], -10000.0f, 10000.0f);
	_fields->insert("parachute_n", &_s->coeffs_n[ServoManager::InParachute], -10000.0f, 10000.0f);

	_fields->insert("custom1_p", &_s->coeffs_p[ServoManager::InCustom1], -10000.0f, 10000.0f);
	_fields->insert("custom1_n", &_s->coeffs_n[ServoManager::InCustom1], -10000.0f, 10000.0f);

	_fields->insert("custom2_p", &_s->coeffs_p[ServoManager::InCustom2], -10000.0f, 10000.0f);
	_fields->insert("custom2_n", &_s->coeffs_n[ServoManager::InCustom2], -10000.0f, 10000.0f);

	_fields->insert("custom3_p", &_s->coeffs_p[ServoManager::InCustom3], -10000.0f, 10000.0f);
	_fields->insert("custom3_n", &_s->coeffs_n[ServoManager::InCustom3], -10000.0f, 10000.0f);

	_fields->insert("custom4_p", &_s->coeffs_p[ServoManager::InCustom4], -10000.0f, 10000.0f);
	_fields->insert("custom4_n", &_s->coeffs_n[ServoManager::InCustom4], -10000.0f, 10000.0f);

};

// params for the L1 Controller
ParameterNames::ParL1Control::ParL1Control(const char *name, L1Params *s, const char *parentPrefix)
    : ParBase(name, true, false), _s(s)
{
    _fields = new ParameterNames(name, 6, parentPrefix);
    _fields->insert("L1_period", &_s->L1_period, 10.0f, 30.0f);
    _fields->insert("L1_damping", &_s->L1_damping, 0.0f, 1.0f);
    _fields->insert("L1_xtrack_i_gain", &_s->L1_xtrack_i_gain, 0.0f, 0.1f);
    _fields->insert("loiter_bank_limit", &_s->loiter_bank_limit, 0.0f, 90.0f);
    _fields->insert("tlmEnable", &_s->tlmEnable);
    _fields->insert("logEnable", &_s->logEnable);
};

// params for the TECS Controller
ParameterNames::ParTECSControl::ParTECSControl(const char* name, TECSParams* s, const char* parentPrefix)
    : ParBase(name, true, false), _s(s)
{
    _fields = new ParameterNames(name, 23, parentPrefix);
    _fields->insert("TECS_airspeed_max", &_s->TECS_airspeed_max, 30.0f, 55.0f);
    _fields->insert("TECS_airspeed_min", &_s->TECS_airspeed_min, 14.0f, 25.0f);
    _fields->insert("throttle_cruise", &_s->throttle_cruise, 0.0f, 100.0f);
    _fields->insert("throttle_max", &_s->throttle_max, 0.0f, 100.0f);
    _fields->insert("throttle_min", &_s->throttle_min, 0.0f, 100.0f);
    _fields->insert("spdCompFiltOmega", &_s->spdCompFiltOmega, 0.5f, 2.0f);
    _fields->insert("maxClimbRate",     &_s->maxClimbRate, 0.1f, 20.0f);
    _fields->insert("minSinkRate",      &_s->minSinkRate, 0.1f, 10.0f);
    _fields->insert("maxSinkRate",      &_s->maxSinkRate, 0.0f, 20.0f);
    _fields->insert("timeConst",        &_s->timeConst, 3.0f, 10.0f);
    _fields->insert("ptchDamp",         &_s->ptchDamp, 0.0f, 1.0f);
    _fields->insert("thrDamp",          &_s->thrDamp, 0.1f, 1.0f);
    _fields->insert("integGain",        &_s->integGain, 0.0f, 0.5f);
    _fields->insert("vertAccLim",       &_s->vertAccLim, 1.0f, 10.0f);
    _fields->insert("rollComp",         &_s->rollComp, 5.0f, 30.0f);
    _fields->insert("spdWeight",        &_s->spdWeight, 0.0f, 2.0f);
    _fields->insert("pitch_max",        &_s->pitch_max, 0.0f, 45.0f);
    _fields->insert("pitch_min",        &_s->pitch_min, -45.0f, 0.0f);
    _fields->insert("pitch_ff_v0",      &_s->pitch_ff_v0, 5.0f, 50.0f);
    _fields->insert("pitch_ff_k",       &_s->pitch_ff_k, -5.0f, 0.0f);
    _fields->insert("tlmEnable",        &_s->tlmEnable);
    _fields->insert("logEnable",        &_s->logEnable);
};


//***************************************************************************
//  ParContainer class
//***************************************************************************

/**
* Parameterized constructor
*/
ParameterNames::ParContainer::ParContainer(const char* name, int maxItems, const char* parentPrefix)
:   ParBase(name, true, false)
{
	_fields = new ParameterNames(name, maxItems, parentPrefix);
}


bool ParameterNames::isParsValid ()
{
	return (_nPars < _maxPars);
}
