#include <PilotIncludes.h>

StorageBase::StorageBase(void)
{
}

/** \name Method reads content of a specified file to the buffer 'buf' of size 'size' (in the case of NIOS-II flash memory - the name of the area).
* Method uses platform specific helper functions.
*/
bool StorageBase::loadFile (const char* fileName, void* buf, int size)
{
    bool result = this->open(fileName); // 'this' isn't necessary but it prevents the use of the global functions with the same name.

    if(!result)
    {
        Log.errorPrintf("StorageBase_loadFile_1");
        return false;
    }

    result = this->load(buf, size);
    this->close();

    if(!result)
    {
        Log.errorPrintf("StorageBase_loadFile_2");
        return false;
    }

    return true;
}

/** \name Method reads content of a specified file to the buffer 'buf' of size 'size' (in the case of NIOS-II flash memory - the name of the area).
* Method uses platform specific helper functions.
*/
bool StorageBase::saveFile (const char* fileName, void* buf, int size)
{
    bool result = this->open(fileName);	// 'this' isn't necessary but it prevents the use of the global functions with the same name.

    if(!result)
    {
        Log.errorPrintf("StorageBase_saveFile_1");
        return false;
    }

    result = this->save(buf, size);
    this->close();

    if(!result)
    {
        Log.errorPrintf("StorageBase_saveFile_2");
        return false;
    }

    return true;
}

/** \name Method returns free log space in bytes. This is a dummy method which should be redefined in derived class.
* \return Free log space in bytes or '-1' when space is infinite.
*/
int StorageBase::getFree (void)
{
    return -1;
}

/** \name Methods tests file saving.
*/
bool StorageBase::saveTest(const char* fname)
{
    struct strTest
    {
        int i;
        float f;
        double d;
        char c[20];
    };

	// Random data generation for each method call
    int t = Os->ticks();
    struct strTest s1 = {t, static_cast<float>(t), static_cast<double>(t)};
    SNPRINTF (s1.c, sizeof(s1.c), "%i", t);
    struct strTest s2 = {0,0,0,""};
    bool ret = false;

    bool b = open(fname);
    if (b)
    {
        bool bs = save (&s1, sizeof(s1));
        if (bs)
        {
            bool bl = load (&s2, sizeof(s2));
            if (bl)
            {
                if (s1.i == s2.i && s1.f == s2.f && strcmp(s1.c, s2.c) == 0)
                    ret = true;
            }
        }
        close ();
    }

    return ret;
}

/** \name Methods tests appending of a line of text to a file.
*/
bool StorageBase::appendTest(const char* fname)
{
    static const int NLINES = 10;

    // Initialization by 0 as additional protection against strcmp going out of the scope of a 'linBuf' array.
    char linBuf[NLINES][LINESIZE] = {0};

    // Time delay for saving data from buffer
    Os->sleepMs(100);
    int offset = getSize();

    // Random data generation for each method call
    unsigned int signature = Os->ticks();
    
    for (int i=0; i<NLINES; i++)
    {
        SNPRINTF (linBuf[i], LINESIZE, "//Linia testowa do kontroli funkcji append. Signature: %u", signature++);
        bool bs = append (linBuf[i]);
        if (!bs)
        {
            return false;
        }
    }

    Os->sleepMs(100);
    int n = getSize() - offset;
    if (n < 0)
    {
        return false;
    }
    

    char line[LINESIZE];
    line[0] = 0;
    for (int i=0; i<NLINES; i++)
    {
        readLine(line, LINESIZE, offset);
        if (strcmp (line, linBuf[i]) != 0)
        {
            return false;
        }
    }

    return true;
}
