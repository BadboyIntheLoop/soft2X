#include "PilotIncludes.h"

#ifndef PILOT_TARGET
	#error "Define PILOT_TARGET in pilot_cfg.h"
#endif

// Compilation on Windows only for a static analysis purpose
#if (PILOT_TARGET == PT_HARDWARE) || (PILOT_TARGET == PT_WIN32)

FlashProgrammer::FlashProgrammer(void)
{
	_address = 0;
	_length = 0; 
	_cur_address = 0;
	_cur_length = 0;
	_cmd_address = 0;
	_cmd_length = 0;
	_cur_page = 0;
	_prepared = false;
	_fastMode = false;
    _prefix = NULL;
    _crc = 0;

	 // Flash memory
    _flashMem = StorageFactory::createConfigMemory();
    if( _flashMem == NULL)
    {
        Log.abort ("Critical Error: FlashProgrammer_1.");
        return;
    }

#if PILOT_TARGET == PT_HARDWARE
		_sdram = new unsigned char[FLASH_MAX_LEN];
#else
	_sdram = new unsigned char[FLASH_MAX_LEN];
#endif	
}

/** \name Method handles the 'smon flash ...' command
*/
FLASH_PROG_RESULT FlashProgrammer::flashCmd(ClassifiedLine *pcl, FParser *pParser)
{
	int addr, len, crc;
    FLASH_PROG_RESULT res = FLASH_PROG_UCOMMAND_ERROR;

	if (pParser->count() >= 6 && STRICMP (pParser->getToken(2), "prepare") == 0)
    {
		_prepared = false;
		if (TypeParser::toInt(pParser->getToken(3), addr) && TypeParser::toInt(pParser->getToken(4), len) && TypeParser::toInt(pParser->getToken(5), crc))
		{
			if (pParser->count() == 6)
			{
				res = cmdPrepare(addr, len, crc, false);  // handling of an 'smon flash prepare <addr> <len> <crc>' command
			}
			else if ((pParser->count() == 7) && STRICMP (pParser->getToken(6), "fast") == 0)
			{
				res = cmdPrepare(addr, len, crc, true);  // handling of an 'smon flash prepare <addr> <len> <crc> FAST' command
			}
			else
			{
				res = FLASH_PROG_UCOMMAND_ERROR;
			}
		}
		else
		{
			res = FLASH_PROG_BADPAR_ERROR;
		}
	}
    else if (pParser->count() == 3 && STRICMP (pParser->getToken(2), "program") == 0)
	{
		res = cmdProgram(pcl); // handling of an 'smon flash program' command
	}
    else if (pParser->count() == 4 && STRICMP (pParser->getToken(2), "get") == 0 && STRICMP (pParser->getToken(3), "address") == 0)
	{
		res = cmdGetAddress(pcl); // handling of an 'smon flash get address' command
	}
	else if (pParser->count() == 3 && pParser->getToken(2)[0] == ':')
	{
		res =  cmdSrecord(pParser->getToken(2));
        if (_fastMode)
            // In fast mode there is no answer for a command
            res = FLASH_PROG_SILENT;
	}
    else if (pParser->count() == 4 && STRICMP (pParser->getToken(2), "read") == 0)
    {
        int offset = pParser->getTokenAsInt(3);
        char buf[97];
        char flashLine[32];
        int count = 0;
        _flashMem->readFlash(flashLine , offset*32, sizeof(flashLine));

        if (Log.secureSnprintf (buf, sizeof(buf), "line %5d: %.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x", count++,
            flashLine[0] & 0xFF,  flashLine[1] & 0xFF,  flashLine[2] & 0xFF,  flashLine[3] & 0xFF,
            flashLine[4] & 0xFF,  flashLine[5] & 0xFF,  flashLine[6] & 0xFF,  flashLine[7] & 0xFF,
            flashLine[8] & 0xFF,  flashLine[9] & 0xFF,  flashLine[10] & 0xFF, flashLine[11] & 0xFF,
            flashLine[12] & 0xFF, flashLine[13] & 0xFF, flashLine[14] & 0xFF, flashLine[15] & 0xFF,
            flashLine[16] & 0xFF, flashLine[17] & 0xFF, flashLine[18] & 0xFF, flashLine[19] & 0xFF,
            flashLine[20] & 0xFF, flashLine[21] & 0xFF, flashLine[22] & 0xFF, flashLine[23] & 0xFF,
            flashLine[24] & 0xFF, flashLine[25] & 0xFF, flashLine[26] & 0xFF, flashLine[27] & 0xFF,
            flashLine[28] & 0xFF, flashLine[29] & 0xFF, flashLine[30] & 0xFF, flashLine[31] & 0xFF) >= 0)
        {
            // Length reduction in the case when the line is too long to pass by communication channel
            buf[96] = 0;
            // Printing without line numbers
            pcl->answer(buf, true, false);
        }
        
        res = FLASH_PROG_OK;
    }
    else if (pParser->count() == 5 && 
        STRICMP (pParser->getToken(2), "read") == 0 && 
        STRICMP (pParser->getToken(4), "file") == 0)
    {
        char buf[97];
        char flashLine[32];
        int count = 0;
        crc = 0x00;
		unsigned int beginAddr = 0;
		res = FLASH_PROG_OK;
		if (STRICMP (pParser->getToken(3), "sof") == 0)
		{
			beginAddr = _address;
		}
		else if (STRICMP (pParser->getToken(3), "elf") == 0)
		{
			beginAddr = FLASH_ADDR_PTF;
		}
		else
		{
			res = FLASH_PROG_UCOMMAND_ERROR;
		}

		if (res != FLASH_PROG_UCOMMAND_ERROR)
		{
	        while(_flashMem->readFlash(flashLine , beginAddr + count*32, sizeof(flashLine)))
	        {
	            crc = 0x00;
	            for (int i=0; i<32; i++)
	                crc += flashLine[i];
	            crc &= 0xFF;

	            if (Log.secureSnprintf (buf, sizeof(buf), "line %d: %.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x", count,
	                flashLine[0] & 0xFF,  flashLine[1] & 0xFF,  flashLine[2] & 0xFF,  flashLine[3] & 0xFF,
	                flashLine[4] & 0xFF,  flashLine[5] & 0xFF,  flashLine[6] & 0xFF,  flashLine[7] & 0xFF,
	                flashLine[8] & 0xFF,  flashLine[9] & 0xFF,  flashLine[10] & 0xFF, flashLine[11] & 0xFF,
	                flashLine[12] & 0xFF, flashLine[13] & 0xFF, flashLine[14] & 0xFF, flashLine[15] & 0xFF,
	                flashLine[16] & 0xFF, flashLine[17] & 0xFF, flashLine[18] & 0xFF, flashLine[19] & 0xFF,
	                flashLine[20] & 0xFF, flashLine[21] & 0xFF, flashLine[22] & 0xFF, flashLine[23] & 0xFF,
	                flashLine[24] & 0xFF, flashLine[25] & 0xFF, flashLine[26] & 0xFF, flashLine[27] & 0xFF,
	                flashLine[28] & 0xFF, flashLine[29] & 0xFF, flashLine[30] & 0xFF, flashLine[31] & 0xFF, crc) >= 0)
	            {
	                count ++;
	                // Length reduction in the case when the line is too long to pass by communication channel
	                buf[96] = 0;
	                // Printing without line numbers
	                pcl->answer(buf, true, false);
	            }

	            if (count >= 180000)
	                break;
	        }
	        
	        res = FLASH_PROG_OK;
		}
    }
	else
	{
		res = FLASH_PROG_UCOMMAND_ERROR;
	}

    return res;
}

/** \name Method handles the 'smon flash prepare ...' command
*/
FLASH_PROG_RESULT FlashProgrammer::cmdPrepare(int addr, int len, int crc, bool fast)
{
	_address = static_cast<unsigned int>(addr);
	_length = static_cast<unsigned int>(len);

	// Parameters validation
	if ((_length == 0) ||(_length > FLASH_MAX_LEN))
	{
		// Incorrect data length
		_prepared = false;
		return FLASH_PROG_LEN_ERROR;
	}
	if ((_address % FLASH_BLOCK_SIZE) != 0)
	{
		// Incorrect addres (it is assumed that address points to the beginning of a block)
		_prepared = false;
		return FLASH_PROG_ADDR_ERROR;
	}

	_cur_address = _address;
	_cur_length = 0;
	_cmd_address = _address;
	_cmd_length = 0;
	_prepared = true;
	_fastMode = fast;
	_crc = static_cast<INT16U>(crc);

	// Setting for the entire buffer a 0xFF value
	memset(_sdram, 0xFF, FLASH_MAX_LEN);
	return FLASH_PROG_OK;
}

/** \name Method converts two successive characters from hex to bytes
*/
bool FlashProgrammer::hh2byte(char* text, unsigned char& val)
{
	char b[3];
    char* ptr=0;
    
	b[0] = text[0];
	b[1] = text[1];
	b[2] = '\0';
	val = static_cast<unsigned char>(strtol(b, &ptr, 16));
	return (*ptr == '\0');
}

/** \name Method handles the 'smon flash S' command
*/
FLASH_PROG_RESULT FlashProgrammer::cmdSrecord(char *srec)
{
	// exemplary MCS File Format:
	//:10EFC0000099FD7F010000002899FD7F086DFF7FFB

	unsigned char data[MAX_SREC_DATA];  // decoded srec (without the first 3 characters)
	unsigned char len;
	unsigned int stringlen, addr;
	unsigned char crc = 0;
	unsigned int sum_crc = 0;
	unsigned char crc_original;

	// Check if the 'prepare' command was executed
	if (!_prepared)
	{
        return FLASH_PROG_NOT_PREP_ERROR;
	}
	_cmd_length++;
	// s-record length validation
	stringlen = strlen(srec);

	char* p = &srec[1];
	if ((stringlen < MIN_SREC_LEN) || (((stringlen-1) % 2) != 0) ||
		(!hh2byte(p, len)) || (len != ((stringlen-1) / 2 - 5)))
	{
		// If too short or
		// if incorrect number of characters or
		// if there was a length conversion error or
		// if the length corresponding to the content of 'srec' does not match the length of the text
		return FLASH_PROG_BADPAR_ERROR;
	}

	// Decoding with CRC sum checking
	int i;
	unsigned char bb;
	sum_crc = len;  // CRC sum is calculated together with the length
	for (i = 0; i<(len+3); i++)
	{
		p = &srec[2*i+3];
		if (!hh2byte(p, bb))
		{
			return FLASH_PROG_BADPAR_ERROR;  // Convertion error (characters other then: 0..9A..F)
		}
		data[i] = bb;
		sum_crc += bb;
	}
	sum_crc %=256;
	char ch = static_cast<char>(sum_crc);
	crc = static_cast<unsigned char>(~ch+1);

	//
	p = &srec[2*(len+3) + 3];
	hh2byte(p, crc_original);
	if(crc != crc_original)
	{
		return FLASH_PROG_BADPAR_ERROR;
	}
	// Record type checking
	p = &srec[7];
	unsigned char recType = 0xFF;
	hh2byte(p, recType);
	if (recType == 0x00)		// Data block
	{
		// Calculation of an address
		addr = (static_cast<unsigned int>(data[0]) <<  8) +
			   (static_cast<unsigned int>(data[1]));
		addr += static_cast<unsigned int>(MCS_EXTENDED_LINEAR_ADDR)*_cur_page;


		// Copying data to the SDRAM memory
		if (addr == _cur_address)  // If the address is consistent with the expected
		{
			// All correct, copying of a s-record data block to the sdram and updating the data
			memcpy(reinterpret_cast<char*>(&_sdram[_cur_address - _address]), reinterpret_cast<char*>(&data[3]), len);
			_cmd_address += len;
			_cur_address += len;
			if(_cur_page == 1)
				return FLASH_PROG_OK;
			return FLASH_PROG_OK;
		}
		else if(addr > _cur_address)
		{
			// All correct, copying of a s-record data block to the sdram and updating the data
			_cur_address = addr;
			memcpy(reinterpret_cast<char*>(&_sdram[_cur_address - _address]), reinterpret_cast<char*>(&data[3]), len);
			_cmd_address += len;
			_cur_address += len;
			return FLASH_PROG_OK;
		}
	}
	else if (recType == 0x04) // record of type 1 (header) is ignored
	{
		// Calculation of an address
		_cur_page = (static_cast<unsigned int>(data[3]) <<  8) +
					(static_cast<unsigned int>(data[4]));
		return FLASH_PROG_OK;
	}
	else if (recType == 0x01) // record of type 1 (header) is ignored
	{
		return FLASH_PROG_OK;
	}

	// In other case - error
	return FLASH_PROG_BADPAR_ERROR;
}

/** \name Method handles the 'smon flash program' command
*/
FLASH_PROG_RESULT FlashProgrammer::cmdProgram(ClassifiedLine* pcl)
{
    if (!_prepared)
	{
		return FLASH_PROG_NOT_PREP_ERROR;
	}
	if (_cmd_length != _length)
	{
		return FLASH_PROG_LEN_ERROR;
	}

	// Calculation of a number of required blocks to write
	int nBlocks = _cur_address/FLASH_PAGE_LEN;
	
	// Programming all blocks
    bool res = false;
	int bytes  = _cur_address%FLASH_PAGE_LEN;;
    char buf[LINESIZE];
    
	res = _flashMem->eraseFlash(_address, _cur_address);
	if (!res) {
		return FLASH_PROG_WRITE_ERROR;
	}
    for(int page = 0; page <= nBlocks; page++)
    {
    	if(page <= nBlocks - 1)
    	{
    		res = _flashMem->writeFlash(&_sdram[page * FLASH_PAGE_LEN] , page * FLASH_PAGE_LEN, FLASH_PAGE_LEN);
    		if (!res) {
    			return FLASH_PROG_WRITE_ERROR;
    		}
    	}
    	else if(page == nBlocks)
    	{
    		res = _flashMem->writeFlash(&_sdram[page * FLASH_PAGE_LEN] , page * FLASH_PAGE_LEN, bytes);
    		if (!res) {
    			return FLASH_PROG_WRITE_ERROR;
    		}
    	}
    	if((page%256) == 0)
    	{
    		if (SNPRINTF (buf, LINESIZE, "%sflash programing %d%%", _prefix, (page+1) * 100 / nBlocks) < 0)
    			return FLASH_PROG_INTERNAL_ERROR;
    		pcl->answer(buf, false, false);
    	}
    }

    _prepared = false;
	return FLASH_PROG_OK; 
}

/** \name Method handles the 'smon flash get address' commad
*/
FLASH_PROG_RESULT FlashProgrammer::cmdGetAddress(ClassifiedLine* pcl)
{
    char buf[LINESIZE];
    
	if (!_prepared) {
		return FLASH_PROG_NOT_PREP_ERROR;
	}
	
	if (SNPRINTF (buf, sizeof(buf), "%sflash address = %u", _prefix, _cur_address) < 0) 
    { 
        return FLASH_PROG_INTERNAL_ERROR;
	}
    
    pcl->answer(buf, false, false); 
    return FLASH_PROG_OK;
}

/** \name Method sets a prefix
*/
void FlashProgrammer::setPrefix(const char * prefix)
{
    _prefix = prefix;
}


#endif  // PILOT_TARGET
