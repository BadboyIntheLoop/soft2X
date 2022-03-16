/**
*                                                                   
* @class FParser			                                                 
*                                                                   
* MP (c) Flytronic 2008                                             
*                                                                   
* @brief Class used to parse textual commands,                             
* takes space separated string as an input, allows for accessing    
* separate tokens via indexes                                       
*/

#include <PilotIncludes.h>  

FParser::FParser(void) : _lastToken(-1),_currToken(0)
{
    _inputStr[0] = 0;
    _tokens[0] = NULL;
};


/**
* Returns token at index
*/
char* FParser::getToken(int index) const
{
    if (index <= _lastToken)
	    return _tokens[index];
    return "";
}

/**
* Returns token as integer or -1 if token is not a number
*/
int FParser::getTokenAsInt(int index)
{
	int val = -1;
	if(index <= _lastToken && TypeParser::toInt(_tokens[index], val))
		return val;
	else
		return -1;
}


/**
* Return number of tokens or 0 if container is empty.
*/
int FParser::count() const
{
	if(_lastToken >= 0)	
		return _lastToken+1;
	else
		return 0;
}



void FParser::reset(void)
{
	_lastToken = -1;		// -1 means empty0 is the first element of the array
	_currToken = 0;
}


/**
* Feed parser with pointer to text line, setting ntok limits number of processed tokens
*/
bool FParser::loadLine(const char* str, int ntok, const char* delimiters)
{
	int strLen = static_cast<int>(strlen(str));	
	if( strLen > PARSER_LINE_LEN - 1)
		return false;

	int		ii = 0;
	char	prevChar = 1;					// any sign except space (0x20)

	_currToken = -1;

	// generates pointers to the next tokens in array of pointers _tokens
	// blank signs stay unchainged
    for(int i = 0; i < strLen; i++)
    {
        if (strchr (delimiters, str[i]) != NULL)
        {
            if(prevChar != 0)	// 
            {
                if(_currToken >= ntok - 1) break;
                _inputStr[ii] = 0;
                prevChar = 0;
                ii++;
            }
        }
        else
		{
			// this is the new token
			if(prevChar == 0 || i == 0)
			{
				if(_currToken < PARSER_MAX_TOKENS)
				{
				_currToken++;
				_tokens[_currToken] = &_inputStr[ii];
				}
                else
				{
					_tokens[_currToken+1] = NULL;						// indicate end of the array
					_lastToken = _currToken;							// set last token
					return false;						// not all tokens were loaded
				}
			}
			
			_inputStr[ii] = str[i];
			ii++;
			prevChar = str[i];
		}
	}
	_tokens[_currToken+1] = NULL;						// indicate end of the array
	_inputStr[ii] = 0;									// add ending zero
	_lastToken = _currToken;							// set last token
	
	return true;
}


/**
* Returns next index of token in container
*/
int FParser::getTokenPosition(const char *token) const
{
	size_t tLen = strlen(token);
	for(int i = 0; i <= _lastToken; i++)
	{
		if(STRNICMP(_tokens[i], token, tLen) == 0)
			return i;
	}
	return -1;
}

/**
* Returns index of specified token in string or -1 when it had not been found
*/
int FParser::getTokenIndex(const char *token, const char *str)
{
	size_t tLen = strlen(token);
	size_t strLen = strlen(str);
	for( size_t i = 0u; i < strLen - tLen; i++)
	{
		if(STRNICMP(str+i, token, tLen) == 0)
			return static_cast<int>(i);
	}
	return -1;	// token not found
	
}

/**
* Returns index of specified token in string stored in class instance.
*/
int	FParser::getTokenIndex(int token) const
{
	if(token >= PARSER_MAX_TOKENS || token > _lastToken || token < 0)
		return -1;

	// _tokens is an array of addresses under which are tokens in array of signs _inputStr
	// position of token is calculated by difference the adrresses
	return _tokens[token] - _tokens[0];
}



//EOF
