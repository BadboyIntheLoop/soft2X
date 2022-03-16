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
#ifndef FPARSER_H
#define FPARSER_H

class FParser
{
    
public:
    FParser(void);
    bool	loadLine(const char* str, int ntok = 100, const char* delimiters = " \t");
    void	reset(void);
    char*	getToken(int index) const;  ///< returns pointer to the token pointed by index (starting from 0)
    int		getTokenAsInt(int index);   ///< returns token converted to integer value
    int		count(void) const;          ///< returns number of tokens in the current instance of FParser

    int				getTokenPosition(const char *token) const;          ///< returns tokens index in array of tokens
    static int		getTokenIndex(const char *token, const char *str);  ///< returns tokens index in input string
    int				getTokenIndex(int token) const;                     ///< returns tokens index of specified number in string

private:
    // parsers statics
    static const int PARSER_LINE_LEN = LINESIZE;
    static const int PARSER_MAX_TOKENS = 100;

    char	_inputStr[PARSER_LINE_LEN];
    char*	_tokens[PARSER_MAX_TOKENS];	
    int		_lastToken;
    int		_currToken;
};


#endif //FPARSER_H
