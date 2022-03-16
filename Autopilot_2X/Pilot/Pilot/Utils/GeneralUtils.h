/*
 * GeneralUtils.h
 *
 *  Created on: Feb 25, 2020
 *      Author: truongnt2
 */

#ifndef PILOT_UTILS_GENERALUTILS_H_
#define PILOT_UTILS_GENERALUTILS_H_


#include <stdint.h>
#include <string>
#include <algorithm>
#include <vector>
namespace truongnt {

class GeneralUtils {
public:
	static bool        base64Decode(const std::string& in, std::string* out);
	static bool        base64Encode(const std::string& in, std::string* out);
	static bool        endsWith(std::string str, char c);
	static void        hexDump(const uint8_t* pData, uint32_t length);
	static std::string ipToString(uint8_t* ip);
	static std::vector<std::string> split(std::string source, char delimiter);
	static std::string toLower(std::string& value);
	static std::string trim(const std::string& str);
	static std::string trim(const std::string& str, char x);
	static std::string replaceString(std::string subject, const std::string& search,
	                          const std::string& replace);
	void replaceStringInPlace(std::string& subject, const std::string& search,
	                          const std::string& replace);
};

} /* namespace truongnt */



#endif /* PILOT_UTILS_GENERALUTILS_H_ */
