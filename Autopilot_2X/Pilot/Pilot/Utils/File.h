/*
 * File.h
 *
 *  Created on: Feb 25, 2020
 *      Author: truongnt2
 */

#ifndef PILOT_UTILS_FILE_H_
#define PILOT_UTILS_FILE_H_


#include <string>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>

#if (PILOT_TARGET == PT_HARDWARE)

namespace truongnt {
/**
 * @brief A logical representation of a file.
 */
class File {
public:
	File(){
		inputFile = NULL;
	}
	File(std::string name, uint8_t type = 0);

	std::string getContent(bool base64Encode = false);
	std::string getContent(uint32_t offset, uint32_t size);
	std::string getName();
	std::string getPath();
	uint8_t     getType();
	bool        isDirectory();
	uint32_t    length();
	int 		readAsStream(uint8_t* buf, int length);
	int 	readAsStream();
	bool openAsStream(char * name);
	void close();

private:
	std::string m_path;
	uint8_t     m_type;
	int 		m_current_pos;
	std::ifstream* inputFile;
};

} /* namespace truongnt */


#endif
#endif /* PILOT_UTILS_FILE_H_ */
