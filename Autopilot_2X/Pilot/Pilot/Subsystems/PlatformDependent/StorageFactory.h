#ifndef STORAGEFACTORY_H
#define STORAGEFACTORY_H

/** \file
* \brief Declaration of an objects factory for hardware specific platforms
*/

/** Class is a factory of an StorageBase type objects. Class has static member functions to create objects
* of type StorageBase which are intended to handle different type of a memories for hardware specific platforms.
* Class is conditionally compiled for each hardware platform.
*/
/// Class implements an object factory for hardware specific platforms
class StorageFactory
{
public:
    static StorageBase* createConfigMemory (bool singleThread=false);	///< Method creates an object used to store subsytem's configuration parameters
    static StorageBase* createFPlanMemory (void);	///< Method creates an object used to store a flight plan
    static StorageBase* createLogMemory (void);		///< Method creates an object used to store a log

private:
	// Constructor is disabled
    StorageFactory(void);
	// Destructor is disabled
    ~StorageFactory(void);
};

#endif  // STORAGEFACTORY_H
