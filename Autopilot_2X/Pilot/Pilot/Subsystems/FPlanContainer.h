/**                                                              
* @class FPlanContainer                                                    
*                                                                   
* @brief Class provides container for storing flight plan and methods for it handling.
* oraz metody do jego oblslugi                                      
*                                                                  
* Marcin Pczycki (c) Flytronic 2008                                
**/

#ifndef FLIGHTPLANCONTAINER_H
#define FLIGHTPLANCONTAINER_H

class FPlanData;
class ClassifiedLine;

/** \file
* \brief Container of a flight plan data
*
* Class implements a container for a flight plan data, access and edition functionality of it.
*/
class FPlanContainer
{
private:
    // Destructor blocking. Object should not be destroy.
    ~FPlanContainer(void){};

	// statics
	static const int	ITEM_NOT_SET = -1;							///< CurrentItem not set
	static const int	ITEM_NOT_FOUND = -1;						///< id not found
	static const int	FPLAN_SEM_TIMEOUT = 10;						///< Timeout waiting for the semaphor.


	// single flight plan line.
	struct FPlanItem
	{
		int				id;											///< New line id.
		char			command[FPlanConf::FPLAN_LINE_LENGTH];		///< Flight plan command with the parameters.
	};

	// line collection, position of the next element (first free element), position of the current element.
	struct FPlanItems
	{
		FPlanItem		items[FPlanConf::FPLAN_LIST_SIZE];			///< Line list
		int				currentIndex;								///< Position of the current element.
		int				nextIndex;									///< Position of the next element (also elements count in the array)
	};

	// variables
	Semaphore			_fpdSem;									///< Semaphor controlling access to the external FPD object.
	FPlanData			_fpd;										///< Local copy of the object to be shared with other subsystems.
	bool				_invalidPlan;								///< Setting to true cause error generation in the getFPlanData() function.

	FPlanConf::FPError		_err;									///< Last error  ostatni blad w metodach klasy
	StorageBase*		_extMem;									///< Instance of the object handling writting to the external memory
	FPlanItems			_plan;										///< Store flight plan.


    
	// private methods.
	bool	verifyCommand(const char* cmd);

public:

	FPlanContainer(void);

    bool	getFPlanData(FPlanData &fpd, FPlanConf::FPStatus s);	///< Return FPD object filled with desired status.
    int	    getItemIndex(int id) const;								///< Return line position of given id or -1 when line does not exists.
    int	    getItemID(int ind) const;								///< Return elements id of given index or ITEM_NOT_SET in case of incorrect index.
    int	    getCurrentItem() const;									///< Get ID of the current element pobiera ID aktualnego elementu

	int		itemCount() const;									    ///< Return count of the lines in the flight plan.
	bool	moveToNextItem();										///< Set pointer of the element to be executed to the next element.
	bool	setAtFirstItem();										///< Set pointer of the element to be executed at the first element in the flight plan.
	bool	setCurrentItem(int id);									///< Set pointer of the element to be executed at the element of given id.

	bool	insert(int newId, int afterId, const char* item);		///< Insert new line after indicated position, rest of the lines would be moved.
	bool	edit(int id, const char* item);							///< Change line of given id text.
	bool	remove(int id);											///< Remove element of given id.
	bool	removeAll();											///< Remove all elements.


	bool	printItems(ClassifiedLine& cl);                         ///< Send line ist to the communication chanel.
	void	printStatus(const char* status,
						ClassifiedLine& cl);                        ///< Send status to the communication chanel, information of status is from the parameter, currently executed command is from the fpcontainer.

	bool	load(const char* name);									///< Load flight plan of given name.
	bool	save();													///< Save default flight plan to the mass storage.
	bool	save(const char* name);									///< Saves given flight plan.

	bool	updateFPD();


    FPlanConf::FPError getLastError() const;                        ///< accessor to the _err variable.

};

#endif //FLIGHTPLANCONTAINER_H

