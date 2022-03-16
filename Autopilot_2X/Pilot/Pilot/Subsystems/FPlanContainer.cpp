/**                                                              
* @class FPlanContainer                                                    
*                                                                   
* @brief Class provides container for storing flight plan and methods for it handling.
* oraz metody do jego oblslugi                                      
*                                                                  
* Marcin Pczycki (c) Flytronic 2008                                
**/

#include <PilotIncludes.h>


//------------------------------------------------------------------------------------------------------------------------------
FPlanContainer::FPlanContainer(void) :
_invalidPlan(false)
{
	// local instance of object status memory and external memory for fplan purposes.
	_extMem = StorageFactory::createFPlanMemory();
	if(_extMem == NULL)
	{
		Log.abort ("Critical Error: FPlanContainer_0");
		return;
	}

	// create semaphor controlling access to the FlightPlanData object.
	if (!_fpdSem.create("FPlanContainer"))
	{
		Log.abort ("Critical Error: FPlanContainer_1");
		return;
	}

	//	pointer to the position in the flight plan.
	_plan.nextIndex = 0;			
	_plan.currentIndex = ITEM_NOT_SET;	
	_fpd.currentStatus = FPlanConf::UNDEFINED;

    _err = FPlanConf::FPErrNoError;
}


/**
* Returns flight pland data  object.
*/
bool FPlanContainer::getFPlanData(FPlanData &fpd, FPlanConf::FPStatus currentStatus)
{
	// Container do not use status field value if in the target object status has to be different it could be changed after copied. (fpd.currentStatus = ???)

	if(_invalidPlan)
		return false;

	// get exclusive access
	if(!_fpdSem.lock()) return false;
			
    fpd = _fpd;     // copy data from local object.
    fpd.currentStatus = currentStatus;	
    
	// release semaphor.
	return _fpdSem.unlock();
}

/**
* Return the position of the element with the given id or ITEM_NOT_FOUND if element dosnt exist in the list.
*/
int FPlanContainer::getItemIndex(int id) const
{
	if(id == ITEM_NOT_SET)
	{
		return ITEM_NOT_FOUND;
	}
	for(int i = 0; i <= _plan.nextIndex - 1; i++)
	{
		if( _plan.items[i].id == id )
			return i;
	}
	return ITEM_NOT_FOUND;	// -1
}

/**
* Return element ID with the given index.
*/
int FPlanContainer::getItemID(int ind) const
{
	if((ind >= _plan.nextIndex - 1) || (ind < 0) || ind >= FPlanConf::FPLAN_LIST_SIZE)
		return ITEM_NOT_FOUND;

	return _plan.items[ind].id;
}

/**
* Get current flight plan item.
*/
int FPlanContainer::getCurrentItem() const
{
	if((_plan.currentIndex > ITEM_NOT_SET) && (_plan.nextIndex > 0))
		return _plan.items[_plan.currentIndex].id;
	else
		return ITEM_NOT_SET;
}

/**
* Returns the number of lines in the flight plan. 
*/
int FPlanContainer::itemCount() const
{
	if(_plan.nextIndex > 0)
		return _plan.nextIndex;
	else
		return 0;
}


/**
* Moves flight plan to the next element. Sets pointer of the executed element to the next element.
*/
bool FPlanContainer::moveToNextItem()									
{
	bool result = false;
	if(_plan.currentIndex < _plan.nextIndex - 1)
	{
		_plan.currentIndex++;
		_err = FPlanConf::FPErrNoError;
		result = true;
	}
	if(result)
	{
		// return error when Flightplan could not be updated.
		if(!updateFPD())
		{
			Log.errorPrint("Error: FPlanContariner_moveToNextItem_1");
			return false;
		}
	}
	return result;
}

/**
* Sets flight plan at the first element.
*/
bool FPlanContainer::setAtFirstItem()
{
	if(_plan.nextIndex < 1)
	{
		_err = FPlanConf::FPErrPlanIsEmpty;
		return false;
	}
	else
	{
		_plan.currentIndex = 0;
		_err = FPlanConf::FPErrNoError;
		return updateFPD();
	}
}

/**
* Set flight plan on the element with given ID
*/
bool FPlanContainer::setCurrentItem(int id)
{
	int position = getItemIndex(id);
	if(position == ITEM_NOT_FOUND)
	{
		_err = FPlanConf::FPErrIdDoesntExist;
		return false;
	} 

	_plan.currentIndex = position;
	_err = FPlanConf::FPErrNoError;
	return updateFPD();
}

/**
* Insert flight plan command after the pointed line by the afterID parameter or at the beginnig when afterId = 0
* If the command is being inserted after current line pointer would be moved at the new inserted line.
* In any case FP_CHANGED message will be send.
*/
bool FPlanContainer::insert(int newId, int afterId, const char* item)			
{
	// check correctness of the newId (if is in range or if is it not used jet)
	// check if afterId is in the list
	// check if there is a free space in the list for the new command and if the command is not to long.
	// if new line has been inserted inside current flight plan, rest of the lines would be moved.
	// set planModified flag, update FPD and send notificatin.


	// id has to be from range 1..MAX
	if((newId > FPlanConf::FPLAN_MAX_ID_VALUE) || (newId <= 0) || (afterId < 0))
	{
		_err = FPlanConf::FPErrWrongIdValue;	
		return false;
	}

	int len = strlen(item);
	int position = 0;

	if((_plan.nextIndex >= FPlanConf::FPLAN_LIST_SIZE) || (len >= FPlanConf::FPLAN_LINE_LENGTH))
	{
		_err = FPlanConf::FPErrLineListLimitExceeded;
		return false;
	}

	if( getItemIndex(newId) != ITEM_NOT_FOUND )
	{
		_err = FPlanConf::FPErrIdExists;
		return false;
	}
	if ( afterId == 0 )
	{
		position = 0;
	}
	else
	{
		position = getItemIndex(afterId);
		if(position == ITEM_NOT_FOUND)
		{
			_err = FPlanConf::FPErrIdDoesntExist;
			return false;
		}

		// inserting after 'afterId' excluding the case when 'afterId = 0' and should be inserted at the beggining.
		else if(_plan.nextIndex > 0) position++;
	}

	// If inserting at the end of the array, loop moving items will not be executed.
	for(int i = _plan.nextIndex-1; i > position-1; i--)
	{
		_plan.items[i+1] = _plan.items[i];
	}

	//copy command and id
	MEMCCPY(_plan.items[position].command, item, 0, len+1);
	_plan.items[position].id = newId;
	_plan.nextIndex++; 

	
	// When item was inserted before current position, current value should be increased to point all the time the same element.
	if(position < _plan.currentIndex)
	{
		_plan.currentIndex++;
	}
	// update fpd
	_err = FPlanConf::FPErrNoError;
	return updateFPD();
}

/**
* Modify current element of the flight plan.
*/
bool FPlanContainer::edit(int id, const char* item)
{
	// calculate and find index of the element of given id
	// check if the new command is not to long.
	// copy command
	int position = getItemIndex(id);
	
	if(position == ITEM_NOT_FOUND)
	{
		_err = FPlanConf::FPErrIdDoesntExist;
		return false;
	}

    if(strlen(item) >= static_cast<unsigned int>(FPlanConf::FPLAN_LINE_LENGTH))
	{
		_err = FPlanConf::FPErrLineListLimitExceeded;
		return false;
	}
	
	MEMCCPY(_plan.items[position].command, item, 0, FPlanConf::FPLAN_LINE_LENGTH);	// copy with ending zero

	_err = FPlanConf::FPErrNoError;
	return updateFPD();
}

/**
* Removes pointed flight plan element and deefragment the array.
* If current line has been removed, pointer would be moved to the next line. 
*/
bool FPlanContainer::remove(int itemId)
{

	int position = getItemIndex(itemId);
	if(position == -1)
	{
		// Error, choosen id does not exists.
		_err = FPlanConf::FPErrIdDoesntExist;	
		return false;
	}
	else if((position == _plan.currentIndex) && (_plan.currentIndex == _plan.nextIndex - 1)		// remove last (that is current) flight plan element.
				|| (_plan.nextIndex == 1))														// or the last one.
	{
		
		_plan.currentIndex = ITEM_NOT_SET;	// position to be executed is not been set.
		_plan.nextIndex--;
	}
	else if(position != _plan.nextIndex && _plan.nextIndex != 1)
	{
		for(int i = position; i < _plan.nextIndex; i++)	// move rest of the elements of the array to be together.
		{
			_plan.items[i] = _plan.items[i+1];
		}
		 
		if(position < _plan.currentIndex)
		{
			// decrease currentIndex that is pointing whole time the same flight plan element.
			_plan.currentIndex--;
		}

		_plan.nextIndex--;
	}


	return updateFPD();
}

/**
* Remove (without phisical overwrite) alle flight plan elements.
*/
bool FPlanContainer::removeAll()
{
	_plan.nextIndex = 0;
	_plan.currentIndex = ITEM_NOT_SET;

	return updateFPD();
}

/**
* Send to th ecommunication chanel list of the whole flight plan elements.
*/
bool FPlanContainer::printItems(ClassifiedLine& cl)
{
	char buffer[LINESIZE];
	int n = itemCount(); 

	// send all commands in the same order as they are saved in the flight plan.
	for(int i = 0; i < n; i++)
	{	
		// check correctness of line to be send.
		if(_plan.items[i].id > FPlanConf::FPLAN_MAX_ID_VALUE)
		{
			// error would be saved in the log and send to the communication chanel.
            _err = FPlanConf::FPErrWrongIdValue;
            cl.answer(FPlanConf::getErrorMsg(_err));
			Log.errorPrint(FPlanConf::getErrorMsg(_err));
			return false;
		}
		SNPRINTF(buffer, sizeof(buffer), "fp: %d %d %s", i, _plan.items[i].id, _plan.items[i].command);
		
        bool sent = cl.answer(buffer, false, false);
		if(!sent) return false;
	}
	return true;
}

/**
* Send status to the communication chanel.
*/
void FPlanContainer::printStatus(const char* currStatus, ClassifiedLine& cl)
{
	// Flight plan line lenght is not the same like lenght of the communication message line so LINESIZE is used always. (with the 0 a the end)
	char buffer[LINESIZE];
	

    if(!_fpdSem.lock())
	{
		_err = FPlanConf::FPErrCantAccessFPlanData;
		return;
	}

	if(_plan.currentIndex >= 0)
	{
        SNPRINTF(buffer, sizeof(buffer), "fp: %s %d %s", currStatus, _plan.items[_plan.currentIndex].id, _plan.items[_plan.currentIndex].command);
	}
	else
	{
		SNPRINTF(buffer, sizeof(buffer), "fp: %s", currStatus);
	}

	_fpdSem.unlock();	
    cl.answer(buffer);
}

/**
* Load flight plan from memory.
*/
bool FPlanContainer::load(const char* name)
{
    bool result =  _extMem->open(name);

    if(!result)
    {
        _err = FPlanConf::FPErrFailedToLoadFPlan;
        return false;
    }

    result = _extMem->load(&_plan, sizeof(_plan));
    _extMem->close();

    if(!result)
    {
        // if loading flight plan fails memory has to be clened.

        _invalidPlan = true;

        // clean flight plan.
        char emptyBuff[FPlanConf::FPLAN_LINE_LENGTH] = {0};
        for(int i = 0; i < FPlanConf::FPLAN_LIST_SIZE; i++)
        {
            MEMCCPY(_plan.items[i].command, emptyBuff, 1, FPlanConf::FPLAN_LINE_LENGTH);
            _plan.items[i].id = 0;
        }
        _plan.nextIndex = ITEM_NOT_SET;
        _plan.currentIndex = ITEM_NOT_SET;
        updateFPD();

        _err = FPlanConf::FPErrFailedToLoadFPlan;
        return false;
    }
    else
        _invalidPlan = false;

    if(!_fpdSem.lock())
    {
        _err = FPlanConf::FPErrCantAccessFPlanData;
        return false;
    }	

    MEMCCPY(&_fpd.fplanName, name, 0, FPlanConf::FPLAN_NAME_LENGTH);

	// lock error is critical so if it occurs restart is needed.
    if(!_fpdSem.unlock())
        return false;

    return updateFPD();

}

/**
* Saves default flght plan to mass storage.
*/
bool FPlanContainer::save()
{
	return save(const_cast<char*>(FPlanConf::DEFAULT_PLAN_NAME));
}

/**
* Saves given flight plan.
*/
bool FPlanContainer::save(const char* name)
{
    if (!SystemNowOnGround)
    {
        Log.msgPrintf ("%sCannot save plan in flight", FPlanConf::SUBS_PREFIX);
        return false;
    }

	bool result = _extMem->open(name);
	if(result)
	{
		result = _extMem->save(&_plan, sizeof(_plan));
		_extMem->close();
	}		
	if(!result)
	{
		return false;
	}
	
	if(!_fpdSem.lock())
	{
		_err = FPlanConf::FPErrCantAccessFPlanData;
		return false;
	}
	MEMCCPY(&_fpd.fplanName, name, 0, FPlanConf::FPLAN_NAME_LENGTH);
	
	return _fpdSem.unlock();
}


/**
* Verify the given command.
*/
bool FPlanContainer::verifyCommand(const char* command)
{
	for(int i = 0; i < FPlanConf::FPLAN_LINE_LENGTH; i++)
	{
		if(command[i] == 0) return true;
	}
	return false;
}

/**
* Update Flight Plan Data
*/
bool FPlanContainer::updateFPD()
{
	if(!_fpdSem.lock())
	{
		_err = FPlanConf::FPErrCantAccessFPlanData;
		return false;	
	}
		
	// set id from the current index.
	if(_plan.currentIndex <= ITEM_NOT_SET)
		_fpd.currentItem = ITEM_NOT_SET;
	else
		_fpd.currentItem = _plan.items[_plan.currentIndex].id;
		
	// set command from the current index.
	if(_fpd.currentItem > 0)
		MEMCCPY(_fpd.command, _plan.items[_plan.currentIndex].command, 0, FPlanConf::FPLAN_LINE_LENGTH);
	else
		_fpd.command[0] = 0;
		
	return _fpdSem.unlock();
}

/**
* Return last error in the container.
*/
FPlanConf::FPError FPlanContainer::getLastError() const
{
	return _err;
}
