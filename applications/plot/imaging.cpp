#include "imaging.h"



//////////////////////////////////////////////////////////////////
// 
// GLOBAL VARIABLES 
// 
////////////////////////////////////////////////////////////////// 


// Allows the system to decide if located in structure or in bulk
// True = bulk      False = structure
bool inside = true;


//Set function for inside bool
void setInside(bool state) {
	inside = state;
}
//Get function for inside bool
bool getInside() {
	return inside;
}