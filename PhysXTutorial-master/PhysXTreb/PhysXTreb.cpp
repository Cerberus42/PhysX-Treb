#include <iostream>
#include "VisualDebugger.h"

using namespace std;
//http://mmmovania.blogspot.com/2011/ NEED TO LOOK AT

int main()
{
	try 
	{ 
		VisualDebugger::Init("PhysXTreb", 600, 600); 
	}
	catch (Exception exc) 
	{ 
		cerr << exc.what() << endl;
		return 0; 
	}

	VisualDebugger::Start();

	return 0;
}