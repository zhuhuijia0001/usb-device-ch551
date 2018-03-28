#include "Type.h"
#include "Task.h"

int main()
{
	InitSystem();

    while(TRUE)
    {					
		ProcessUartData();

		ProcessKeyboardLed();

		FeedWdt();
    }

    return 0;
}

