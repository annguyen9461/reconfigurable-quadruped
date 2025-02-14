#include "joystick.hpp"
#include <unordered_set>

int main()
{   
    int num_legs = 4;
    std::unordered_set<int> btn_set;

	const unsigned int maxJoysticks = 32;
	Joystick joysticks[maxJoysticks] = {0};

	char fileName[32];
	for (unsigned int i=0; i<maxJoysticks; ++i)
	{
		sprintf(fileName, "/dev/input/js%d", i);
		joysticks[i] = openJoystick(fileName);
	}

	while (1)
	{
		for (unsigned int i=0; i<maxJoysticks; ++i)
		{
			if (joysticks[i].connected)
			{
				readJoystickInput(&joysticks[i]);

				printf("Axes: ");
                int val_updown = joysticks[i].axisStates[RIGHT_JS_UP_DOWN];
                int val_left_right = joysticks[i].axisStates[RIGHT_JS_LEFT_RIGHT];

                printf("%d:% 6d ", RIGHT_JS_LEFT_RIGHT, val_left_right);
                printf("%d:% 6d ", RIGHT_JS_UP_DOWN, val_updown);
                
				printf("Button pressed: ");
				for (char buttonIndex=0; buttonIndex<joysticks[i].buttonCount; ++buttonIndex) {
					// if pressed
					if (joysticks[i].buttonStates[buttonIndex]) {   
                        printf("%d ", buttonIndex);
                        if (btn_set.find(buttonIndex) == btn_set.end()) { // Add the button if not present
                            btn_set.insert(buttonIndex);
                        } else {
                            btn_set.erase(buttonIndex);
                        }
                    }
				}
				printf("\n");

                printf("Button list: ");
                for (auto btn : btn_set) {
                    printf("%d, ",btn);
                }
                printf("\n");

                if (val_updown == 0 && val_left_right == 0 ) {
                }
                else {
                    if (abs(val_updown) > abs(val_left_right)) {
                        if (val_updown < 0) 
                        {
                            printf("UP\n");
                        } else if(val_updown > 0) 
                        {
                            printf("DOWN\n");
                        }
                    } else {
                        if (val_left_right < 0) {
                            printf("LEFT\n");
                        } 
                        else if (val_left_right > 0)
                        {
                            printf("RIGHT\n");
                        }
                    }
                }
			}
		}
		fflush(stdout);
		usleep(16000);
	}
}