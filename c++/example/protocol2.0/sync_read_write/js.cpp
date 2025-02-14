#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

#define RIGHT_JS_UP_DOWN 4
#define RIGHT_JS_LEFT_RIGHT 3

struct Joystick
{
	bool connected;
	char buttonCount;
	short* buttonStates;
	char axisCount;
	short* axisStates;
	char name[128];
	int file;
};

Joystick openJoystick(const char* fileName)
{
	Joystick j = {0};
	int file = open(fileName, O_RDONLY | O_NONBLOCK);
	if (file != -1)
	{
		ioctl(file, JSIOCGBUTTONS, &j.buttonCount);
		j.buttonStates = (short*)calloc(j.buttonCount, sizeof(short));
		ioctl(file, JSIOCGAXES, &j.axisCount);
		j.axisStates = (short*)calloc(j.axisCount, sizeof(short));
		ioctl(file, JSIOCGNAME(sizeof(j.name)), j.name);
		j.file = file;
		j.connected = true;
	}
	return j;
}

void readJoystickInput(Joystick* joystick)
{
	while (1)
	{
		js_event event;
		int bytesRead = read(joystick->file, &event, sizeof(event));
		if (bytesRead == 0 || bytesRead == -1) return;

		if (event.type == JS_EVENT_BUTTON && event.number < joystick->buttonCount) {
			joystick->buttonStates[event.number] = event.value;
		}
		if (event.type == JS_EVENT_AXIS && event.number < joystick->axisCount) {
			joystick->axisStates[event.number] = event.value;
		}
	}
}

int main()
{
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
                
				printf("Buttons: ");
				for (char buttonIndex=0; buttonIndex<joysticks[i].buttonCount; ++buttonIndex) {
					// if pressed
					if (joysticks[i].buttonStates[buttonIndex]) printf("%d ", buttonIndex);
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