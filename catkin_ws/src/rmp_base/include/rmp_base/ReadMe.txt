This is intended to serve as a basic example on how to interface with the
RMP platform via ethernet.

The files to modify for your own application would be:


EXAMPLE_MAIN.py: the main application that shows how to start the RMP thread, communicate with it
		 and handle the events

user_event_handlers.py: defines what to do with the signals from the RMP communication thread

rmp_config_params.py: define the user configurable parameters that are loaded and verified at start

The example does the following:
1. Creates an RMP thread
2. Initiates ethernet UDP communication	with the platform
3. Checks configuration against user configuration in rmp_config_params.py
4. Loads and mismatched parameters and verifies them
5. Puts the platform in tractor mode
6. Cycles through all the audio commands while commanding zero motion in between
7. Puts the platform in standby mode
8. Collects all feedback while running, logs it, and prints just the inertial data
8. Kills the thread and the main loop	
	
