Sleepy
==========

Arduino low power sleep utility, based on WTD (watchdog timer interrupt)


//Enable WDT
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

void loop(){
	...
	
	//sleep 10s
	sleepDelay(10)
	...
}
