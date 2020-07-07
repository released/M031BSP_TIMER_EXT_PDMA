# M031BSP_TIMER_EXT_PDMA
 M031BSP_TIMER_EXT_PDMA

update @ 2020/07/07

1. use TM1_EXT (PA10) with PDMA , to capture external signal freq.

	- test signal : LED (PB14) and PWM0_CH0 (PB5)

2. increase TIMER 1 PSC value to 0xFF , to increase detect freq range under 0 (check PDMA_TimerCapture_Process) 

	- ex : if TIMER 1 PSC set 0 , lowest freq. is TIMER1 CLOCK 48000000/0xFFF FFF = 3

	- ex : if TIMER 1 PSC set 0xFF , lowest freq. is TIMER1 CLOCK (48000000/(0xFF + 1))/0xFFF FFF = 0.011

3. Connect LED (PB14) and PWM0_CH0 (PB5) to TM1_EXT (PA10) , to simulate external signal

	- Default PWM0_CH0 freq = 100 , press A or a , to increase PWM freq (step : 100)
	
	- press D or d , to decrease PWM freq (step : 1)
	
	- LED default toggle : 0.5 Hz , press 1 or 2 or 3 to change toggle speed
	
4. below is waveform capture	

![image](https://github.com/released/M031BSP_TIMER_EXT_PDMA/blob/master/EXT_PIN_waveform.jpg)
	
5. below is log capture		
	
![image](https://github.com/released/M031BSP_TIMER_EXT_PDMA/blob/master/EXT_PIN_log.jpg)