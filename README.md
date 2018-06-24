# Low-Frequency-Signal-Generator-and-Scalar-Network-Analyzer
The Aim of the project is to built a Signal Generator and a network analyser under 30$ using a TM4c123gxl board and couple of op-amps.
The entire project is divided into 12 steps.

Steps 1 to 4:
Includes extracting the frequency voltage and type of wave from the string inputted by the user. 
Also includes configuring the uart

Steps 5 to 8:
It includes actually getting the waves - DC, Sine, sqaure, triagular on the analyser.
For this, a pre-fed 1V, 1khz wave is stored in an array and then, according to the required frequency, the delphi variable - representing the index of the pre-fed array is increased in terms of the frequency. 
Also, DAC gives 0-2.5V output, hence, using op-amps, it is mapped from -5 to +5. Hence to get the desired voltage (from -5 to 5) the value fed into the DAV is calculated. E.G. for DAC input 0, the output of opamp is -5V. Also, some calibration is needed as the voltage won't be exactly -5V for 0 DAC value.

Steps 9 and 10:
Step 9,10 includes increasing frequency of a wave from f1 to f2 frequency in steps. This is done with the help of inbuilt ADC and sweep function. The sweep function starts with f1 frequency. If the frequency is less than 1kHz, it starts with a slow ADC. It keeps on increasing the frequency in the power of 2 i.e freq+2^i, where i =0,1,..N and N depends on f2 frequency. Once the frequency is past 1Khz, the ADC starts fast ADC. Hence, the ouput wave will sweep from frequency f1 to f2.
Slow ADC - voltage is converted every 3 sec Fast ADC - voltage is converted every 0.5 sec.

Step 11 and 12:
Step 11,12 measures voltage of input wave using ADC and error checking.
