# Testing
### Power Switch

#### Switching Capacitive Load
The following is a breadboard test with an MIC2544 load switch, with a 1k&Omega; limit resistor (0.23A) switching 5v onto a discharged 680uF capacitor. The power supply which drives this circuit was set for 5v and 0.5A current limit (in order to prevent external limiting). In this capture we see the load switch limiting the current to the capacitor, increasing the rise time (to 12ms) and preventing problematic current draws onto the data logger. 
![MIC2544 Capacitor Switch](TEK00100.PNG)
* Red -> `EN`
* Yellow -> `FLG`
* Blue -> I<sub>In</sub> (10V/A)
* Green -> V<sub>Out</sub>

The following shows the same event, but a detailed view of the turn on event to illustrate the timing and current peak in more detail

![MIC2544 Capacitor Switch](TEK00102.PNG)
* Red -> `EN`
* Yellow -> `FLG`
* Blue -> I<sub>In</sub> (10V/A)
* Green -> V<sub>Out</sub>

##### v0.1 Update
Testing for v0.1 hardware (switched from MOSFET control to MIC2544 load switch with current limiting)

Inrush current with Margay v2.2 output rail used as power supply. R<sub>Lim</sub> = 2.2k&Omega; 
Note, initial inrush spike, then reduced current. Initial spike does not result in brown out of Margay. LiDAR Lite static current consumption is able to be supported by the Margay v2.2 external 3.3v rail.
![v0.1 Power On, Margay Supply](TEK00109.PNG)
* Yellow -> I<sub>In</sub> (10V/A)
* Blue -> V<sub>In</sub> 

This figure shows the same waveform as above, but a detailed view of the switching 
![v0.1 Power On Detail, Margay Supply](TEK00110.PNG)
* Yellow -> I<sub>In</sub> (10V/A)
* Blue -> V<sub>In</sub> 

This figure shows the risetime of the 5v switched rail on board the Symbiont board. In this case we see the rise time which is controlled by the limiting of the current from the MIC2544 switch (R<sub>Lim</sub> = 2.2k&Omega;)
![v0.1 5v SW Rise Time, Margay Supply](TEK00111.PNG)
* Red -> V<sub>5v_SW</sub>
* Blue -> V<sub>In</sub> 

The following shows the problem if the current limit is set too high (R<sub>Lim</sub> = 1.05k&Omega;), the core 5v rail sags (due to hitting the current limit of the boost converter), which causes the brown out detect to engage and lock the device into a perpetual reset state. After the reset is triggered due to the sag of the core voltage below 4.3v (brownout limit), we see oscillation occur and the device held in reset.
![Excess current limit, Power Supply](TEK00112.PNG)
* Red -> V<sub>5v_Core</sub>
* Yellow -> I<sub>In</sub> 