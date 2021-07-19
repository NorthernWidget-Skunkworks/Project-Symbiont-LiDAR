[![DOI](https://zenodo.org/badge/196273476.svg)](https://zenodo.org/badge/latestdoi/196273476)

# Project-Symbiont-LiDAR
Design files for a support and interface unit for the Garmin LiDAR Lite

![Symbiosis](https://upload.wikimedia.org/wikipedia/commons/0/0d/Reindeer-Lichen_%286176257165%29.gif)

***The symbiont.*** *Lichens, like this Reindeer Lichen found in northeastern
Minnesota, are a symbiosis between fungi and algae. Our symbiosis is between
silicon, copper, aluminum, and laser. But be it nature or machine, some
partnerships are truly mutual in their benefits. Photo by Homer Edward Price.*

The [LiDAR Lite](https://buy.garmin.com/en-US/US/p/578152/pn/010-01722-10) is a rangefinder from Garmin capable of returning distances to objects up to 40 m away. It is an effective sensor for water, snow, and ice levels, among other purposes. Extensive work to characterize the LiDAR Lite for environmental sensing of water levels has been performed by:

Paul, J. D., Buytaert, W., & Sah, N.(2020). A technical evaluation of lidar-based measurement of riverwater levels. *Water Resources Research*, *56*, e2019WR026810. https://doi.org/10.1029/2019WR026810

## Technical specifications

### Electronic Hardware

* Microcontroller (computer) core
  * ATTiny1634
  * [Firmware](Firmware) written in Arduino-compatible C++
  * 12 MHz
* Sensors
  * Externally connected: [Garmin LiDAR Lite v3 HP](https://buy.garmin.com/en-US/US/p/578152/pn/010-01722-10)
  * On board
    * MEMS accelerometer to detect board angle
    * Hall-effect sensor to for user to trigger with a magnet after providing a known angle to the accelerometer. This corrects for offset errors in the accelerometer, significantly improving its angular accuracy.
* Connections and communications protocols
  * [LiDAR Lite v3 HP](https://buy.garmin.com/en-US/US/p/578152/pn/010-01722-10)
    * 6-pin screw-terminal header
    * Switchable 5V power supply
    * I2C
  * Data logger
    * Power in and Ground
    * Digital communications
      * I2C
* Power
  * Voltage limits: 3.3 ~ 5.5V
  * Power consumption: ~0.5mA @ 4.5V, take reading every 60 seconds, then power down
  * Power conditioning: Provides an onboard, high power, step up to 5V to allow for interface to 3.3V loggers
* Fault recovery: Using intermediate system to emulate an I2C connection prevents a global lockup of the logger system
* Status LED
* Open-source licensing via CC BY-SA 4.0

![Symbiont board](Documentation/images/SymbiontLiDAR_v010_top_annotated_20210408.png)

### Electronic Software and Firmware

* Programmable using the Arduino IDE https://www.arduino.cc/en/main/software
* [Firmware](Firmware) available in this repository. At the time of writing, "LiDAR_InterfaceDemo" is the most up-to-date prototype firmware.
* [Software ](https://github.com/NorthernWidget-Skunkworks/Symbiont-LiDAR_Library) to use the Symbiont-LiDAR with Arduino-compatible devices
* Open-source licensing via GNU GPL 3.0

## Assembly

Assembling this sensor is possible by hand with sufficient skill and the following tools:
* Temperature-controlled soldering iron
* Hot-air rework station
* Equipment for stenciling with solder paste
* ESD-safe tweezers and workstation
* Solder wick

Most of the components on this board are coarse enough in pitch that assembly by hand is expected to be relatively straightforward. However, if you are concerned about this, there are PCB assembly workshops located in many parts of the world.
>> THOUGHT: I have basic instructions for assembly on the [Project Walrus](https://github.com/NorthernWidget-Skunkworks/Project-TPDH-Walrus) README; I will not reproduce these here just yet, as we might want to set up a separate assembly guide, mostly with good links to existing resources, since our writing something of this nature would be to reinvent the wheel.

## Firmware

### Downloading and installing the Arduino IDE

Go to https://www.arduino.cc/en/main/software. Choose the proper IDE version for your computer. For Windows, we suggest the non-app version to have more control over Arduino; this might change in the future. You will have to add custom libraries, so the web version will not work (at least, as of the time of writing). Download and install the Arduino IDE. Open it to begin the next steps.

### AVR ISP

To install firmware on the Symbiont board, you use the [2x3-pin 6-pin ICSP (also called ISP) header](https://www.digikey.com/product-detail/en/3m/929665-09-03-I/3M156313-06-ND/681796) with a special device called an "in-circuit system programmer" (or just "in-system programmer; yup, that's what the acronym stands for).

Many devices exist to upload firmware, including:
* The official [AVR ISP mkII](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42093-AVR-ISP-mkII_UserGuide.pdf) (no longer produced but available used)
* Using an [Arduino as an ISP](https://www.arduino.cc/en/tutorial/arduinoISP)
* The versatile [Olimex AVR-ISP-MK2](https://www.olimex.com/Products/AVR/Programmers/AVR-ISP-MK2/open-source-hardware)
* The [Adafruit USBtinyISP](https://www.adafruit.com/product/46)

### Uploading the firmware

Using this ISP, upload (as of the time of writing): [the Arduino sketch in this folder](Firmware/AdditionalFirmware/LiDAR_Accelerometer_InterfaceDemo). To do so, follow these steps:

1. Open the Arduino IDE.
2. Follow [these instructions](https://github.com/SpenceKonde/ATTinyCore/blob/master/Installation.md) to install the [ATTinyCore board definitions](https://github.com/SpenceKonde/ATTinyCore)
3. Select [ATTiny1634](https://github.com/SpenceKonde/ATTinyCore/blob/master/avr/extras/ATtiny_1634.md) **(No bootloader)**
4. Plug your ISP of choice into your computer (via a USB cable) and onto the 6-pin header. There are two ways to place it on; the header is aligned such that the ribbon cable should be facing away from the board while programming. If this fails without being able to upload, try flipping the header around. This should both power the board and provide communications.
5. Go to Tools --> Programmer and select the appropriate programmer based on what you are using.
6. Go to Tools --> Burn Bootloader. Yes, we know that you just selected "ATTiny1634 (No bootloader)". But this step sets the fuses, which configure their internal oscillator and brown-out detection.
7. Go to Sketch --> Upload Using Programmer. After several seconds, you learn whether you succeeded or failed. Hopefully it worked!

![Upload using programmer](Documentation/images/UploadUsingProgrammer.png)

***Uploading using the in-system programmer.***

***Important note for Linux users:*** You must supply permissions to the Arduino IDE for it to be able to use the ICSP, or you will have to run it using `sudo`. The former option is better; the latter is easier in the moment.

***Note: be sure to download and/or update drivers for your ISP***


## Writing a program to connect to the Symbiont-LiDAR

Once it is bootloaded and connected with a LiDAR Lite sensor, you should be able to use any standard Arduino device to connect to it and read its data.

### Very simple Arduino code

This code is intended for any generic Arduino system.

```c++
// Include the Symbiont library
#include "SymbiontLiDAR.h"

// Declare variables -- just as strings
String header;
String data;

// Instantiate class
SymbiontLiDAR myLaser;

void setup(){
    // Begin Serial connection to computer at 38400 baud
    Serial.begin(38400);
    // Obtain the header just once
    header = myLaser.getHeader();
    // Print the header to the serial monitor
    Serial.println(header);
}

void loop(){
    // Take one reading every (10 + time to take reading) seconds
    // and print it to the screen
    myLaser.updateMeasurements();
    data = myLaser.getString();
    Serial.println(data);
    delay(10000); // Wait 10 seconds before the next reading, inefficiently
}
```

### Northern Widget Margay code

The [Margay data logger](github.com/NorthernWidget-Skunkworks/Project-Margay) is the lightweight and low-power open-source data-logging option from Northern Widget. It saves data to a local SD card and includes on-board status measurements and a low-drift real-time clock. We have written [a library to interface with the Margay](github.com/NorthernWidget-Skunkworks/Margay_Library), which can in turn be used to link the Margay with sensors.

```c++
// Include the Symbiont library
#include "Margay.h"
#include "SymbiontLiDAR.h"

// Declare variables -- just as strings
String header;
String data;

// Instantiate classes
SymbiontLiDAR myLaser;
Margay Logger; // Margay v2.2; UPDATE CODE TO INDICATE THIS

// Empty header to start; will include sensor labels and information
String Header = "";

// I2CVals for Symbiont
uint8_t I2CVals[] = {0x50}; // DEFAULT

//Number of seconds between readings
uint32_t updateRate = 60;

void setup(){
    Header = Header + myLaser.GetHeader();
    Logger.begin(I2CVals, sizeof(I2CVals), Header);
    initialize();
}

void loop(){
    Logger.run(update, updateRate);
}

String update() {
    initialize();
    return myLaser.getString();
}

void initialize(){
    myLaser.begin();
}
```

### Northern Widget Resnik code

>> Currently nearly identical to Margay code, by design. Add telemetry with Particle Boron.

```c++
// Include the Symbiont library
#include "Resnik.h"
#include "SymbiontLiDAR.h"

// Declare variables -- just as strings
String header;
String data;

// Instantiate classes
SymbiontLiDAR myLaser;
Resnik Logger;

// Empty header to start; will include sensor labels and information
String Header = "";

// I2CVals for Symbiont
uint8_t I2CVals[] = {0x50}; // DEFAULT

//Number of seconds between readings
uint32_t updateRate = 60;

void setup(){
    Header = Header + myLaser.GetHeader();
    Logger.begin(I2CVals, sizeof(I2CVals), Header);
    initialize();
}

void loop(){
    Logger.run(update, updateRate);
}

String update() {
    initialize();
    return myLaser.getString();
}

void initialize(){
    myLaser.begin();
}
```

## Housing and cabling

### Parts required

This is what we used for our build; you can be creative based on materials and availability.

* Main enclosure
  * Polycase box [WC-20F (clear lid)](https://www.polycase.com/wc-20f)
  * [2x \#4 screws](https://www.polycase.com/screws-mbr-100) to mount Symbiont in box
  * Cable gland ([Heyco M4365](https://www.heyco.com/Liquid_Tight_Cordgrips/product.cfm?product=Liquid-Tight-Cordgrips-Metric&section=Liquid_Tight_Cordgrips)) for cable to LiDAR Lite
  * Strain-relieved cable gland ([Heyco M4425](https://www.heyco.com/Liquid_Tight_Cordgrips/product.cfm?product=Liquid-Tight-Cordgrips-Pigtail-Metric&section=Liquid_Tight_Cordgrips)) for cable to logger
  * Desiccant packs
* LiDAR Rangefinder and attachment to enclosure
  * [LiDAR Lite sensor](https://www.sparkfun.com/products/14599)
  * [4 sealing screws](https://www.mcmaster.com/90825A144/): \#4-40 x 3/8"
  * [4 washers](https://www.mcmaster.com/90107A005) for \#4 screws
  * [4 cap nuts](https://www.mcmaster.com/99022A101) for \#4-40 screws
* Mounting plate
  * Material: Acetal (Delrin) sheet: 1/4" thick. We commmonly use [12x24" black](https://www.eplastics.com/ACTLBLK0-25012X24); the black pigment increases its UV resistance
  * Rectangular dimensions for mount: 127 x 95.25 x 6.35 mm (5.00" x 3.75" x 0.25")
  * [Design](https://easel.inventables.com/projects/VMmCoOyJyiiKTospk1NBBQ) on Easel for X-carve (see also [CNC files here](CNC)). Note the 7 mm depth to ensure that the bit cuts all the way through the Acetal; use a piece of scrap material on your cutting bed if you want to protect it.
* Mounting plate fasteners
  * 1/4"-20 hardware to attach the box to the mounting plate
    * Bolts: [1" long; hex head convenient; zinc-plated medium-grade (Grade 5) recommended for lower price with good corrosion resistance](https://www.mcmaster.com/92865A542)
    * Nuts: [Zinc-plated Grade 5](https://www.mcmaster.com/95462A029)
    * Washers: Zinc-plated [SAE](https://www.mcmaster.com/90126A029) or [USS](https://www.mcmaster.com/90108A413)
    * Lock washers: We typically use [split-lock washers](https://www.mcmaster.com/zinc-plated-steel-washers/), but [tooth-lock washers](https://www.mcmaster.com/91113A029) are good for high-vibration environments.
  * 2 U bolts: 1/4"-20, inner diameter 3/4" to 1 3/4", to attach the mounting plate to a pipe. We typically mount this sensor on 3/4" pipe or conduit, and recommend a U bolt designed to fit this ([1/4"-20, 1 1/8" inner diameter](https://www.mcmaster.com/u-bolts/u-bolts-with-mounting-plates/for-pipe-size~3-4/)). Weather/corrosion resistance is helpful, especially if you are concerned about later removing the U bolts.
* Cable to logger
  * 3 m (or less) [4-conductor AlphaWire](https://www.digikey.com/product-detail/en/alpha-wire/5004C-SL001/5004CSL001-ND/484976), stripped and tinned at both ends. Other cables will work too; this is what we have found to be highest quality and reliability. We cannot guarantee successful I<sup>2</sup>C communications over cables longer than 3 meters.

![Mounting plate perspective view in Easel](Documentation/images/EaselSymbiontMount.png)

***Mounting plate perspective view in [Easel](https://easel.inventables.com/projects/VMmCoOyJyiiKTospk1NBBQ)*** *for easy integration with the X-carve series of lower cost CNC routers.*

### Assembly

Prior to assembly, ensure that you have:
* [Uploaded the firmware to the Symbiont board](#Uploading-the-firmware)
* Fabricated [the mounting plate](CNC) (see also the [Easel online CNC setup](https://easel.inventables.com/projects/VMmCoOyJyiiKTospk1NBBQ)) if desired.

1. Drill and tap 12 mm holes in the side of the box. Use a M12-1.5 tap for the threads.

2. Install the Symbiont board as shown below using two of the \#4 self-tapping screws.

![Symbiont prototype in box](Documentation/images/PrototypeInBox_top_withScale_20200501.jpg)

3. Using the sealing screws, cap nuts, and washers, install the LiDAR Lite sensor onto the lid. By mounting the LiDAR Lite at an angle, you can fix it to the box lid in a way that still allows the box to open and close properly. The cap nuts go on the outside of the housing.

![Prototype sensor assembly](Documentation/images/PrototypeBoxAssembly_orig2019-09-05_20200501.jpg)

4. Thread the cables through the cable glands and attach them to the screw terminals. A 1.8 mm flat-head screwdriver can be very useful for this. Note that these cables cross the board in the above image (upper left) -- and that (unlike in the image!) you should do this *after* the board is in the box.

5. Using the switch in the box, select the desired communications protocol. For most of our uses, this is I2C.

6. Attach the cable to the data logger.

7. Test the LiDAR Lite unit.

8. When satisfied with the tests, turn off the data logger and place a protective cover over the lenses of the LiDAR Lite for safety during transport. We have [a set of "safety glasses" available for 3D printing](3Dprint). You may want to secure these in place with electrical tape.

9. Place desiccant in the box. I typically install two small desiccant packs on the side of the board above the switch and large capacitor and pack them in so they are unlikely to move; this is in order to keep the indicator LED visible.

10. Securely screw the lid onto the box to seal the LiDAR Lite + Symbiont unit.

11. Mark the corner of the box next to the Hall-effect sensor, and mark the orientations of the roll and pitch axes. The pitch axis should be positive when the long strain-relieved cable gland tilts upwards. The roll axis should be positive when, when looking at the side of the assembly such that the long strain-relieved cable gland is on the left and the laser rangefinder is pointed up, the laser rangefinder rolls towards you.

>> Add an annotated picture of roll, pitch, and Hall-Effect sensor locations

12. Place the box on a measured flat surface and tap the magnet to the marked location by the Hall Effect sensor. This will appropriately zero the offsets for the sensor and increase its near-horizontal accuracy significantly. This must be done when the sensor is powered; the LED will light up during the full duration of magnetic contact. When connected to a logger, this can be done by putting the magnet in place, hitting the "RESET" button (e.g., on a Margay logger), and then holding the magnet in place with the read light on until the first reading is complete. Doing this while connected to a computer is recommended in order to see the first reading on the serial monitor and double check that the zeroing/calibration is appropriate. For a convenient magnet holder, you can use our [3D-printable magnetic wand][3Dprint], which holds a small rare-Earth magnet. This may be a generic part, though this [3/8" x 1/8" Neodymium Disk Magnet](https://www.apexmagnets.com/magnets/3-8-x-1-8-disc-neodymium-magnet) works well in our experience.

13. Use the 1/4"-20 hardware to attach the LiDAR Lite box to the mounting plate. The bolts pass through the center holes on the tabs on either side of the box, with their heads towards the box lids.

>> Note: Mounting plate holes currently too small for these; will need to be updated. Currently using #8 hardware.

![Side view: long axis](Documentation/images/SideView_LongAxis.png)

![Side view: short axis, LiDAR Lite cable](Documentation/images/SideView_LiDARcableSide.png)

14. (Can wait for field installation) attach the unit via its mounting plate to the appropriate pipe, post, etc. This typically involves the U bolts, noted above. Curved EMT conduit can be helpful for providing a way to select the angle of the sensor. Although we use a single 45-degree bend piece of conduit in the images below, we might suggest attaching a 90-degree bend conduit first, and then a 45-degree bend as necessary to adjust the sensor away from a direct down-looking view; the sensor will attach directly to the convex side of the bend instead of bridging over airspace, which required us to use rocks as shims to reduce mounting-plate flexure. (This suggestion, however, increases torque on the mast; future field testing is needed to determine the best method.) ***Important note: the LiDAR Lite unit will likely give a return only if its angle to the surface that it is measuring is steeper (i.e., more orthogonal) than 45 degrees.***

![Field installation prototype in the lab: Sensor only](Documentation/images/FieldInstallation_LabPrototype_SensorOnly_orig2019-09-09_20200501.jpg)

## Field installation

![Field installation prototype in the lab](Documentation/images/FieldInstallation_LabPrototype_orig2019-09-09_20200501.jpg)

***Lab mock-up of field installation for LiDAR Lite.*** *The boxes at right contain [Northern Widget Margay](https://github.com/NorthernWidget-Skunkworks/Project-Margay) data loggers with a single cable gland to connect to the Symbiont box. The two posts next to them provide examples of how to connect the 3/4" EMT conduit to some fixed point in the environment, either the side of a flat(ish) wall (right) -- though the brackets aren't necessary or even always good, since we can bolt right through the pipe -- or to a flat(ish) surface using a floor flange. We drilled holes through the conduit to attach 1/4" eye bolts (1/4"-20, 1.5" long) using nuts, lock washers, and washers, to the conduit. These eye bolts then held turnbuckles to which we attached cables (lower left). The other end of the cables can be attached via sleeve or wedge anchors to rock, or to other sturdy structures. The LiDAR Lite + Symbiont is in the upper left corner, albeit attached in a way that would have it looking up... unless it were attached to the end of the 90-degree bend.*

### Mast construction

#### Supplies

* 3/4" EMT rigid conduit
  * Straight: main mast
  * 90-degree bend
  * 45-degree bend
* 3/4" Conduit screw-down connectors (to join multiple pieces of conduit)
* 3/4" Conduit-to-threaded connectors (to link smooth conduit to threaded 3/4" plumbing, needed only if you use a floor flange)
* Floor flange (1, if you want to attach to a flat surface)
* 1/4-20 x 1.5" eye bolts (3, +1 in case you bend/break one)
* 1/4" x 2+" long sleeve or wedge anchors; 4 for the floor flange or 2 for bolting into the side of a rock. Extras suggested, as these can be broken during installation.
* Turnbuckles; I suggest getting 3 that are threaded all the way down (as shown in the picture above). Get an extra in case you break one.
* 1/8" metal cable
* 1/8" cable clamps (x6, +1 in case you lose the nuts)
* 1/8" metal cable turns (x6; if you lose one, don't worry; these are nice for durability but not totally necessary)
* 3/8" x 2.5+" long sleeve or wedge anchors for the cables; these are less breakable than their 1/4" cousins, but you might still want an extra one.
* Extra 1/4"-20 hardware. I almost always end up needing this.
* Cable management (keep secure and prevent from flapping in the wind, which can cause damage)
  * Cable ties (zip ties). Ensure that these are UV-resistant; many black ones are.
  * Electrical tape
* *Optional:* 2x ~1-3/4" hose clamps if mounting a data-logger box for the [Margay](https://github.com/NorthernWidget-Skunkworks/Project-Margay) on the main mast. Longer hose clamps (or multiple 1-3/4" clamps in series) for larger logger boxes, so long as they can remain stable on the mast.

You might not need the guy wires and associated hardware if you bolt your assembly to the side of something (like a rock or wall). Some guy wires are generally encouraged.

#### Assembly

![Leveling the base](Documentation/images/PlacingBase_BillyLevel_2019-09-14_11.19.27.jpg)
***Placing and leveling the base.***

![Base is installed using wedge anchors](Documentation/images/BaseInstalled_WedgeAnchors_2019-09-14_11.43.26_modified.jpg)
***Installing the base using wedge anchors.***

![Conduit installed and anchored with eye bolts and turbuckles.](Documentation/images/Conduit_Standing_EyeBolts_2019-09-14_13.39.42.jpg)
***Installing the conduit and anchoring it with eye bolts and turnbuckles.*** *Before going out to the field, we strongly suggest you pre-drill the holes and pre-install the eye bolts.*

![Cable to rock bolt](Documentation/images/CableTurnaround_RockBolt_2019-09-14_15.28.48.jpg)
***Tightening the cable around the bolt in the rock using the cable clamp.***

![Aiming the LiDAR Lite](Documentation/images/AimToRiver_Billy_2019-09-14_14.39.31.jpg)
***Aiming the LiDAR Lite unit at the river.*** *In future models, we are considering adding peep sights to the mounting bracket. [Professor Billy Armstrong](https://earth.appstate.edu/faculty-staff/dr-william-h-armstrong) in the photo.*

The quality of any zeroing with the Hall-effect sensor will be limited by the ~1-degree precision of the accelerometer that is used as an inclinometer; as an additional check, it is recommended to measure and record the orientation of the field-mounted unit by hand.

![Perspective view upvalley](Documentation/images/Perspective_upvalley_2019-09-14_15.20.41.jpg)
***Fully installed clifftop unit***

![Perspective view after installation.](Documentation/images/Perspective_view_installed_2019-09-14_15.30.02.jpg)
***Fully installed clifftop unit*** *Note data-logger box attached with cable ties, as well as the mounting-plate attachment. For the latter, we used a rock as a shim with a 45-degree piece of EMS conduit. However, it may have been better to use a 90-degree piece of conduit and a 45-degree piece of conduit together to keep us from needing this shim -- though this would have increased torque on the main mast.*

![](Documentation/images/RockSideDrill_Billy_2019-09-15_14.14.45.jpg)
***Alternative method: installing on the side of a rock.***

![Culvert deployment with context](Documentation/images/DeploymentCulvert_2019-09-15_16.32.40.jpg)
***Full mast installed on the side of a rock.*** *Note the LiDAR Lite + Symbiont and the data-logger box.

## Acknowledgments

Support for this project provided by:

<img src="https://pbs.twimg.com/profile_images/1139626463932637186/qCak0yvY_400x400.png" alt="UMN ESCI" width="240px">

<img src="https://ane4bf-datap1.s3-eu-west-1.amazonaws.com/wmocms/s3fs-public/styles/featured_media_detail/public/advanced_page/featured_media/wmologo2016_fulltext_horizontal_rgb_en-2.jpg?C4guHHfFZ0Uv029Eo5AvJLFg6nMR47hI&itok=NVNNIT7H" alt="WMO" width="240px">

<br/>
<br/>
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
