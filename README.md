ADSIS8300bpm
============

The home of the Struck SIS8300 BPM digitizer support for areaDetector software.

[EPICS](http://www.aps.anl.gov/epics/) 

[areaDetector](http://cars.uchicago.edu/software/epics/areaDetector.html)

It contains this driver and IOC directory.

Additional information:
* [Documentation for ADSIS8300](???).
* [Documentation for ADCSimDetector](http://cars.uchicago.edu/software/epics/ADCSimDetectorDoc.html).
* [Release notes and links to source and binary releases](RELEASE.md).


Notes
-----

* Make sure that clock source is SMA, clock divider 1, trigger source External,
  RTM type DWC8VM1, all channels enabled, time point ~260000.
* Use timing receiver to generate triggers on backplane
* See documentation/Screenshot_2016-10-03_13-50-46.png

