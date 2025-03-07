# FraunhoferRadar {#fraunhofer_radar}

====================================================================================================
KNOWN BEHAVIOUR
====================================================================================================

1. When radar is connected to PC, consider 5-10 seconds timeout before it starts "talking".
2. Radar WILL GET CONSIDERABLY HOT (can get painful to touch) 5-10 minutes after connecting it.
    The metallic enclosure takes care of heat transfer away from electronics. THIS IS EXPECTED AND
    NORMAL. I believe this is due to the compactness of sensor and that the emitted frequency is
    very high (up to 92 GHz). The electromagnetic emission of radar is within EU norms and is
    not dangerous to person in any circumstances.
3. White plastic bulb serves to focus the radar beam to 2.5 degrees (azimuth and elevation) and
    must be handled with caution - any big scratches on surface might lead to performance loss.

====================================================================================================
IMPORTANT RADAR PARAMETERS (DEFINED IN CONFIG)
====================================================================================================

RUB_Version:
    Radar version
Start_Freq:
    defines the start frequency (in GHz in the range between 68.0 GHz to 92 GHz)
    of the desired FMCW ramp.
Stop_Freq:
    defines the stop frequency (in GHz in the range between 68.0 GHz to 92 GHz)
    of the desired FMCW ramp.
Ramp_Length:
    defines the ramp length or rather chirp duration (in seconds in the range between 0.001 sec to
    0.016 sec) of the desired FMCW ramp.
Ramp_Cnt:
    defines the number of sequential ramps (15000 ramps in maximum)
Ramp_wait:
    defines the duty cycle in seconds (ramp length + pause between consecutive ramps) and will be
    used by the system for Ramp_Cnt unequal zero. It must be ensured that Ramp_wait is at least
    1000 microseconds longer than the actual ramp length, otherwise a reliable operation of the
    radar is not guaranteed.
radar_delay:
    defines the delay (NOT duty cycle) in microseconds between ramps, when system is in endless
    measurement mode (Ramp_Cnt equal zero).
    Note: Since this delay is realized in software, there will be some jitter in the timing
    (especially for short delays). Therefore, for time-critical measurements, the timing should be
    specified via Ramp_Cnt and Ramp_wait.
selfReflectionBins:
    part of raw data that contains reflections from radar enclosure itself (not usable, therefore
    usually it is neglected)
radarSampleTime:
    radar data sampling frequency

Note: The maximum measurement rate depends on the computing power of the used computer
hardware.

====================================================================================================
KNOWN ISSUES
====================================================================================================

1. The First Radar initialization (just after connecting sensor) usually fails - this behaviour is
    not understood.

# XethruAdapter

====================================================================================================
LIBRARY INSTALLATION
====================================================================================================

To use the device please install `ModuleConnector` library from https://gitlab.cern.ch/glunghi/CERNRoboticFramework_libs/tree/master/Sensors/XethruModuleConnector
