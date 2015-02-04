# $Id: WeatherStationFSK.py $
# Copyright 2015 Matthew Wire
# Adapted from fousb.py
# See the file LICENSE.txt for your full rights.
#

"""Classes and functions for interfacing with WeatherStationFSK (WH1080 + BMP085 + DHT22).

Rainfall and Spurious Sensor Readings

The rain counter occasionally reports incorrect rainfall.  On some stations,
the counter decrements then increments.  Or the counter may increase by more
than the number of bucket tips that actually occurred.  The max_rain_rate
helps filter these bogus readings.  This filter is applied to any sample
period.  If the volume of the samples in the period divided by the sample
period interval are greater than the maximum rain rate, the samples are
ignored.

Spurious rain counter decrements often accompany what appear to be noisy
sensor readings.  So if we detect a spurious rain counter decrement, we ignore
the rest of the sensor data as well.  The suspect sensor readings appear
despite the double reading (to ensure the read is not happening mid-write)
and do not seem to correlate to unstable reads.

A single bucket tip is equivalent to 0.3 mm of rain.  The default maximum
rate is 24 cm/hr (9.44 in/hr).  For a sample period of 5 minutes this would
be 2 cm (0.78 in) or about 66 bucket tips, or one tip every 4 seconds.  For
a sample period of 30 minutes this would be 12 cm (4.72 in)

The rain counter is two bytes, so the maximum value is 0xffff or 65535.  This
translates to 19660.5 mm of rainfall (19.66 m or 64.9 ft).  The console would
have to run for two years with 2 inches of rainfall a day before the counter
wraps around.
MJW: Serial reading gives mm reading directly

Pressure Calculations

Pressures are calculated and reported differently by pywws and wview.  These
are the variables:

 - abs_pressure - the raw sensor reading
 - fixed_block_rel_pressure - value entered in console, then follows
     abs_pressure as it changes
 - fixed_block_abs_pressure - seems to follow abs_pressure, sometimes
     with a lag of a minute or two
 - pressure - station pressure (SP) - adjusted raw sensor reading
 - barometer - sea level pressure derived from SP using temperaure and altitude
 - altimeter - sea level pressure derived from SP using altitude

wview reports the following:

  pressure = abs_pressure * calMPressure + calCPressure
  barometer = sp2bp(pressure, altitude, temperature)
  altimeter = sp2ap(pressure, altitude)

pywws reports the following:

  pressure = abs_pressure + pressure_offset

where pressure_offset is

  pressure_offset = fixed_block_relative_pressure - fixed_block_abs_pressure

so that

  pressure = fixed_block_relative_pressure

pywws does not do barometer or altimeter calculations.

this implementation does the following:

  pressure = abs_pressure + offset
  barometer = sp2bp(adjp, altitude, temperature)
  altimeter = sp2ap(adjp, altitude)

where 'offset' is specified in weewx.conf (default is 0), 'altitude' is
specified in weewx.conf, and 'temperature' is read from the sensors.

The 'barometer' value is reported to wunderground, cwop, etc.
"""

import datetime
import sys
import syslog
import serial
import time
import io

import weewx.drivers
import weewx.wxformulas

DRIVER_NAME = 'WeatherStationFSK'
DRIVER_VERSION = '1.0'

DEBUG_RAIN = 1
DEBUG = 0

def loader(config_dict, engine):
    return WeatherStationFSK(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return WeatherStationFSKConfEditor()

# =============================================================================
#                      Class WeatherStationFSKConfEditor
# =============================================================================
class WeatherStationFSKConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WeatherStationFSK]
    # This section is for the WH1080 connected via a WeatherStationFSK Arduino.

    #   Debian, Ubuntu, Redhat, Fedora, and SuSE:
    #     /dev/ttyUSB0 is a common USB port name
    #     /dev/ttyS0   is a common serial port name
    #   BSD:
    #     /dev/cuaU0   is a common serial port name
    port = /dev/ttyUSB0

    ######################################################
    # The rest of this section rarely needs any attention. 
    # You can safely leave it "as is."
    ######################################################

    # Serial baud rate (usually 9600)
    baudrate = 9600

    # How long to wait for a response from the station before giving up (in
    # seconds; must be greater than 2)
    timeout = 5

    # How long to wait before trying again (in seconds)
    wait_before_retry = 1.2

    # How many times to try before giving up:
    max_tries = 4
    
    # The driver to use:
    driver = weewx.drivers.weatherstationfsk
"""

def logmsg(level, msg):
    syslog.syslog(level, 'WeatherStationFSK: %s' % msg)

def logdbg(msg):
    if DEBUG:
        logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

def _format(buf):
    return ' '.join(["%0.2X" % ord(c) for c in buf])

class WeatherStationFSK(weewx.drivers.AbstractDevice):
    """Driver for FineOffset USB stations."""
    
    def __init__(self, **stn_dict) :
        """Initialize the station object.

        port: The serial port of the device. [Required]
        
        baudrate: Baudrate of the port. [Optional. Default 9600]
        
        timeout: How long to wait before giving up on a response from the
        serial port. [Optional. Default is 15]
        
        wait_before_retry: How long to wait before retrying. [Optional.
        Default is 10 seconds]

        max_tries: How many times to try again before giving up. [Optional.
        Default is 3]
        """

        self.port              = stn_dict.get('port', '/dev/ttyUSB0')
        self.baudrate          = int(stn_dict.get('baudrate', 9600))
        self.timeout           = float(stn_dict.get('timeout', 15.0))
        self.wait_before_retry = float(stn_dict.get('wait_before_retry', 10.0))
        self.max_tries         = int(stn_dict.get('max_tries', 3))

        self._last_rain_loop = None

        logmsg(syslog.LOG_INFO, 'driver version is %s' % DRIVER_VERSION)

    # Unfortunately there is no provision to obtain the model from the station
    # itself, so use what is specified from the configuration file.
    @property
    def hardware_name(self):
        return 'WH1080 WeatherStationFSK'

    def genLoopPackets(self):
        """Generator function that continuously returns decoded packets."""

        ntries = 0
        while ntries < self.max_tries:
            ntries += 1
            try:
                packet = {'dateTime': int(time.time() + 0.5),
                          'usUnits': weewx.METRICWX}
                # open a new connection to the station for each reading
                with Station(self.port, self.baudrate, self.timeout) as station:
                    packet = station.get_readings(packet, self._last_rain_loop)
                    self._last_rain_loop = packet['rainTotal']
                ntries = 0
                yield packet
            except (serial.serialutil.SerialException, weewx.WeeWxIOError), e:
                logerr("Failed attempt %d of %d to get LOOP data: %s" %
                       (ntries, self.max_tries, e))
                time.sleep(self.wait_before_retry)
        else:
            msg = "Max retries (%d) exceeded for LOOP data" % self.max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)

class Station(object):

    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def read(self, nchar=1):
        buf = self.serial_port.read(nchar)
        n = len(buf)
        if n != nchar:
            if DEBUG and n:
                logdbg("partial buffer: '%s'" % _format(buf))
            raise weewx.WeeWxIOError("Read expected %d chars, got %d" %
                                     (nchar, n))
        return buf        

# inHumidity: DHT,C,<Humidity%>
# inTemp: DHT,<C>
# outHumidity: WS4,ID,C,<Humidity%>
# outTemp: WS4,ID,<C>
# pressure: BMP,C,<Pres,mBar?, abs/rel?> # station is mbar
# windSpeed: WS4,ID,C,H,<WindVel(m/s)*3.6=km/h> # station is m/s, weewx wants km/h
# windGust: WS4,ID,C,H,WVel,<WMax(m/s)*3.6=km/h # station is m/s, weewx wants km/h
# windDir: WS4,ID,C,H,WVel,WMax,WCompass,<WBearing(deg)>
# rain: WS4,ID,C,H,WVel,WMax,WCompass,WBearing(deg),<Rain(mm)*0.1=cm> # station is mm, weewx wants cm
    def get_readings(self, packet, last_rain):
        """Get data from the station.
        """
        b = ''
        a= ''
        dhtPacket = False
        bmpPacket = False
        ws4Packet = False
        pktType = 0 # 1=WS4,2=DHT,3=BMP
      
        while True:
            b += self.read(1)

            if pktType == 0:
                # Match token for packet type and remove everything before it
                if "WS4" in b:
                    pktType = 1
                    b = b[b.rfind("WS4"):]
                elif "DHT" in b:
                    pktType = 2
                    b = b[b.rfind("DHT"):]
                elif "BMP" in b:
                    pktType = 3
                    b = b[b.rfind("BMP"):]
            elif "\r" in b:
                # Wait for a \r character and extract the resulting string
                if pktType == 1:
                    a = b[b.rfind("WS4"):b.rfind("\r")]
                    csvData = a.split(',')
                    if csvData.count > 8:
                        ws4Packet = True
                        logdbg("csvdta: %s" % csvData)
                elif pktType == 2:
                    a = b[b.rfind("DHT"):b.rfind("\r")]
                    dhtData = a.split(',')
                    if dhtData.count > 2:
                        dhtPacket = True
                        logdbg("dhtdta: %s" % dhtData)
                elif pktType == 3:
                    a = b[b.rfind("BMP"):b.rfind("\r")]
                    bmpData = a.split(',')
                    if bmpData.count > 1:
                        bmpPacket = True
                        logdbg("bmpdta: %s" % bmpData)
                
                # Clear previous data from buffer
                b = b[b.rfind("\r")+1:]
                # Reset packet type
                pktType = 0

            if ws4Packet and dhtPacket and bmpPacket:
                break
            
        # Read WS4, DHT and BMP data.  Only return a packet when we get WS4 data and already have data for DHT/BMP
        
        if ((csvData[0] == 'WS4') and (csvData.count > 8)):
            # WS4 packet WS4,<ID>,<Temp C>,<rel Humidity>,<Wind Velocity>,<Wind Max>,<Wind compass>,<Wind bearing>,<Rain>
            packet['outHumidity'] = float(csvData[3])
            packet['outTemp'] = float(csvData[2])
            packet['windSpeed'] = float(csvData[4])# station is m/s
            packet['windGust'] = float(csvData[5])# station is m/s
            packet['windDir'] = float(csvData[7])
            
            # No windGustDir from station so use windDir
            packet['windGustDir'] = packet['windDir']
            # if windspeed is zero there is no wind direction
            if packet['windSpeed'] is None or packet['windSpeed'] == 0:
                packet['windDir'] = None
            if packet['windGust'] is None or packet['windGust'] == 0:
                packet['windGustDir'] = None
                
            # calculate the rain increment from the rain total
            # watch for spurious rain counter decrement.  if decrement is significant
            # then it is a counter wraparound.  a small decrement is either a sensor
            # glitch or a read from a previous record.  if the small decrement persists
            # across multiple samples, it was probably a firmware glitch rather than
            # a sensor glitch or old read.  a spurious increment will be filtered by
            # the bogus rain rate check.
            packet['rain'] = float(csvData[8])
            total = packet['rain']
            packet['rainTotal'] = packet['rain']
            if packet['rain'] is not None and last_rain is not None:
                if packet['rain'] < last_rain:
                    pstr = '0x%04x' % packet['ptr'] if packet['ptr'] is not None else 'None'
                    if last_rain - packet['rain'] < rain_max * 0.5:
                        logmsg(syslog.LOG_INFO, 'ignoring spurious rain counter decrement (%s): '
                               'new: %s old: %s' % (pstr, packet['rain'], last_rain))
                    else:
                        logmsg(syslog.LOG_INFO, 'rain counter wraparound detected (%s): '
                               'new: %s old: %s' % (pstr, packet['rain'], last_rain))
                        total += rain_max
            packet['rain'] = weewx.wxformulas.calculate_rain(total, last_rain)
            if not packet['rain'] == None:
                packet['rain'] = float(packet['rain'])# mm

            # report rainfall in log to diagnose rain counter issues
            if DEBUG_RAIN and packet['rain'] is not None and packet['rain'] > 0:
                logdbg('got rainfall of %.2f cm (new: %.2f old: %.2f)' %
                       (packet['rain'], packet['rainTotal'], last_rain))


        if ((dhtData[0] == 'DHT') and (dhtData.count > 2)):
            # DHT sensor packet DHT,<Temp C>,<Humidity>,<Heat Index> ( http://en.wikipedia.org/wiki/Heat_index )
            packet['inHumidity'] = float(dhtData[2])
            packet['inTemp'] = float(dhtData[1])

        if ((bmpData[0] == 'BMP') and (bmpData.count > 1)):
            # BMP sensor packet BMP,<Temp C>,<Pressure P>
            packet['pressure'] = float(bmpData[2])              

        # Only return a valid packet if we already have readings from WS4, DHT and BMP sensors
        if ((csvData.count > 8) and (dhtData.count > 2) and (bmpData.count > 1)):
            return packet
