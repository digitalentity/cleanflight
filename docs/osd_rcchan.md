OSD_RCCHAN
==========

Allows the display of up to 4 rc channels on the OSD. The channel values can be
scaled as required and a prefix and/or postfix can be added. Max/min alarms can
also be specified.

The parameters are specified in the cli as follows:

**\#osd_rcchan index channel minimum maximum decimals minAlarm maxAlarm prefix
postfix**

Where the parameters are as follows

| **Name** | **Description**                                                                                         | **Default** | **Range**          |
|----------|---------------------------------------------------------------------------------------------------------|-------------|--------------------|
| index    | The index of the osd_rcchan.                                                                            |             | 0 to 3             |
| channel  | The rc channel number to be displayed                                                                   |             | 1 to 16            |
| minimum  | The value to be displayed in the OSD when the channel value is 1000                                     | 1000        | \-5000 to 5000     |
| minimum  | The value to be displayed in the OSD when the channel value is 2000                                     | 2000        | \-5000 to 5000     |
| decimals | Number of decimals to be displayed.                                                                     | 0           | 0 to 4             |
| minAlarm | Values below minAlarm will blink in OSD. Disabled if zero                                               | 0           | minimum to maximum |
| maxAlarm | Values above maxAlarm will blink in OSD. Disabled if zero                                               | 0           | minimum to maximum |
| prefix   | Prefix to display before value. If prefix is an integer then the corresponding symbol will be displayed |             | 3 character max    |
| postfix  | Text to display after value. If prefix is an integer then the corresponding symbol will be displayed    |             | 3 character max    |

Examples:
---------

### Get parameter list for all indices

**\#osd_rcchan**

osd_rcchan 0 4 1000 2000 0 1200 1800 THR

osd_rcchan 1 4 0 100 2 0 0 LFR %

osd_rcchan 2 0 1000 2000 0 0 0

osd_rcchan 3 0 1000 2000 0 0 0

### Get parameter list for one index

**\#osd_rcchan 1 4 0 100 2 0 0 LFR %**

osd_rcchan 1 4 0 100 2 0 0 LFR %

### Display the value of channel 4 (Throttle?) with no decimal places. The display will blink at throttle levels \>1800 or \< 1200. 

**\#osd_rcchan 0 4 1000 2000 0 1200 1800 THR**

osd_rcchan 0 4 1000 2000 0 1200 1800 THR

### Display the value of channel 9 with 2 decimal places. The prefix is left arrow. The values will range from 0 to 100 and will blink if above 50. The postfix is % .

**\#osd_rcchan 0 9 0 100 2 0 50 2 %**

osd_rcchan 0 9 0 100 2 0 50 2 %

### Reset channel 1 to default

\#osd_rcchan 1 0

osd_rcchan 1 0 1000 2000 0 0 0

**NB**

In order to have the values displayed in the OSD, the screen coordinates for
each osd_rcchan item must be set in the cli. Unlike other OSD components, this
is not currently possible using the configurator.

In INav 2.4, the item numbers for the four osd_rcchan entries are 107, 108, 109,
110 and the settings must be viewed and modified using the cli. These indices
will probably be different in other releases

**\#osd_layout 0 107 2 9 V**

osd_layout 0 107 2 9 V
