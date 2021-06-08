/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
 

#include "io/adsb.h"
#include "navigation/navigation.h"
#include "common/maths.h"
#include "math.h"
#include "common/time.h"
#include "drivers/time.h"

adsbVehicle_t adsbNearest;
static timeUs_t last_adsb = 0;

void gpsDistanceCmBearing(int32_t currentLat1, int32_t currentLon1, int32_t destinationLat2, int32_t destinationLon2, uint32_t *dist, int32_t *bearing)
{
    #define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR    1.113195f  // MagicEarthNumber from APM
    float gps_scale_lon_down = cos_approx((fabsf((float)gpsSol.llh.lat) / 10000000.0f) * 0.0174532925f);
    const float d_lat = destinationLat2 - currentLat1; // difference of latitude in 1/10 000 000 degrees
    const float d_lon = (float)(destinationLon2 - currentLon1) * gps_scale_lon_down;

    *dist = sqrtf(sq(d_lat) + sq(d_lon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
    *bearing = 9000.0f + RADIANS_TO_CENTIDEGREES(atan2_approx(-d_lat, d_lon));      // Convert the output radians to 100xdeg
    *bearing = wrap_36000(*bearing);
}

void adsbNewVehicle(uint32_t avicao, int32_t avlat, int32_t avlon, int32_t avalt)
{
    uint32_t avdist; 
    int32_t avdir; 
    uint8_t avupdate = 1; 
    if (STATE(GPS_FIX) && gpsSol.numSat >= 5) {
        gpsDistanceCmBearing(gpsSol.llh.lat, gpsSol.llh.lon, avlat, avlon, &avdist, &avdir); 
    }
    avdist /= 100; 
    avdir /= 100;
#if defined(USE_NAV)
    avalt -= getEstimatedActualPosition(Z) + GPS_home.alt;
#elif defined(USE_BARO)
    avalt -= baro.alt + GPS_home.alt;
#endif
    avalt /= 1000;
    if (avdist > adsbNearest.dist) {
        avupdate = 0;
    }   
    if (adsbNearest.ttl <= 1) {
        avupdate = 1;
    }    
    if (avicao == adsbNearest.icao) {
        avupdate = 1;
    }      
    if (avdist > 50000) { // limit display to aircraft < 50Km
        avupdate = 0; 
    }  
    if (avupdate == 1) {
        adsbNearest.icao = avicao;
        adsbNearest.dist = avdist;       
        adsbNearest.alt = avalt; 
        adsbNearest.dir = avdir; 
        adsbNearest.ttl = 10;    // 10 secs default timeout    
    }   
}

void adsbExpiry()
{
    timeUs_t currentTimeUs = micros();
    if ((currentTimeUs - last_adsb) > 1000 * 1000) {
        last_adsb = currentTimeUs;
    if (adsbNearest.ttl > 0) {
        adsbNearest.ttl--;
    }
    else {
        adsbNearest.icao = 0;
        adsbNearest.dist = 0;       
        adsbNearest.alt = 0; 
        adsbNearest.dir = 0;       
    }
  }
}

