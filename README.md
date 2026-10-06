# Sky Weather

A 7 inch touchscreen desk display for the weather at one place. You type a US place, pick the match, and the clock, sunrise, sunset, and alert times follow that place.

The sketch is `firmware/7 inch version/sky_weather/sky_weather.ino`.

## Board

Arduino IDE, board package ESP32 by Espressif (this sketch was built with 3.3.11).

| Setting | Value |
| --- | --- |
| Board | ESP32S3 Dev Module |
| PSRAM | OPI PSRAM |
| Flash Size | 16MB |
| USB CDC On Boot | Enabled |
| Partition Scheme | Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS) |

The world picture is stored in the sketch (`iss_world.h`). Together with the program it uses most of that 1.2 MB app slot. A smaller app partition will not fit.

Libraries, from the Library Manager:

- GFX Library for Arduino
- TJpg_Decoder

Wi-Fi, HTTP, Preferences, LittleFS, and FFat come with the ESP32 board package.

Open this sketch on its own. The board settings above are also in `firmware/7 inch version/settings to change.txt`.

Saved settings use the name `skywx`. Factory reset on the setup page clears this desk only.

## What it shows

Home shows the sky in plain words: sun, night, clear, partly cloudy, cloudy, wind, rain, snow, thunder, or hail. It also shows the temperature, the high and the low, the moon, the air pressure, the UV, and the rain. A quiet clear day says all clear. If the sky is not clear and there is no alert, it says no warnings. An alert shows the event and when it ends.

The foot buttons are Place, Radar, Rain, Sky, Wi-Fi, and Set.

- **Radar** has three pictures. Satellite is the overhead photo. Clouds keeps that photo and adds the bright cloud. Storm keeps that photo and adds the colored radar. The selected place and the largest nearby cities are named on those pictures, along with rivers and lakes.
- **Rain** shows the next 12 hours. The home rain card opens the next 24 hours. The lower temperature card opens the next 7 days. The pressure bar opens wind and air pressure.
- **Sky** opens a menu. Clock shows the time, sun up and sun down, the sky, the temperature, and the wind. Launches shows upcoming launches. Night sky lists the planets that are up, then the moon, stars, and clusters, on two pages. ISS shows the stored world picture with day and night, the orbit path, and the visible passes.
- **Set** has a dark or light look, lamp dim, mid, or bright, F or C, and factory reset.

It needs 2.4 GHz Wi-Fi and USB power. There is no account. Place search and tide stations are for the United States. A tide card appears for a place near a NOAA tide station.

## Data

Forecast and air quality come from Open-Meteo. Alerts come from weather.gov. Aurora Kp comes from NOAA SWPC. Tides come from NOAA. Place search, rivers, and lakes come from Nominatim. Nearby cities come from OpenStreetMap through Overpass. Satellite pictures come from Esri. Clouds come from NASA GIBS (GOES-East). Storm radar comes from Iowa State (NEXRAD). The ISS path and passes use a Celestrak element set and wheretheiss.at. Launches come from The Space Devs.

The ISS world picture is already in the sketch, so that page does not download the map. The path, the day and night shading, and the pass list still update over Wi-Fi.

## Listing pictures

`etsy/` holds example screens for a listing. The place on those pictures says YOUR TOWN. The numbers are not a live forecast. Listing text is in `etsy/ETSY-LISTING.txt`.
