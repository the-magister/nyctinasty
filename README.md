# Nyctinasty Oasis

Submitting for an Honoraria grant for Burning Man 2018.  An interactive playground celebrating the rhythm of night and day. By day the piece provides climbing fun, shade, and misters powered by movement. At night the lights come on, the sound enlivens and the fire and smoke show begins. The final night, the flower-like structure will wilt and burn.

![3D render of Sepal](http://games-with-fire.com/wp-content/uploads/2017/11/outrigger-300x300.png "Logo")

## Physical Description

Three climbable outriggers around a 32 ft radius circle surround a central flower. 16 ft tall, the outriggers support 20 people dancing and climbing. Built of wood and covered in shade cloth, they feature water misters and hammocks for daytime refuge. Misters are housed in vines, flowers decorate the structure, and leaf-like streamers unfurl overhead.

The center flower is a 16 ft tall metal structure comprised of three metal petals with flame and fog effect heads. Each petal is 3 ft W x 6ft L * 16 ft H. They each have a secured metal base, topped with a foldable rigid origami paneling crafted from 16 gauge metal with metal hinging. A propane accumulator is stored within each petal and plumbed to the tip of the paneling. Each base has 50 meters of RGB LEDs wrapped around the accumulator, diffused by semi-opaque HDPE panels.

## Interactivity and Mission

Nyctinasty Oasis is a day/night playground. Each outrigger is a climbable shade structure. With chill day music, participants can climb the structure, play with it to coax cooling mists, or take a break in the hammocks underneath.
Also incorporated is a sound puzzle, where the participants use movement to mix together multiple tracks of sounds to complete a song. Each outrigger contains three sensor bars which detect the frequency and location of the participants using infrared sensors. The game incorporates collaborative movement to create a coherent whole. Songs composed of three tracks (e.g., bass/melody/vocals) will have each track randomly assigned. As the dancers move and align their tracks, the lights and music will synchronize, rewarding with cooling mists and sounds during the day, and coordinated light at night.

The central flower coordinates with fire and fog. At the end of the week, Nyctinasty Oasis is burned, leaving participants to wonder if it was all a mirage.

## Philosophy

Nyctinasty is the circadian rhythmic movement of plants in response to day and night. An oasis is a fertile habitat found in a desert. Together, a nyctinasty oasis is a retreat that provides both respite and reactivity that changes and is available from day to night. We seek to express how experience varies depending on time of day, point of view, activity level, and community involvement. Nyctinasty Oasis has diurnal features that return upon sunrise and sundown, as well as constant features. During the day, Nyctinasty Oasis provides solace with shade, hammocks, and water misters. Chill, low-volume ambient music is controlled by interaction with the outrigger sensors. At night, the lights come out to play along with a multi-track interactive musical and fire experience that varies based on the number of participants at points along the outriggers. The Nyctinasty Oasis experience is externally guided by regular circadian elements, but also influenced by user participation and determination. The Nyctinasty Oasis also provides constants, a playground standing ready for climbing, hiding, and chilling day or night. We wish to communicate to participants that the experience will be regular, changeable, and replenishing.

## Hardware 

Nyctinasty functions as a distributed set of microcontrollers (uC).  In this sense, the project shares the archetecture of an IoT system.  However, processing is all conducted locally (as opposed to typical cloud-based IoT processing).

### Devices

* Networking: WiFi router, RFM69H(C)W peer-to-peer, FM for sound.
* Wemos D1 Mini (ESP8266): and mainstay microcontroller (uC).
* Arduino Nano (Atmel 328P): analog-to-digitial conversion
* Moteino R3 (Atmel 328P): bridge to legacy HopeRF-mediated peer-to-peer hardware
* Tsunami Super WAV Trigger (ARM Cortex m7): audio generation
* Sharp IR Distance Sensor (GP2Y0A02YK0F): range and motion detection
* Addressable LEDs (WS2811): lighting
* pump (TBD): mister system
