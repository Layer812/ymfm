# picoaudio - VGM playback example for Pico Audio Pack

This is an experimental implementation for playback VGM files on [Pico Audio Pack](https://shop.pimoroni.com/products/pico-audio-pack?variant=32369490853971) with Raspberry Pi Pico.
Almost of all code is based on [YMFM](https://github.com/aaronsgiles/ymfm). 
The code for handling TAR file from flash is based on [pico-infones](https://github.com/shuichitakano/pico-infones) and [PicoSystem_InfoNes](https://github.com/fhoedemakers/PicoSystem_InfoNes).
Pico has limited power for handling FM emulation, some of VGM files (using multi chip) will not be able to listen normaly:(
In the other words, There are rooms for improve, So I appreciate your warm support :)

Outrun for YM2151(with out SEGA PCM) sounds good:)
Not good for AY-3-8910 or YM2419...

## Important notice
This software gives Raspberry PI Pico a higher core voltage & over clock, It may cause damege on your device. No warranties.

## Build & Install & Customise
### Build UF2 file
```
$ git clone https://github.com/Layer812/ymfm
$ cd ymfm/examples/picoaudio
$ mkdir build
$ cd build
$ cmake ..
$ make
```
### Install UF2 File
Put "ymfm/examples/picoaudio/build/picoaudio.uf2" into Raspberry Pico.
You can download from here >> [picoaudio.uf2](https://github.com/Layer812/ymfm/blob/main/examples/picoaudio/picoaudio.uf2)

### Customise
You can customise something by edit "ymfm/examples/picoaudio/CMakeLists.txt"

```
About over clocking
$       PICO_FLASH_SPI_CLKDIV=3
$       CORE_CLOCK=304000
$       CORE_VOLTAGE=VREG_VOLTAGE_1_30
```
```
About Stereo output (1 means MONO(default) / 0 means STEREO)
$              PICO_AUDIO_I2S_MONO_OUTPUT=1
```
After edit, you need build UF2 file again.

## Import VGM files and ROM file
### Import VGM files
Before Import, you need extract VGZ file to VGM.
```
$ mv some.vgz some.vgm.gz
$ gzip -d some.vgm.gz
```
The VGM file should be placed in some way from 0x10100000, and can be easily transferred using [picotool](https://github.com/raspberrypi/picotool).
```
picotool load some.vgm -t bin -o 0x10100000
```
You can either place the VGM files directly or place a tar file containing multiple VGM files. 
```
picotool load somevgms.tar -t bin -o 0x10100000
```
### Import ROM file
If you have ROM file for YM2608, you can store it from 0x10098000
```
picotool load ym2608.rom -t bin -o 0x10098000
```

## Thanks
- [Aaron Giles](https://github.com/aaronsgiles/ymfm) Greate project!
- [shuichitakano](https://github.com/shuichitakano/pico-infones) I could know lot of inspiration from his code.
- [yunkya2](https://github.com/yunkya2/pico-mdx/tree/master/pico-mdx) I'm enjoying to listen mdx.
- [Frank](https://github.com/fhoedemakers/) and [newschooldev](https://github.com/newschooldev) They motivated me to do this:)

## Things to do
- Rewrite some code with interpolater for improve speed.
- Support vgm command(repeat etc)
- Support buttons.
- Support vgz files.

## Change logs
### 0.1-alpha
Initial realease.
