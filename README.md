# wifimic

## Preparing the Raspberry Pi zero w

I used the regular instructions to install Raspbian "Buster". Subsequently I did

    sudo apt-get update
    sudo apt-get upgrade

and used `raspi-config` to enable ssh and set the hostname, timezone, wifi country, locale and keyboard.

The wifimic server is implemented with Python3 and requires some extra packages to be installed.

    sudo apt-get install python3-pip
    sudo apt-get install python3-pyaudio
    sudo apt-get install python3-numpy

## Enable audio

To enable the HifiBerry hat, I followed the instructions from [here](https://www.hifiberry.com/docs/software/configuring-linux-3-18-x/) and removed

    dtparam=audio=on

and added

    dtoverlay=hifiberry-dac

I also enabled alsa by creating an `/etc/asound.conf` file with

    pcm.!default {
      type hw card 0
    }
    ctl.!default {
      type hw card 0
    }

The device showed up like this

    pi@wifimic:~ $ aplay -l
    **** List of PLAYBACK Hardware Devices ****
    card 0: sndrpihifiberry [snd_rpi_hifiberry_dac], device 0: HifiBerry DAC HiFi pcm5102a-hifi-0 [HifiBerry DAC HiFi pcm5102a-hifi-0]
      Subdevices: 1/1
      Subdevice #0: subdevice #0

and was able to play a stereo wav file that I downloaded from [onlinetonegenerator.com](https://onlinetonegenerator.com).

## Set up as wifi access point

I followed the instructions [here](https://www.raspberrypi.org/documentation/configuration/wireless/access-point-routed.md) to set it up as a wireless access point. Since this involves downloading packages from internet, but also changing the wifi settings, I used an OTG adapter and a USB ethernet adapter to implement a wired network connection. This allows headless access via ssh to debug the wireless setup, downloading the required files and keeping up to date with the code on github.

_Somehow my 2013 MacBook Pro will not connect to the wireless network of the RaspBerry Pi. Connecting to the access point with an iPad worked fine, and also the ESP32 modules don't show problems when connecting._

## Running the wifimic server as service

The wifimic server should start with the system and should always be running. I am using systemd for that, following these [instructions](https://www.raspberrypi.org/documentation/linux/usage/systemd.md).
