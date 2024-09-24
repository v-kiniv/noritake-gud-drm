
# Linux DRM driver for Noritake GU-D series displays
<img align="right" src="./kmscube.gif" alt="kmskube" width="192" style="width:192px; border-radius: 6px;"/>

- Touchscreen
- Brightness
- GPU accelerated rendering


*The following installation instructions is intended for Raspberry Pi 4 and Raspbian OS 32 and 64 bit.*

### Get Driver Source
```bash
cd ~
git clone https://github.com/v-kiniv/noritake-gud-drm.git
cd ~/noritake-gud-drm
```

### Install build tools and Linux headers
```bash
sudo apt install build-essential raspberrypi-kernel-headers
```

### Build
```bash
make -j4
```

### Install Overlay
```bash
sudo dtc -@ -I dts -O dtb -o /boot/overlays/noritake-gud-drm.dtbo ./gud.dts
```

### Test
```bash
sudo dtoverlay noritake-gud-drm
sudo insmod ./gud.ko
```
If it's working, proceed with *Install Driver* and *Configure* sections to load driver on boot.

### Install Driver
```bash
sudo make install
```

### Configure
Driver can be enabled and configured using `dtoverlay` in `/boot/config.txt`.

Enable overlay with default configuration:
```
dtoverlay=noritake-gud-drm
```

Enable overlay with custom configuration(optional):
```
dtoverlay=noritake-gud-drm,speed=400000,busy-gpios=27,dithering=0
```

Set fixed clock for the GPU to avoid SPI clock exceeding maximum throughput of the display bus:
```
gpu_freq=250
```

### Reboot
And, finally, reboot device to load the driver:
```bash
sudo reboot
```

### Mesa
To enable GPU accelerated rendering, you will also need to build Mesa adapter driver:
*Adapter driver act as a bridge between Mesa and display driver.*

UPD: as of kernel 6.6 and Mesa 23 this is not required, but when you run an OpenGL application, MESA will complain about missing `gud_dri.so`.

```bash
# make sure you have uncommented deb-src in /etc/apt/sources.list before running apt-get build-dep
sudo apt-get build-dep mesa
wget https://archive.mesa3d.org/mesa-20.0.0.tar.xz
tar xf mesa-20.0.0.tar.xz
```
*You should be able to compile adapter driver for any version of Mesa, here we are compiling for 20.0.0*

1. Add the following line after `#if defined(GALLIUM_KMSRO)` in file `mesa-20.0.0/src/gallium/targets/dri/target.c`
```
DEFINE_LOADER_DRM_ENTRYPOINT(gud)
```

2. Add the following line after `foreach d : [[with_gallium_kmsro, [` in file `mesa-20.0.0/src/gallium/targets/dri/meson.build`:
```
'gud_dri.so',
```

3. Build:
```bash
cd mesa-20.0.0/
meson --prefix="${PWD}/build/install" build/
ninja -j4 -C build/
ninja -C build/ install
```

4. Install adapter driver

For 32bit OS run:
```bash
sudo cp build/install/lib/arm-linux-gnueabihf/dri/gud_dri.so /lib/arm-linux-gnueabihf/dri/
```

For 64bit OS run:
```bash
sudo cp build/install/lib/aarch64-linux-gnu/dri/gud_dri.so /lib/aarch64-linux-gnu/dri/
```

5. Test
```bash
sudo apt install kmscube
kmscube -D /dev/dri/card1
```

### Console Blank
Add `consoleblank=600` to `/boot/cmdline.txt` so it looks something like this:
```txt
console=serial0,115200 console=tty1 consoleblank=600 root=PARTUUID=a5507f56-02 rootfstype=ext4 fsck.repair=yes rootwait
```

### Configure Console Font
```bash
sudo dpkg-reconfigure console-setup
```
Proceed with wizard and select `Termius`->`6x12 (framebuffer only)` font

### Backlight / Display Power
Create udev rules to be able to change display brightness and power without sudo
```bash
echo 'SUBSYSTEM=="backlight",RUN+="/bin/chmod 666 /sys/class/backlight/%k/brightness /sys/class/backlight/%k/bl_power"' | sudo tee -a /etc/udev/rules.d/backlight-permissions.rules
```

Now you can control brightness by writing to the `/brightness`. Valid values are are 1 through 8.
```bash
# min brightness
echo 1 > /sys/class/backlight/gud/brightness

# max brightness
echo 8 > /sys/class/backlight/gud/brightness
```
And power by writing to the `/bl_power`
```bash
# power OFF
echo 4 > /sys/class/backlight/gud/bl_power

# power ON
echo 0 > /sys/class/backlight/gud/bl_power
```