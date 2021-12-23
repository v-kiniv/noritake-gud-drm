
# Linux DRM driver for Noritake GU-D series displays
<img align="right" src="./kmscube.gif" alt="kmskube" style="width:192px; border-radius: 6px;"/>

- Touchscreen
- Brightness
- GPU accelerated rendering


*The following installation instructions is intended for Raspberry Pi 4.*

### Get Driver Source
```bash
cd ~
git clone https://github.com/v-kiniv/noritale-gud-drm.git
cd ~/noritake-gud-drm
```

### Build
```bash
make -j4
```

### Test
```bash
sudo insmod ~/noritake-gud-drm/gud.ko
```

### Install Driver
```bash
sudo make install
```

### Install Overlay
To load the driver automatically on system boot, install overlay.
```bash
sudo dtc -@ -I dts -O dtb -o /boot/overlays/noritake-gud-drm.dtbo ./gud.dts
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

### Mesa
To enable GPU accelerated rendering, you will also need to build Mesa adapter driver:
*Adapter driver act as a bridge between Mesa and display driver.*

```bash
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
```

4. Install adapter driver:
```bash
sudo cp build/install/lib/arm-linux-gnueabihf/dri/gud_dri.so /lib/arm-linux-gnueabihf/dri/
```


