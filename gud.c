#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/dev_printk.h>
#include <linux/property.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/backlight.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/input/mt.h>

#include <drm/drm_drv.h>
#include <drm/drm_connector.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_rect.h>

#define GUD_RX_BUF_SIZE   512
#define TOUCH_SW_NUM      32
#define TOUCH_X_MAX       256
#define TOUCH_Y_MAX       128
#define ENABLE_BACKLIGHT  1
#define ENABLE_TOUCH      1
#define WHITE             0xff
#define BLACK             0

#define DATA_WRITE_CMD 0x44
#define DATA_READ_CMD 0x54
#define STATUS_READ_CMD 0x58

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
#define write_reg(vfd, ...)                                            \
  (gud_write_register(vfd, NUMARGS(__VA_ARGS__), __VA_ARGS__))

/* diffusing error (Floyd-Steinberg) */
#define DIFFUSING_MATRIX_WIDTH  2
#define DIFFUSING_MATRIX_HEIGHT  2

static const signed char
diffusing_matrix[DIFFUSING_MATRIX_WIDTH][DIFFUSING_MATRIX_HEIGHT] = {
  {-1, 3},
  {3, 2},
};

static const struct drm_mode_config_funcs gud_mode_config_funcs = {
  .fb_create = drm_gem_fb_create_with_dirty,
  .atomic_check = drm_atomic_helper_check,
  .atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_display_mode gud_mode = {
  DRM_SIMPLE_MODE(256, 128, 84, 42),
};

static const uint32_t gud_formats[] = {
  DRM_FORMAT_XRGB8888,
};

DEFINE_DRM_GEM_DMA_FOPS(gud_fops);

struct gud_vfd {
  struct drm_display_mode mode;
  struct drm_connector connector;
  struct drm_device drm;
  struct drm_simple_display_pipe pipe;
  struct spi_device *spi;
  struct touchscreen_properties touch_props;
  unsigned int rotation;
  bool   dithering;
  bool   enabled;
  bool   bl_power_on;
  struct backlight_device *bl_dev;
  struct input_dev    *input_dev;
  struct mutex cmdlock;
  struct work_struct poll_work;
  void * rx_buf;
  void * tx_buf;

  struct gpio_desc *busy;
  struct gpio_desc *reset;

  unsigned int height;
  unsigned int width;

  struct {
    struct drm_rect rect;
    u8 *buf;
  } last_frame;
};

static const struct drm_driver gud_driver = {
  .driver_features  = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
  .fops      = &gud_fops,
  DRM_GEM_DMA_DRIVER_OPS_VMAP,
  .name      = "gud",
  .desc      = "Noritake dot matrix VFD displays",
  .date      = "20210405",
  .major      = 1,
  .minor      = 0,
};

static bool gud_wait_busy(struct gud_vfd *vfd)
{
  int i;

  if(!vfd->busy) {
    return true;
  }

  for (i = 100; i > 0; i--) {
    if (!gpiod_get_value_cansleep(vfd->busy))
      break;

    usleep_range(10, 100);
  }

  if (!i) {
    dev_err(&vfd->spi->dev, "%s busy timeout", __func__);
    return false;
  }

  return true;
}

static int gud_available(struct gud_vfd *vfd)
{
  u8 txbuf[8] = { 0 };
  u8 rxbuf[8] = { 0 };
  int len = -1;
  int res = 0;

  struct spi_transfer t = {
    .tx_buf = txbuf,
    .rx_buf = rxbuf,
    .len = 8,
    .speed_hz = vfd->spi->max_speed_hz,
  };
  struct spi_message m;

  txbuf[0] = STATUS_READ_CMD;

  if (!vfd->spi) {
    return -1;
  }

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);

  if(!gud_wait_busy(vfd)) {
    return -1;
  }
  res = spi_sync(vfd->spi, &m);
  
  // 6th bit should be 0, invliad status otherwise
  if(res < 0 || (rxbuf[6] & 0x40)) { 
    return -1;
  }

  len = rxbuf[6] & 0x3F; // only lower 5 bits is used to indicate transmit data len
  if(len < 0 || len > 63) {
    dev_err(&vfd->spi->dev, "%s: illegal transmit data len: %d\n", __func__, len);
    return -1;
  }

  return len;
}

static int gud_read(struct gud_vfd *vfd, void *buf, int len)
{
  u8 txbuf[64] = { 0 };
  u8 rxbuf[64] = { 0 };
  int res = 0;

  struct spi_transfer t = {
    .tx_buf = txbuf,
    .rx_buf = rxbuf,
    .len = len + 3,
    .speed_hz = vfd->spi->max_speed_hz,
  };
  struct spi_message m;

  if(len < 1 || len > 63) {
    dev_err(&vfd->spi->dev, "%s: illegal read len: %d\n", __func__, len);
    return -1;
  }

  txbuf[0] = DATA_READ_CMD;

  if (!vfd->spi) {
    return -1;
  }

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);

  if(!gud_wait_busy(vfd)) {
    return -1;
  }
  res = spi_sync(vfd->spi, &m);
  if(res < 0) {
    return -1;
  }

  memcpy(buf, &rxbuf[2], len);

  return 0;
}

static int gud_write(struct gud_vfd *vfd, void *buf, size_t len)
{
  int res = 0;
  u8 *txbuf = buf;
  size_t i;

  struct spi_transfer t = {
    .tx_buf = txbuf,
    .len = len + 1,
    .speed_hz = vfd->spi->max_speed_hz,
  };
  struct spi_message m;

  for (i = len; i > 0; i--) {
    txbuf[i] = txbuf[i - 1];
  }

  txbuf[0] = DATA_WRITE_CMD;

  if (!vfd->spi) {
    pr_err("%s: vfd->spi is unexpectedly NULL\n", __func__);
    return -1;
  }

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);
  
  if(!gud_wait_busy(vfd)) {
    return -1;
  }
  res = spi_sync(vfd->spi, &m);

  return res;
}

static int gud_write_register(struct gud_vfd *vfd, int len, ...)
{
  va_list args;
  u8 *buf = vfd->tx_buf;
  int res, i = 0;

  va_start(args, len);
  for (i = 0; i < len; i++) {
    buf[i] = ((u8) va_arg(args, unsigned int));
  }
  va_end(args);

  res = gud_write(vfd, buf, len);
  if(res < 0) {
    dev_err(&vfd->spi->dev, "%s: gud_write returned %d", __func__, res);
  }

  return res;
}

static int gud_read_display_status(struct gud_vfd *vfd) {
  uint32_t status = 0;
  uint8_t *rx = vfd->rx_buf;
  int len = 0;

  memset(rx, 0, GUD_RX_BUF_SIZE);

  // Display status send, a = 0x30 (Product type information)
  write_reg(vfd, 0x1f, 0x28, 0x65, 0x40, 0x30);
  len = gud_available(vfd);

  // check for valid header
  if(len == 18 && gud_read(vfd, rx, len) == 0 && rx[0] == 0x28 && rx[1] == 0x65 && rx[2] == 0x40) {
    dev_info(&vfd->spi->dev, "%s: product type: %*s", __func__, 15, &rx[3]);
  } else {
    dev_err(&vfd->spi->dev, "%s: invalid response (len: %d)", __func__, len);
    status = -1;
  }

  return status;
}

static int gud_reset(struct gud_vfd *vfd)
{
  if (!vfd->reset) {
    dev_err(&vfd->spi->dev, "No reset pin present");
    return -1;
  }

  dev_info(&vfd->spi->dev, "Reset display");
  gpiod_set_value_cansleep(vfd->reset, 0);
  msleep(2);
  gpiod_set_value_cansleep(vfd->reset, 1);
  msleep(120);

  return 0;
}

static int gud_blank(struct gud_vfd *vfd, bool on)
{
  dev_info(&vfd->spi->dev, "(%s=%s)\n", __func__, on ? "true" : "false");

  mutex_lock(&vfd->cmdlock);
  write_reg(vfd, 0x1f, 0x28, 0x61, 0x40, on ? 0x00 : 0x01);
  mutex_unlock(&vfd->cmdlock);

  vfd->bl_power_on = !on;
  return 0;
}

static const struct of_device_id gud_of_match[] = {
  { .compatible = "noritake,gu256x128d" },
  {},
};
MODULE_DEVICE_TABLE(of, gud_of_match);

static const struct spi_device_id gud_id[] = {
  { "gu256x128d", 0 },
  { },
};
MODULE_DEVICE_TABLE(spi, gud_id);

static inline struct gud_vfd *drm_to_vfd(struct drm_device *drm)
{
  return container_of(drm, struct gud_vfd, drm);
}

static void iterate_diffusion_matrix(u32 xres, u32 yres, int x,
             int y, signed short *convert_buf,
             signed short pixel, signed short error)
{
  u16 i, j;

  /* diffusion matrix row */
  for (i = 0; i < DIFFUSING_MATRIX_WIDTH; ++i) {
    /* diffusion matrix column */
    for (j = 0; j < DIFFUSING_MATRIX_HEIGHT; ++j) {
      signed short *write_pos;
      signed char coeff;

      /* skip pixels out of zone */
      if (x + i < 0 || x + i >= xres || y + j >= yres)
        continue;
      write_pos = &convert_buf[(y + j) * xres + x + i];
      coeff = diffusing_matrix[i][j];
      if (-1 == coeff) {
        /* pixel itself */
        *write_pos = pixel;
      } else {
        signed short p = *write_pos + error * coeff;

        if (p > WHITE) {
          p = WHITE;
        }
        if (p < BLACK) {
          p = BLACK;
        }
        *write_pos = p;
      }
    }
  }
}

static void gud_dither(u8 *buf, signed short *convert_buf, 
                       int width, int height) {
  int x, y;
  signed short pixel;
  signed short error_b;
  signed short error_w;
  signed short error;

  for (x = 0; x < width; ++x)
    for (y = 0; y < height; ++y) {
      convert_buf[y * width + x] = buf[y * width + x];
    }

  for (x = 0; x < width; ++x)
    for (y = 0; y < height; ++y) {
      pixel = convert_buf[y * width + x];
      if (pixel > 169) {
        pixel = 255;
      }
      error_b = pixel - BLACK;
      error_w = pixel - WHITE;

      /* what color close? */
      if (abs(error_b) >= abs(error_w)) {
        /* white */
        error = error_w;
        pixel = 0xff;
      } else {
        /* black */
        error = error_b;
        pixel = 0;
      }

      error /= 8;

      iterate_diffusion_matrix(width, height, x, y, convert_buf, pixel, error);
    }

  for (x = 0; x < width; ++x)
    for (y = 0; y < height; ++y) {
      int p = convert_buf[y * width + x];
      if (p > WHITE){
        p = WHITE;
      }
      if (p < BLACK){
        p = BLACK;
      }
      buf[y * width + x] = p;
    }
}

static int gud_write_vmem(struct gud_vfd *vfd, struct drm_rect *rect, void *fb_buf)
{
  unsigned int height = drm_rect_height(rect);
  unsigned int width = drm_rect_width(rect);
  u8 *vmem8 = (u8 *)(fb_buf);
  u32 xres = width;
  u32 yres = height;
  u8 *buf = vfd->tx_buf;
  int x, y;
  int ret = 0;
  int destPos = 0;
  u16 b;

  mutex_lock(&vfd->cmdlock);

  write_reg(vfd, 0x1f, 0x28, 0x64, 0x21);
  write_reg(vfd, rect->x1, rect->x1 >> 8, rect->y1, rect->y1 >> 8);
  write_reg(vfd, xres, xres >> 8, yres, yres >> 8);
  write_reg(vfd, 0x01);

  for (x = 0; x < xres; x++) {
     b = 0;
     for (y = 0; y < yres; y++) {
      b = (unsigned char)((b << 1) | (vmem8[xres * y + x] >> 7));

       if (y % 8 == 7) {
         /* just did a full byte */
         buf[destPos++] = b;
       } /* if */
     } /* for */
     if (yres % 8 != 0) {
       b <<= (8 - yres % 8);
       buf[destPos++] = b;
     } /* if */
   } /* for */

  ret = gud_write(vfd, vfd->tx_buf, (height / 8) * width);
  if (ret < 0)
    dev_err(&vfd->spi->dev, "write failed and returned: %d\n", ret);

  mutex_unlock(&vfd->cmdlock);

  return ret;
}

static void gud_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
  struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
  struct gud_vfd *vfd = drm_to_vfd(fb->dev);
  unsigned int dst_pitch = 0;
  struct iosys_map dst, vmap;
  struct drm_rect clip;
  unsigned int height = drm_rect_height(rect);
  unsigned int width = drm_rect_width(rect);
  int idx, ret = 0;
  u8 *buf = NULL;
  signed short *convert_buf = NULL;

  if (!vfd->enabled)
    return;

  clip.x1 = rect->x1;
  clip.x2 = rect->x2;
  clip.y1 = rect->y1;
  clip.y2 = rect->y2;

  if(height % 8 != 0) {
    clip.y2 += 8 - (height % 8);
    height = drm_rect_height(&clip);
  }

  if(fb->format->format != DRM_FORMAT_XRGB8888) {
    dev_err(&vfd->spi->dev, "%s: invalid format (%u)", __func__, 
              fb->format->format);
    return;
  }

  if (!drm_dev_enter(fb->dev, &idx))
    return;

  buf = kmalloc_array(fb->width * fb->height, sizeof(unsigned char), GFP_KERNEL);
  if (!buf) {
    goto out_exit;
  }

  ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
  if (ret)
    goto out_free;

  iosys_map_set_vaddr(&dst, buf);
	iosys_map_set_vaddr(&vmap, dma_obj->vaddr);
  drm_fb_xrgb8888_to_gray8(&dst, &dst_pitch, &vmap, fb, &clip);

  drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);

  if(vfd->dithering) {
    convert_buf = kmalloc_array(fb->width * fb->height, sizeof(signed short),
                                GFP_KERNEL);
    if (!convert_buf) {
      goto out_free;
    }
    gud_dither(buf, convert_buf, width, height);
    kfree(convert_buf);
  }

  ret = gud_write_vmem(vfd, &clip, buf);

out_free:
  kfree(buf);
out_exit:
  drm_dev_exit(idx);
}


static void gud_enable_flush(struct gud_vfd *vfd,
         struct drm_plane_state *plane_state)
{
  struct drm_framebuffer *fb = plane_state->fb;
  struct drm_rect rect = {
    .x1 = 0,
    .x2 = fb->width,
    .y1 = 0,
    .y2 = fb->height,
  };

  vfd->enabled = true;
  gud_fb_dirty(fb, &rect);
}

static void gud_pipe_enable(struct drm_simple_display_pipe *pipe,
        struct drm_crtc_state *crtc_state,
        struct drm_plane_state *plane_state)
{
  struct gud_vfd *vfd = drm_to_vfd(pipe->crtc.dev);

  dev_info(&vfd->spi->dev, "gud_pipe_enable");

  mutex_lock(&vfd->cmdlock);
  write_reg(vfd, 0x0c); // clear
  mutex_unlock(&vfd->cmdlock);
  gud_blank(vfd, false);

  gud_enable_flush(vfd, plane_state);
}

static void gud_pipe_disable(struct drm_simple_display_pipe *pipe)
{
  struct gud_vfd *vfd = drm_to_vfd(pipe->crtc.dev);

  dev_info(&vfd->spi->dev, "gud_pipe_disable");

  vfd->enabled = false;
  gud_blank(vfd, true);
}

static void gud_pipe_update(struct drm_simple_display_pipe *pipe,
        struct drm_plane_state *old_state)
{
  struct drm_plane_state *state = pipe->plane.state;
  struct drm_rect rect;

  if (!pipe->crtc.state->active)
    return;

  if (drm_atomic_helper_damage_merged(old_state, state, &rect)) {
    gud_fb_dirty(state->fb, &rect);
  }
}

static const struct drm_simple_display_pipe_funcs gud_pipe_funcs = {
  .enable = gud_pipe_enable,
  .disable = gud_pipe_disable,
  .update = gud_pipe_update,
};

static int gud_connector_get_modes(struct drm_connector *connector)
{
  struct gud_vfd *vfd = drm_to_vfd(connector->dev);
  struct drm_display_mode *mode;

  mode = drm_mode_duplicate(connector->dev, &vfd->mode);
  if (!mode) {
    DRM_ERROR("Failed to duplicate mode\n");
    return 0;
  }

  if (mode->name[0] == '\0')
    drm_mode_set_name(mode);
  
  mode->type |= DRM_MODE_TYPE_PREFERRED;
  drm_mode_probed_add(connector, mode);

  if (mode->width_mm) {
    connector->display_info.width_mm = mode->width_mm;
    connector->display_info.height_mm = mode->height_mm;
  }

  return 1;
}

static const struct drm_connector_helper_funcs gud_connector_hfuncs = {
  .get_modes = gud_connector_get_modes,
};

static const struct drm_connector_funcs gud_connector_funcs = {
  .reset = drm_atomic_helper_connector_reset,
  .fill_modes = drm_helper_probe_single_connector_modes,
  .destroy = drm_connector_cleanup,
  .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
  .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

#ifdef ENABLE_BACKLIGHT
void unregister_backlight(struct gud_vfd *vfd)
{
  gud_blank(vfd, true);
  if (vfd->bl_dev) {
    vfd->bl_dev->props.power = FB_BLANK_POWERDOWN;
    backlight_update_status(vfd->bl_dev);
    backlight_device_unregister(vfd->bl_dev);
    vfd->bl_dev = NULL;
  }
}

static int update_onboard_backlight(struct backlight_device *bd)
{
  struct gud_vfd *vfd = bl_get_data(bd);
  int level = bd->props.brightness;
  dev_info(&vfd->spi->dev, "%s: level=%d, power=%d\n", __func__, level, bd->props.power);

  if (bd->props.power <= FB_BLANK_UNBLANK && !vfd->bl_power_on && vfd->enabled) {
    return gud_blank(vfd, false);
  } else if(bd->props.power > FB_BLANK_UNBLANK) {
    if(vfd->bl_power_on) {
      return gud_blank(vfd, true);
    }
    return 0;
  }

  if(level < 1 || level > 8) {
    return 0;
  }

  mutex_lock(&vfd->cmdlock);
  write_reg(vfd, 0x1f, 'X', level);
  mutex_unlock(&vfd->cmdlock);

  return 0;
}

static const struct backlight_ops bl_ops = {
  .update_status = update_onboard_backlight,
};

static void register_onboard_backlight(struct gud_vfd *vfd)
{
  struct backlight_device *bd;
  struct backlight_properties bl_props = {
    .max_brightness = 8,
    .type = BACKLIGHT_RAW,
  };

  bl_props.type = BACKLIGHT_RAW;
  bl_props.power = FB_BLANK_UNBLANK;

  bd = backlight_device_register("gud",
               &vfd->spi->dev, vfd, &bl_ops,
               &bl_props);
  
  if (IS_ERR(bd)) {
    dev_err(&vfd->spi->dev, "cannot register backlight device (%ld)\n",
      PTR_ERR(bd));
    return;
  }
  int level = bd->props.brightness;
  vfd->bl_dev = bd;
}
#else
static void register_onboard_backlight(struct gud_vfd *vfd) { };
#endif

static void gud_setup_touch_input(struct gud_vfd *vfd)
{
  struct input_dev *input_dev;
  int error;

  input_dev = devm_input_allocate_device(&vfd->spi->dev);
  if (!input_dev) {
    dev_err(&vfd->spi->dev, "Failed to allocate memory\n");
    return;
  }

  vfd->input_dev = input_dev;

  input_dev->name = "gud_ts";
  input_dev->phys = "input/ts";

  input_set_abs_params(input_dev, ABS_X, 0, TOUCH_X_MAX, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, 0, TOUCH_Y_MAX, 0, 0);

  input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TOUCH_X_MAX, 0, 0);
  input_abs_set_res(input_dev, ABS_MT_POSITION_X, TOUCH_X_MAX);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TOUCH_Y_MAX, 0, 0);
  input_abs_set_res(input_dev, ABS_MT_POSITION_Y, TOUCH_Y_MAX);
  input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 1, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 1, 0, 0);

  input_mt_init_slots(input_dev, TOUCH_SW_NUM, INPUT_MT_DIRECT);
  touchscreen_parse_properties(input_dev, false, &vfd->touch_props);


  error = input_register_device(vfd->input_dev);
  if (error) {
    dev_err(&vfd->spi->dev, "Failed to register interrupt\n");
  }
}

static int64_t gud_read_touch_status(struct gud_vfd *vfd) {
  int64_t status = -1;
  uint8_t *rx = vfd->rx_buf;
  int len = 0;
  bool valid = false;

  mutex_lock(&vfd->cmdlock);
  memset(rx, 0, GUD_RX_BUF_SIZE);

  if(write_reg(vfd, 0x1f, 0x4b, 0x10) == 0) {
    len = gud_available(vfd);
    if(len > 0 && gud_read(vfd, rx, len) == 0) {
      if(len == 6 && rx[0] == 0x10 && rx[1] == 4) {
        status =  (rx[2] <<  24)  |
                  (rx[3] <<  16)  |
                  (rx[4] <<  8)   |
                  (rx[5] <<  0);
        valid = true;
      }
    }
  }

  mutex_unlock(&vfd->cmdlock);
  return status;
}

static void gud_poll_touch(struct gud_vfd *vfd)
{
  struct input_dev *dev = vfd->input_dev;
  int64_t status = 0;
  uint16_t x = 0;
  uint16_t y = 0;
  bool curr = false;
  int i = 0;

  if(dev == NULL) {
    return;
  }

  status = gud_read_touch_status(vfd);

  if(status > -1) {
    for (i = 0; i < TOUCH_SW_NUM; i++) {
      curr = ((u32) status) & (1 << i);
      y = (i / 8) % 4;
      x = i % 8;

      input_mt_slot(dev, i);
      touchscreen_report_pos(dev, &vfd->touch_props, (x*32+16), (y*32+16), true);
      input_mt_report_slot_state(dev, MT_TOOL_FINGER, curr);
    }

    input_mt_sync_frame(dev);
    input_sync(dev);
  }
}

static void gud_poll_thread(struct work_struct *work){
  struct gud_vfd *vfd = container_of(work, struct gud_vfd, poll_work);

  set_current_state(TASK_INTERRUPTIBLE);
  schedule_timeout(HZ / 20);
  gud_poll_touch(vfd);
  schedule_work(&vfd->poll_work);

  return;
}

static int gud_probe_spi(struct spi_device *spi)
{
  int ret;
  struct drm_device *drm;
  struct gud_vfd *vfd;
  struct device *dev = &spi->dev;
  u32 dithering = 0;

  dev_info(dev, "gud_probe_spi: SPI speed: %uKHz\n", spi->max_speed_hz / 1000);

  device_property_read_u32(dev, "dithering", &dithering);

  // DEVICE
  vfd = devm_drm_dev_alloc(dev, &gud_driver, struct gud_vfd, drm);
  if (IS_ERR(vfd)) {
    dev_err(dev, "devm_drm_dev_alloc failed: %d\n", ret);
    return PTR_ERR(vfd);
  }
  mutex_init(&vfd->cmdlock);
  vfd->enabled = false;
  vfd->dithering = dithering;
  vfd->spi = spi;
  vfd->width = 256;
  vfd->height = 128;

  drm = &vfd->drm;
  memset(&vfd->last_frame, 0, sizeof(struct drm_rect));

  // SPI
  /* The SPI device is used to allocate dma memory */
  if (!dev->coherent_dma_mask) {
    ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
    if (ret) {
      dev_warn(dev, "Failed to set dma mask %d\n", ret);
      return ret;
    }
  }
  spi_set_drvdata(spi, drm);

  // GPIO
  vfd->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
  if (IS_ERR(vfd->reset)) {
    ret = PTR_ERR(vfd->reset);
    if (ret != -EPROBE_DEFER)
      dev_err(dev, "Failed to get gpio 'reset'\n");
    return ret;
  }

  vfd->busy = devm_gpiod_get(dev, "busy", GPIOD_IN);
  if (IS_ERR(vfd->busy)) {
    ret = PTR_ERR(vfd->busy);
    if (ret != -EPROBE_DEFER)
      dev_err(dev, "Failed to get gpio 'busy', driver will skip busy line check\n");
  }

  // IO BUFFERS
  vfd->tx_buf = devm_kzalloc(dev, vfd->width * vfd->height / 8, GFP_KERNEL);
  if (!vfd->tx_buf)
    return -ENOMEM;

  vfd->rx_buf = devm_kzalloc(dev, GUD_RX_BUF_SIZE, GFP_KERNEL);
  if (!vfd->rx_buf)
    return -ENOMEM;

  vfd->last_frame.buf = devm_kzalloc(dev, vfd->width * vfd->height * 2, GFP_KERNEL);
  if (!vfd->last_frame.buf)
    return -ENOMEM;

  // DISPLAY
  mutex_lock(&vfd->cmdlock);
  gud_reset(vfd);
  ret = gud_read_display_status(vfd);
  if(ret < 0) {
    return -EPROBE_DEFER;
  }
  
  // init
  write_reg(vfd, 0x1b, 0x40);

  // configure touch
  write_reg(vfd, 0x1f, 0x4b, 0x70, 0x00, 0x02); // configure touch sensitivity
  write_reg(vfd, 0x1f, 0x4b, 0x70, 0x01, 0x01); // configure touch ON decision
  write_reg(vfd, 0x1f, 0x4b, 0x70, 0x02, 0x01); // configure touch OFF decision
  write_reg(vfd, 0x1f, 0x4b, 0x70, 0x03, 0x00); // configure calibration period
  mutex_unlock(&vfd->cmdlock);
  
  register_onboard_backlight(vfd);

  #if ENABLE_TOUCH == 1
  gud_setup_touch_input(vfd);
  INIT_WORK(&vfd->poll_work, gud_poll_thread);
  schedule_work(&vfd->poll_work);
  #endif

  // DRM
  ret = drmm_mode_config_init(drm);
  if (ret) {
    dev_err(dev, "drmm_mode_config_init failed: %d\n", ret);
    return ret;
  }

  drm_mode_copy(&vfd->mode, &gud_mode);

  ret = drm_connector_init(drm, &vfd->connector, &gud_connector_funcs,
         DRM_MODE_CONNECTOR_SPI);
  if (ret) {
    dev_err(dev, "drm_connector_init failed: %d\n", ret);
    return ret;
  }
  drm_connector_helper_add(&vfd->connector, &gud_connector_hfuncs);

  ret = drm_simple_display_pipe_init(drm, &vfd->pipe, &gud_pipe_funcs,
             gud_formats, ARRAY_SIZE(gud_formats),
             NULL, &vfd->connector);
  if (ret) {
    dev_err(dev, "drm_simple_display_pipe_init failed: %d\n", ret);
    return ret;
  }

  drm_plane_enable_fb_damage_clips(&vfd->pipe.plane);

  drm->mode_config.preferred_depth = 32;
  drm->mode_config.min_width = vfd->mode.hdisplay;
  drm->mode_config.max_width = vfd->mode.hdisplay;
  drm->mode_config.min_height = vfd->mode.vdisplay;
  drm->mode_config.max_height = vfd->mode.vdisplay;
  drm->mode_config.funcs = &gud_mode_config_funcs;
  
  drm_mode_config_reset(drm);

  ret = drm_dev_register(drm, 0);
  if (ret) {
    dev_err(dev, "drm_dev_register failed: %d\n", ret);
    return ret;
  }

  drm_fbdev_generic_setup(drm, 0);

  return 0;
}

static void gud_remove(struct spi_device *spi)
{
  struct drm_device *drm = spi_get_drvdata(spi);
  struct gud_vfd *vfd = drm_to_vfd(drm);
  DRM_DEBUG_DRIVER("GUD REMOVE");
  dev_err(&spi->dev, "GUD REMOVE");

  drm_dev_unplug(drm);
  drm_atomic_helper_shutdown(drm);

  #ifdef ENABLE_BACKLIGHT
  unregister_backlight(vfd);
  #endif

  #ifdef ENABLE_TOUCH
  cancel_work_sync(&vfd->poll_work);
  #endif
}

static void gud_shutdown(struct spi_device *spi)
{
  struct drm_device *drm = spi_get_drvdata(spi);
  struct gud_vfd *vfd = drm_to_vfd(drm);

  DRM_DEBUG_DRIVER("GUD SHUTDOWN");
  dev_err(&spi->dev, "GUD SHUTDOWN");
  drm_atomic_helper_shutdown(drm);

  #ifdef ENABLE_BACKLIGHT
  unregister_backlight(vfd);
  #endif
}

static struct spi_driver gud_spi_driver = {
  .driver = {
    .name   = "gud",
    .owner  = THIS_MODULE,
    .of_match_table = gud_of_match,
  },
  .id_table = gud_id,
  .probe = gud_probe_spi,
  .remove = gud_remove,
  .shutdown = gud_shutdown,
};
module_spi_driver(gud_spi_driver);

MODULE_DESCRIPTION("DRM driver for the Noritake GU-D series display");
MODULE_AUTHOR("Vasily Kiniv");
MODULE_LICENSE("GPL");
