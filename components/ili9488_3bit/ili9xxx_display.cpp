#include "ili9xxx_display.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ili9xxx {

static const uint16_t SPI_SETUP_US = 100;         // estimated fixed overhead in microseconds for an SPI write
static const uint16_t SPI_MAX_BLOCK_SIZE = 4092;  // Max size of continuous SPI transfer

// store a 16 bit value in a buffer, big endian.
static inline void put16_be(uint8_t *buf, uint16_t value) {
  buf[0] = value >> 8;
  buf[1] = value;
}

void ILI9XXXDisplay::set_madctl() {
  // custom x/y transform and color order
  uint8_t mad = this->color_order_ == display::COLOR_ORDER_BGR ? MADCTL_BGR : MADCTL_RGB;
  if (this->swap_xy_)
    mad |= MADCTL_MV;
  if (this->mirror_x_)
    mad |= MADCTL_MX;
  if (this->mirror_y_)
    mad |= MADCTL_MY;
  this->command(ILI9XXX_MADCTL);
  this->data(mad);
  esph_log_d(TAG, "Wrote MADCTL 0x%02X", mad);
}

void ILI9XXXDisplay::setup() {
  ESP_LOGD(TAG, "Setting up ILI9xxx");

  this->setup_pins_();
  this->init_lcd_(this->init_sequence_);
  this->init_lcd_(this->extra_init_sequence_.data());
  
  this->command(ILI9XXX_PIXFMT);
  this->data(0x51); // RGB interface to 16bits as it does not support 3 bits
                    // MCU interface to 3 bits.
  this->is_18bitdisplay_ = false;
  this->is_3bitdisplay_ = true;


  this->set_madctl();
  this->command(this->pre_invertcolors_ ? ILI9XXX_INVON : ILI9XXX_INVOFF);
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}

void ILI9XXXDisplay::resetup() {
  ESP_LOGD(TAG, "Setting up ILI9xxx");

  //this->setup_pins_();
  this->init_lcd_(this->init_sequence_);
  this->init_lcd_(this->extra_init_sequence_.data());
  
  this->command(ILI9XXX_PIXFMT);
  this->data(0x51); // RGB interface to 16bits as it does not support 3 bits
                    // MCU interface to 3 bits.
  this->is_18bitdisplay_ = false;
  this->is_3bitdisplay_ = true;


  this->set_madctl();
  this->command(this->pre_invertcolors_ ? ILI9XXX_INVON : ILI9XXX_INVOFF);
  this->x_low_ = 0;
  this->y_low_ = 0;
  this->x_high_ = this->width_;
  this->y_high_ = this->height_;
}

void ILI9XXXDisplay::alloc_buffer_() {

  this->init_internal_(this->get_buffer_length_() >> 1);

  if (this->buffer_ == nullptr) {
    this->mark_failed();
  }
}

void ILI9XXXDisplay::setup_pins_() {
  this->dc_pin_->setup();  // OUTPUT
  this->dc_pin_->digital_write(false);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();  // OUTPUT
    this->reset_pin_->digital_write(true);
  }

  this->spi_setup();

  this->reset_();
}

void ILI9XXXDisplay::dump_config() {
  LOG_DISPLAY("", "ili9xxx", this);
  ESP_LOGCONFIG(TAG, "  Width Offset: %u", this->offset_x_);
  ESP_LOGCONFIG(TAG, "  Height Offset: %u", this->offset_y_);

  ESP_LOGCONFIG(TAG, "  3-Bit Mode");
  
  ESP_LOGCONFIG(TAG, "  Data rate: %dMHz", (unsigned) (this->data_rate_ / 1000000));

  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  ESP_LOGCONFIG(TAG, "  Color order: %s", this->color_order_ == display::COLOR_ORDER_BGR ? "BGR" : "RGB");
  ESP_LOGCONFIG(TAG, "  Swap_xy: %s", YESNO(this->swap_xy_));
  ESP_LOGCONFIG(TAG, "  Mirror_x: %s", YESNO(this->mirror_x_));
  ESP_LOGCONFIG(TAG, "  Mirror_y: %s", YESNO(this->mirror_y_));
  ESP_LOGCONFIG(TAG, "  Invert colors: %s", YESNO(this->pre_invertcolors_));

  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  => Failed to init Memory: YES!");
  }
  LOG_UPDATE_INTERVAL(this);
}

float ILI9XXXDisplay::get_setup_priority() const { return setup_priority::HARDWARE; }

void ILI9XXXDisplay::fill(Color color) {
  if (!this->check_buffer_())
    return;
  uint16_t new_color = 0;
  this->x_low_ = 0;
  this->y_low_ = 0;
  this->x_high_ = this->get_width_internal() - 1;
  this->y_high_ = this->get_height_internal() - 1;
  {
    uint8_t color3 = ((color.r >> 5) & 0x04) | ((color.g >> 6) & 0x02) | ((color.b >> 7) & 0x01);
    uint8_t packed = (color3 << 3) | color3; // Fill both 3-bit slots in byte;

    const uint32_t buffer_length_3_bits = this->get_buffer_length_() >> 1;
    memset(this->buffer_, packed, buffer_length_3_bits);
  }
  return;
}


void ILI9XXXDisplay::filled_rectangle_3bit(int x1, int y1, int width, int height, Color color) {
  if (!this->check_buffer_())
    return;
  if (width <= 0 || height <= 0)
    return;

  this->x_low_ = std::min((uint16_t) x1, this->x_low_);
  this->y_low_ = std::min((uint16_t)y1, this->y_low_);
  this->x_high_ = std::max((uint16_t)(x1+width-1), this->x_high_);
  this->y_high_ = std::max((uint16_t)(y1+height-1), this->y_high_);


  if (color != this->last_color)
  {
    this->last_color = color.raw_32;
    this->color_packed_right = ((color.r >> 5) & 4) | ((color.g >> 6) & 2) | (color.b >> 7);
    this->color_packed_left = (this->color_packed_right << 3);
    this->color_packed = this->color_packed_left | this->color_packed_right;
  }

  uint8_t x1_parity = x1 & 1;  // 0 for even, 1 for odd
  uint8_t length_parity = (width & 1) ^ x1_parity; // 0 for even, 1 for odd

  uint8_t width_bytes = (width >> 1) - length_parity;  // 2 pixels per byte
  uint8_t screen_width_bytes = this->get_width_internal() >> 1;

  uint8_t* buffer_start = (this->buffer_ + (y1 * screen_width_bytes) + (x1 >> 1)) ;
  uint8_t* buffer = buffer_start + x1_parity;

  for (uint16_t y = 0; y < height; y++) {
    memset(buffer, this->color_packed, width_bytes);
    buffer += screen_width_bytes;
  }
  buffer = buffer_start;
  for (uint16_t y = 0; x1_parity && y < height; y++) {
    *buffer = (*buffer & 0x38) | color_packed_right; // Clear upper 3 bits, set upper bits to color
    buffer += screen_width_bytes;
  }
  buffer = buffer_start + width_bytes + length_parity;
  for (uint16_t y = 0; x1_parity && !length_parity && y < height && x1_parity; y++) {
    *buffer = (*buffer & 0x07) | color_packed_left; // Clear upper 3 bits, set upper bits to color
    buffer += screen_width_bytes;
  }
}

void HOT ILI9XXXDisplay::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || x < 0 || y >= this->get_height_internal() || y < 0) {
    return;
  }
  if (!this->check_buffer_())
    return;

  uint32_t pos = ((y * width_) + x)>>1;

  if (color != this->last_color)
  {
    this->last_color = color.raw_32;
    this->color_packed_right = ((color.r >> 5) & 4) | ((color.g >> 6) & 2) | (color.b >> 7);
    this->color_packed_left = (this->color_packed_right << 3);
    this->color_packed = this->color_packed_left | this->color_packed_right;
  }
  uint8_t new_color = 0;
  bool updated = false;

  //check for parity
  if (!(x & 1)) {
      // Clear upper 3 bits, set upper bits to color
      new_color = (this->buffer_[pos] & 0x07) | this->color_packed_left;
  } else {
      // Clear lower 3 bits, set lower bits to color
      new_color = (this->buffer_[pos] & 0x38) | this->color_packed_right;
  }

  //always override, as the text print calls this method
  // values change often so just rewrite them
  this->buffer_[pos] = new_color;

  if (x < this->x_low_)
    this->x_low_ = x;
  if (y < this->y_low_)
    this->y_low_ = y;
  if (x > this->x_high_)
    this->x_high_ = x;
  if (y > this->y_high_)
    this->y_high_ = y;

  return;
}

void ILI9XXXDisplay::update() {
  if (this->prossing_update_) {
    this->need_update_ = true;
    return;
  }
  this->prossing_update_ = true;
  do {
    this->need_update_ = false;
    this->do_update_();
  } while (this->need_update_);
  this->prossing_update_ = false;
  this->display_();
}

void ILI9XXXDisplay::display_() {
  // check if something was displayed
  if ((this->x_high_ < this->x_low_) || (this->y_high_ < this->y_low_)) {
    return;
  }

  this->x_low_ = 0;
  this->y_low_ = 0;
  this->x_high_ = 479;
  this->y_high_ = 319;
  // we will only update the changed rows to the display
  size_t const w = this->x_high_ - this->x_low_ + 1;
  size_t const h = this->y_high_ - this->y_low_ + 1;

  ESP_LOGV(TAG,
           "Start display(xlow:%d, ylow:%d, xhigh:%d, yhigh:%d, width:%d, "
           "height:%zu)",
           this->x_low_, this->y_low_, this->x_high_, this->y_high_, w, h);
  auto now = millis();

  // buffer internal 3bit so direct write
  ESP_LOGV(TAG, "3bit mode, Doing single write of %zu bytes", (this->width_ * h) >> 1);
  set_addr_window_(0, this->y_low_, this->width_ - 1, this->y_high_);
  this->write_array(this->buffer_ + (this->y_low_ * (this->width_ >> 1)), (h * (this->width_>> 1)));
  // set_addr_window_(0, 0, 479, 319);
  // this->write_array(this->buffer_ + (this->y_low_ * (this->width_ >> 1)), (h * (this->width_>> 1)));

  this->end_data_();
  ESP_LOGV(TAG, "Data write took %dms", (unsigned) (millis() - now));
  // invalidate watermarks
  this->x_low_ = this->width_;
  this->y_low_ = this->height_;
  this->x_high_ = 0;
  this->y_high_ = 0;
}

// note that this bypasses the buffer and writes directly to the display.
void ILI9XXXDisplay::draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr,
                                    display::ColorOrder order, display::ColorBitness bitness, bool big_endian,
                                    int x_offset, int y_offset, int x_pad) {
  if (w <= 0 || h <= 0)
    return;
  
  // if color mapping or software rotation is required, hand this off to the parent implementation. This will
  // do color conversion pixel-by-pixel into the buffer and draw it later. If this is happening the user has not
  // configured the renderer well.
  display::Display::draw_pixels_at(x_start, y_start, w, h, ptr, order, bitness, big_endian, x_offset, y_offset,
                                     x_pad);
  return;
}

// should return the total size: return this->get_width_internal() * this->get_height_internal() * 2 // 16bit color
// values per bit is huge
uint32_t ILI9XXXDisplay::get_buffer_length_() { return this->get_width_internal() * this->get_height_internal(); }

void ILI9XXXDisplay::command(uint8_t value) {
  this->start_command_();
  this->write_byte(value);
  this->end_command_();
}

void ILI9XXXDisplay::data(uint8_t value) {
  this->start_data_();
  this->write_byte(value);
  this->end_data_();
}

void ILI9XXXDisplay::send_command(uint8_t command_byte, const uint8_t *data_bytes, uint8_t num_data_bytes) {
  this->command(command_byte);  // Send the command byte
  this->start_data_();
  this->write_array(data_bytes, num_data_bytes);
  this->end_data_();
}

void ILI9XXXDisplay::start_command_() {
  this->dc_pin_->digital_write(false);
  this->enable();
}
void ILI9XXXDisplay::start_data_() {
  this->dc_pin_->digital_write(true);
  this->enable();
}

void ILI9XXXDisplay::end_command_() { this->disable(); }
void ILI9XXXDisplay::end_data_() { this->disable(); }

void ILI9XXXDisplay::reset_() {
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(false);
    delay(20);
    this->reset_pin_->digital_write(true);
    delay(20);
  }
}

void ILI9XXXDisplay::init_lcd_(const uint8_t *addr) {
  if (addr == nullptr)
    return;
  uint8_t cmd, x, num_args;
  while ((cmd = *addr++) != 0) {
    x = *addr++;
    if (x == ILI9XXX_DELAY_FLAG) {
      cmd &= 0x7F;
      ESP_LOGV(TAG, "Delay %dms", cmd);
      delay(cmd);
    } else {
      num_args = x & 0x7F;
      ESP_LOGV(TAG, "Command %02X, length %d, bits %02X", cmd, num_args, *addr);
      this->send_command(cmd, addr, num_args);
      addr += num_args;
      if (x & 0x80) {
        ESP_LOGV(TAG, "Delay 150ms");
        delay(150);  // NOLINT
      }
    }
  }
}

// Tell the display controller where we want to draw pixels.
void ILI9XXXDisplay::set_addr_window_(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  x1 += this->offset_x_;
  x2 += this->offset_x_;
  y1 += this->offset_y_;
  y2 += this->offset_y_;
  this->command(ILI9XXX_CASET);
  this->data(x1 >> 8);
  this->data(x1 & 0xFF);
  this->data(x2 >> 8);
  this->data(x2 & 0xFF);
  this->command(ILI9XXX_PASET);  // Page address set
  this->data(y1 >> 8);
  this->data(y1 & 0xFF);
  this->data(y2 >> 8);
  this->data(y2 & 0xFF);
  this->command(ILI9XXX_RAMWR);  // Write to RAM
  this->start_data_();
}

void ILI9XXXDisplay::invert_colors(bool invert) {
  this->pre_invertcolors_ = invert;
  if (is_ready()) {
    this->command(invert ? ILI9XXX_INVON : ILI9XXX_INVOFF);
  }
}

int ILI9XXXDisplay::get_width_internal() { return this->width_; }
int ILI9XXXDisplay::get_height_internal() { return this->height_; }

}  // namespace ili9xxx
}  // namespace esphome
