class Video {
private:
  bool frame_hires;
  bool frame_interlace;
  unsigned line_width[240];

  void update();
  void scanline();
  void init() NALL_COLD;

  static const uint8_t cursor[15 * 15];
  void draw_cursor(uint16_t color, int x, int y);

  friend class System;
};

extern Video video;
