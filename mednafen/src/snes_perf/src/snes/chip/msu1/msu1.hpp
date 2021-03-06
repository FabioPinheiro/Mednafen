class MSU1 : public Coprocessor, public MMIO {
public:
  static void Enter();
  void enter();
  void init() NALL_COLD;
  void enable();
  void power() NALL_COLD;
  void reset() NALL_COLD;

  uint8 mmio_read(unsigned addr);
  void mmio_write(unsigned addr, uint8 data);

  void serialize(serializer&) NALL_COLD;

private:
  file datafile;
  file audiofile;

  enum Flag {
    DataBusy       = 0x80,
    AudioBusy      = 0x40,
    AudioRepeating = 0x20,
    AudioPlaying   = 0x10,
    Revision       = 0x01,
  };

  struct MMIO {
    uint32 data_offset;
    uint32 audio_offset;
    uint32 audio_loop_offset;

    uint16 audio_track;
    uint8 audio_volume;

    bool data_busy;
    bool audio_busy;
    bool audio_repeat;
    bool audio_play;
  } mmio;
};

extern MSU1 msu1;
