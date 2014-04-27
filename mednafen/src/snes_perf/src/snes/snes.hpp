namespace SNES {
  namespace Info {
    static const char Name[] = "bsnes";
    static const char Version[] = "073";
    static const unsigned SerializerVersion = 15;
  }
}

//#define DEBUGGER
#undef DEBUGGER
//#define CHEAT_SYSTEM
#undef CHEAT_SYSTEM

#include <libco/libco.h>

#include <nall/algorithm.hpp>
#include <nall/array.hpp>
#include <nall/detect.hpp>
#include <nall/dl.hpp>
#include <nall/endian.hpp>
#include <nall/file.hpp>
#include <nall/foreach.hpp>
#include <nall/function.hpp>
#include <nall/moduloarray.hpp>
#include <nall/platform.hpp>
#include <nall/priorityqueue.hpp>
#include <nall/property.hpp>
#include <nall/serializer.hpp>
#include <nall/stdint.hpp>
#include <nall/string.hpp>
#include <nall/utility.hpp>
#include <nall/varint.hpp>
#include <nall/vector.hpp>
using namespace nall;

#ifdef DEBUGGER
  #define debugvirtual virtual
#else
  #define debugvirtual
#endif

namespace SNES {
  typedef int8_t   int8;
  typedef int16_t  int16;
  typedef int32_t  int32;
  typedef int64_t  int64;

  typedef uint8_t  uint8;
  typedef uint16_t uint16;
  typedef uint32_t uint32;
  typedef uint64_t uint64;

  typedef uint_t< 1> uint1;
  typedef uint_t< 2> uint2;
  typedef uint_t< 3> uint3;
  typedef uint_t< 4> uint4;
  typedef uint_t< 5> uint5;
  typedef uint_t< 6> uint6;
  typedef uint_t< 7> uint7;

  typedef uint_t< 9> uint9;
  typedef uint_t<10> uint10;
  typedef uint_t<11> uint11;
  typedef uint_t<12> uint12;
  typedef uint_t<13> uint13;
  typedef uint_t<14> uint14;
  typedef uint_t<15> uint15;

  typedef uint_t<17> uint17;
  typedef uint_t<18> uint18;
  typedef uint_t<19> uint19;
  typedef uint_t<20> uint20;
  typedef uint_t<21> uint21;
  typedef uint_t<22> uint22;
  typedef uint_t<23> uint23;
  typedef uint_t<24> uint24;
  typedef uint_t<25> uint25;
  typedef uint_t<26> uint26;
  typedef uint_t<27> uint27;
  typedef uint_t<28> uint28;
  typedef uint_t<29> uint29;
  typedef uint_t<30> uint30;
  typedef uint_t<31> uint31;

  typedef uint_t<40> uint40;
  typedef uint_t<48> uint48;
  typedef uint_t<56> uint56;

  struct Processor {
    cothread_t thread;
    unsigned frequency;
    int64 clock;

    inline void create(void (*entrypoint_)(), unsigned frequency_) NALL_COLD {
      if(thread) co_delete(thread);
      thread = co_create(65536 * sizeof(void*), entrypoint_);
      frequency = frequency_;
      clock = 0;
    }

    inline void serialize(serializer &s) NALL_COLD {
      s.integer(frequency);
      s.integer(clock);
    }

    inline Processor() : thread(0) {}
  };

  struct ChipDebugger {
    virtual bool property(unsigned id, string &name, string &value) = 0;
  };

  #include <memory/memory.hpp>
  #include <cpu/core/core.hpp>
  //#include <smp/core/core.hpp>
  #include <ppu/counter/counter.hpp>

  #if defined(PROFILE_ACCURACY)
  #include "profile-accuracy.hpp"
  #elif defined(PROFILE_COMPATIBILITY)
  #include "profile-compatibility.hpp"
  #elif defined(PROFILE_PERFORMANCE)
  #include "profile-performance.hpp"
  #endif

  #include <system/system.hpp>
  #include <chip/chip.hpp>
  #include <cartridge/cartridge.hpp>
  #include <cheat/cheat.hpp>

  #include <memory/memory-inline.hpp>
  #include <ppu/counter/counter-inline.hpp>
  #include <cheat/cheat-inline.hpp>
}

namespace nall {
  template<> struct has_size<SNES::MappedRAM> { enum { value = true }; };
  template<> struct has_size<SNES::StaticRAM> { enum { value = true }; };
}

#undef debugvirtual
