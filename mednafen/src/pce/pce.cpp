/* Mednafen - Multi-system Emulator
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "pce.h"
#include "vce.h"
#include "pce_psg/pce_psg.h"
#include "input.h"
#include "huc.h"
#include "subhw.h"
#include "pcecd.h"
#include "../cdrom/scsicd.h"
#include "hes.h"
#include "debug.h"
#include "tsushin.h"
#include "arcade_card/arcade_card.h"
#include "../mempatcher.h"
#include "../cdrom/cdromif.h"
#include "../md5.h"
#include "../FileStream.h"
#include "../sound/OwlResampler.h"
#include <math.h>

#define PCE_DEBUG(x, ...) {  /* printf(x, ## __VA_ARGS__); */ }

namespace MDFN_IEN_PCE
{

static const MDFNSetting_EnumList PSGRevisionList[] =
{
 { "huc6280", PCE_PSG::REVISION_HUC6280, "HuC6280", gettext_noop("HuC6280 as found in the original PC Engine.") },
 { "huc6280a", PCE_PSG::REVISION_HUC6280A, "HuC6280A", gettext_noop("HuC6280A as found in the SuperGrafx and CoreGrafx I.  Provides proper channel amplitude centering, but may cause clicking in a few games designed with the original HuC6280's sound characteristics in mind.") },
 { "match", PCE_PSG::_REVISION_COUNT, gettext_noop("Match emulation mode."), gettext_noop("Selects \"huc6280\" for non-SuperGrafx mode, and \"huc6280a\" for SuperGrafx(full) mode.") },
 { NULL, 0 },
};

static std::vector<CDIF*> *cdifs = NULL;
static bool CD_TrayOpen;
static int CD_SelectedDisc;	// -1 for no disc

HuC6280 *HuCPU;

VCE *vce = NULL;

static PCE_PSG *psg = NULL;

extern ArcadeCard *arcade_card;	// Bah, lousy globals.

static OwlBuffer* HRBufs[2] = { NULL, NULL };
static RavenBuffer* ADPCMBuf = NULL;
static RavenBuffer* CDDABufs[2] = { NULL, NULL };
static OwlResampler* HRRes = NULL;

static bool SetSoundRate(double rate);


bool PCE_ACEnabled;
uint32 PCE_InDebug = 0;
uint64 PCE_TimestampBase;	// Only used with the debugger for the time being.

extern MDFNGI EmulatedPCE;
static bool IsSGX;
static bool IsHES;

// Accessed in debug.cpp
static uint8 BaseRAM[32768]; // 8KB for PCE, 32KB for Super Grafx
uint8 PCE_PeekMainRAM(uint32 A)
{
 return BaseRAM[A & ((IsSGX ? 32768 : 8192) - 1)];
}

void PCE_PokeMainRAM(uint32 A, uint8 V)
{
 BaseRAM[A & ((IsSGX ? 32768 : 8192) - 1)] = V;
}



HuC6280::readfunc NonCheatPCERead[0x100];

static DECLFR(PCEBusRead)
{
 if(!PCE_InDebug)
 {
  PCE_DEBUG("Unmapped Read: %02x %04x\n", A >> 13, A);
 }
 return(0xFF);
}

static DECLFW(PCENullWrite)
{
 if(!PCE_InDebug)
 {
  PCE_DEBUG("Unmapped Write: %02x, %08x %02x\n", A >> 13, A, V);
 }
}

static DECLFR(BaseRAMReadSGX)
{
 return(BaseRAM[A & 0x7FFF]);
}

static DECLFW(BaseRAMWriteSGX)
{
 BaseRAM[A & 0x7FFF] = V;
}

static DECLFR(BaseRAMRead)
{
 return(BaseRAM[A & 0x1FFF]);
}

static DECLFW(BaseRAMWrite)
{
 BaseRAM[A & 0x1FFF] = V;
}

static DECLFR(IORead)
{
 A &= 0x1FFF;

 switch(A & 0x1c00)
 {
  case 0x0000: if(!PCE_InDebug)
		HuCPU->StealCycle(); 
	       return(vce->ReadVDC(A));

  case 0x0400: if(!PCE_InDebug)
		HuCPU->StealCycle(); 
	       return(vce->Read(A));

  case 0x0800: if(HuCPU->InBlockMove())
		return(0);
	       return(HuCPU->GetIODataBuffer());

  case 0x0c00: if(HuCPU->InBlockMove())
                return(0);
	       {
		uint8 ret = HuCPU->TimerRead(A, PCE_InDebug);
                if(!PCE_InDebug)
		 HuCPU->SetIODataBuffer(ret);
                return(ret);
               }

  case 0x1000: if(HuCPU->InBlockMove())
                return(0);
	       {
	        uint8 ret = INPUT_Read(HuCPU->Timestamp(), A);
                if(!PCE_InDebug)
		 HuCPU->SetIODataBuffer(ret);
                return(ret);
               }

  case 0x1400: if(HuCPU->InBlockMove())
                return(0);
	       {
	        uint8 ret = HuCPU->IRQStatusRead(A, PCE_InDebug);
		if(!PCE_InDebug)
		 HuCPU->SetIODataBuffer(ret);
	        return(ret);
	       }

  case 0x1800: if(IsTsushin)
		return(PCE_TsushinRead(A));

	       if(!PCE_IsCD)
		break;
	       if((A & 0x1E00) == 0x1A00)
	       {
		if(arcade_card)
		 return(arcade_card->Read(A, PCE_InDebug));
		else
		 return(0);
	       }
	       else
	       {
		int32 next_cd_event;
		uint8 ret;

		ret = PCECD_Read(HuCPU->Timestamp(), A, next_cd_event, PCE_InDebug);

		vce->SetCDEvent(next_cd_event);

		return(ret);
	       }

  case 0x1C00: if(IsHES)
		return(ReadIBP(A)); 
	       return(SubHW_ReadIOPage(A));
	       break; // Expansion
 }

 if(!PCE_InDebug)
 {
  PCE_DEBUG("I/O Unmapped Read: %04x\n", A);
 }

 return(0xFF);
}

static DECLFW(IOWrite)
{
 switch(A & 0x1c00)
 {
  case 0x0000: HuCPU->StealCycle();
	       vce->WriteVDC(A & 0x80001FFF, V);
	       break;

  case 0x0400: HuCPU->StealCycle(); 
	       vce->Write(A & 0x1FFF, V);
	       break;

  case 0x0800: HuCPU->SetIODataBuffer(V); 
	       psg->Write(HuCPU->Timestamp() / 3, A & 0x1FFF, V);
	       break;

  case 0x0c00: HuCPU->SetIODataBuffer(V);
	       HuCPU->TimerWrite(A & 0x1FFF, V);
	       break;

  case 0x1000: HuCPU->SetIODataBuffer(V);
	       INPUT_Write(HuCPU->Timestamp(), A & 0x1FFF, V);
	       break;

  case 0x1400: HuCPU->SetIODataBuffer(V);
	       HuCPU->IRQStatusWrite(A & 0x1FFF, V);
	       break;

  case 0x1800: if(IsTsushin)
                PCE_TsushinWrite(A & 0x1FFF, V);

	       if(!PCE_IsCD)
	       {
		if(!PCE_InDebug)
		{
		 PCE_DEBUG("I/O Unmapped Write: %04x %02x\n", A, V);
		}
		break;
	       }

	       if((A & 0x1E00) == 0x1A00)
	       {
		if(arcade_card)
		 arcade_card->Write(A& 0x1FFF, V);
	       }
	       else
	       {
	        int32 next_cd_event = PCECD_Write(HuCPU->Timestamp(), A & 0x1FFF, V);

		vce->SetCDEvent(next_cd_event);
	       }

	       break;

  case 0x1C00:  //if(!PCE_InDebug)
		//{
		// PCE_DEBUG("I/O Unmapped Write: %04x %02x\n", A, V);
		//}
		SubHW_WriteIOPage(A, V);
		break;
 }
}

static void PCECDIRQCB(bool asserted)
{
 if(asserted)
  HuCPU->IRQBegin(HuC6280::IQIRQ2);
 else
  HuCPU->IRQEnd(HuC6280::IQIRQ2);
}

static bool LoadCustomPalette(const char *path)
{
 MDFN_printf(_("Loading custom palette from \"%s\"...\n"),  path);
 MDFN_indent(1);

 try
 {
  FileStream fp(path, FileStream::MODE_READ);
  uint8 CustomColorMap[1024 * 3];
  uint32 CustomColorMapLen = 0;

  fp.read(CustomColorMap, 512 * 3, true);
  if(fp.read(CustomColorMap, 512 * 3, false) == 512 * 3)
   CustomColorMapLen = 1024;
  else
  {
   MDFN_printf(_("Palette is missing the full set of 512 greyscale entries.  Strip-colorburst entries will be calculated.\n"));
   CustomColorMapLen = 512;
  }

  vce->SetCustomColorMap(CustomColorMap, CustomColorMapLen);
 }
 catch(std::exception &e)
 {
  MDFN_printf("%s\n", e.what());
  MDFN_indent(-1);
  return(false);
 }

 MDFN_indent(-1);

 return(true);
}

static int LoadCommon(void);
static void LoadCommonPre(void);

static bool TestMagic(const char *name, MDFNFILE *fp)
{
 if(memcmp(fp->data, "HESM", 4) && strcasecmp(fp->ext, "pce") && strcasecmp(fp->ext, "sgx"))
  return(FALSE);

 return(TRUE);
}

static void SetCDSettings(bool silent_status = false)
{
 PCECD_Settings cd_settings;
 memset(&cd_settings, 0, sizeof(PCECD_Settings));

 cd_settings.CDDA_Volume = (double)MDFN_GetSettingUI("pce.cddavolume") / 100;
 cd_settings.ADPCM_Volume = (double)MDFN_GetSettingUI("pce.adpcmvolume") / 100;
 cd_settings.ADPCM_ExtraPrecision = MDFN_GetSettingB("pce.adpcmextraprec");

 PCECD_SetSettings(&cd_settings);

 if(!silent_status)
 {
  if(cd_settings.CDDA_Volume != 1.0)
   MDFN_printf(_("CD-DA Volume: %d%%\n"), (int)(100 * cd_settings.CDDA_Volume));

  if(cd_settings.ADPCM_Volume != 1.0)
   MDFN_printf(_("ADPCM Volume: %d%%\n"), (int)(100 * cd_settings.ADPCM_Volume));
 }


 unsigned int cdpsgvolume = MDFN_GetSettingUI("pce.cdpsgvolume");

 if(cdpsgvolume != 100)
 {
  MDFN_printf(_("CD PSG Volume: %d%%\n"), cdpsgvolume);
 }

 psg->SetVolume(0.678 * cdpsgvolume / 100);
}

static void CDSettingChanged(const char *name)
{
 SetCDSettings(true);
}

static int Load(const char *name, MDFNFILE *fp)
{
 uint32 headerlen = 0;
 uint32 r_size;

 IsHES = 0;
 IsSGX = 0;

 if(!memcmp(fp->data, "HESM", 4))
  IsHES = 1;

 LoadCommonPre();

 if(!IsHES)
 {
  if(fp->size & 0x200) // 512 byte header!
   headerlen = 512;
 }

 r_size = fp->size - headerlen;
 if(r_size > 4096 * 1024) r_size = 4096 * 1024;

 uint32 crc = crc32(0, fp->data + headerlen, fp->size - headerlen);


 if(IsHES)
 {
  if(!PCE_HESLoad(fp->data, fp->size))
   return(0);

  ADPCMBuf = new RavenBuffer();
  PCE_IsCD = 1;
  PCECD_Init(NULL, PCECDIRQCB, PCE_MASTER_CLOCK, ADPCMBuf->Buf(), NULL, NULL);
 }
 else
 {
  HuCLoad(fp->data + headerlen, fp->size - headerlen, crc, MDFN_GetSettingB("pce.disable_bram_hucard"));
  #if 0	// For testing
  PCE_IsCD = 1;
  PCECD_Init(NULL, PCECDIRQCB, PCE_MASTER_CLOCK, &sbuf[0], &sbuf[1]);
  #endif
 }
 if(!strcasecmp(fp->ext, "sgx"))
  IsSGX = TRUE;

 if(fp->size >= 8192 && !memcmp(fp->data + headerlen, "DARIUS Version 1.11b", strlen("DARIUS VERSION 1.11b")))
 {
  MDFN_printf("SuperGfx:  Darius Plus\n");
  IsSGX = 1;
 }

 if(crc == 0x4c2126b0)
 {
  MDFN_printf("SuperGfx:  Aldynes\n");
  IsSGX = 1;
 }

 if(crc == 0x8c4588e2)
 {
  MDFN_printf("SuperGfx:  1941 - Counter Attack\n");
  IsSGX = 1;
 }
 if(crc == 0x1f041166)
 {
  MDFN_printf("SuperGfx:  Madouou Granzort\n");
  IsSGX = 1;
 }
 if(crc == 0xb486a8ed)
 {
  MDFN_printf("SuperGfx:  Daimakaimura\n");
  IsSGX = 1;
 }
 if(crc == 0x3b13af61)
 {
  MDFN_printf("SuperGfx:  Battle Ace\n");
  IsSGX = 1;
 }

 return(LoadCommon());
}

static void LoadCommonPre(void)
{
 // Initialize sound buffers
 for(unsigned ch = 0; ch < 2; ch++)
  HRBufs[ch] = new OwlBuffer();

 // FIXME:  Make these globals less global!
 PCE_ACEnabled = MDFN_GetSettingB("pce.arcadecard");

 HuCPU = new HuC6280(IsHES);

 for(int x = 0; x < 0x100; x++)
 {
  HuCPU->SetFastRead(x, NULL);
  HuCPU->SetReadHandler(x, PCEBusRead);
  HuCPU->SetWriteHandler(x, PCENullWrite);
 }

 MDFNMP_Init(1024, (1 << 21) / 1024);
}

static int LoadCommon(void)
{ 
 IsSGX |= MDFN_GetSettingB("pce.forcesgx") ? 1 : 0;

 if(IsHES)
  IsSGX = 1;
 // Don't modify IsSGX past this point.
 
 vce = new VCE(IsSGX, MDFN_GetSettingB("pce.nospritelimit"));

 if(IsSGX)
  MDFN_printf("SuperGrafx Emulation Enabled.\n");

 for(int i = 0xF8; i < 0xFC; i++)
 {
  HuCPU->SetReadHandler(i, IsSGX ? BaseRAMReadSGX : BaseRAMRead);
  HuCPU->SetWriteHandler(i, IsSGX ? BaseRAMWriteSGX : BaseRAMWrite);

  if(IsSGX)
   HuCPU->SetFastRead(i, BaseRAM + (i & 0x3) * 8192);
  else
   HuCPU->SetFastRead(i, BaseRAM);
 }

 LoadCustomPalette(MDFN_MakeFName(MDFNMKF_PALETTE, 0, NULL).c_str());

 MDFNMP_AddRAM(IsSGX ? 32768 : 8192, 0xf8 * 8192, BaseRAM);

 HuCPU->SetReadHandler(0xFF, IORead);
 HuCPU->SetWriteHandler(0xFF, IOWrite);

 {
  int psgrevision = MDFN_GetSettingI("pce.psgrevision");

  if(psgrevision == PCE_PSG::_REVISION_COUNT)
  {
   psgrevision = IsSGX ? PCE_PSG::REVISION_HUC6280A : PCE_PSG::REVISION_HUC6280;
  }

  for(const MDFNSetting_EnumList *el = PSGRevisionList; el->string; el++)
  {
   if(el->number == psgrevision)
   {
    MDFN_printf(_("PSG Revision: %s\n"), el->description);
    break;
   }
  }
  psg = new PCE_PSG(HRBufs[0]->Buf(), HRBufs[1]->Buf(), psgrevision);
 }

 psg->SetVolume(1.0);

 if(PCE_IsCD)
  SetCDSettings();

 PCEINPUT_Init();

 //
 //
 //
 SubHW_Init();
 HuCPU->SetReadHandler(0xFE, SubHW_ReadFEPage);
 HuCPU->SetWriteHandler(0xFE, SubHW_WriteFEPage);
 //
 //
 //

 PCE_Power();

 //
 // REMOVE ME, debugging stuff
 //
 #if 0
 {
  uint8 er[8] = { 0x47, 0x6E, 0x31, 0x14, 0xB3, 0xEB, 0xEC, 0x2B };

  for(int i = 0; i < 8; i++)
   SubHW_WriteIOPage(0x1C00, er[i]);

  SubHW_WriteIOPage(0x1FF7, 0x80);
 }
 #endif
 //
 //
 //

 MDFNGameInfo->LayerNames = IsSGX ? "BG0\0SPR0\0BG1\0SPR1\0" : "Background\0Sprites\0";
 MDFNGameInfo->fps = (uint32)((double)7159090.90909090 / 455 / 263 * 65536 * 256);


 for(unsigned int i = 0; i < 0x100; i++)
  NonCheatPCERead[i] = HuCPU->GetReadHandler(i);

 if(!IsHES)
 {
  MDFNGameInfo->nominal_height = MDFN_GetSettingUI("pce.slend") - MDFN_GetSettingUI("pce.slstart") + 1;
  MDFNGameInfo->nominal_width = MDFN_GetSettingB("pce.h_overscan") ? 320 : 288;

  MDFNGameInfo->lcm_width = MDFN_GetSettingB("pce.h_overscan") ? 1120 : 1024;
  MDFNGameInfo->lcm_height = MDFNGameInfo->nominal_height;
 }

 vce->SetShowHorizOS(MDFN_GetSettingB("pce.h_overscan")); 

#ifdef WANT_DEBUGGER
 PCEDBG_Init(IsSGX, psg);
#endif


 return(1);
}

static bool DetectGECD(CDIF *cdiface)	// Very half-assed detection until(if) we get ISO-9660 reading code.
{
 uint8 sector_buffer[2048];
 CDUtility::TOC toc;

 cdiface->ReadTOC(&toc);

 // Now, test for the Games Express CD games.  The GE BIOS seems to always look at sector 0x10, but only if the first track is a
 // data track.
 if(toc.first_track == 1 && (toc.tracks[1].control & 0x4))
 {
  if(cdiface->ReadSector(sector_buffer, 0x10, 1) == 0x1)
  {
   if(!memcmp((char *)sector_buffer + 0x8, "HACKER CD ROM SYSTEM", 0x14))
    return(true);

   if(!memcmp((char *)sector_buffer + 0x01, "CD001", 0x5))
   {
    if(cdiface->ReadSector(sector_buffer, 0x14, 1) == 0x1)
    {
     static const uint32 known_crcs[] =
     {
      0xd7b47c06,	// AV Tanjou
      0x86aec522,	// Bishoujo Jyanshi [...]
      0xc8d1b5ef,	// CD Bishoujo [...]
      0x0bdbde64,	// CD Pachisuro [...]
     };
     uint32 zecrc = crc32(0, sector_buffer, 2048);

     //printf("%04x\n", zecrc);
     for(unsigned int i = 0; i < sizeof(known_crcs) / sizeof(uint32); i++)
      if(known_crcs[i] == zecrc)
       return(true);
    }
   }
  }
 }

 return(false);
}

static bool TestMagicCD(std::vector<CDIF *> *CDInterfaces)
{
 static const uint8 magic_test[0x20] = { 0x82, 0xB1, 0x82, 0xCC, 0x83, 0x76, 0x83, 0x8D, 0x83, 0x4F, 0x83, 0x89, 0x83, 0x80, 0x82, 0xCC,  
				   	 0x92, 0x98, 0x8D, 0xEC, 0x8C, 0xA0, 0x82, 0xCD, 0x8A, 0x94, 0x8E, 0xAE, 0x89, 0xEF, 0x8E, 0xD0
				       };
 CDIF *cdiface = (*CDInterfaces)[0];
 uint8 sector_buffer[2048];
 CDUtility::TOC toc;
 bool ret = FALSE;

 memset(sector_buffer, 0, sizeof(sector_buffer));

 cdiface->ReadTOC(&toc);

 for(int32 track = toc.first_track; track <= toc.last_track; track++)
 {
  if(toc.tracks[track].control & 0x4)
  {
   if(cdiface->ReadSector(sector_buffer, toc.tracks[track].lba, 1) != 0x1)
    break;

   if(!memcmp((char*)sector_buffer, (char *)magic_test, 0x20))
    ret = TRUE;

   // PCE CD BIOS apparently only looks at the first data track.
   break;
  }
 }

 // If it's a PC-FX CD(Battle Heat), return false.
 // This is very kludgy.
 for(int32 track = toc.first_track; track <= toc.last_track; track++)
 {
  if(toc.tracks[track].control & 0x4)
  {
   cdiface->ReadSector(sector_buffer, toc.tracks[track].lba, 1);
   if(!strncmp("PC-FX:Hu_CD-ROM", (char*)sector_buffer, strlen("PC-FX:Hu_CD-ROM")))
   {
    return(false);
   }
  }
 }

 if(DetectGECD(cdiface))
  ret = true;

 return(ret);
}

static int LoadCD(std::vector<CDIF *> *CDInterfaces)
{
 static const FileExtensionSpecStruct KnownBIOSExtensions[] =
 {
  { ".pce", gettext_noop("PC Engine ROM Image") },
  { ".bin", gettext_noop("PC Engine ROM Image") },
  { ".bios", gettext_noop("BIOS Image") },
  { NULL, NULL }
 };
 md5_context md5;
 uint32 headerlen = 0;

 MDFNFILE fp;

 IsHES = 0;
 IsSGX = 0;

 LoadCommonPre();

 const char *bios_sname = DetectGECD((*CDInterfaces)[0]) ? "pce.gecdbios" : "pce.cdbios";
 std::string bios_path = MDFN_MakeFName(MDFNMKF_FIRMWARE, 0, MDFN_GetSettingS(bios_sname).c_str() );

 if(!fp.Open(bios_path, KnownBIOSExtensions, _("CD BIOS")))
 {
  return(0);
 }

 if(fp.Size() & 0x200)
  headerlen = 512;


 //md5.starts();
 //md5.update(fp.Data(), fp.Size());
 //md5.update(MDFNGameInfo->MD5, 16);
 //md5.finish(MDFNGameInfo->MD5);

 bool disable_bram_cd = MDFN_GetSettingB("pce.disable_bram_cd");

 if(disable_bram_cd)
  MDFN_printf(_("Warning: BRAM is disabled per pcfx.disable_bram_cd setting.  This is simulating a malfunction.\n"));

 if(!HuCLoad(fp.Data() + headerlen, fp.Size() - headerlen, 0, disable_bram_cd, PCE_ACEnabled ? SYSCARD_ARCADE : SYSCARD_3))
 {
  return(0);
 }

 ADPCMBuf = new RavenBuffer();
 for(unsigned lr = 0; lr < 2; lr++)
  CDDABufs[lr] = new RavenBuffer();

 PCE_IsCD = 1;

 if(!PCECD_Init(NULL, PCECDIRQCB, PCE_MASTER_CLOCK, ADPCMBuf->Buf(), CDDABufs[0]->Buf(), CDDABufs[1]->Buf()))
 {
  HuCClose();
  return(0);
 }

 MDFNGameInfo->GameType = GMT_CDROM;
 CD_TrayOpen = false;
 CD_SelectedDisc = 0;
 cdifs = CDInterfaces;

 SCSICD_SetDisc(true, NULL, true);
 SCSICD_SetDisc(false, (*CDInterfaces)[0], true);


 MDFN_printf(_("CD Layout:   0x%s\n"), md5_context::asciistr(MDFNGameInfo->MD5, 0).c_str());
 MDFN_printf(_("Arcade Card Emulation:  %s\n"), PCE_ACEnabled ? _("Enabled") : _("Disabled"));

 return(LoadCommon());
}


static void CloseGame(void)
{
 if(PCE_IsCD)
 {
  PCECD_Close();
 }

 if(IsHES)
  HES_Close();
 else
 {
  HuCClose();
 }

 if(vce)
 {
  delete vce;
  vce = NULL;
 }

 if(psg)
 {
  delete psg;
  psg = NULL;
 }

 if(HuCPU)
 {
  delete HuCPU;
  HuCPU = NULL;
 }

 for(unsigned ch = 0; ch < 2; ch++)
 {
  if(HRBufs[ch])
  {
   delete HRBufs[ch];
   HRBufs[ch] = NULL;
  }

  if(CDDABufs[ch])
  {
   delete CDDABufs[ch];
   CDDABufs[ch] = NULL;
  }
 }

 if(ADPCMBuf)
 {
  delete ADPCMBuf;
  ADPCMBuf = NULL;
 }

 if(HRRes)
 {
  delete HRRes;
  HRRes = NULL;
 }
}

#if 0
void TestThing(unsigned count)
{
 int poo = 0;

 //for(int x = 0; x < count; x++)
 {
  //static const double pinc = 44100.0 / 1789772.7272;
  //static double p = 0;

  //p += pinc;
  //if(p >= 1.0)
  //{
  // p -= 1.0;
  static const uint32 ptv = (int64)(1024 * 1024) * 21477272 / 44100;
  static int64 p = 0;

  //if(x == (count - 1) || 1) //!(rand() & 1))
  //{
  // p -= (12 * (x - poo)) << 20;
  // poo = x;
  //}
  //p -= (12 << 20);
  int x = count;

  p -= ((int64)12 * x) << 20;

  while(p <= 0)
  {
   //const int synthtime = x;
   //const int synthtime_phase = ((((1 << 20) + p) >> 4) - 0x80;
   //const int synthtime_phase_int = synthtime_phase >> 8;
   //const int synthtime_phase_fract = synthtime_phase & 0xFF;
   const uint64 synthtime_ex = ((((uint64)((x + 1) * 12) << 20) + p) / 3) >> (4 + 2);
   const int synthtime = synthtime_ex >> 16;
   const int synthtime_phase = (int)(synthtime_ex & 0xFFFF) - 0x80;
   const int synthtime_phase_int = synthtime_phase >> 8;
   const int synthtime_phase_fract = synthtime_phase & 0xFF;
   static unsigned counter = 0;
   static int32 wv = 0x6000;

   //printf("%d %d %lld\n", x, synthtime, ptv);

   p += ptv;

   if(!counter)
   {
    wv = -wv;
    counter = 8;
   }
   else
    counter--;

//   HRBufs[0][HRBUF_LEFTOVER_PADDING + x + 0] += wv * 41 * 256;
//   HRBufs[0][HRBUF_LEFTOVER_PADDING + x + 1] += 0 - (wv * 41 * 256);
#if 1
   for(unsigned ch = 0; ch < 2; ch++)
   {
    int32 prev = 0;
    for(unsigned c = 0; c < CDDA_Filter_NumConvolutions; c++)
    {
     int32 coeff;
     int32 mr;

     //coeff = CDDA_Filter[1 + synthtime_phase_int + 0][c];
     coeff = (CDDA_Filter[1 + synthtime_phase_int + 0][c] * (256 - synthtime_phase_fract) +
              CDDA_Filter[1 + synthtime_phase_int + 1][c] * (synthtime_phase_fract)) >> 8;

     mr = (wv * coeff) >> (16 - 6 - 8);
     HRBufs[ch][HRBUF_LEFTOVER_PADDING + synthtime + c] += mr - prev;
     prev = mr;
    }
    HRBufs[ch][HRBUF_LEFTOVER_PADDING + synthtime + CDDA_Filter_NumConvolutions] += 0 - prev;
   }
#endif
  }
 }

#if 0
 for(unsigned x = 0; x < count; x++)
 {
  HRBufs[0][HRBUF_LEFTOVER_PADDING + x] = 0;
  HRBufs[1][HRBUF_LEFTOVER_PADDING + x] = 0;
 }
#endif
}
#endif

static EmulateSpecStruct *es;
static void Emulate(EmulateSpecStruct *espec)
{
 es = espec;

 espec->MasterCycles = 0;
 espec->SoundBufSize = 0;

 MDFNMP_ApplyPeriodicCheats();

 if(espec->VideoFormatChanged)
  vce->SetPixelFormat(espec->surface->format);

 if(espec->SoundFormatChanged)
  SetSoundRate(espec->SoundRate);

 //int t = MDFND_GetTime();

 vce->StartFrame(espec->surface, &espec->DisplayRect, espec->LineWidths, IsHES ? 1 : espec->skip);

 // Begin loop here:
 //for(int i = 0; i < 2; i++)
 bool rp_rv;
 do
 {
  assert(HuCPU->Timestamp() < 12);
  //printf("ST: %d\n", HuCPU->Timestamp());

  INPUT_Frame();

  //vce->RunFrame(espec->surface, &espec->DisplayRect, espec->LineWidths, IsHES ? 1 : espec->skip);
  rp_rv = vce->RunPartial();

  const uint32 end_timestamp = HuCPU->Timestamp();
  const uint32 end_timestamp_div12 = end_timestamp / 12;
  const uint32 end_timestamp_mod12 = end_timestamp % 12;

  INPUT_AdjustTS((int32)end_timestamp_mod12 - (int32)end_timestamp);	// Careful with this!

  SubHW_EndFrame(end_timestamp, end_timestamp_mod12);

  psg->Update(end_timestamp / 3);
  psg->ResetTS(end_timestamp_mod12 / 3);

  HuC_Update(end_timestamp);
  HuC_ResetTS(end_timestamp_mod12);

  {
   const unsigned rsc = std::min<unsigned>(65536, end_timestamp_div12);
   int32 new_sc;

   if(ADPCMBuf)
    PCECD_ProcessADPCMBuffer(rsc);

   for(unsigned ch = 0; ch < 2; ch++)
   {
    if(HRRes)
    {
     //
     // These filter parameters cause much less of a lowpass and much much less of a highpass filter effect than what I've tested on my Turbo Duo,
     // but I think it's probably broken(in need of capacitor replacements).
     // 
     HRBufs[ch]->Integrate(rsc, 2 /* lp shift, lower = less lp effect */, 14 /* hp shift, higher = less hp effect*/, ADPCMBuf, CDDABufs[ch]);
    }

#if 0
    for(unsigned x = 0; x < end_timestamp_div12; x++)
    {
     static double phase[2] = { 0};
     static double phase_inc[2] = { 0 };
     static double phase_inc_inc = 0.000000001; //0.000003;

     if(1)
     {
      static int zoom = 32767 * 0.75 * 256 / 4;
	
      //if(!(rand() & 0xFFFFFF))
      // zoom = -zoom;
      *(float*)&HRBufs[ch]->Buf()[x] = zoom;
     }
     // *(float*)&HRBufs[ch]->Buf()[x] = 256 * 32767 * 0.75 * sin(phase[ch]);
     else
      *(float*)&HRBufs[ch]->Buf()[x] = 256 * 0.75 * ((int16)rand());

     phase[ch] += phase_inc[ch];
     phase_inc[ch] += phase_inc_inc;
    }
#endif

#if 0
    for(unsigned x = 0; x < rsc; x++)
    {
     static int32 wv[2] = { 0x6000 * 256, 0x6000 * 256 };
     static int counter[2] = { 0, 0 };

     *(float*)&HRBufs[ch]->Buf()[x] = wv[ch];

     if(!counter[ch])
     {
      wv[ch] = -wv[ch];
      counter[ch] = 80;
     }
     else
      counter[ch]--;
    }

#endif


    if(espec->SoundBuf && HRRes)
     new_sc = HRRes->Resample(HRBufs[ch], rsc, espec->SoundBuf + (espec->SoundBufSize * 2) + ch, espec->SoundBufMaxSize - espec->SoundBufSize);
    else
    {
     HRBufs[ch]->ResampleSkipped(rsc);
     new_sc = 0;
    }
   }

   if(ADPCMBuf)
    ADPCMBuf->Finish(rsc);

   if(CDDABufs[0])
   {
    CDDABufs[0]->Finish(rsc);
    CDDABufs[1]->Finish(rsc);
   }

   espec->SoundBufSize += new_sc;
  }

  if(PCE_IsCD)
   PCECD_ResetTS(end_timestamp_mod12);

  vce->ResetTS(end_timestamp_mod12);

  HuCPU->SyncAndResetTimestamp(end_timestamp_mod12);

  //
  //
  //
  PCE_TimestampBase += end_timestamp - end_timestamp_mod12;
  espec->MasterCycles += end_timestamp - end_timestamp_mod12;

  if(!rp_rv)
  {
   //MDFN_MidSync(espec);
  }
 } while(!rp_rv);

 //printf("%d\n", MDFND_GetTime() - t);

 // End loop here.
 //printf("%d\n", vce->GetScanlineNo());

 if(IsHES)
  HES_Update(espec, INPUT_HESHack());	//Draw(espec->skip ? NULL : espec->surface, espec->skip ? NULL : &espec->DisplayRect, espec->SoundBuf, espec->SoundBufSize, INPUT_HESHack());
}

void PCE_MidSync(void)
{
 INPUT_Frame();

 es->MasterCycles = HuCPU->Timestamp();

 //MDFN_MidSync(es);
}

static int StateAction(StateMem *sm, int load, int data_only)
{
 SFORMAT StateRegs[] =
 {
  SFARRAY(BaseRAM, IsSGX? 32768 : 8192),
  SFVAR(PCE_TimestampBase),

  SFVAR(CD_TrayOpen),
  SFVAR(CD_SelectedDisc),

  SFEND
 };

#ifdef WANT_DEBUGGER
 PCEDBG_MachineStateChanged();
#endif

 int ret = MDFNSS_StateAction(sm, load, data_only, StateRegs, "MAIN");

 ret &= HuCPU->StateAction(sm, load, data_only);
 ret &= vce->StateAction(sm, load, data_only);
 ret &= psg->StateAction(sm, load, data_only);
 ret &= INPUT_StateAction(sm, load, data_only);
 ret &= HuC_StateAction(sm, load, data_only);
 ret &= SubHW_StateAction(sm, load, data_only);


 if(load)
 {
  if(cdifs)
  {
   // Sanity check.
   if(CD_SelectedDisc >= (int)cdifs->size())
    CD_SelectedDisc = (int)cdifs->size() - 1;

   SCSICD_SetDisc(CD_TrayOpen, (CD_SelectedDisc >= 0 && !CD_TrayOpen) ? (*cdifs)[CD_SelectedDisc] : NULL, true);
  }
 }

 return(ret);
}

void PCE_Power(void)
{
#ifdef WANT_DEBUGGER
 PCEDBG_MachineStateChanged();
#endif

 memset(BaseRAM, 0x00, sizeof(BaseRAM));

 HuCPU->Power();
 PCE_TimestampBase = 0;	// FIXME, move to init.
 const int32 timestamp = HuCPU->Timestamp();

 vce->Reset(timestamp);
 psg->Power(timestamp / 3);

 if(IsHES)
  HES_Reset();
 else
  HuC_Power();

 PCEINPUT_Power(timestamp);

 SubHW_Power();

 if(PCE_IsCD)
 {
  vce->SetCDEvent(PCECD_Power(timestamp));
 }
 //printf("%d\n", HuCPU->Timestamp());
}

static void CDInsertEject(void)
{
 CD_TrayOpen = !CD_TrayOpen;

 for(unsigned disc = 0; disc < cdifs->size(); disc++)
 {
  if(!(*cdifs)[disc]->Eject(CD_TrayOpen))
  {
   MDFN_DispMessage(_("Eject error."));
   CD_TrayOpen = !CD_TrayOpen;
  }
 }

 if(CD_TrayOpen)
  MDFN_DispMessage(_("Virtual CD Drive Tray Open"));
 else
  MDFN_DispMessage(_("Virtual CD Drive Tray Closed"));

 SCSICD_SetDisc(CD_TrayOpen, (CD_SelectedDisc >= 0 && !CD_TrayOpen) ? (*cdifs)[CD_SelectedDisc] : NULL);
}

static void CDEject(void)
{
 if(!CD_TrayOpen)
  CDInsertEject();
}

static void CDSelect(void)
{
 if(cdifs && CD_TrayOpen)
 {
  CD_SelectedDisc = (CD_SelectedDisc + 1) % (cdifs->size() + 1);

  if((unsigned)CD_SelectedDisc == cdifs->size())
   CD_SelectedDisc = -1;

  if(CD_SelectedDisc == -1)
   MDFN_DispMessage(_("Disc absence selected."));
  else
   MDFN_DispMessage(_("Disc %d of %d selected."), CD_SelectedDisc + 1, (int)cdifs->size());
 }
}

static void DoSimpleCommand(int cmd)
{
 switch(cmd)
 {
  case MDFN_MSC_RESET: PCE_Power(); break;
  case MDFN_MSC_POWER: PCE_Power(); break;

  case MDFN_MSC_INSERT_DISK:
	CDInsertEject();
        break;

  case MDFN_MSC_SELECT_DISK:
	CDSelect();
        break;

  case MDFN_MSC_EJECT_DISK:
	CDEject();
        break;
 }
}

static MDFNSetting PCESettings[] = 
{
  { "pce.input.multitap", MDFNSF_EMU_STATE | MDFNSF_UNTRUSTED_SAFE, gettext_noop("Enable multitap(TurboTap) emulation."), NULL, MDFNST_BOOL, "1" },

  { "pce.slstart", MDFNSF_NOFLAGS, gettext_noop("First rendered scanline."), NULL, MDFNST_UINT, "4", "0", "239" },
  { "pce.slend", MDFNSF_NOFLAGS, gettext_noop("Last rendered scanline."), NULL, MDFNST_UINT, "235", "0", "239" },

  { "pce.h_overscan", MDFNSF_NOFLAGS, gettext_noop("Show horizontal overscan area."), NULL, MDFNST_BOOL, "0" },

  { "pce.mouse_sensitivity", MDFNSF_NOFLAGS, gettext_noop("Emulated mouse sensitivity."), NULL, MDFNST_FLOAT, "0.50", NULL, NULL, NULL, PCEINPUT_SettingChanged },
  { "pce.disable_softreset", MDFNSF_NOFLAGS, gettext_noop("If set, when RUN+SEL are pressed simultaneously, disable both buttons temporarily."), NULL, MDFNST_BOOL, "0", NULL, NULL, NULL, PCEINPUT_SettingChanged },

  { "pce.disable_bram_cd", MDFNSF_EMU_STATE | MDFNSF_UNTRUSTED_SAFE, gettext_noop("Disable BRAM(saved game memory) for CD games."), gettext_noop("It is intended for viewing CD games' error screens that may be different from simple BRAM full and uninitialized BRAM error screens, though it can cause the game to crash outright."), MDFNST_BOOL, "0" },
  { "pce.disable_bram_hucard", MDFNSF_EMU_STATE | MDFNSF_UNTRUSTED_SAFE, gettext_noop("Disable BRAM(saved game memory) for HuCard games."), gettext_noop("It is intended for changing the behavior(passwords vs save games) of some HuCard games."), MDFNST_BOOL, "0" },

  { "pce.forcesgx", MDFNSF_EMU_STATE | MDFNSF_UNTRUSTED_SAFE, gettext_noop("Force SuperGrafx emulation."), 
		gettext_noop("Enabling this option is not necessary to run unrecognized PCE ROM images in SuperGrafx mode, and enabling it is discouraged; ROM images with a file extension of \".sgx\" will automatically enable SuperGrafx emulation."), MDFNST_BOOL, "0" },

  { "pce.arcadecard", MDFNSF_EMU_STATE | MDFNSF_UNTRUSTED_SAFE, gettext_noop("Enable Arcade Card emulation."), 
	gettext_noop("Leaving this option enabled is recommended, unless you want to see special warning screens on ACD games, or you prefer the non-enhanced modes of ACD-enhanced SCD games.  Additionally, you may want to disable it you you wish to use state rewinding with a SCD ACD-enhanced game on a slow CPU, as the extra 2MiB of RAM the Arcade Card offers is difficult to compress in real-time."), MDFNST_BOOL, "1" },

  { "pce.nospritelimit", MDFNSF_NOFLAGS, gettext_noop("Remove 16-sprites-per-scanline hardware limit."), 
					 gettext_noop("WARNING: Enabling this option may cause undesirable graphics glitching on some games(such as \"Bloody Wolf\")."), MDFNST_BOOL, "0" },

  { "pce.cdbios", MDFNSF_EMU_STATE, gettext_noop("Path to the CD BIOS"), NULL, MDFNST_STRING, "syscard3.pce" },
  { "pce.gecdbios", MDFNSF_EMU_STATE, gettext_noop("Path to the GE CD BIOS"), gettext_noop("Games Express CD Card BIOS (Unlicensed)"), MDFNST_STRING, "gecard.pce" },

  { "pce.psgrevision", MDFNSF_NOFLAGS, gettext_noop("Select PSG revision."), gettext_noop("WARNING: HES playback will always use the \"huc6280a\" revision if this setting is set to \"match\", since HES playback is always done with SuperGrafx emulation enabled."), MDFNST_ENUM, "match", NULL, NULL, NULL, NULL, PSGRevisionList  },

  { "pce.cdpsgvolume", MDFNSF_NOFLAGS, gettext_noop("PSG volume when playing a CD game."), gettext_noop("Setting this volume control too high may cause sample clipping."), MDFNST_UINT, "100", "0", "200", NULL, CDSettingChanged },
  { "pce.cddavolume", MDFNSF_NOFLAGS, gettext_noop("CD-DA volume."), gettext_noop("Setting this volume control too high may cause sample clipping."), MDFNST_UINT, "100", "0", "200", NULL, CDSettingChanged },
  { "pce.adpcmvolume", MDFNSF_NOFLAGS, gettext_noop("ADPCM volume."), gettext_noop("Setting this volume control too high may cause sample clipping."), MDFNST_UINT, "100", "0", "200", NULL, CDSettingChanged },
  { "pce.adpcmextraprec", MDFNSF_NOFLAGS, gettext_noop("Output the full 12-bit ADPCM predictor."), gettext_noop("Enabling this option causes the MSM5205 ADPCM predictor to be outputted with full precision of 12-bits, rather than only outputting 10-bits of precision(as an actual MSM5205 does).  Enable this option to reduce whining noise during ADPCM playback."), MDFNST_BOOL, "0" },

  { "pce.resamp_quality", MDFNSF_NOFLAGS, gettext_noop("Sound quality."), gettext_noop("Higher values correspond to better SNR and better preservation of higher frequencies(\"brightness\"), at the cost of increased computational complexity and a negligible increase in latency.\n\nHigher values will also slightly increase the probability of sample clipping(relevant if Mednafen's volume control settings are set too high), due to increased (time-domain) ringing."), MDFNST_INT, "3", "0", "5" },
  { "pce.resamp_rate_error", MDFNSF_NOFLAGS, gettext_noop("Sound output rate tolerance."), gettext_noop("Lower values correspond to better matching of the output rate of the resampler to the actual desired output rate, at the expense of increased RAM usage and poorer CPU cache utilization."), MDFNST_FLOAT, "0.0000009", "0.0000001", "0.0000350" },

  { "pce.vramsize", MDFNSF_EMU_STATE | MDFNSF_UNTRUSTED_SAFE | MDFNSF_SUPPRESS_DOC, gettext_noop("Size of emulated VRAM per VDC in 16-bit words.  DO NOT CHANGE THIS UNLESS YOU KNOW WTF YOU ARE DOING."), NULL, MDFNST_UINT, "32768", "32768", "65536" },
  { NULL }
};

static DECLFR(CheatReadFunc)
{
  std::vector<SUBCHEAT>::iterator chit;
  uint8 retval = NonCheatPCERead[(A / 8192) & 0xFF](A);

  for(chit = SubCheats[A & 0x7].begin(); chit != SubCheats[A & 0x7].end(); chit++)
  {
   if(A == chit->addr)
   {
    if(chit->compare == -1 || chit->compare == retval)
    {
     retval = chit->value;
     break;
    }
   }
  }
 return(retval);
}

static uint8 MemRead(uint32 addr)
{
 return(NonCheatPCERead[(addr / 8192) & 0xFF](addr));
}

static void InstallReadPatch(uint32 address, uint8 value, int compare)
{
 HuCPU->SetFastRead(address >> 13, NULL);
 HuCPU->SetReadHandler(address >> 13, CheatReadFunc);
}

static void RemoveReadPatches(void)
{
 for(int x = 0; x < 0x100; x++)
  HuCPU->SetReadHandler(x, NonCheatPCERead[x]);
}

static void SetLayerEnableMask(uint64 mask)
{
 vce->SetLayerEnableMask(mask);
}

static const FileExtensionSpecStruct KnownExtensions[] =
{
 { ".pce", gettext_noop("PC Engine ROM Image") },
 { ".hes", gettext_noop("PC Engine Music Rip") },
 { ".sgx", gettext_noop("SuperGrafx ROM Image") },
 { NULL, NULL }
};

static bool SetSoundRate(double rate)
{
 if(HRRes)
 {
  delete HRRes;
  HRRes = NULL;
 }

 if(rate > 0)
 {
  HRRes = new OwlResampler(PCE_MASTER_CLOCK / 12, rate, MDFN_GetSettingF("pce.resamp_rate_error"), 20, MDFN_GetSettingUI("pce.resamp_quality"));
  for(unsigned i = 0; i < 2; i++)
   HRRes->ResetBufResampState(HRBufs[i]);
 }

 return(TRUE);
}

};

using namespace MDFN_IEN_PCE;

MDFNGI EmulatedPCE =
{
 "pce",
 "PC Engine (CD)/TurboGrafx 16 (CD)/SuperGrafx",
 KnownExtensions,
 MODPRIO_INTERNAL_HIGH,
 #ifdef WANT_DEBUGGER
 &PCEDBGInfo,
 #else
 NULL,
 #endif
 &PCEInputInfo,
 Load,
 TestMagic,
 LoadCD,
 TestMagicCD,
 CloseGame,
 SetLayerEnableMask,
 NULL,
 NULL,
 NULL,
 InstallReadPatch,
 RemoveReadPatches,
 MemRead,
 NULL,
 false,
 StateAction,
 Emulate,
 PCEINPUT_SetInput,
 DoSimpleCommand,
 PCESettings,
 MDFN_MASTERCLOCK_FIXED(PCE_MASTER_CLOCK),
 0,
 TRUE,  // Multires possible?

 0,   // lcm_width
 0,   // lcm_height
 NULL,  // Dummy

 320,	// Nominal width
 232,	// Nominal height
 1365,	// Framebuffer width
 270,	// Framebuffer height(TODO: decrease to 264(263 + spillover line))

 2,     // Number of output sound channels
};

