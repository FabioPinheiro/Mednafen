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

/* Major organizational differences compared to real HuC6280:
	PSG emulation is in a completely separate file and class.

	Timer and IRQ read/write handlers are called externally from the main(external) PC Engine I/O page memory handler function, for
	speed reasons.

	Bus cycle extension on VDC and VCE access is simulated/handled in the main(external) PC Engine I/O page memory handler function, for
	speed reasons.

	Input port emulation is done externally.

  Bugs:
	The 21MHz master clock penalty when executing(not explicitly branching) across a 8KiB page boundary is not emulated.
*/

#include "mednafen/mednafen.h"
#include "huc6280.h"

#include <string.h>

#ifdef WANT_DEBUGGER
 #include        <trio/trio.h>
#endif

//#define IncPC()	{ PC++; if(!(PC & 0x1FFF)) printf("Crossing: %04x %02x\n", PC - 1, lastop); }

#define LASTCYCLE /*assert(((P & I_FLAG) ? 0 : (uint32)~0) == PIMaskCache);*/ IRQSample = (IRQlow & IRQMask) & PIMaskCache; IFlagSample = P & I_FLAG; ADDCYC(1);


#include <map>


std::map<uint8,void*> map;
std::map<uint8,int> mapTo;
unsigned int contador = 0;
double jumps = 0;
double normal = 0;
void HuC6280::Add(uint8 pc){
	/*
	map[0x92]=&lab92;
	map[0x94]=&lab94;
	map[0x00]=&lab00;
	map[0xae]=&labae;
	map[0xb0]=&labb0;*/


	
	
	
	if(mapTo.count(pc)==0)
		mapTo[pc]=1;
	else
		mapTo[pc]++;
	
	if(contador == 10000000){
		printMap();
	}
}
void HuC6280::printMap(){
	printf("%f   %f   %f  %f  contador: %d\n", normal,jumps, jumps/(jumps+normal), (jumps+normal), contador);
	for (std::map<uint8,int>::iterator it=mapTo.begin(); it!=mapTo.end(); ++it)
		printf("%02x => %d\n",it->first , it->second );
	
}
/*
void* HuC6280::checkMap(){

	
	uint8 newOp = RdOp(PC);
	if(map.count(newOp)==1){
		PC++;
		goto *(map[newOp]);
	}
		
}*/
/*	
	uint8 aux; \
	aux = 0x92; \
	map[aux]=&&lab92; \
	aux = 0xa5; \
	map[aux]=&&laba5; \
	aux = 0x94; \
	map[aux]=&&lab94; \
	aux = 0x00; \
	map[aux]=&&lab00; \
	aux = 0xae; \
	map[aux]=&&labae; \
	aux = 0xb0; \
	map[aux]=&&labb0; \
	*/

/*#define checkMap {  \
	uint8 newOp; \
	newOp = RdOp(PC);\
	if(map.count(newOp)==1 && contador%2==0){\
		PC++;  \
		jumps++; \
		goto *(map[newOp]);  \
	}else{ \
		normal++;\
		goto after;\
	}\
}*/

#define checkMap {  \
	uint8 newOp; \
	newOp = RdOp(PC);\
	PC++;  \
	goto *(map[newOp]);  \
}
//history *h = new history();


void HuC6280::StealCycle(void)
{
 ADDCYC(1);
}

void HuC6280::StealCycles(const int count)
{
 ADDCYC(count);
}

void HuC6280::StealMasterCycles(const int count)
{
 ADDCYC_MASTER(count);
}

void HuC6280::FlushMPRCache(void)
{
 for(int x = 0; x < 9; x++)
  SetMPR(x, MPR[x & 0x7]);
}

INLINE void HuC6280::PUSH(const uint8 V)
{
 WrMem(0x2100 | S, V);
 S--;
}       

INLINE uint8 HuC6280::POP(void)
{
 S++;

 return(RdMem(0x2100 | S));
}

static uint8 ZNTable[256];
/* Some of these operations will only make sense if you know what the flag
   constants are. */

INLINE void HuC6280::X_ZN(const uint8 zort)
{
 P &= ~(Z_FLAG|N_FLAG);
 P |= ZNTable[zort];
}

INLINE void HuC6280::X_ZNT(const uint8 zort)
{
 P |= ZNTable[zort];
}

template<bool DebugMode>
INLINE void HuC6280::JR(const bool cond, const bool BBRS)
{
 if(cond)
 {
  int32 disp;
  disp=(int8)RdOp(PC);
  PC++;
  ADDCYC(3);
  PC+=disp;

  if(DebugMode && ADDBT)
   ADDBT(PC - disp - 2 - (BBRS ? 1 : 0), PC, 0);
 }
 else
 {
  ADDCYC(1);
  PC++;
 }
 LASTCYCLE;
}

template<bool DebugMode>
INLINE void HuC6280::BBRi(const uint8 val, const unsigned int bitto)
{
 JR<DebugMode>(!(val & (1 << bitto)), true);
}

template<bool DebugMode>
INLINE void HuC6280::BBSi(const uint8 val, const unsigned int bitto)
{
 JR<DebugMode>(val & (1 << bitto), true);
}

// Total cycles for ST0/ST1/ST2 is effectively 5(4 here, +1 stealcycle in the PC Engine memory handler logic)
#define ST0	{ ADDCYC(3); LASTCYCLE; WrMemPhysical(0x80000000 | (0xFF * 8192 + 0), x); }
#define ST1	{ ADDCYC(3); LASTCYCLE; WrMemPhysical(0x80000000 | (0xFF * 8192 + 2), x); }
#define ST2	{ ADDCYC(3); LASTCYCLE; WrMemPhysical(0x80000000 | (0xFF * 8192 + 3), x); }

#define LDA	   A=x;X_ZN(A)
#define LDX	   X=x;X_ZN(X)
#define LDY        Y=x;X_ZN(Y)


#define IMP(op) op; break;

#define SAX	{ uint8 tmp = X; X = A; A = tmp; ADDCYC(2); LASTCYCLE; }
#define SAY	{ uint8 tmp = Y; Y = A; A = tmp; ADDCYC(2); LASTCYCLE; }
#define SXY	{ uint8 tmp = X; X = Y; Y = tmp; ADDCYC(2); LASTCYCLE; }

#define TAX	{ X = A; X_ZN(A); ADDCYC(1); LASTCYCLE; }
#define TXA	{ A = X; X_ZN(A); ADDCYC(1); LASTCYCLE; }
#define TAY	{ Y = A; X_ZN(A); ADDCYC(1); LASTCYCLE; }
#define TYA	{ A = Y; X_ZN(A); ADDCYC(1); LASTCYCLE; }
#define TSX	{ X = S; X_ZN(X); ADDCYC(1); LASTCYCLE; }
#define TXS	{ S = X;	  ADDCYC(1); LASTCYCLE; }

#define DEX	{ X--; X_ZN(X);  ADDCYC(1); LASTCYCLE; }
#define DEY	{ Y--; X_ZN(Y);  ADDCYC(1); LASTCYCLE; }
#define INX	{ X++; X_ZN(X);  ADDCYC(1); LASTCYCLE; }
#define INY	{ Y++; X_ZN(Y);  ADDCYC(1); LASTCYCLE; }



// Combined cycle total of TPREFIX and TPOSTFIX must be 3.
// WARNING: LASTCYCLE is called twice in instructions that make use TPREFIX/TPOSTFIX, so allow for that in the LASTCYCLE
// 	    macro!
#define TPREFIX { uint8 Abackup = A; if(P & T_FLAG) { ADDCYC(1); A = RdMem(0x2000 + X); }
#define TPOSTFIX if(P & T_FLAG) { ADDCYC(1); WrMem(0x2000 + X, A); LASTCYCLE; A = Abackup; } }

/*  All of the freaky arithmetic operations. */
#define AND        TPREFIX; A&=x;X_ZN(A); TPOSTFIX;
#define BIT        P &= ~(Z_FLAG|V_FLAG|N_FLAG); P|=ZNTable[x&A]&Z_FLAG; P|=x&(V_FLAG|N_FLAG);
#define EOR        TPREFIX; A^=x;X_ZN(A); TPOSTFIX;
#define ORA        TPREFIX; A|=x;X_ZN(A); TPOSTFIX;


// ADC and SBC in decimal mode take 1 extra CPU cycle...we'll add it by using "LASTCYCLE".  So now that makes
// LASTCYCLE being called at most 3 times.  Not very LASTy, is it!!
#define ADC  TPREFIX; {	\
	      if(P & D_FLAG)	\
	      {		\
		uint32 low = (A & 0x0F) + (x & 0x0F) + (P & 1);	\
		uint32 high = (A & 0xF0) + (x & 0xF0);	\
		P &= ~(Z_FLAG | C_FLAG | N_FLAG);	\
		if(low > 0x09) { high += 0x10; low += 0x06; }	\
		if(high > 0x90) { high += 0x60; }	\
		P |= (high >> 8) & C_FLAG;	\
		A = (low & 0x0F) | (high & 0xF0);	\
		X_ZNT(A);	\
		LASTCYCLE;	\
	      }	\
	      else	\
	      {	\
	       uint32 l=A+x+(P&1);	\
	       P&=~(Z_FLAG|C_FLAG|N_FLAG|V_FLAG);	\
               P|=((((A^x)&0x80)^0x80) & ((A^l)&0x80))>>1;	\
               P|=(l>>8)&C_FLAG;	\
	       A=l;	\
	       X_ZNT(A);	\
	      }	\
	     } TPOSTFIX;

#define SBC  if(P & T_FLAG) {puts("SET misuse"); } if(P & D_FLAG)	\
	     {		\
	      uint32 c = (P & 1) ^ 1;	\
	      uint32 l = A - x - c;	\
	      uint32 low = (A & 0x0f) - (x & 0x0f) - c;	\
	      uint32 high = (A & 0xf0) - (x & 0xf0);	\
	      P &= ~(Z_FLAG | C_FLAG | N_FLAG);	\
	      if(low & 0xf0) low -= 0x06;	\
	      if(low & 0x80) high -= 0x10;	\
	      if(high & 0x0f00) high -= 0x60;	\
	      P |= ((l >> 8) & C_FLAG) ^ C_FLAG;	\
	      A = (low & 0x0F) | (high & 0xf0);	\
	      X_ZNT(A);	\
              LASTCYCLE;      \
	     }	else {	\
	      uint32 l=A-x-((P&1)^1);	\
	      P&=~(Z_FLAG|C_FLAG|N_FLAG|V_FLAG);	\
	      P|=((A^l)&(A^x)&0x80)>>1;	\
	      P|=((l>>8)&C_FLAG)^C_FLAG;	\
	      A=l;	\
	      X_ZNT(A);	\
	     }

#define CMPL(a1,a2) {	\
		     uint32 t=a1-a2;	\
		     X_ZN(t&0xFF);	\
		     P&=~C_FLAG;	\
		     P|=((t>>8)&C_FLAG)^C_FLAG;	\
		    }

#define TAM     for(int i = 0; i < 8; i ++) {               \
                        if(x & (1 << i))        \
			{	\
				SetMPR(i, A);	\
			}	\
	        } SetMPR(8, MPR[0]);	\
		ADDCYC(4);	\
		LASTCYCLE;

#define TMA	for(int i = 0; i < 8; i ++) {		\
			if(x & (1 << i))	\
				A = MPR[i];	\
		}		\
		ADDCYC(3);	\
		LASTCYCLE;

// Note: CSL/CSH's speed timing changes take effect for the last CPU cycle of CSL/CSH.  Be cautious
// not to change the order here:
#define CSL	{ /*printf("CSL: %04x\n", PC);*/ ADDCYC(2); speed = 0; REDOSPEEDCACHE(); LASTCYCLE; }
#define CSH	{ /*printf("CSH: %04x\n", PC);*/ ADDCYC(2); speed = 1; REDOSPEEDCACHE(); LASTCYCLE; }

#define RMB(bitto)	x &= ~(1 << (bitto & 7))
#define SMB(bitto)	x |= 1 << (bitto & 7)

#define TSB   { P &= ~(Z_FLAG | V_FLAG | N_FLAG); P |= (x | A) ? 0 : Z_FLAG; P |= x & (N_FLAG | V_FLAG); x |= A; }
#define TRB     { P &= ~(Z_FLAG | V_FLAG | N_FLAG); P |= (x & ~A) ? 0 : Z_FLAG; P |= x & (N_FLAG | V_FLAG); x &= ~A; }

#define TST	{ P &= ~(Z_FLAG | V_FLAG | N_FLAG); P |= (x & zoomhack) ? 0: Z_FLAG; P |= x & (V_FLAG | N_FLAG); }

#define CMP		CMPL(A,x)
#define CPX		CMPL(X,x)
#define CPY	      	CMPL(Y,x)

/* The following operations modify the byte being worked on. */
#define DEC       	x--;X_ZN(x)
#define INC		x++;X_ZN(x)

#define ASL        P&=~C_FLAG;P|=x>>7;x<<=1;X_ZN(x)
#define LSR	P&=~(C_FLAG|N_FLAG|Z_FLAG);P|=x&1;x>>=1;X_ZNT(x)

#define ROL	{	\
		 uint8 l=x>>7;	\
		 x<<=1;	\
		 x|=P&C_FLAG;	\
		 P&=~(Z_FLAG|N_FLAG|C_FLAG);	\
		 P|=l;	\
		 X_ZNT(x);	\
		}
#define ROR	{	\
		 uint8 l=x&1;	\
		 x>>=1;	\
		 x|=(P&C_FLAG)<<7;	\
		 P&=~(Z_FLAG|N_FLAG|C_FLAG);	\
		 P|=l;	\
		 X_ZNT(x);	\
		}
		 
/* Absolute */
#define GetAB(target) 	\
{	\
 target=RdOp(PC);	\
 PC++;	\
 target|=RdOp(PC)<<8;	\
 PC++;	\
}

/* Absolute Indexed(for reads) */
#define GetABI(target, i)	\
{	\
 unsigned int tmp;	\
 GetAB(tmp);	\
 target=tmp;	\
 target+=i;	\
}

/* Zero Page */
#define GetZP(target)	\
{	\
 target=0x2000 | RdOp(PC); 	\
 PC++;	\
}

/* Zero Page Indexed */
#define GetZPI(target,i)	\
{	\
 target=0x2000 | ((i+RdOp(PC)) & 0xFF);	\
 PC++;	\
}

/* Indirect */
#define GetIND(target)   \
{       \
 uint8 tmp;     \
 tmp=RdOp(PC);        \
 PC++; \
 target=RdMem(0x2000 + tmp);	\
 tmp++;         \
 target|=RdMem(0x2000 + tmp)<<8;       \
}


/* Indexed Indirect */
#define GetIX(target)	\
{	\
 uint8 tmp;	\
 tmp=RdOp(PC);	\
 PC++;	\
 tmp+=X;	\
 target=RdMem(0x2000 + tmp);	\
 tmp++;		\
 target|=RdMem(0x2000 + tmp) <<8;	\
}

/* Indirect Indexed(for reads) */
#define GetIY(target)	\
{	\
 unsigned int rt;	\
 uint8 tmp;	\
 tmp=RdOp(PC);	\
 rt=RdMem(0x2000 + tmp);	\
 tmp++;	\
 rt|=RdMem(0x2000 + tmp)<<8;	\
 target = (rt + Y);	\
 PC++;	\
}

/* Now come the macros to wrap up all of the above stuff addressing mode functions
   and operation macros.  Note that operation macros will always operate(redundant
   redundant) on the variable "x".
*/

#define RMW_A(op) 	{ uint8 x = A; op; A = x; ADDCYC(1); LASTCYCLE; break; } /* Meh... */
#define RMW_AB(op) 	{ unsigned int EA; uint8 x; GetAB(EA); ADDCYC(6); x=RdMem(EA); op; LASTCYCLE; WrMem(EA,x); break; }
#define RMW_ABI(reg,op) { unsigned int EA; uint8 x; GetABI(EA,reg); ADDCYC(6); x=RdMem(EA); op; LASTCYCLE; WrMem(EA,x); break; }
#define RMW_ABX(op)	RMW_ABI(X,op)
#define RMW_ABY(op)	RMW_ABI(Y,op)
#define RMW_ZP(op)  	{ unsigned int EA; uint8 x; GetZP(EA); ADDCYC(5); x=RdMem(EA); op; LASTCYCLE; WrMem(EA,x); break; }
#define RMW_ZPX(op) 	{ unsigned int EA; uint8 x; GetZPI(EA, X); ADDCYC(5); x=RdMem(EA); op; LASTCYCLE; WrMem(EA,x); break;}

// For RMB/SMB...
#define RMW_ZP_B(op)	{ unsigned int EA; uint8 x; GetZP(EA); ADDCYC(5); x=RdMem(EA); ADDCYC(1); op; LASTCYCLE; WrMem(EA,x); break; }


// A LD_IM for complex immediate instructions that take care of cycle consumption in their operation(TAM, TMA, ST0, ST1, ST2)
#define LD_IM_COMPLEX(op)	{ uint8 x = RdOp(PC); PC++; op; break; }

#define LD_IM(op)	{uint8 x; x=RdOp(PC); PC++; ADDCYC(1); LASTCYCLE; op; break;}
#define LD_ZP(op)	{unsigned int EA; uint8 x; GetZP(EA); ADDCYC(3); LASTCYCLE; x=RdMem(EA); op; break;}
#define LD_ZPX(op)  	{unsigned int EA; uint8 x; GetZPI(EA, X); ADDCYC(3); LASTCYCLE; x=RdMem(EA); op; break;}
#define LD_ZPY(op)  	{unsigned int EA; uint8 x; GetZPI(EA, Y); ADDCYC(3); LASTCYCLE; x=RdMem(EA); op; break;}
#define LD_AB(op)	{unsigned int EA; uint8 x; GetAB(EA); ADDCYC(4); LASTCYCLE; x=RdMem(EA); op; /*checkMap;*/ break; }
#define LD_ABI(reg,op)  {unsigned int EA; uint8 x; GetABI(EA,reg); ADDCYC(4); LASTCYCLE; x=RdMem(EA); op; break;}
#define LD_ABX(op)	LD_ABI(X, op)
#define LD_ABY(op)	LD_ABI(Y, op)

#define LD_IND(op)	{unsigned int EA; uint8 x; GetIND(EA); ADDCYC(6); LASTCYCLE; x=RdMem(EA); op; break;}
#define LD_IX(op)	{unsigned int EA; uint8 x; GetIX(EA); ADDCYC(6); LASTCYCLE; x=RdMem(EA); op; break;}
#define LD_IY(op)	{unsigned int EA; uint8 x; GetIY(EA); ADDCYC(6); LASTCYCLE; x=RdMem(EA); op; break;}

// For the funky TST instruction
#define LD_IM_TST(op, lt)       { uint8 lt = RdOp(PC); PC++; ADDCYC(3); op; }
#define LD_IM_ZP(op)	LD_IM_TST(LD_ZP(TST), zoomhack);
#define LD_IM_ZPX(op)	LD_IM_TST(LD_ZPX(TST), zoomhack);
#define LD_IM_AB(op)	LD_IM_TST(LD_AB(TST), zoomhack);
#define LD_IM_ABX(op)	LD_IM_TST(LD_ABX(TST), zoomhack);


#define BMT_PREFIX(pork) in_block_move = IBM_##pork;
#define BMT_LOOPCHECK(pork) if(!runrunrun) { TimerSync(); return; } continue_the_##pork:

#define BMT_TDD BMT_PREFIX(TDD); do { ADDCYC(6); WrMem(bmt_dest, RdMem(bmt_src)); bmt_src--; bmt_dest--; BMT_LOOPCHECK(TDD); bmt_length--; } while(bmt_length);
#define BMT_TAI BMT_PREFIX(TAI); {bmt_alternate = 0; do { ADDCYC(6); WrMem(bmt_dest, RdMem(bmt_src + bmt_alternate)); bmt_dest++; bmt_alternate ^= 1; BMT_LOOPCHECK(TAI); bmt_length--; } while(bmt_length); }
#define BMT_TIA BMT_PREFIX(TIA); {bmt_alternate = 0; do { ADDCYC(6); WrMem(bmt_dest + bmt_alternate, RdMem(bmt_src)); bmt_src++; bmt_alternate ^= 1; BMT_LOOPCHECK(TIA); bmt_length--; } while(bmt_length); } 
#define BMT_TII BMT_PREFIX(TII); do { ADDCYC(6); WrMem(bmt_dest, RdMem(bmt_src)); bmt_src++; bmt_dest++; BMT_LOOPCHECK(TII); bmt_length--; } while(bmt_length); 
#define BMT_TIN BMT_PREFIX(TIN); do { ADDCYC(6); WrMem(bmt_dest, RdMem(bmt_src)); bmt_src++; BMT_LOOPCHECK(TIN); bmt_length--; } while(bmt_length);

// Block memory transfer load
#define LD_BMT(op)	{ PUSH(Y); PUSH(A); PUSH(X); GetAB(bmt_src); GetAB(bmt_dest); GetAB(bmt_length); ADDCYC(14); op; in_block_move = 0; X = POP(); A = POP(); Y = POP(); ADDCYC(2); LASTCYCLE; break; }

#define ST_ZP(r)	{unsigned int EA; GetZP(EA); ADDCYC(3); LASTCYCLE; WrMem(EA, r); break;}
#define ST_ZPX(r)	{unsigned int EA; GetZPI(EA,X); ADDCYC(3); LASTCYCLE; WrMem(EA, r); break;}
#define ST_ZPY(r)	{unsigned int EA; GetZPI(EA,Y); ADDCYC(3); LASTCYCLE; WrMem(EA, r); break;}
#define ST_AB(r)	{unsigned int EA; GetAB(EA); ADDCYC(4); LASTCYCLE; WrMem(EA, r); break;}
#define ST_ABI(reg,r)	{unsigned int EA; GetABI(EA,reg); ADDCYC(4); LASTCYCLE; WrMem(EA,r); break; }
#define ST_ABX(r)	ST_ABI(X, r)
#define ST_ABY(r)	ST_ABI(Y, r)

#define ST_IND(r)	{unsigned int EA; GetIND(EA); ADDCYC(6); LASTCYCLE; WrMem(EA,r); /*checkMap;*/ break; }
#define ST_IX(r)	{unsigned int EA; GetIX(EA); ADDCYC(6); LASTCYCLE; WrMem(EA,r); break; }
#define ST_IY(r)	{unsigned int EA; GetIY(EA); ADDCYC(6); LASTCYCLE; WrMem(EA,r); break; }

void HuC6280::Reset(void)
{
 timer_inreload = FALSE;
 timer_div = 1024;
 timer_load = 0;
 timer_value = 0;
 timer_status = 0;
 in_block_move = 0;

 IRQSample = IQRESET;
 IRQlow = IQRESET;
}
  
HuC6280::HuC6280(const bool emulate_wai) : EmulateWAI(emulate_wai)
{
	timestamp = 0;
	next_user_event = 0;
	next_event = 0;

	timer_lastts = 0;
	timer_inreload = 0;
	timer_status = 0;
	timer_value = 0;
	timer_load = 0;
	timer_div = 0;
	
	
	in_block_move = 0;
	isopread = 0;

	LastLogicalReadAddr = 0;
	LastLogicalWriteAddr = 0;

	for(int x = 0; x < 256; x++)
	{
	 if(!x)
          ZNTable[x] = Z_FLAG;
	 else if (x&0x80)
          ZNTable[x]=N_FLAG;
	 else
          ZNTable[x]=0;
	}

	SetCPUHook(NULL, NULL);
}

HuC6280::~HuC6280()
{
 #if 0
 for(int op = 0; op < 256; op++)
 {
  printf("%02x: Nominal: %d, Real: ", op, CycTable[op]);

  for(int i = 0; i < 256; i++)
  {
   if(CycTimes[op][i])
    printf("%d, ", i);
  }

  printf("\n");
 }
 #endif
}

void HuC6280::Power(void)
{
 IODataBuffer = 0xFF;

 IRQlow = 0;

 PC = 0;
 A = 0;
 X = 0;
 Y = 0;
 S = 0;
 P = 0;

 REDOPIMCACHE();

 for(int i = 0; i < 9; i++)
 {
  MPR[i] = 0;
  FastPageR[i] = NULL;
 }  
 Reset();
}

// TimerSync() doesn't call CalcNextEvent(), so we'll need to call it some time after TimerSync() (TimerSync() is
// used in HappySync(), TimerRead(), and TimerWrite().
void HuC6280::TimerSync(void)
{
 int32 clocks = timestamp - timer_lastts;

 timer_div -= clocks;

 while(timer_div <= 0)
 {
  int32 reload_div = 1024 * 3;

  if(timer_inreload)
  {
   timer_value = timer_load;
   reload_div = reload_div - 1; //1023;
   timer_inreload = FALSE;
  }
  else
  {
   if(timer_status)
   {
    timer_value --;
    if(timer_value < 0)
    {
     timer_inreload = TRUE;
     reload_div = 1;
     IRQBegin(IQTIMER);
    }
   }
  }
  timer_div += reload_div;
 }

 timer_lastts = timestamp;
}

void HuC6280::HappySync(void)
{
 TimerSync();

 if(next_user_event <= 0)
  next_user_event = EventHandler->Sync(timestamp);

 CalcNextEvent();
}


template<bool DebugMode>
void HuC6280::RunSub(void)
{	

	uint8 aux;
	aux = 0x40; map[aux] = &&after;	 aux = 0x60; map[aux] = &&after;	 aux = 0x48; map[aux] = &&after;	 aux = 0x08; map[aux] = &&after;	
	aux = 0xDA; map[aux] = &&after;	 aux = 0x5A; map[aux] = &&after;	 aux = 0x68; map[aux] = &&after;	 aux = 0xFA; map[aux] = &&after;	
	aux = 0x7A; map[aux] = &&after;	 aux = 0x28; map[aux] = &&after;	 aux = 0x4C; map[aux] = &&after;	 aux = 0x6C; map[aux] = &&after;	
	aux = 0x7C; map[aux] = &&after;	 aux = 0x20; map[aux] = &&after;	 aux = 0xAA; map[aux] = &&after;	 aux = 0x8A; map[aux] = &&after;	
	aux = 0xA8; map[aux] = &&after;	 aux = 0x98; map[aux] = &&after;	 aux = 0xBA; map[aux] = &&after;	 aux = 0x9A; map[aux] = &&after;	
	aux = 0xCA; map[aux] = &&after;	 aux = 0x88; map[aux] = &&after;	 aux = 0xE8; map[aux] = &&after;	 aux = 0xC8; map[aux] = &&after;	
	aux = 0x54; map[aux] = &&after;	 aux = 0xD4; map[aux] = &&after;	 aux = 0x62; map[aux] = &&after;	 aux = 0x82; map[aux] = &&after;	
	aux = 0xC2; map[aux] = &&after;	 aux = 0x18; map[aux] = &&after;	 aux = 0xD8; map[aux] = &&after;	 aux = 0x58; map[aux] = &&after;	
	aux = 0xB8; map[aux] = &&after;	 aux = 0x38; map[aux] = &&after;	 aux = 0xF8; map[aux] = &&after;	 aux = 0x78; map[aux] = &&after;	
	aux = 0xF4; map[aux] = &&after;	 aux = 0xEA; map[aux] = &&after;	 aux = 0x0A; map[aux] = &&after;	 aux = 0x06; map[aux] = &&after;	
	aux = 0x16; map[aux] = &&after;	 aux = 0x0E; map[aux] = &&after;	 aux = 0x1E; map[aux] = &&after;	 aux = 0x3A; map[aux] = &&after;	
	aux = 0xC6; map[aux] = &&after;	 aux = 0xD6; map[aux] = &&after;	 aux = 0xCE; map[aux] = &&after;	 aux = 0xDE; map[aux] = &&after;	
	aux = 0x1A; map[aux] = &&after;	 aux = 0xE6; map[aux] = &&after;	 aux = 0xF6; map[aux] = &&after;	 aux = 0xEE; map[aux] = &&after;	
	aux = 0xFE; map[aux] = &&after;	 aux = 0x4A; map[aux] = &&after;	 aux = 0x46; map[aux] = &&after;	 aux = 0x56; map[aux] = &&after;	
	aux = 0x4E; map[aux] = &&after;	 aux = 0x5E; map[aux] = &&after;	 aux = 0x2A; map[aux] = &&after;	 aux = 0x26; map[aux] = &&after;	
	aux = 0x36; map[aux] = &&after;	 aux = 0x2E; map[aux] = &&after;	 aux = 0x3E; map[aux] = &&after;	 aux = 0x6A; map[aux] = &&after;	
	aux = 0x66; map[aux] = &&after;	 aux = 0x76; map[aux] = &&after;	 aux = 0x6E; map[aux] = &&after;	 aux = 0x7E; map[aux] = &&after;	
	aux = 0x69; map[aux] = &&after;	 aux = 0x65; map[aux] = &&after;	 aux = 0x75; map[aux] = &&after;	 aux = 0x6D; map[aux] = &&after;	
	aux = 0x7D; map[aux] = &&after;	 aux = 0x79; map[aux] = &&after;	 aux = 0x72; map[aux] = &&after;	 aux = 0x61; map[aux] = &&after;	
	aux = 0x71; map[aux] = &&after;	 aux = 0x29; map[aux] = &&after;	 aux = 0x25; map[aux] = &&after;	 aux = 0x35; map[aux] = &&after;	
	aux = 0x2D; map[aux] = &&after;	 aux = 0x3D; map[aux] = &&after;	 aux = 0x39; map[aux] = &&after;	 aux = 0x32; map[aux] = &&after;	
	aux = 0x21; map[aux] = &&after;	 aux = 0x31; map[aux] = &&after;	 aux = 0x89; map[aux] = &&after;	 aux = 0x24; map[aux] = &&after;	
	aux = 0x34; map[aux] = &&after;	 aux = 0x2C; map[aux] = &&after;	 aux = 0x3C; map[aux] = &&after;	 aux = 0xC9; map[aux] = &&after;	
	aux = 0xC5; map[aux] = &&after;	 aux = 0xD5; map[aux] = &&after;	 aux = 0xCD; map[aux] = &&after;	 aux = 0xDD; map[aux] = &&after;	
	aux = 0xD9; map[aux] = &&after;	 aux = 0xD2; map[aux] = &&after;	 aux = 0xC1; map[aux] = &&after;	 aux = 0xD1; map[aux] = &&after;	
	aux = 0xE0; map[aux] = &&after;	 aux = 0xE4; map[aux] = &&after;	 aux = 0xEC; map[aux] = &&after;	 aux = 0xC0; map[aux] = &&after;	
	aux = 0xC4; map[aux] = &&after;	 aux = 0xCC; map[aux] = &&after;	 aux = 0x49; map[aux] = &&after;	 aux = 0x45; map[aux] = &&after;	
	aux = 0x55; map[aux] = &&after;	 aux = 0x4D; map[aux] = &&after;	 aux = 0x5D; map[aux] = &&after;	 aux = 0x59; map[aux] = &&after;	
	aux = 0x52; map[aux] = &&after;	 aux = 0x41; map[aux] = &&after;	 aux = 0x51; map[aux] = &&after;	 aux = 0xA9; map[aux] = &&after;	
	aux = 0xB5; map[aux] = &&after;	 aux = 0xAD; map[aux] = &&after;	 aux = 0xBD; map[aux] = &&after;	 aux = 0xB9; map[aux] = &&after;	
	aux = 0xB2; map[aux] = &&after;	 aux = 0xA1; map[aux] = &&after;	 aux = 0xB1; map[aux] = &&after;	 aux = 0xA2; map[aux] = &&after;	
	aux = 0xA6; map[aux] = &&after;	 aux = 0xB6; map[aux] = &&after;	 aux = 0xBE; map[aux] = &&after;	 aux = 0xA0; map[aux] = &&after;	
	aux = 0xA4; map[aux] = &&after;	 aux = 0xB4; map[aux] = &&after;	 aux = 0xAC; map[aux] = &&after;	 aux = 0xBC; map[aux] = &&after;	
	aux = 0x09; map[aux] = &&after;	 aux = 0x05; map[aux] = &&after;	 aux = 0x15; map[aux] = &&after;	 aux = 0x0D; map[aux] = &&after;	
	aux = 0x1D; map[aux] = &&after;	 aux = 0x19; map[aux] = &&after;	 aux = 0x12; map[aux] = &&after;	 aux = 0x01; map[aux] = &&after;	
	aux = 0x11; map[aux] = &&after;	 aux = 0xE9; map[aux] = &&after;	 aux = 0xE5; map[aux] = &&after;	 aux = 0xF5; map[aux] = &&after;	
	aux = 0xED; map[aux] = &&after;	 aux = 0xFD; map[aux] = &&after;	 aux = 0xF9; map[aux] = &&after;	 aux = 0xF2; map[aux] = &&after;	
	aux = 0xE1; map[aux] = &&after;	 aux = 0xF1; map[aux] = &&after;	 aux = 0x85; map[aux] = &&after;	 aux = 0x95; map[aux] = &&after;	
	aux = 0x8D; map[aux] = &&after;	 aux = 0x9D; map[aux] = &&after;	 aux = 0x99; map[aux] = &&after;	 aux = 0x81; map[aux] = &&after;	
	aux = 0x91; map[aux] = &&after;	 aux = 0x86; map[aux] = &&after;	 aux = 0x96; map[aux] = &&after;	 aux = 0x8E; map[aux] = &&after;	
	aux = 0x84; map[aux] = &&after;	 aux = 0x8C; map[aux] = &&after;	 aux = 0x0F; map[aux] = &&after;	 aux = 0x1F; map[aux] = &&after;	
	aux = 0x2F; map[aux] = &&after;	 aux = 0x3F; map[aux] = &&after;	 aux = 0x4F; map[aux] = &&after;	 aux = 0x5F; map[aux] = &&after;	
	aux = 0x6F; map[aux] = &&after;	 aux = 0x7F; map[aux] = &&after;	 aux = 0x8F; map[aux] = &&after;	 aux = 0x9F; map[aux] = &&after;	
	aux = 0xAF; map[aux] = &&after;	 aux = 0xBF; map[aux] = &&after;	 aux = 0xCF; map[aux] = &&after;	 aux = 0xDF; map[aux] = &&after;	
	aux = 0xEF; map[aux] = &&after;	 aux = 0xFF; map[aux] = &&after;	 aux = 0x80; map[aux] = &&after;	 aux = 0x44; map[aux] = &&after;	
	aux = 0x90; map[aux] = &&after;	 aux = 0xF0; map[aux] = &&after;	 aux = 0xD0; map[aux] = &&after;	 aux = 0x30; map[aux] = &&after;	
	aux = 0x10; map[aux] = &&after;	 aux = 0x50; map[aux] = &&after;	 aux = 0x70; map[aux] = &&after;	 aux = 0x07; map[aux] = &&after;	
	aux = 0x17; map[aux] = &&after;	 aux = 0x27; map[aux] = &&after;	 aux = 0x37; map[aux] = &&after;	 aux = 0x47; map[aux] = &&after;	
	aux = 0x57; map[aux] = &&after;	 aux = 0x67; map[aux] = &&after;	 aux = 0x77; map[aux] = &&after;	 aux = 0x87; map[aux] = &&after;	
	aux = 0x97; map[aux] = &&after;	 aux = 0xa7; map[aux] = &&after;	 aux = 0xb7; map[aux] = &&after;	 aux = 0xc7; map[aux] = &&after;	
	aux = 0xd7; map[aux] = &&after;	 aux = 0xe7; map[aux] = &&after;	 aux = 0xf7; map[aux] = &&after;	 aux = 0x64; map[aux] = &&after;	
	aux = 0x74; map[aux] = &&after;	 aux = 0x9C; map[aux] = &&after;	 aux = 0x9E; map[aux] = &&after;	 aux = 0x14; map[aux] = &&after;	
	aux = 0x1C; map[aux] = &&after;	 aux = 0x04; map[aux] = &&after;	 aux = 0x0C; map[aux] = &&after;	 aux = 0x83; map[aux] = &&after;	
	aux = 0xA3; map[aux] = &&after;	 aux = 0x93; map[aux] = &&after;	 aux = 0xB3; map[aux] = &&after;	 aux = 0x02; map[aux] = &&after;	
	aux = 0x22; map[aux] = &&after;	 aux = 0x42; map[aux] = &&after;	 aux = 0x73; map[aux] = &&after;	 aux = 0xC3; map[aux] = &&after;	
	aux = 0xD3; map[aux] = &&after;	 aux = 0xE3; map[aux] = &&after;	 aux = 0xF3; map[aux] = &&after;	 aux = 0x43; map[aux] = &&after;	
	aux = 0x53; map[aux] = &&after;	 aux = 0x03; map[aux] = &&after;	 aux = 0x13; map[aux] = &&after;	 aux = 0x23; map[aux] = &&after;	
	aux = 0xCB; map[aux] = &&after;	
	
	aux = 0x92;
	map[aux]=&&lab92;
	aux = 0xa5;
	map[aux]=&&laba5;
	aux = 0x94;
	map[aux]=&&lab94;
	aux = 0x00;
	map[aux]=&&lab00;
	aux = 0xae;
	map[aux]=&&labae;
	aux = 0xb0;
	map[aux]=&&labb0; 

	
	uint32 old_PC;

        if(in_block_move)
        {
	 IBM_Dispatch: ;
         switch(in_block_move)
         {
          default: exit(1);
          case IBM_TIA: goto continue_the_TIA;
          case IBM_TAI: goto continue_the_TAI;
          case IBM_TDD: goto continue_the_TDD;
          case IBM_TII: goto continue_the_TII;
          case IBM_TIN: goto continue_the_TIN;
         }
        }

	do
    {
	 #include "huc6280_step.inc"
	} while(runrunrun > 0);
}

void HuC6280::Run(bool StepMode)
{
 if(StepMode)
  runrunrun = -1;        // Needed so a BMT isn't interrupted.
 else
  runrunrun = 1;

 if(CPUHook || ADDBT)
  RunSub<true>();
 else
  RunSub<false>();
}

uint8 HuC6280::TimerRead(unsigned int address, bool peek)
{
 if(!peek)
 {
  TimerSync();
  CalcNextEvent();
 }

 return(timer_value | (IODataBuffer & 0x80));
}

void HuC6280::TimerWrite(unsigned int address, uint8 V)
{
 TimerSync();

 switch(address & 1)
 {
  case 0: timer_load = (V & 0x7F); break;
  case 1: if(V & 1) // Enable counter
	  {
	  	if(timer_status == 0)
		{
			//if(timer_inreload)
			// puts("Oops");
			timer_div = 1024 * 3;
			timer_value = timer_load;
		}
	  }
	  timer_status = V & 1;
	  break;
 }

 CalcNextEvent();
}


uint8 HuC6280::IRQStatusRead(unsigned int address, bool peek)
{
 if(!(address & 2)) 
  return(IODataBuffer);

 switch(address & 1)
 {
  case 0:
	 if(!peek)
	  IRQEnd(IQTIMER); 
	 return(IRQMask ^ 0x7);
  case 1: 
	{
	 int status = 0;
	 if(IRQlow & IQIRQ1) status |= 2;
	 if(IRQlow & IQIRQ2) status |= 1;
	 if(IRQlow & IQTIMER) status |= 4;
	 return(status | (IODataBuffer & ~(1 | 2 | 4)));
	}
 }
 return(IODataBuffer);
}

void HuC6280::IRQStatusWrite(unsigned int address, uint8 V)
{
 if(!(address & 2))
  return;

 switch(address & 1)
 {
  case 0: IRQMask = (V & 0x7) ^ 0x7;
	  break;

  case 1: IRQEnd(IQTIMER); 
	  break;
 }
}

int HuC6280::StateAction(StateMem *sm, int load, int data_only)
{
 uint16 tmp_PC = PC;

 SFORMAT StateRegs[]=
 {
  SFVAR(runrunrun),	// For the benefit of save states while in step mode in the debugger.

  SFVARN(tmp_PC, "PC"),
  SFVARN(A, "A"),
  SFVARN(P, "P"),
  SFVARN(IFlagSample, "IFlagSample"),
  SFVARN(X, "X"),
  SFVARN(Y, "Y"),
  SFVARN(S, "S"),

  SFVARN(lastop, "lastop"),

  SFVARN(IRQSample, "IRQSample"),
  SFVARN(IRQlow, "IRQlow"),
  SFVARN(IRQMask, "IRQMask"),
  SFARRAYN(MPR, 8, "MPR"),
  SFVARN(speed, "speed"),

  SFVARN(timer_inreload, "timer_inreload"),
  SFVARN(timer_status, "timer_status"),
  SFVARN(timer_value, "timer_value"),
  SFVARN(timer_load, "timer_load"),
  SFVARN(timer_div, "timer_div"),

  SFVARN(in_block_move, "IBM"),
  SFVARN(bmt_src, "IBM_SRC"),
  SFVARN(bmt_dest, "IBM_DEST"),
  SFVARN(bmt_length, "IBM_LENGTH"),
  SFVARN(bmt_alternate, "IBM_ALTERNATE"),

  SFVARN(next_event, "next_event"),
  SFVARN(next_user_event, "next_user_event"),

  SFVAR(IODataBuffer),
  SFEND
 };

 int ret = MDFNSS_StateAction(sm, load, data_only, StateRegs, "CPU");

 if(load)
 {
  PC = tmp_PC;

  // Update MPR cache
  FlushMPRCache();
  REDOSPEEDCACHE();
  REDOPIMCACHE();
 }
 return(ret);
}

void HuC6280::DumpMem(char *filename, uint32 start, uint32 end)
{
 FILE *fp=fopen(filename, "wb");

 for(uint32 x = start; x <= end; x++)
  fputc(RdMem(x), fp);

 fclose(fp);
}

void HuC6280::SetRegister(const unsigned int id, uint32 value)
{
	 switch(id)
	 {
	  case GSREG_PC:
		PC = value & 0xFFFF;
		break;

	  case GSREG_A:
		A = value & 0xFF;
		break;

	  case GSREG_X:
		X = value & 0xFF;
		break;

	  case GSREG_Y:
		Y = value & 0xFF;
		break;

  	case GSREG_SP:
		S = value & 0xFF;
		break;

	  case GSREG_P:
		P = value & 0xFF;
		REDOPIMCACHE();
		break;

	  case GSREG_SPD:
		speed = value & 0x01;
		REDOSPEEDCACHE();
		break;

	  case GSREG_MPR0:
	  case GSREG_MPR1:
	  case GSREG_MPR2:
	  case GSREG_MPR3:
	  case GSREG_MPR4:
	  case GSREG_MPR5:
	  case GSREG_MPR6:
	  case GSREG_MPR7:
		MPR[id - GSREG_MPR0] = value & 0xFF;
		FlushMPRCache();
		break;

	  case GSREG_IRQM:
		IRQMask = (value & 0x7) ^ 0x7;
		break;

	  case GSREG_TIMS:
		timer_status = value & 0x1;
		break;

	  case GSREG_TIMV:
		timer_value = value & 0x7F;
		break;

	  case GSREG_TIML:
		timer_load = value & 0x7F;
		break;

	  case GSREG_TIMD:
		timer_div = value & 1023;
		break;
	 }
}
