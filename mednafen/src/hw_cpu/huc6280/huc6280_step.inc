         if(DebugMode)
          old_PC = PC;

         if(DebugMode && CPUHook)
         {
          TimerSync();
          CalcNextEvent();
          if(CPUHook(PC))
	  {
	   if(in_block_move)
	    goto IBM_Dispatch;
	  }
         }

	 if(IRQSample | IRQlow)
	 {
          if(IRQSample & IQRESET)
          {
           speed = 0;
           REDOSPEEDCACHE();

           IRQMask = 7;
           SetMPR(7, 0);
           PC=RdMem(0xFFFE);
           PC|=RdMem(0xFFFF)<<8;
           P=I_FLAG;
	   REDOPIMCACHE();
           IRQSample &= ~IQRESET;
           IRQlow &= ~IQRESET;

	   if(DebugMode && ADDBT)
	    ADDBT(old_PC, PC, 0xFFFE);

	   continue;
	  }
	  else
	  {
 	   uint32 tmpa = 0;

	   if((IRQlow & IQTIMER & IRQMask) && !IFlagSample) //IRQSample & IQTIMER)
	    tmpa = 0xFFFA;
	   else if(IRQSample & IQIRQ1)
	    tmpa = 0xFFF8;
	   else if(IRQSample & IQIRQ2)
	    tmpa = 0xFFF6;
	   //else
	   // puts("DANGER WILL ROBINSON DANGER");

	   //printf("IRQ: %04x\n", tmpa);

	   if(tmpa)
	   {
	    // Total: 8 cycles(7 ADDCYC(1), 1 LASTCYCLE)

	    ADDCYC(1);	// Cycle 1
	    RdMem(PC);	//	Dummy read

	    ADDCYC(1);	// Cycle 2
	    RdMem(PC + 1); //	Dummy read

	    ADDCYC(1);	// Cycle 3
	    PUSH(PC>>8); //	Push PCH

	    ADDCYC(1);	// Cycle 4
	    PUSH(PC); //	Push PCL

	    ADDCYC(1);	// Cycle 5
	    PUSH((P & ~B_FLAG)); //	Push P
	    P |= I_FLAG;
	    REDOPIMCACHE();
	    P &= ~(T_FLAG | D_FLAG);

	    ADDCYC(1);	// Cycle 6
	    PC=RdMem(tmpa);	// Fetch vector PCL

	    ADDCYC(1);	// Cycle 7
	    PC|=RdMem(tmpa + 1) << 8; // Fect vector PCH

	    LASTCYCLE;	// Cycle 8(internal operation?)

            if(DebugMode && ADDBT)
             ADDBT(old_PC, PC, tmpa);

	    continue;
           }
	  }
	 }
         PC &= 0xFFFF;     // Our cpu core can only handle PC going about 8192 bytes over, so make sure it never gets that far...
		
		
		
	
		
	//printf("%x\n", PC);
	lastop = RdOp(PC);
	
	PC++;

	//checkMap;
    switch(lastop)
         {
          #include "ops.inc"
         }

     
	 P &= ~T_FLAG;
skip_T_flag_clear:	// goto'd by the SET code
	contador++;
//	jumps++;
	 checkMap;
after:
	PC--;
