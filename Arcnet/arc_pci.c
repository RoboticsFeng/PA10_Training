/*******************************************************
	     
	     ARC Net Driver for PA-10

********************************************************/

#include "../include.h"

#include "../main_cfg.h"

#include "arc_pci.h"
#include "arc_pci_cfg.h"
 
#include <hw/inout.h> 
 
/*==================== $BJQ?t$N@k8@(B ================================================*/

unsigned char	_SendPacket[256];    /* $B%5!<%\%I%i%$%P$X$NAw?.%P%C%U%!(B */
unsigned char	_RecvPacket[256];    /* $B%5!<%\%I%i%$%P$+$i$N<u?.%P%C%U%!(B */

SEND_C   _Ccom;          /* $BAw?.%G!<%?(B */
RECV_C   _Axis;          /* $B<u?.%G!<%?(B */

int  RecvFlag = FALSE;
int  RvCNT    = FALSE;

unsigned char  SampleData[256];

static	int	PageNo;

const double TORQ_TO_DIGIT[7] = { TORQ_TO_DIGIT1, TORQ_TO_DIGIT2, TORQ_TO_DIGIT3, 
				  TORQ_TO_DIGIT4, TORQ_TO_DIGIT5, TORQ_TO_DIGIT6,
				  TORQ_TO_DIGIT7 };

/*==================================================================================
  $B%"!<%/%M%C%H$N=i4|2=(B
  
            $BLa$jCM(B
	        0$B!'@5>o=i4|2==*N;(B
		1,2,3,4$B!'FbIt<+8J?GCG$K$h$j!"%(%i!<M-$j(B
==================================================================================*/
int
_arc_init(int BaseAddress)
{
  unsigned int	i;
  unsigned int	str, cmr, diag, ahr, alr, dtr, sar, cfr, ndr, stup1, stup2;
  
/*--- $B3F%l%8%9%?$N%"%I%l%9@_Dj(B ---*/  

  str   = BaseAddress + 0x00;  /* $B%9%F!<%?%9!&%l%8%9%?(B */
  cmr   = BaseAddress + 0x04;  /* $B%3%^%s%I!&%l%8%9%?(B */
  diag  = BaseAddress + 0x04;  /* $B%@%$%"%0%N%9%F%#%C%/!&%l%8%9%?(B */
  ahr   = BaseAddress + 0x08;  /* $B%"%I%l%9%]%$%s%?!J>e0L!K!&%l%8%9%?(B */
  alr   = BaseAddress + 0x0C;  /* $B%"%I%l%9%]%$%s%?!J2<0L!K!&%l%8%9%?(B */
  dtr   = BaseAddress + 0x10;  /* $B%G!<%?!&%l%8%9%?(B */
  sar   = BaseAddress + 0x14;  /* $B%5%V%"%I%l%9!&%l%8%9%?(B */
  cfr   = BaseAddress + 0x18;  /* $B%3%s%U%#%0%l!<%7%g%s!&%l%8%9%?(B */

  ndr   = BaseAddress + 0x1c;  /* $B%N!<%I(BID$B!&%l%8%9%?(B */
  stup1 = BaseAddress + 0x1c;  /* $B%;%C%H%"%C%W(B1$B!&%l%8%9%?(B */
  stup2 = BaseAddress + 0x1c;  /* $B%;%C%H%"%C%W(B2$B!&%l%8%9%?(B */

/*----- $B=i4|2=(B ----------*/

  out8( cfr, 0x80 );             /* (I?LD3*1X>/D(B */
  iodelay();
  out8( cfr, 0x02 );             /* Software Reset $B$N2r=|(B */
                                       /* (I>/D1/L_(B1$B!&(IZ<^=@$B$r(I18>=$B2DG=$K$9$k(B */
  delay1(1);          	               /* (I?LD3*1X>/D$B8e$N;~4VBT$A(B(800ns) */

  out8( stup1, 0x81 );	       /* (I>/D1/L_(B1$B!&(IZ<^=@$B$N@_Dj(B */
                                       /* BP$B;~(B nPULSE1$BC<;R(B:push/pull$B=PNO(B*/
                                       /* (I10K^DZ0<.]$B$,(B2(I8[/8%;28Y(B(10MHz) */
  
  out8( cfr, 0x19 | 0x04 );      /* TXEN=0 (ID$B7hDj8e$KM-8z$K$9$k(B)*/
                                       /* (II0D^(BID$B!&(IZ<^=@$B$r(I18>=$B2DG=$K$9$k(B */
                                       /* (IJ^/8L_Z0]S0D^(B(BP)$B$NA*Br(B */
  out8( ndr, _MyID );            /* (II0D^(BID$B!&(IZ<^=@$B$K(B_MyID(0xff)$B$r@_Dj(B */
  delay1(3);                            /* (II0D^(BID$B!&(IZ<^=@$B$N@_Dj(B */

/*----- $BFbIt<+8J?GCG(B -----*/ 

  out8( ahr, 0x80 );             /* $B<!$N(IC^0@$B!&(IZ<^=@$B$X$N(I18>=$B$r(B"Read"$B$K@_Dj(B */

/* $B%A%'%C%/#1(B */
  out8( alr, 0x00 );	       /* (I1D^Z=(B 000H = D1H ? */
  for(i=0;i<TIME_OUT;i++){             
    if( in8( dtr ) == 0xd1 )    break;
  }
  if(i==TIME_OUT)   return(1);	

/* $B%A%'%C%/#2(B */  
  out8( alr, 0x01 );	       /* (I1D^Z=(B 001H = _MyID ? */
  for(i=0;i<TIME_OUT;i++){
    if( in8( dtr ) == _MyID )   break;
    }
  if(i==TIME_OUT)   return(2);	

/* $B%A%'%C%/#3(B */
  for(i=0;i<TIME_OUT;i++){
    if(( in8( diag ) & 0x30 ) == 0x00 )  break;
  }
  if(i==TIME_OUT)   return(3);

/* $B%A%'%C%/#4(B */  
  delay1(10);                           /* TXEN=0$B"*(B840ms$B7P2aBT$A(BWait */
  for(i=0;i<TIME_OUT;i++){	       /* DUP-ID = 0 ?	*/
    if(( in8( diag ) & 0x40 ) == 0x00)    break;
  }
  if(i==TIME_OUT)   return(4);

/*----- $B@_Dj(B -----*/
  
  out8( sar, 0x04 );	       /* (I>/D1/L_(B2$B!&(IZ<^=@$B$r(I18>=$B2DG=$K$9$k(B */
  out8( stup2, 0xbc );	       /* CKUP1,0:$B$F$$G\2sO)$N@_Dj(B:10MHz*/
                                       /* $B9bB.(BCPU$BBP1~$K$9$k(B */
  delay1(1);                            /* CKUP1,0$B$NJQ998e$N(BWait(1ms) */
  out8( cmr, 0x18 );	       /* CKUP0,CKUP1$BJQ998e$N:F5/F0(B */

  out8( cfr, 0x39 | 0x04 );      /* "TXEN"=1$B!J(IH/D\08$B$K;22C$9$k!K(B*/
                                       /* (IJ^/8L_Z0]S0D^(B(BP)$B$NA*Br(B */
  out8( cmr, 0x0d );             /* (I<.0D$B!&(I[]8^J_9/D$B$NN>J}$r<u?.2DG=$K(B */
  out8( cmr, 0x84 );             /* (IL^[0D^7,=DJ_9/D$B$b<u?.2DG=$K$9$k(B */
  
  PageNo= 0;

  return(0);
}

/*=================================================================================
  $B%"!<%/%M%C%H$X$N%G!<%?Aw?.!J%7%g!<%H%Q%1%C%H;HMQ!K(B

       $BAw?.MQ%P%C%U%!!'(B_SendPacket[0$B!A(B255]
            _SendPacket[0] ... _MyID          = 0xff
            _SendPacket[1] ... _SDID          = 0xfe
            _SendPacket[2] ... (I:O]D^$B@hF,%"%I%l%9!J(B256 - $B%G!<%??t!K(B
                    /
                    /
            _SendPacket[_SendPacket[2]]   ... (I<09]<,Y(BNo.
            _SendPacket[_SendPacket[2]+1] ... (I:O]D^(B = 'S','C','E'...etc
	    _SendPacket[_SendPacket[2]+2] ... (IC^0@(B
	            /
	    _SendPacket[255]                ... (IC^0@(B

       $BLa$jCM!!(B
           0$B!'@5>oAw?.=*N;(B
           1$B!'Aw?.2DG=>uBV$G$O$J$$!JBT$A>uBV$NAw?.%3%^%s%IM-$j!K(B
	   2$B!'Aj<j$+$i(BACK$B$NJV?.$,$J$$(B
==================================================================================*/
int
_arc_send(int BaseAddress, unsigned char *_SendPacket1)
{
  unsigned int	i;
  unsigned int	str, mask, cmr, diag, ahr, alr, dtr, sar, cfr, ndr, stup1, stup2;
  
/*--- $B3F%l%8%9%?$N%"%I%l%9@_Dj(B ---*/  

  str   = BaseAddress + 0x00;  /* $B%9%F!<%?%9!&%l%8%9%?(B */
  mask  = BaseAddress + 0x00;  /* $B3d$j9~$_%^%9%/!&%l%8%9%?(B */
  cmr   = BaseAddress + 0x04;  /* $B%3%^%s%I!&%l%8%9%?(B */
  diag  = BaseAddress + 0x04;  /* $B%@%$%"%0%N%9%F%#%C%/!&%l%8%9%?(B */
  ahr   = BaseAddress + 0x08;  /* $B%"%I%l%9%]%$%s%?!J>e0L!K!&%l%8%9%?(B */
  alr   = BaseAddress + 0x0C;  /* $B%"%I%l%9%]%$%s%?!J2<0L!K!&%l%8%9%?(B */
  dtr   = BaseAddress + 0x10;  /* $B%G!<%?!&%l%8%9%?(B */
  sar   = BaseAddress + 0x14;  /* $B%5%V%"%I%l%9!&%l%8%9%?(B */
  cfr   = BaseAddress + 0x18;  /* $B%3%s%U%#%0%l!<%7%g%s!&%l%8%9%?(B */

  ndr   = BaseAddress + 0x1c;  /* $B%N!<%I(BID$B!&%l%8%9%?(B */
  stup1 = BaseAddress + 0x1c;  /* $B%;%C%H%"%C%W(B1$B!&%l%8%9%?(B */
  stup2 = BaseAddress + 0x1c;  /* $B%;%C%H%"%C%W(B2$B!&%l%8%9%?(B */
  
/*----- $BAw?.2DG=>uBV$N%A%'%C%/(B -----*/

  for(i=0;i<TIME_OUT;i++){              /* $BA0$NAw?.MW5aL?Na!&=*N;(B ? */
    if((in8(str) & 0x01) != 0x00)   break;
  }
  if(i==TIME_OUT)    return(1);         /* $BAw?.Cf(B(TA=0) */
  
/*----- 2$B%Z!<%8L\$KAw?.%G!<%?$r%;%C%H$9$k(B -----*/

  out8( ahr, 0x04 );              /* $B<!$N(IC^0@$B!&(IZ<^=@$B$X$N(I18>=$B$r(B"Write"$B$K@_Dj(B */
                                        /* 2$B%Z!<%8L\(B(0x4??H)$B$K@_Dj(B */
  out8( alr, 0x00 );		/* (I1D^Z=$B$r(B0x400H$B$K@_Dj(B */
  out8( dtr, _MyID );		/* $BAw?.85#I#D(B(_MyID) */

  out8( alr, 0x01 );              /* (I1D^Z=$B$r(B0x401H$B$K@_Dj(B */
  out8( dtr, _SendPacket1[1] );  /* $BAw?.@h#I#D(B (_SDID) */

  out8( alr, 0x02 );              /* (I1D^Z=$B$r(B0x402H$B$K@_Dj(B */
  out8( dtr, _SendPacket1[2] );  /* $B%G!<%?@hF,%"%I%l%9(B */

  out8( ahr, 0x44 );              /* (I50D2]8XR]D$B!!!\!!(I1D^Z=>/D(B(0x400H) */
  out8( alr, _SendPacket1[2] );  /* $B%G!<%?@hF,%"%I%l%9%;%C%H(B */
  for(i=_SendPacket[2];i!=0x100;i++){ /* $B%G!<%?%;%C%H(B*/
    out8( dtr, _SendPacket1[i] );
  }

  out8( mask, 0x00 );		/* $B3d$j9~$_$N6X;_(B */
  
/*----- $BAw?.2DG=>uBV$r%A%'%C%/(B -----*/

  if((in8(str) & 0x01) == 0x00 ){ /* $BA0$NAw?.MW5aL?Na!&=*N;(B ? */
    out8( cmr, 0x01 );	        /* $BAw?.2DG=>uBV$G$J$$$?$a!"Aw?.L?Na$r%-%c%s%;%k$9$k(B */
    return(1);
  }

/*----- $BAw?.(B -----*/

  out8( cmr, 0x13 );		/* 2$B%Z!<%8L\$rAw?.$9$k(B */

  if(_SendPacket1[1] != 0x00){         /* BroadCast$BJ}<0$G$J$$>l9g(B*/           
    for(i=0;i<TIME_OUT;i++){            /* ACK$B!J@5>o1~Ez!K$NJV?.%A%'%C%/(B */
      if(( in8( str ) & 0x03 ) == 0x03 )	break;  /* $B@5>o$KAw?.40N;(B */
    }
    if(i == TIME_OUT){
      out8( cmr, 0x01 );	        /* $BAw?.2DG=>uBV$G$J$$$?$a!"Aw?.L?Na$r%-%c%s%;%k$9$k(B */
      return(2);
    }
  }
  return(0);
}

/*=================================================================================
  $B%"!<%/%M%C%H$N%G!<%?<u?.(B

            $B<u?.MQ%P%C%U%!!'(B_RecvPacket[0$B!A(B255]

	         _RecvPacket[0] ... _SDID
		 _RecvPacket[1] ... _MyID
		 _RecvPacket[2] ... (I:O]D^$B@hF,(I1D^Z=$B!J(B256 - (IC^0@$B?t!K(B
		         /
			 /
		 _RecvPacket[_RecvPacket[2]]   ... (I<09]<,Y(BNo.
		 _RecvPacket[_RecvPacket[2]+1] ... (I:O]D^(B = 'S','C','E'...etc
		 _RecvPacket[_RecvPacket[2]+2] ... (IC^0@(B
		 $B!!!!!!!!(B/
		 _RecvPacket[255]                ... (IC^0@(B

	    $BLa$jCM(B
	        0$B!'@5>o<u?.=*N;(B
		1$B!'BT$A>uBV$N<u?.%3%^%s%IM-$j(B
==================================================================================*/
int
_arc_read(int BaseAddress)
{
  unsigned int	i;
  unsigned char  datAdd;
  unsigned char  *ptr;
  unsigned int	str, mask, cmr, diag, ahr, alr, dtr, sar, cfr, ndr, stup1, stup2;
  
/*--- $B3F%l%8%9%?$N%"%I%l%9@_Dj(B ---*/  

  str   = BaseAddress + 0x00;  /* $B%9%F!<%?%9!&%l%8%9%?(B */
  mask  = BaseAddress + 0x00;  /* $B3d$j9~$_%^%9%/!&%l%8%9%?(B */
  cmr   = BaseAddress + 0x04;  /* $B%3%^%s%I!&%l%8%9%?(B */
  diag  = BaseAddress + 0x04;  /* $B%@%$%"%0%N%9%F%#%C%/!&%l%8%9%?(B */
  ahr   = BaseAddress + 0x08;  /* $B%"%I%l%9%]%$%s%?!J>e0L!K!&%l%8%9%?(B */
  alr   = BaseAddress + 0x0C;  /* $B%"%I%l%9%]%$%s%?!J2<0L!K!&%l%8%9%?(B */
  dtr   = BaseAddress + 0x10;  /* $B%G!<%?!&%l%8%9%?(B */
  sar   = BaseAddress + 0x14;  /* $B%5%V%"%I%l%9!&%l%8%9%?(B */
  cfr   = BaseAddress + 0x18;  /* $B%3%s%U%#%0%l!<%7%g%s!&%l%8%9%?(B */

  ndr   = BaseAddress + 0x1c;  /* $B%N!<%I(BID$B!&%l%8%9%?(B */
  stup1 = BaseAddress + 0x1c;  /* $B%;%C%H%"%C%W(B1$B!&%l%8%9%?(B */
  stup2 = BaseAddress + 0x1c;  /* $B%;%C%H%"%C%W(B2$B!&%l%8%9%?(B */
   
/*----- $BBT$A>uBV$N<u?.%3%^%s%I$N%A%'%C%/(B -----*/

  out8( mask, 0x00 );	           /* $B<u?.3d$j9~$_$N6X;_(B */
  if(( in8( str ) & 0x80 ) == 0x00 ) /* $BBT$A>uBV$N<u?.%3%^%s%I(B $BM-(B ? */
    return(1);
  
/*----- $B<u?.%Z!<%8$NA*Br(B -----*/

  out8( mask, 0x00 );              /* $B<u?.3d$j9~$_$N6X;_(B */
  
  if(PageNo==1){                         /* 1$B%Z!<%8L\$rA*Br(B */
    out8( ahr, 0xc2 );             /* $B<!$N(IC^0@$B!&(IZ<^=@$B$X$N(I18>=$B$r(B"Read"$B$K@_Dj(B */
                                         /* (I50D2]8XR]D(B */
                                         /* 1$B%Z!<%8L\(B(0x200H)$B$K@_Dj(B */
    out8( cmr, 0x84 );             /* $B<!$O(B0$B%Z!<%8L\(B(0x000H)$B$r<u?.$9$k(B */
    PageNo=0;                            /* $B<!$O(B0$B%Z!<%8L\$rA*Br$9$k(B */
  }
  else{                                  /* 0$B%Z!<%8L\$rA*Br(B */
    out8( ahr, 0xc0 );             /* $B<!$N(IC^0@$B!&(IZ<^=@$B$X$N(I18>=$B$r(B"Read"$B$K@_Dj(B */
                                         /* (I50D2]8XR]D(B */
                                         /* 0$B%Z!<%8L\(B(0x000H)$B$K@_Dj(B */
    out8( cmr, 0x8c );             /* $B<!$O(B1$B%Z!<%8L\(B(0x200H)$B$r<u?.$9$k(B */
    PageNo=1;                            /* $B<!$O(B1$B%Z!<%8L\$rA*Br$9$k(B */
  }

/*----- $B<u?.(B -----*/
  
  ptr = _RecvPacket;
  RecvFlag = TRUE;

  out8( alr, 0x00 );               /* (I1D^Z=$B$r(B0x?00H$B$K@_Dj(B */
  *(ptr+0) = in8( dtr );           /* $BAw?.85(BID$B$NFI$_9~$_(B */
  
  out8( alr, 0x01 );               /* (I1D^Z=$B$r(B0x?01H$B$K@_Dj(B */
  *(ptr+1) = in8( dtr );           /* $BAw?.@h(BID$B$NFI$_9~$_(B */

  out8( alr, 0x02 );               /* (I1D^Z=$B$r(B0x?02H$B$K@_Dj(B */
  datAdd = in8(dtr);               /* (I<09]<,Y(BNo.$B$N(I1D^Z=$B$NFI$_9~$_(B */
  *(ptr+2) = datAdd;                     /* (I<09]<,Y(BNo.$B$N(I1D^Z=$B$r(I>/D(B */

  out8( alr, datAdd );
  for(i=datAdd; i!=0x100; i++){          /* $B%G!<%?FI$_9~$_(B */
    *(ptr+i) = in8(dtr);
  }

  return(0);
}
 
/*==================================================================================
  $B%"!<%/%M%C%H(B(ARC-PCI/22)$B$N%j%;%C%H(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
_arc_reset(int BaseAddress)
{
  unsigned int	Config;                    /* $B%3%s%U%#%0%l!<%7%g%s!&%l%8%9%?(B */
  
  Config = BaseAddress + 0x18;
  out8( Config, 0x80 );	           /* $B%=%U%H%&%'%"!&%j%;%C%H(B */
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$K!H@)8f3+;O%3%^%s%I!I$rAw?.$9$k(B

            $BLa$jCM(B
	        0$B!'%"!<%/%M%C%HAw?.2DG=(B
		1$B!'Aw?.%(%i!<(B
==================================================================================*/
int
iSend_S(void)
{
  int  stat1, rvstat;
  static int  timecnt = 0;
  unsigned int  TopAdr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));   /*$BAw?.%G!<%?NN0h(BClear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 0xff - (sizeof(char));
  _SendPacket[2] = TopAdr;
  _SendPacket[TopAdr  ] = 'r';
  _SendPacket[TopAdr+1] = 'S';
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);          /* $BAw?.(B */

  if(stat1 != 0){
    printf("\n write error   (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  
  while(1){
    rvstat = iRecv_S();                            /* $B3NG'MQ$K%G!<%?$r<u?.$9$k(B */ 
    if (RecvFlag == TRUE){
      timecnt = 0;
      printf("Get Ready to Start! \n\n");
      return(0);
    }
    timecnt++;
    if(timecnt > TIME_OUT2 ){
      printf("\ndata receive timeout  - iSend_S -\n");
      timecnt = 0;
      return(1);
    }
  }
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$+$i%G!<%?$r<u?.$9$k(B

           $BLa$jCM!J(B0$B!K(B
==================================================================================*/
int
iRecv_S(void)
{
  int  stat1;
  
  stat1 = _arc_read(ARC_ADDR);                          /* $B%G!<%?$N<u?.(B */

  if(stat1 == 0 ){
    printf("\nArcnet for PA10 O.K! -> ");
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$K!H@)8f%3%^%s%I!I$*$h$S@)8f%G!<%?$rAw?.$9$k(B

            $BLa$jCM(B
	        0$B!'@5>oAw?.=*N;(B
		1$B!'Aw?.%(%i!<(B
==================================================================================*/
int
iSend_C(void)
{
  int  jnt, stat1;
  unsigned int  TopAdr, adr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));    /* $BAw?.%G!<%?NN0h(BClear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 210;
  _SendPacket[2] = TopAdr;                           /* $B@hF,(I1D^Z=>/D(B */
  _SendPacket[TopAdr  ] = 'r';                       /* (I<09]<,Y(BNo. */
  _SendPacket[TopAdr+1] = 'C';                       /* $B%G!<%?<oN`(B */
  
  for(jnt=0;jnt<7;jnt++){
    adr = TopAdr + 2 + 6*jnt;
    _SendPacket[adr  ] = (unsigned char)((_Ccom.Drive[jnt].Status & 0xff00) >> 8);
    _SendPacket[adr+1] = (unsigned char) (_Ccom.Drive[jnt].Status & 0x00ff);
    _SendPacket[adr+2] = (unsigned char)((_Ccom.Drive[jnt].Torq   & 0xff00) >> 8);
    _SendPacket[adr+3] = (unsigned char) (_Ccom.Drive[jnt].Torq   & 0x00ff);
    _SendPacket[adr+4] = (unsigned char)((_Ccom.Drive[jnt].Speed  & 0xff00) >> 8);
    _SendPacket[adr+5] = (unsigned char) (_Ccom.Drive[jnt].Speed  & 0x00ff);
  }
  _SendPacket[254] = (unsigned char)((_Ccom.Do & 0xff00) >> 8);
  _SendPacket[255] = (unsigned char) (_Ccom.Do & 0x00ff);
  
  memcpy(SampleData,_SendPacket,sizeof(SampleData));
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);          /* $BAw?.(B */

  if(stat1 != 0){
    printf("\n write error (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  RvCNT = TRUE;

  return OK;
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$+$i%G!<%?$r<u?.$7!"<u?.MQ(IJ^/L'$B$K3JG<$9$k(B

            $BLa$jCM(B(0)
==================================================================================*/
int
iRecv_C(void)
{
  int  jnt, stat1;
  unsigned char  RecvCNT;
  unsigned short  status;
  long  rezolver;
  short  speed, torq;
  
  stat1 = _arc_read(ARC_ADDR);                         /* $B%G!<%?$N<u?.(B */
  
  if(stat1 == 0 ){
    if(RecvFlag == TRUE){
      if(RvCNT == TRUE){
		  for(jnt=0;jnt<7;jnt++){   
	  RecvCNT = _RecvPacket[2] + 2 + 10*jnt;
	  
	  status   = (_RecvPacket[RecvCNT  ] <<  8) | (_RecvPacket[RecvCNT+1]);
	  
	  rezolver = (_RecvPacket[RecvCNT+2] << 24) | (_RecvPacket[RecvCNT+3] << 16) |
	    (_RecvPacket[RecvCNT+4] <<  8) | (_RecvPacket[RecvCNT+5]);
	  
	  speed    = (_RecvPacket[RecvCNT+6] <<  8) | (_RecvPacket[RecvCNT+7]);
	  
	  torq     = (_RecvPacket[RecvCNT+8] <<  8) | (_RecvPacket[RecvCNT+9]);
	  
	  _Axis.sts[jnt].Status = status;
	  _Axis.sts[jnt].Rez	  = rezolver;
	  _Axis.sts[jnt].Speed  = speed;
	  _Axis.sts[jnt].Torq	  = torq;
	}
	
	_Axis.MStatus = (_RecvPacket[_RecvPacket[2]+80] << 8) | 
	  (_RecvPacket[_RecvPacket[2]+81]);
	
	RvCNT = FALSE;
      }
    }
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$+$i%G!<%?$r<u?.$7!"<u?.MQ(IJ^/L'$B$K3JG<$9$k(B

            $BLa$jCM(B(0)
==================================================================================*/
int
RecData(void)
{
  int rvstat;
  static int timecnt = 0;
  
  while(1){
    rvstat = iRecv_C();                        /* $B3NG'MQ$K%G!<%?$r<u?.$9$k(B */ 
    if (RecvFlag == TRUE){
      timecnt = 0;
	  //printf("aho\n");
      return(0);
    }
    timecnt++;
    if(timecnt > TIME_OUT2 ){
      printf("\ndata receive timeout  - iSend_C -\n");
      timecnt = 0;
      return(1);
    }
  }
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$K!H%V%l!<%-2r=|%3%^%s%I!I$rAw?.$9$k(B

            $BLa$jCM(B
	        0$B!'@5>oAw?.=*N;(B
		1$B!'Aw?.%(%i!<(B
==================================================================================*/
int
iSend_T(void)
{
  int  stat1,rvstat;
  static int  timecnt = 0;
  unsigned int  TopAdr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));      /* $BAw?.%G!<%?NN0h(BClear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 0xff - (sizeof(char));
  _SendPacket[2] = TopAdr;
  _SendPacket[TopAdr  ] = 'r';
  _SendPacket[TopAdr+1] = 'T';
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);             /* $BAw?.(B */

  if(stat1 != 0){
    printf("\n write error (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  
  while(1){
    rvstat = iRecv_T();                             /* $B3NG'MQ$K%G!<%?$r<u?.$9$k(B */ 
    if (RecvFlag == TRUE){
      timecnt = 0;
      printf("\nBrakeOFF mode 'T' -> Get Ready to Start!\n\n");
      return(0);
    }
    timecnt++;
    if(timecnt > TIME_OUT2 ){
      printf("\ndata receive timeout  - iSend_T -\n");
      timecnt = 0;
      return(1);
    }
  }
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$+$i%G!<%?$r<u?.$9$k(B

            $BLa$jCM(B
	        0$B!'@5>o<u?.=*N;(B
		1$B!'<u?.%(%i!<(B
==================================================================================*/
int
iRecv_T(void)
{
  int  stat1;
  
  stat1 = _arc_read(ARC_ADDR);                           /* $B%G!<%?$N<u?.(B */
  
  if(stat1 == 0 ){
    printf("\nSERVO_DRIVER BRAKE OFF!!\n\n");
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$K!H@)8f=*N;%3%^%s%I!I$rAw?.$9$k(B

            $BLa$jCM(B
	        0$B!'@5>oAw?.=*N;(B
		1$B!'Aw?.%(%i!<(B
==================================================================================*/
int
iSend_E(void)
{
  int  stat1, rvstat;
  static int  timecnt = 0;
  unsigned int  TopAdr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));      /* $BAw?.%G!<%?NN0h(BClear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 0xff - (sizeof(char));
  _SendPacket[2] = TopAdr;
  _SendPacket[TopAdr  ] = 'r';
  _SendPacket[TopAdr+1] = 'E';
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);             /* $BAw?.(B */

  if(stat1 != 0){
    printf("\n write error (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  
  while(1){
    rvstat = iRecv_E();                             /* $B3NG'MQ$K%G!<%?$r<u?.$9$k(B */ 
    if (RecvFlag == TRUE){
      timecnt = 0;
      printf("Get Ready to Finish!\n\n");
      return(0);
    }
    timecnt++;
    if(timecnt > TIME_OUT2 ){
      printf("\ndata receive timeout  - iSend_E -\n");
      timecnt = 0;
      return(1);
    }
  }
}

/*=================================================================================
  $B%5!<%\%I%i%$%P$+$i%G!<%?$r<u?.$9$k(B

            $BLa$jCM(B(0)
==================================================================================*/
int
iRecv_E(void)
{
  int  stat1;
  
  stat1 = _arc_read(ARC_ADDR);                            /* $B%G!<%?$N<u?.(B */

  if(stat1 == 0 ){
    printf("\nBrake ON! -> ");
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  $B%"!<%/%M%C%H$N=i4|2=(B

            $BLa$jCM(B
	        0$B!'@5>o=i4|2==*N;(B
		1$B!'=i4|2=%(%i!<(B
==================================================================================*/
int
Arc_Init(void)
{
  int  jnt, stat1;
  
  stat1 = _arc_init(ARC_ADDR);                            /* $B%"!<%/%M%C%H$N=i4|2=(B */
  
  if (stat1 !=0){
    printf("Arc_Init -> error!! (status=%d)\n",stat1);
    return(ERROR);
  }
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status   = 0x0005;  /* $B%V%l!<%-(BON$B!"%5!<%\(BOFF$B!"%H%k%/@)8f(B */
    _Ccom.Drive[jnt].Torq     = 0;
    _Ccom.Drive[jnt].Speed    = 0;
  }
  return(OK);

}

/*=================================================================================
  $B3F4X@a$N8=:_3QEY>pJs$r<hF@$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
GetPosition(double *cur_pos)
{
  cur_pos[0] = _Axis.sts[0].Rez*REZ_TO_DEG*DEG_TO_RAD;
  cur_pos[1] = _Axis.sts[1].Rez*REZ_TO_DEG*DEG_TO_RAD;
  cur_pos[2] = _Axis.sts[2].Rez*REZ_TO_DEG*DEG_TO_RAD;
  cur_pos[3] = _Axis.sts[3].Rez*REZ_TO_DEG*DEG_TO_RAD;
  cur_pos[4] = _Axis.sts[4].Rez*REZ_TO_DEG*DEG_TO_RAD;
  cur_pos[5] = _Axis.sts[5].Rez*REZ_TO_DEG*DEG_TO_RAD;
  cur_pos[6] = _Axis.sts[6].Rez*REZ_TO_DEG*DEG_TO_RAD;
}

/*=================================================================================
  $B;XNa%H%k%/$r;XNaMQ%P%C%U%!$K%;%C%H$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
SetTorq(double *torq)
{
  int  jnt;
  double  dmytorq[7];
  
  for(jnt=0;jnt<7;jnt++){
    dmytorq[jnt] = torq[jnt] * TORQ_TO_DIGIT[jnt]; 

    _Ccom.Drive[jnt].Status = 0x0006;   /* $B%V%l!<%-(BOFF$B!"%5!<%\(BON$B!"%H%k%/@)8f(B */
    _Ccom.Drive[jnt].Torq   = (short)dmytorq[jnt];
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }
  
  _Ccom.Do = 0x0000;

  return;
}

/*=================================================================================
  $B%V%l!<%-(BON$B$r;XNa$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
NoTorq(void)
{
  int  jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0007;   /* $B%V%l!<%-(BON$B!"%5!<%\(BON$B!"%H%k%/@)8f(B */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  $B;XDj$7$?4X@a$r%V%l!<%-2r=|$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
BrakeOFF(int joint)
{
  int jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0007;   /* $B%V%l!<%-(BON$B!"%5!<%\(BON$B!"%H%k%/@)8f(B */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }
  
/*--- $B;XDj$5$l$?4X@a(B(joint)$B$r%V%l!<%-2r=|$9$k(B ---*/

  _Ccom.Drive[joint].Status = 0x0006;   /* $B%V%l!<%-(BOFF$B!"%5!<%\(BON$B!"%H%k%/@)8f(B */
  _Ccom.Drive[joint].Torq   = 0x0000;
  _Ccom.Drive[joint].Speed  = 0x0000;

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  $BA4<4%V%l!<%-2r=|$r;XNa$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
AllBrakeOFF(void)
{
  int jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0006;   /* $B%V%l!<%-(BOFF$B!"%5!<%\(BON$B!"%H%k%/@)8f(B */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  $B;XNa$7$?B.EY$rAw?.MQ%P%C%U%!$K%;%C%H$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
SetSpeed(int joint, double speed)
{
  int jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0003;   /* $B%V%l!<%-(BON$B!"%5!<%\(BON$B!"B.EY@)8f(B */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }
  
/*--- $B;XDj$5$l$?4X@a(B(joint)$B$rB.EY(B(speed)$B$GB.EY@)8f$9$k(B ---*/

  _Ccom.Drive[joint].Status = 0x0002;   /* $B%V%l!<%-(BOFF$B!"%5!<%\(BON$B!"B.EY@)8f(B */
  _Ccom.Drive[joint].Torq   = 0x0000;
  _Ccom.Drive[joint].Speed  = (short)speed;  /* digit */

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  $B%V%l!<%-(BON$B$r;XNa$9$k(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
NoSpeed(void)
{
  int  jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0003;   /* $B%V%l!<%-(BON$B!"%5!<%\(BON$B!"B.EY@)8f(B */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }

  _Ccom.Do = 0x0000;
}


/*=================================================================================
  $B%"!<%/%M%C%H$N%j%;%C%H(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
Arc_Reset(void)
{
  _arc_reset(ARC_ADDR);    
}

/*=================================================================================
  $BAw?.%G!<%?$NI=<((B

            $BLa$jCM$J$7(B
==================================================================================*/
void
PrintSampleData(void)
{
  unsigned int  SampleCNT1,SampleCNT2,SampleCNT3;
  unsigned int  SampleCNT4,SampleCNT5,SampleCNT6,SampleCNT7;
  

  SampleCNT1 = SampleData[2] + 2;
  SampleCNT2 = SampleCNT1 + 6;
  SampleCNT3 = SampleCNT2 + 6;
  SampleCNT4 = SampleCNT3 + 6;
  SampleCNT5 = SampleCNT4 + 6;
  SampleCNT6 = SampleCNT5 + 6;
  SampleCNT7 = SampleCNT6 + 6;
  
  printf("\n_MyID(0xFF) = 0x%4x,  _SDID(0xFE) = 0x%4x,  S_No = %x\n",
	 SampleData[0], SampleData[1], SampleData[2]);

  printf("Data_Code = '%c'\n\n", SampleData[SampleCNT1-1]);

  printf("Status1 = 0x%4x,     Torque1 = 0x%4x,     Speed1 = 0x%4x\n",
	 (SampleData[SampleCNT1  ] << 8) | SampleData[SampleCNT1+1],
	 (SampleData[SampleCNT1+2] << 8) | SampleData[SampleCNT1+3],
	 (SampleData[SampleCNT1+4] << 8) | SampleData[SampleCNT1+5]);
  
  printf("Status2 = 0x%4x,     Torque2 = 0x%4x,     Speed2 = 0x%4x\n",
	 (SampleData[SampleCNT2  ] << 8) | SampleData[SampleCNT2+1],
	 (SampleData[SampleCNT2+2] << 8) | SampleData[SampleCNT2+3],
	 (SampleData[SampleCNT2+4] << 8) | SampleData[SampleCNT2+5]);
  
  printf("Status3 = 0x%4x,     Torque3 = 0x%4x,     Speed3 = 0x%4x\n",
	 (SampleData[SampleCNT3  ] << 8) | SampleData[SampleCNT3+1],
	 (SampleData[SampleCNT3+2] << 8) | SampleData[SampleCNT3+3],
	 (SampleData[SampleCNT3+4] << 8) | SampleData[SampleCNT3+5]);
  
  printf("Status4 = 0x%4x,     Torque4 = 0x%4x,     Speed4 = 0x%4x\n",
	 (SampleData[SampleCNT4  ] << 8) | SampleData[SampleCNT4+1],
	 (SampleData[SampleCNT4+2] << 8) | SampleData[SampleCNT4+3],
	 (SampleData[SampleCNT4+4] << 8) | SampleData[SampleCNT4+5]);
  
  printf("Status5 = 0x%4x,     Torque5 = 0x%4x,     Speed5 = 0x%4x\n",
	 (SampleData[SampleCNT5  ] << 8) | SampleData[SampleCNT5+1],
	 (SampleData[SampleCNT5+2] << 8) | SampleData[SampleCNT5+3],
	 (SampleData[SampleCNT5+4] << 8) | SampleData[SampleCNT5+5]);
  
  printf("Status6 = 0x%4x,     Torque6 = 0x%4x,     Speed6 = 0x%4x\n",
	 (SampleData[SampleCNT6  ] << 8) | SampleData[SampleCNT6+1],
	 (SampleData[SampleCNT6+2] << 8) | SampleData[SampleCNT6+3],
	 (SampleData[SampleCNT6+4] << 8) | SampleData[SampleCNT6+5]);
  
  printf("Status7 = 0x%4x,     Torque7 = 0x%4x,     Speed7 = 0x%4x\n\n",
	 (SampleData[SampleCNT7  ] << 8) | SampleData[SampleCNT7+1],
	 (SampleData[SampleCNT7+2] << 8) | SampleData[SampleCNT7+3],
	 (SampleData[SampleCNT7+4] << 8) | SampleData[SampleCNT7+5]);
  
}

/*=================================================================================
  $BAw?.%G!<%?$NI=<((B

            $BLa$jCM$J$7(B
==================================================================================*/
void
PrintCcomData(void)
{
  printf("\nStatus1 = 0x%4x,     Torque1 = 0x%4x,     Speed1 = 0x%4x\n",
	 _Ccom.Drive[0].Status, _Ccom.Drive[0].Torq, _Ccom.Drive[0].Speed);

  printf("Status2 = 0x%4x,     Torque2 = 0x%4x,     Speed2 = 0x%4x\n",
	 _Ccom.Drive[1].Status, _Ccom.Drive[1].Torq, _Ccom.Drive[1].Speed);

  printf("Status3 = 0x%4x,     Torque3 = 0x%4x,     Speed3 = 0x%4x\n",
	 _Ccom.Drive[2].Status, _Ccom.Drive[2].Torq, _Ccom.Drive[2].Speed);

  printf("Status4 = 0x%4x,     Torque4 = 0x%4x,     Speed4 = 0x%4x\n",
	 _Ccom.Drive[3].Status, _Ccom.Drive[3].Torq, _Ccom.Drive[3].Speed);

  printf("Status5 = 0x%4x,     Torque5 = 0x%4x,     Speed5 = 0x%4x\n",
	 _Ccom.Drive[4].Status, _Ccom.Drive[4].Torq, _Ccom.Drive[4].Speed);

  printf("Status6 = 0x%4x,     Torque6 = 0x%4x,     Speed6 = 0x%4x\n",
	 _Ccom.Drive[5].Status, _Ccom.Drive[5].Torq, _Ccom.Drive[5].Speed);

  printf("Status7 = 0x%4x,     Torque7 = 0x%4x,     Speed7 = 0x%4x\n\n",
	 _Ccom.Drive[6].Status, _Ccom.Drive[6].Torq, _Ccom.Drive[6].Speed);

}

/*=================================================================================
  $B<u?.%G!<%?$NI=<((B

            $BLa$jCM$J$7(B
==================================================================================*/
void
PrintRecvData(void)
{
  int  RecvCNT1,RecvCNT2,RecvCNT3,RecvCNT4,RecvCNT5,RecvCNT6,RecvCNT7;
  
  RecvCNT1 = _RecvPacket[2] + 2;
  RecvCNT2 = RecvCNT1 + 10;
  RecvCNT3 = RecvCNT2 + 10;
  RecvCNT4 = RecvCNT3 + 10;
  RecvCNT5 = RecvCNT4 + 10;
  RecvCNT6 = RecvCNT5 + 10;
  RecvCNT7 = RecvCNT6 + 10;
  
  printf("\n_SDID(0xFE) = 0x%4x,  _MyID(0xFF) = 0x%4x,  S_No = %d\n",
	 _RecvPacket[0],_RecvPacket[1],_RecvPacket[2]);
  
  printf("Data_Code = '%c'\n\n", _RecvPacket[RecvCNT1-1]);

  printf("Status1 = 0x%4x,     Rezolver1 = 0x%8x,     Speed1 = 0x%4x,     Torque1 = 0x%4x\n",
	 (_RecvPacket[RecvCNT1  ] <<  8) | (_RecvPacket[RecvCNT1+1]),
	 (_RecvPacket[RecvCNT1+2] << 24) | (_RecvPacket[RecvCNT1+3] << 16) |
	 (_RecvPacket[RecvCNT1+4] <<  8) | (_RecvPacket[RecvCNT1+5]),
	 (_RecvPacket[RecvCNT1+6] <<  8) | (_RecvPacket[RecvCNT1+7]),
	 (_RecvPacket[RecvCNT1+8] <<  8) | (_RecvPacket[RecvCNT1+9]));
    
  printf("Status2 = 0x%4x,     Rezolver2 = 0x%8x,     Speed2 = 0x%4x,     Torque2 = 0x%4x\n",
	 (_RecvPacket[RecvCNT2  ] <<  8) | (_RecvPacket[RecvCNT2+1]),
	 (_RecvPacket[RecvCNT2+2] << 24) | (_RecvPacket[RecvCNT2+3] << 16) |
	 (_RecvPacket[RecvCNT2+4] <<  8) | (_RecvPacket[RecvCNT2+5]),
	 (_RecvPacket[RecvCNT2+6] <<  8) | (_RecvPacket[RecvCNT2+7]),
	 (_RecvPacket[RecvCNT2+8] <<  8) | (_RecvPacket[RecvCNT2+9]));
   
  printf("Status3 = 0x%4x,     Rezolver3 = 0x%8x,     Speed3 = 0x%4x,     Torque3 = 0x%4x\n",
	 (_RecvPacket[RecvCNT3  ] <<  8) | (_RecvPacket[RecvCNT3+1]),
	 (_RecvPacket[RecvCNT3+2] << 24) | (_RecvPacket[RecvCNT3+3] << 16) |
	 (_RecvPacket[RecvCNT3+4] <<  8) | (_RecvPacket[RecvCNT3+5]),
	 (_RecvPacket[RecvCNT3+6] <<  8) | (_RecvPacket[RecvCNT3+7]),
	 (_RecvPacket[RecvCNT3+8] <<  8) | (_RecvPacket[RecvCNT3+9]));
  
  printf("Status4 = 0x%4x,     Rezolver4 = 0x%8x,     Speed4 = 0x%4x,     Torque4 = 0x%4x\n",
	 (_RecvPacket[RecvCNT4  ] <<  8) | (_RecvPacket[RecvCNT4+1]),
	 (_RecvPacket[RecvCNT4+2] << 24) | (_RecvPacket[RecvCNT4+3] << 16) |
	 (_RecvPacket[RecvCNT4+4] <<  8) | (_RecvPacket[RecvCNT4+5]),
	 (_RecvPacket[RecvCNT4+6] <<  8) | (_RecvPacket[RecvCNT4+7]),
	 (_RecvPacket[RecvCNT4+8] <<  8) | (_RecvPacket[RecvCNT4+9]));

  printf("Status5 = 0x%4x,     Rezolver5 = 0x%8x,     Speed5 = 0x%4x,     Torque6 = 0x%4x\n",
	 (_RecvPacket[RecvCNT5  ] <<  8) | (_RecvPacket[RecvCNT5+1]),
	 (_RecvPacket[RecvCNT5+2] << 24) | (_RecvPacket[RecvCNT5+3] << 16) |
	 (_RecvPacket[RecvCNT5+4] <<  8) | (_RecvPacket[RecvCNT5+5]),
	 (_RecvPacket[RecvCNT5+6] <<  8) | (_RecvPacket[RecvCNT5+7]),
	 (_RecvPacket[RecvCNT5+8] <<  8) | (_RecvPacket[RecvCNT5+9]));

  printf("Status6 = 0x%4x,     Rezolver6 = 0x%8x,     Speed6 = 0x%4x,     Torque6 = 0x%4x\n",
	 (_RecvPacket[RecvCNT6  ] <<  8) | (_RecvPacket[RecvCNT6+1]),
	 (_RecvPacket[RecvCNT6+2] << 24) | (_RecvPacket[RecvCNT6+3] << 16) |
	 (_RecvPacket[RecvCNT6+4] <<  8) | (_RecvPacket[RecvCNT6+5]),
	 (_RecvPacket[RecvCNT6+6] <<  8) | (_RecvPacket[RecvCNT6+7]),
	 (_RecvPacket[RecvCNT6+8] <<  8) | (_RecvPacket[RecvCNT6+9]));

  printf("Status7 = 0x%4x,     Rezolver7 = 0x%8x,     Speed7 = 0x%4x,     Torque7 = 0x%4x\n\n",
	 (_RecvPacket[RecvCNT7  ] <<  8) | (_RecvPacket[RecvCNT7+1]),
	 (_RecvPacket[RecvCNT7+2] << 24) | (_RecvPacket[RecvCNT7+3] << 16) |
	 (_RecvPacket[RecvCNT7+4] <<  8) | (_RecvPacket[RecvCNT7+5]),
	 (_RecvPacket[RecvCNT7+6] <<  8) | (_RecvPacket[RecvCNT7+7]),
	 (_RecvPacket[RecvCNT7+8] <<  8) | (_RecvPacket[RecvCNT7+9]));

}

/*=================================================================================
  $B<u?.%G!<%?$NI=<((B

            $BLa$jCM$J$7(B
==================================================================================*/
void
PrintAxisData(void)
{
  printf("\nStatus1 = 0x%4x,     Rezolver1 = 0x%8lx,     Speed1 = 0x%4x,     Torque1 = 0x%4x\n",
	 _Axis.sts[0].Status, _Axis.sts[0].Rez, _Axis.sts[0].Speed, _Axis.sts[0].Torq);

  printf("Status2 = 0x%4x,     Rezolver2 = 0x%8lx,     Speed2 = 0x%4x,     Torque2 = 0x%4x\n",
	 _Axis.sts[1].Status, _Axis.sts[1].Rez, _Axis.sts[1].Speed, _Axis.sts[1].Torq);

  printf("Status3 = 0x%4x,     Rezolver3 = 0x%8lx,     Speed3 = 0x%4x,     Torque3 = 0x%4x\n",
	 _Axis.sts[2].Status, _Axis.sts[2].Rez, _Axis.sts[2].Speed, _Axis.sts[2].Torq);

  printf("Status4 = 0x%4x,     Rezolver4 = 0x%8lx,     Speed4 = 0x%4x,     Torque4 = 0x%4x\n",
	 _Axis.sts[3].Status, _Axis.sts[3].Rez, _Axis.sts[3].Speed, _Axis.sts[3].Torq);

  printf("Status5 = 0x%4x,     Rezolver5 = 0x%8lx,     Speed5 = 0x%4x,     Torque5 = 0x%4x\n",
	 _Axis.sts[4].Status, _Axis.sts[4].Rez, _Axis.sts[4].Speed, _Axis.sts[4].Torq);

  printf("Status6 = 0x%4x,     Rezolver6 = 0x%8lx,     Speed6 = 0x%4x,     Torque6 = 0x%4x\n",
	 _Axis.sts[5].Status, _Axis.sts[5].Rez, _Axis.sts[5].Speed, _Axis.sts[5].Torq);

  printf("Status7 = 0x%4x,     Rezolver7 = 0x%8lx,     Speed7 = 0x%4x,     Torque7 = 0x%4x\n\n",
	 _Axis.sts[6].Status, _Axis.sts[6].Rez, _Axis.sts[6].Speed, _Axis.sts[6].Torq);

}

/*=================================================================================
  $B%^%9%?!<%5!<%\%9%F!<%?%9$NI=<((B

            $BLa$jCM$J$7(B
==================================================================================*/
void
MS(void)
{
  printf("Master Status = 0x%2x\n", _Axis.MStatus);
}

/*=================================================================================
  $B;~4VBT$A(B1

            $BLa$jCM$J$7(B
==================================================================================*/
void
delay1(int time1)
{
  int i;
  /*
  for(i=0;i<100000*time1;i++);
  */
  for(i=0;i<2000000*time1;i++);

}

/*=================================================================================
  $B;~4VBT$A(B2

            $BLa$jCM$J$7(B
==================================================================================*/
void
iodelay(void)
{
  int i;
  /*
  for(i=0;i<100;i++);
  */
  for(i=0;i<2000;i++);

}

/*=================================================================================
  $B%"!<%/%M%C%H$N=i4|2=(B

            $BLa$jCM$J$7(B
==================================================================================*/
int
arcInit(void)
{
  int stat1;

  stat1 = Arc_Init();    /* $B%"!<%/%M%C%H$N=i4|2=(B */
  if(stat1 == ERROR){
    printf("ERROR ARC-PCI/22 DRIVER: initialization faile\n");
    return ERROR;
  }
  printf("ARC-PCI/22   Initialized at base address 0x%x\n", ARC_ADDR);

  return OK;
}

/*=================================================================================
  $B%"!<%/%M%C%H!&DL?.3+;O(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
arcnet_start(void)
{
  int stat1;

  switch(CtrlMode){
  case operation:
    stat1 = iSend_S();
    if (stat1 != 0){
      printf("\narcnet cannot start\n");
    }
    break;
  case simulation:
    break;
 
  default:
    break;
  }
}

/*=================================================================================
  $B%"!<%/%M%C%H$N=*N;(B

            $BLa$jCM$J$7(B
==================================================================================*/
void
arcFin(void)
{
  int stat1;

  switch(CtrlMode){
  case operation:
    stat1 = iSend_E();
    break;
  case simulation:
    break;
 
  default:
    break;
  }
}
