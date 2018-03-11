#ifndef _ARC_PCI_CFG_H
#define _ARC_PCI_CFG_H

#ifndef SHORT_PACKET
#define	SHORT_PACKET  (256)
#endif

#ifndef _MyID
#define	_MyID  (0xff)     /* $B%"!<%/%M%C%H%\!<%I$N(BID No. */
#endif

#ifndef _SDID
#define	_SDID  (0xfe)     /* $B%5!<%\%I%i%$%P$N(BID No. */
#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0xec00)//(0xc400)	/* Acrnet board base address (coral) 2015 3/30 ‹ààV’Ç‰Á */
//#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0x5000)//(0xc400)	/* Acrnet board base address (jade) 2015 3/30 ‹ààV’Ç‰Á */
//#endif

//#ifndef ARC_ADDR
//#define ARC_ADDR	(0xb000)//(0xc400)	/* Acrnet board base address (finger) 2015 3/30 ‹ààV’Ç‰Á */
//#endif

#ifndef ARC_ADDR
#define	ARC_ADDR    (0xc800)  /* $B%"!<%/%M%C%H%\!<%I(B(PCI3)$B!&%Y!<%9%"%I%l%9(B */
#endif

/* #ifndef ARC_ADDR */
/* #define	ARC_ADDR_R  (0xe000)  /\* $B%"!<%/%M%C%H%\!<%I(B(PCI2)$B!&%Y!<%9%"%I%l%9(B *\/ */
/* #endif */

#ifndef PI
#define	PI  (3.14159265358979323846)
#endif

#ifndef REZ_TO_DEG
#define	REZ_TO_DEG     (1.0/2275.55555)
#endif

#ifndef DEG_TO_RAD
#define	DEG_TO_RAD     (PI/180.0)
#endif

#ifndef TORQ_TO_DIGIT1
#define	TORQ_TO_DIGIT1  ((4096./4.60) / 50.)     /* 0.00112[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT2
#define	TORQ_TO_DIGIT2  ((4096./4.60) / 50.)     /* 0.00112[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT3
#define	TORQ_TO_DIGIT3  ((4096./2.00) / 50.)     /* 0.000488[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT4
#define	TORQ_TO_DIGIT4  ((4096./2.00) / 50.)     /* 0.000488[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT5
#define	TORQ_TO_DIGIT5  ((4096./0.29) / 50.)    /* 0.0000708[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT6
#define	TORQ_TO_DIGIT6  ((4096./0.29) / 50.)    /* 0.0000708[Nm/digit] */
#endif

#ifndef TORQ_TO_DIGIT7
#define	TORQ_TO_DIGIT7  ((4096./0.29) / 50.)    /* 0.0000708[Nm/digit] */
#endif

#ifndef TIMEOUT
#define	TIMEOUT	   (-1)
#endif
/*
#ifndef TIME_OUT
#define	TIME_OUT   (65535)
#endif
*/
#ifndef TIME_OUT
#define	TIME_OUT   (655350*2)
#endif
/*
#ifndef TIME_OUT2
#define	TIME_OUT2  (1048575)
#endif
*/
#ifndef TIME_OUT2
#define	TIME_OUT2  (10485750*2)
#endif


/*========== $B%5!<%\%I%i%$%P$H$NDL?.%G!<%?%U%)!<%^%C%H(B =======================*/

/*---------- $BAw?.%G!<%?(B ----------*/

typedef struct {
    unsigned short  Status;        /* $B%9%F!<%?%9(B */
    short           Torq;          /* $B%H%k%/;XNaCM(B($BEEN.CM!K(B*/
    short           Speed;         /* $B4X@aB.EY;XNaCM(B */
} C_COM;

typedef struct {
    unsigned char   ChkNo;        /* (I<09]<,Y(BNo. */
    unsigned char   Code;         /* $B%G!<%?$N<oN`(B */
    C_COM           Drive[7];     /* $B#1!A(B8$B<4;XNaCM(B */
    unsigned short  Do;           /* (IC^<^@Y$B=PNO!'(B8ch */
} SEND_C;

/*---------- $B<u?.%G!<%?(B ----------*/

typedef struct {
    unsigned short  Status;       /* $B%9%F!<%?%9(B */
    long            Rez;          /* $B4X@a3QEY!J%l%>%k%PCM!K(B*/
    short           Speed;        /* $B8=:_B.EYCM(B */ 
    short           Torq;         /* $B8=:_%H%k%/CM(B */
} RC_COM;


typedef struct {
    unsigned char   ChkNo;        /* (I<09]<,Y(BNo.	*/
    unsigned char   Code;         /* $B%G!<%?$N<oN`(B */
    RC_COM          sts[7];       /* 1$B!A(B8$B<4%9%F!<%?%9(B */
    unsigned short  MStatus;     /* Master$B%9%F!<%?%9(B */
    unsigned short  Di;           /* (IC^<^@Y$BF~NO!'(B8ch */
    unsigned short  Ai[4];        /* $B%"%J%m%0F~NO!'(B4ch */
} RECV_C;

#endif  /* _ARC_PCI_CFG_H */
