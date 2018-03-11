/*******************************************************
	     
	     ARC Net Driver for PA-10

********************************************************/

#include "../include.h"

#include "../main_cfg.h"

#include "arc_pci.h"
#include "arc_pci_cfg.h"
 
#include <hw/inout.h> 
 
/*==================== 変数の宣言 ================================================*/

unsigned char	_SendPacket[256];    /* サーボドライバへの送信バッファ */
unsigned char	_RecvPacket[256];    /* サーボドライバからの受信バッファ */

SEND_C   _Ccom;          /* 送信データ */
RECV_C   _Axis;          /* 受信データ */

int  RecvFlag = FALSE;
int  RvCNT    = FALSE;

unsigned char  SampleData[256];

static	int	PageNo;

const double TORQ_TO_DIGIT[7] = { TORQ_TO_DIGIT1, TORQ_TO_DIGIT2, TORQ_TO_DIGIT3, 
				  TORQ_TO_DIGIT4, TORQ_TO_DIGIT5, TORQ_TO_DIGIT6,
				  TORQ_TO_DIGIT7 };

/*==================================================================================
  アークネットの初期化
  
            戻り値
	        0：正常初期化終了
		1,2,3,4：内部自己診断により、エラー有り
==================================================================================*/
int
_arc_init(int BaseAddress)
{
  unsigned int	i;
  unsigned int	str, cmr, diag, ahr, alr, dtr, sar, cfr, ndr, stup1, stup2;
  
/*--- 各レジスタのアドレス設定 ---*/  

  str   = BaseAddress + 0x00;  /* ステータス・レジスタ */
  cmr   = BaseAddress + 0x04;  /* コマンド・レジスタ */
  diag  = BaseAddress + 0x04;  /* ダイアグノスティック・レジスタ */
  ahr   = BaseAddress + 0x08;  /* アドレスポインタ（上位）・レジスタ */
  alr   = BaseAddress + 0x0C;  /* アドレスポインタ（下位）・レジスタ */
  dtr   = BaseAddress + 0x10;  /* データ・レジスタ */
  sar   = BaseAddress + 0x14;  /* サブアドレス・レジスタ */
  cfr   = BaseAddress + 0x18;  /* コンフィグレーション・レジスタ */

  ndr   = BaseAddress + 0x1c;  /* ノードID・レジスタ */
  stup1 = BaseAddress + 0x1c;  /* セットアップ1・レジスタ */
  stup2 = BaseAddress + 0x1c;  /* セットアップ2・レジスタ */

/*----- 初期化 ----------*/

  out8( cfr, 0x80 );             /* ｿﾌﾄｳｪｱﾘｾｯﾄ */
  iodelay();
  out8( cfr, 0x02 );             /* Software Reset の解除 */
                                       /* ｾｯﾄｱｯﾌﾟ1・ﾚｼﾞｽﾀをｱｸｾｽ可能にする */
  delay1(1);          	               /* ｿﾌﾄｳｪｱﾘｾｯﾄ後の時間待ち(800ns) */

  out8( stup1, 0x81 );	       /* ｾｯﾄｱｯﾌﾟ1・ﾚｼﾞｽﾀの設定 */
                                       /* BP時 nPULSE1端子:push/pull出力*/
                                       /* ｱｰﾋﾞﾄﾚｰｼｮﾝが2ｸﾛｯｸ･ｻｲｸﾙ(10MHz) */
  
  out8( cfr, 0x19 | 0x04 );      /* TXEN=0 (ID決定後に有効にする)*/
                                       /* ﾉｰﾄﾞID・ﾚｼﾞｽﾀをｱｸｾｽ可能にする */
                                       /* ﾊﾞｯｸﾌﾟﾚｰﾝﾓｰﾄﾞ(BP)の選択 */
  out8( ndr, _MyID );            /* ﾉｰﾄﾞID・ﾚｼﾞｽﾀに_MyID(0xff)を設定 */
  delay1(3);                            /* ﾉｰﾄﾞID・ﾚｼﾞｽﾀの設定 */

/*----- 内部自己診断 -----*/ 

  out8( ahr, 0x80 );             /* 次のﾃﾞｰﾀ・ﾚｼﾞｽﾀへのｱｸｾｽを"Read"に設定 */

/* チェック１ */
  out8( alr, 0x00 );	       /* ｱﾄﾞﾚｽ 000H = D1H ? */
  for(i=0;i<TIME_OUT;i++){             
    if( in8( dtr ) == 0xd1 )    break;
  }
  if(i==TIME_OUT)   return(1);	

/* チェック２ */  
  out8( alr, 0x01 );	       /* ｱﾄﾞﾚｽ 001H = _MyID ? */
  for(i=0;i<TIME_OUT;i++){
    if( in8( dtr ) == _MyID )   break;
    }
  if(i==TIME_OUT)   return(2);	

/* チェック３ */
  for(i=0;i<TIME_OUT;i++){
    if(( in8( diag ) & 0x30 ) == 0x00 )  break;
  }
  if(i==TIME_OUT)   return(3);

/* チェック４ */  
  delay1(10);                           /* TXEN=0→840ms経過待ちWait */
  for(i=0;i<TIME_OUT;i++){	       /* DUP-ID = 0 ?	*/
    if(( in8( diag ) & 0x40 ) == 0x00)    break;
  }
  if(i==TIME_OUT)   return(4);

/*----- 設定 -----*/
  
  out8( sar, 0x04 );	       /* ｾｯﾄｱｯﾌﾟ2・ﾚｼﾞｽﾀをｱｸｾｽ可能にする */
  out8( stup2, 0xbc );	       /* CKUP1,0:てい倍回路の設定:10MHz*/
                                       /* 高速CPU対応にする */
  delay1(1);                            /* CKUP1,0の変更後のWait(1ms) */
  out8( cmr, 0x18 );	       /* CKUP0,CKUP1変更後の再起動 */

  out8( cfr, 0x39 | 0x04 );      /* "TXEN"=1（ﾈｯﾄﾜｰｸに参加する）*/
                                       /* ﾊﾞｯｸﾌﾟﾚｰﾝﾓｰﾄﾞ(BP)の選択 */
  out8( cmr, 0x0d );             /* ｼｮｰﾄ・ﾛﾝｸﾞﾊﾟｹｯﾄの両方を受信可能に */
  out8( cmr, 0x84 );             /* ﾌﾞﾛｰﾄﾞｷｬｽﾄﾊﾟｹｯﾄも受信可能にする */
  
  PageNo= 0;

  return(0);
}

/*=================================================================================
  アークネットへのデータ送信（ショートパケット使用）

       送信用バッファ：_SendPacket[0〜255]
            _SendPacket[0] ... _MyID          = 0xff
            _SendPacket[1] ... _SDID          = 0xfe
            _SendPacket[2] ... ｺﾏﾝﾄﾞ先頭アドレス（256 - データ数）
                    /
                    /
            _SendPacket[_SendPacket[2]]   ... ｼｰｹﾝｼｬﾙNo.
            _SendPacket[_SendPacket[2]+1] ... ｺﾏﾝﾄﾞ = 'S','C','E'...etc
	    _SendPacket[_SendPacket[2]+2] ... ﾃﾞｰﾀ
	            /
	    _SendPacket[255]                ... ﾃﾞｰﾀ

       戻り値　
           0：正常送信終了
           1：送信可能状態ではない（待ち状態の送信コマンド有り）
	   2：相手からACKの返信がない
==================================================================================*/
int
_arc_send(int BaseAddress, unsigned char *_SendPacket1)
{
  unsigned int	i;
  unsigned int	str, mask, cmr, diag, ahr, alr, dtr, sar, cfr, ndr, stup1, stup2;
  
/*--- 各レジスタのアドレス設定 ---*/  

  str   = BaseAddress + 0x00;  /* ステータス・レジスタ */
  mask  = BaseAddress + 0x00;  /* 割り込みマスク・レジスタ */
  cmr   = BaseAddress + 0x04;  /* コマンド・レジスタ */
  diag  = BaseAddress + 0x04;  /* ダイアグノスティック・レジスタ */
  ahr   = BaseAddress + 0x08;  /* アドレスポインタ（上位）・レジスタ */
  alr   = BaseAddress + 0x0C;  /* アドレスポインタ（下位）・レジスタ */
  dtr   = BaseAddress + 0x10;  /* データ・レジスタ */
  sar   = BaseAddress + 0x14;  /* サブアドレス・レジスタ */
  cfr   = BaseAddress + 0x18;  /* コンフィグレーション・レジスタ */

  ndr   = BaseAddress + 0x1c;  /* ノードID・レジスタ */
  stup1 = BaseAddress + 0x1c;  /* セットアップ1・レジスタ */
  stup2 = BaseAddress + 0x1c;  /* セットアップ2・レジスタ */
  
/*----- 送信可能状態のチェック -----*/

  for(i=0;i<TIME_OUT;i++){              /* 前の送信要求命令・終了 ? */
    if((in8(str) & 0x01) != 0x00)   break;
  }
  if(i==TIME_OUT)    return(1);         /* 送信中(TA=0) */
  
/*----- 2ページ目に送信データをセットする -----*/

  out8( ahr, 0x04 );              /* 次のﾃﾞｰﾀ・ﾚｼﾞｽﾀへのｱｸｾｽを"Write"に設定 */
                                        /* 2ページ目(0x4??H)に設定 */
  out8( alr, 0x00 );		/* ｱﾄﾞﾚｽを0x400Hに設定 */
  out8( dtr, _MyID );		/* 送信元ＩＤ(_MyID) */

  out8( alr, 0x01 );              /* ｱﾄﾞﾚｽを0x401Hに設定 */
  out8( dtr, _SendPacket1[1] );  /* 送信先ＩＤ (_SDID) */

  out8( alr, 0x02 );              /* ｱﾄﾞﾚｽを0x402Hに設定 */
  out8( dtr, _SendPacket1[2] );  /* データ先頭アドレス */

  out8( ahr, 0x44 );              /* ｵｰﾄｲﾝｸﾘﾒﾝﾄ　＋　ｱﾄﾞﾚｽｾｯﾄ(0x400H) */
  out8( alr, _SendPacket1[2] );  /* データ先頭アドレスセット */
  for(i=_SendPacket[2];i!=0x100;i++){ /* データセット*/
    out8( dtr, _SendPacket1[i] );
  }

  out8( mask, 0x00 );		/* 割り込みの禁止 */
  
/*----- 送信可能状態をチェック -----*/

  if((in8(str) & 0x01) == 0x00 ){ /* 前の送信要求命令・終了 ? */
    out8( cmr, 0x01 );	        /* 送信可能状態でないため、送信命令をキャンセルする */
    return(1);
  }

/*----- 送信 -----*/

  out8( cmr, 0x13 );		/* 2ページ目を送信する */

  if(_SendPacket1[1] != 0x00){         /* BroadCast方式でない場合*/           
    for(i=0;i<TIME_OUT;i++){            /* ACK（正常応答）の返信チェック */
      if(( in8( str ) & 0x03 ) == 0x03 )	break;  /* 正常に送信完了 */
    }
    if(i == TIME_OUT){
      out8( cmr, 0x01 );	        /* 送信可能状態でないため、送信命令をキャンセルする */
      return(2);
    }
  }
  return(0);
}

/*=================================================================================
  アークネットのデータ受信

            受信用バッファ：_RecvPacket[0〜255]

	         _RecvPacket[0] ... _SDID
		 _RecvPacket[1] ... _MyID
		 _RecvPacket[2] ... ｺﾏﾝﾄﾞ先頭ｱﾄﾞﾚｽ（256 - ﾃﾞｰﾀ数）
		         /
			 /
		 _RecvPacket[_RecvPacket[2]]   ... ｼｰｹﾝｼｬﾙNo.
		 _RecvPacket[_RecvPacket[2]+1] ... ｺﾏﾝﾄﾞ = 'S','C','E'...etc
		 _RecvPacket[_RecvPacket[2]+2] ... ﾃﾞｰﾀ
		 　　　　/
		 _RecvPacket[255]                ... ﾃﾞｰﾀ

	    戻り値
	        0：正常受信終了
		1：待ち状態の受信コマンド有り
==================================================================================*/
int
_arc_read(int BaseAddress)
{
  unsigned int	i;
  unsigned char  datAdd;
  unsigned char  *ptr;
  unsigned int	str, mask, cmr, diag, ahr, alr, dtr, sar, cfr, ndr, stup1, stup2;
  
/*--- 各レジスタのアドレス設定 ---*/  

  str   = BaseAddress + 0x00;  /* ステータス・レジスタ */
  mask  = BaseAddress + 0x00;  /* 割り込みマスク・レジスタ */
  cmr   = BaseAddress + 0x04;  /* コマンド・レジスタ */
  diag  = BaseAddress + 0x04;  /* ダイアグノスティック・レジスタ */
  ahr   = BaseAddress + 0x08;  /* アドレスポインタ（上位）・レジスタ */
  alr   = BaseAddress + 0x0C;  /* アドレスポインタ（下位）・レジスタ */
  dtr   = BaseAddress + 0x10;  /* データ・レジスタ */
  sar   = BaseAddress + 0x14;  /* サブアドレス・レジスタ */
  cfr   = BaseAddress + 0x18;  /* コンフィグレーション・レジスタ */

  ndr   = BaseAddress + 0x1c;  /* ノードID・レジスタ */
  stup1 = BaseAddress + 0x1c;  /* セットアップ1・レジスタ */
  stup2 = BaseAddress + 0x1c;  /* セットアップ2・レジスタ */
   
/*----- 待ち状態の受信コマンドのチェック -----*/

  out8( mask, 0x00 );	           /* 受信割り込みの禁止 */
  if(( in8( str ) & 0x80 ) == 0x00 ) /* 待ち状態の受信コマンド 有 ? */
    return(1);
  
/*----- 受信ページの選択 -----*/

  out8( mask, 0x00 );              /* 受信割り込みの禁止 */
  
  if(PageNo==1){                         /* 1ページ目を選択 */
    out8( ahr, 0xc2 );             /* 次のﾃﾞｰﾀ・ﾚｼﾞｽﾀへのｱｸｾｽを"Read"に設定 */
                                         /* ｵｰﾄｲﾝｸﾘﾒﾝﾄ */
                                         /* 1ページ目(0x200H)に設定 */
    out8( cmr, 0x84 );             /* 次は0ページ目(0x000H)を受信する */
    PageNo=0;                            /* 次は0ページ目を選択する */
  }
  else{                                  /* 0ページ目を選択 */
    out8( ahr, 0xc0 );             /* 次のﾃﾞｰﾀ・ﾚｼﾞｽﾀへのｱｸｾｽを"Read"に設定 */
                                         /* ｵｰﾄｲﾝｸﾘﾒﾝﾄ */
                                         /* 0ページ目(0x000H)に設定 */
    out8( cmr, 0x8c );             /* 次は1ページ目(0x200H)を受信する */
    PageNo=1;                            /* 次は1ページ目を選択する */
  }

/*----- 受信 -----*/
  
  ptr = _RecvPacket;
  RecvFlag = TRUE;

  out8( alr, 0x00 );               /* ｱﾄﾞﾚｽを0x?00Hに設定 */
  *(ptr+0) = in8( dtr );           /* 送信元IDの読み込み */
  
  out8( alr, 0x01 );               /* ｱﾄﾞﾚｽを0x?01Hに設定 */
  *(ptr+1) = in8( dtr );           /* 送信先IDの読み込み */

  out8( alr, 0x02 );               /* ｱﾄﾞﾚｽを0x?02Hに設定 */
  datAdd = in8(dtr);               /* ｼｰｹﾝｼｬﾙNo.のｱﾄﾞﾚｽの読み込み */
  *(ptr+2) = datAdd;                     /* ｼｰｹﾝｼｬﾙNo.のｱﾄﾞﾚｽをｾｯﾄ */

  out8( alr, datAdd );
  for(i=datAdd; i!=0x100; i++){          /* データ読み込み */
    *(ptr+i) = in8(dtr);
  }

  return(0);
}
 
/*==================================================================================
  アークネット(ARC-PCI/22)のリセット

            戻り値なし
==================================================================================*/
void
_arc_reset(int BaseAddress)
{
  unsigned int	Config;                    /* コンフィグレーション・レジスタ */
  
  Config = BaseAddress + 0x18;
  out8( Config, 0x80 );	           /* ソフトウェア・リセット */
}

/*=================================================================================
  サーボドライバに“制御開始コマンド”を送信する

            戻り値
	        0：アークネット送信可能
		1：送信エラー
==================================================================================*/
int
iSend_S(void)
{
  int  stat1, rvstat;
  static int  timecnt = 0;
  unsigned int  TopAdr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));   /*送信データ領域Clear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 0xff - (sizeof(char));
  _SendPacket[2] = TopAdr;
  _SendPacket[TopAdr  ] = 'r';
  _SendPacket[TopAdr+1] = 'S';
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);          /* 送信 */

  if(stat1 != 0){
    printf("\n write error   (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  
  while(1){
    rvstat = iRecv_S();                            /* 確認用にデータを受信する */ 
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
  サーボドライバからデータを受信する

           戻り値（0）
==================================================================================*/
int
iRecv_S(void)
{
  int  stat1;
  
  stat1 = _arc_read(ARC_ADDR);                          /* データの受信 */

  if(stat1 == 0 ){
    printf("\nArcnet for PA10 O.K! -> ");
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  サーボドライバに“制御コマンド”および制御データを送信する

            戻り値
	        0：正常送信終了
		1：送信エラー
==================================================================================*/
int
iSend_C(void)
{
  int  jnt, stat1;
  unsigned int  TopAdr, adr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));    /* 送信データ領域Clear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 210;
  _SendPacket[2] = TopAdr;                           /* 先頭ｱﾄﾞﾚｽｾｯﾄ */
  _SendPacket[TopAdr  ] = 'r';                       /* ｼｰｹﾝｼｬﾙNo. */
  _SendPacket[TopAdr+1] = 'C';                       /* データ種類 */
  
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
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);          /* 送信 */

  if(stat1 != 0){
    printf("\n write error (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  RvCNT = TRUE;

  return OK;
}

/*=================================================================================
  サーボドライバからデータを受信し、受信用ﾊﾞｯﾌｧに格納する

            戻り値(0)
==================================================================================*/
int
iRecv_C(void)
{
  int  jnt, stat1;
  unsigned char  RecvCNT;
  unsigned short  status;
  long  rezolver;
  short  speed, torq;
  
  stat1 = _arc_read(ARC_ADDR);                         /* データの受信 */
  
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
  サーボドライバからデータを受信し、受信用ﾊﾞｯﾌｧに格納する

            戻り値(0)
==================================================================================*/
int
RecData(void)
{
  int rvstat;
  static int timecnt = 0;
  
  while(1){
    rvstat = iRecv_C();                        /* 確認用にデータを受信する */ 
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
  サーボドライバに“ブレーキ解除コマンド”を送信する

            戻り値
	        0：正常送信終了
		1：送信エラー
==================================================================================*/
int
iSend_T(void)
{
  int  stat1,rvstat;
  static int  timecnt = 0;
  unsigned int  TopAdr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));      /* 送信データ領域Clear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 0xff - (sizeof(char));
  _SendPacket[2] = TopAdr;
  _SendPacket[TopAdr  ] = 'r';
  _SendPacket[TopAdr+1] = 'T';
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);             /* 送信 */

  if(stat1 != 0){
    printf("\n write error (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  
  while(1){
    rvstat = iRecv_T();                             /* 確認用にデータを受信する */ 
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
  サーボドライバからデータを受信する

            戻り値
	        0：正常受信終了
		1：受信エラー
==================================================================================*/
int
iRecv_T(void)
{
  int  stat1;
  
  stat1 = _arc_read(ARC_ADDR);                           /* データの受信 */
  
  if(stat1 == 0 ){
    printf("\nSERVO_DRIVER BRAKE OFF!!\n\n");
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  サーボドライバに“制御終了コマンド”を送信する

            戻り値
	        0：正常送信終了
		1：送信エラー
==================================================================================*/
int
iSend_E(void)
{
  int  stat1, rvstat;
  static int  timecnt = 0;
  unsigned int  TopAdr;
  
  memset(_SendPacket,0x00,sizeof(_SendPacket));      /* 送信データ領域Clear */
  
  _SendPacket[0] = _MyID;
  _SendPacket[1] = _SDID;
  
  TopAdr = 0xff - (sizeof(char));
  _SendPacket[2] = TopAdr;
  _SendPacket[TopAdr  ] = 'r';
  _SendPacket[TopAdr+1] = 'E';
  
  stat1 = _arc_send(ARC_ADDR,_SendPacket);             /* 送信 */

  if(stat1 != 0){
    printf("\n write error (status=%d)\n",stat1);
    return(ERROR);
  }
  
  RecvFlag = FALSE;
  
  while(1){
    rvstat = iRecv_E();                             /* 確認用にデータを受信する */ 
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
  サーボドライバからデータを受信する

            戻り値(0)
==================================================================================*/
int
iRecv_E(void)
{
  int  stat1;
  
  stat1 = _arc_read(ARC_ADDR);                            /* データの受信 */

  if(stat1 == 0 ){
    printf("\nBrake ON! -> ");
    return(0);
  }
  else 
    return(1);
}

/*=================================================================================
  アークネットの初期化

            戻り値
	        0：正常初期化終了
		1：初期化エラー
==================================================================================*/
int
Arc_Init(void)
{
  int  jnt, stat1;
  
  stat1 = _arc_init(ARC_ADDR);                            /* アークネットの初期化 */
  
  if (stat1 !=0){
    printf("Arc_Init -> error!! (status=%d)\n",stat1);
    return(ERROR);
  }
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status   = 0x0005;  /* ブレーキON、サーボOFF、トルク制御 */
    _Ccom.Drive[jnt].Torq     = 0;
    _Ccom.Drive[jnt].Speed    = 0;
  }
  return(OK);

}

/*=================================================================================
  各関節の現在角度情報を取得する

            戻り値なし
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
  指令トルクを指令用バッファにセットする

            戻り値なし
==================================================================================*/
void
SetTorq(double *torq)
{
  int  jnt;
  double  dmytorq[7];
  
  for(jnt=0;jnt<7;jnt++){
    dmytorq[jnt] = torq[jnt] * TORQ_TO_DIGIT[jnt]; 

    _Ccom.Drive[jnt].Status = 0x0006;   /* ブレーキOFF、サーボON、トルク制御 */
    _Ccom.Drive[jnt].Torq   = (short)dmytorq[jnt];
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }
  
  _Ccom.Do = 0x0000;

  return;
}

/*=================================================================================
  ブレーキONを指令する

            戻り値なし
==================================================================================*/
void
NoTorq(void)
{
  int  jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0007;   /* ブレーキON、サーボON、トルク制御 */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  指定した関節をブレーキ解除する

            戻り値なし
==================================================================================*/
void
BrakeOFF(int joint)
{
  int jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0007;   /* ブレーキON、サーボON、トルク制御 */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }
  
/*--- 指定された関節(joint)をブレーキ解除する ---*/

  _Ccom.Drive[joint].Status = 0x0006;   /* ブレーキOFF、サーボON、トルク制御 */
  _Ccom.Drive[joint].Torq   = 0x0000;
  _Ccom.Drive[joint].Speed  = 0x0000;

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  全軸ブレーキ解除を指令する

            戻り値なし
==================================================================================*/
void
AllBrakeOFF(void)
{
  int jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0006;   /* ブレーキOFF、サーボON、トルク制御 */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  指令した速度を送信用バッファにセットする

            戻り値なし
==================================================================================*/
void
SetSpeed(int joint, double speed)
{
  int jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0003;   /* ブレーキON、サーボON、速度制御 */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }
  
/*--- 指定された関節(joint)を速度(speed)で速度制御する ---*/

  _Ccom.Drive[joint].Status = 0x0002;   /* ブレーキOFF、サーボON、速度制御 */
  _Ccom.Drive[joint].Torq   = 0x0000;
  _Ccom.Drive[joint].Speed  = (short)speed;  /* digit */

  _Ccom.Do = 0x0000;
}

/*=================================================================================
  ブレーキONを指令する

            戻り値なし
==================================================================================*/
void
NoSpeed(void)
{
  int  jnt;
  
  for(jnt=0;jnt<7;jnt++){
    _Ccom.Drive[jnt].Status = 0x0003;   /* ブレーキON、サーボON、速度制御 */
    _Ccom.Drive[jnt].Torq   = 0x0000;
    _Ccom.Drive[jnt].Speed  = 0x0000;
  }

  _Ccom.Do = 0x0000;
}


/*=================================================================================
  アークネットのリセット

            戻り値なし
==================================================================================*/
void
Arc_Reset(void)
{
  _arc_reset(ARC_ADDR);    
}

/*=================================================================================
  送信データの表示

            戻り値なし
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
  送信データの表示

            戻り値なし
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
  受信データの表示

            戻り値なし
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
  受信データの表示

            戻り値なし
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
  マスターサーボステータスの表示

            戻り値なし
==================================================================================*/
void
MS(void)
{
  printf("Master Status = 0x%2x\n", _Axis.MStatus);
}

/*=================================================================================
  時間待ち1

            戻り値なし
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
  時間待ち2

            戻り値なし
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
  アークネットの初期化

            戻り値なし
==================================================================================*/
int
arcInit(void)
{
  int stat1;

  stat1 = Arc_Init();    /* アークネットの初期化 */
  if(stat1 == ERROR){
    printf("ERROR ARC-PCI/22 DRIVER: initialization faile\n");
    return ERROR;
  }
  printf("ARC-PCI/22   Initialized at base address 0x%x\n", ARC_ADDR);

  return OK;
}

/*=================================================================================
  アークネット・通信開始

            戻り値なし
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
  アークネットの終了

            戻り値なし
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
