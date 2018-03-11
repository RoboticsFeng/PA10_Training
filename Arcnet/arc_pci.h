#ifndef _ARC_PCI_H
#define _ARC_PCI_H

#include "arc_pci_cfg.h"

extern int _arc_init(int BaseAddress);
extern int _arc_send(int BaseAddress, unsigned char *_SendPacket);
extern int _arc_read(int BaseAddress);
extern void _arc_reset(int BaseAddress);

extern int iSend_S(void);
extern int iRecv_S(void);
extern int iSend_C(void);
extern int iRecv_C(void);
extern int RecData(void);
extern int iSend_T(void);
extern int iRecv_T(void);
extern int iSend_E(void);
extern int iRecv_E(void);

extern int  Arc_Init(void);
extern void SetData(char);
extern void GetPosition(double *cur_pos);
extern void SetTorq(double *torq);
extern void NoTorq(void);
extern void BrakeOFF(int joint);
extern void AllBrakeOFF(void);
extern void SetSpeed(int joint, double speed);
extern void NoSpeed(void);
extern void Arc_Reset(void);

extern void PrintSampleData(void);
extern void PrintCcomData(void);
extern void PrintRecvData(void);
extern void PrintAxisData(void);

extern void MS(void);
extern void delay1(int);
extern void iodelay(void);

extern int arcInit(void);
extern void arcnet_start(void);
extern void arcFin(void);

#endif /* _ARCPCI_H */




