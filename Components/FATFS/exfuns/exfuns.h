#ifndef __EXFUNS_H
#define __EXFUNS_H 		   
#include "ff.h"
#include "hal_board.h"
//////////////////////////////////////////////////////////////////////////////////

//V1.0 ��Ҫ����һЩȫ�ֱ���������FATFS��ʹ�ã�ͬʱʵ��һЩ�����������ȡ�Ĺ��ܺ���

//////////////////////////////////////////////////////////////////////////////////	
extern FATFS *fs;  
extern FIL *file;	  
extern UINT br,bw;
extern FILINFO fileinfo;
extern DIR dir;
//extern uint8 *fatbuf;//SD�����ݻ�����


//fypetell���ص����Ͷ���
//���ݱ�FILEYPEBL���.��exfuns.c���涨��
#define T_BIN		0X00	//bin�ļ�
#define T_LRC		0X10	//lrc�ļ�
#define T_NES		0X20	//nes�ļ�
#define TEXT		0X30	//.txt�ļ�
#define T_C			0X31	//.c�ļ�
#define T_H			0X32    //.h�ļ�
#define T_FLAC		0X4C	//flac�ļ�
#define T_BMP		0X50	//bmp�ļ�
#define T_JPG		0X51	//jpg�ļ�
#define T_JPEG		0X52	//jpeg�ļ�		 
#define T_GIF		0X53	//gif�ļ�  

 
uint8 exfuns_init(void);		//�����ڴ�
uint8 f_typetell(uint8 *fname);	//ʶ���ļ�����
uint8 exf_getfree(uint8 *drv,uint32 *total,uint32 *free);//�õ�������������ʣ������
uint8 char_upper(uint8 c);      //ת����Сд
//uint32 exf_fdsize(uint8 *fdname);																				   //�õ��ļ��д�С
//uint8* exf_get_src_dname(uint8* dpfn);																		   
//uint8 exf_copy(uint8(*fcpymsg)(uint8*pname,uint8 pct,uint8 mode),uint8 *psrc,uint8 *pdst,uint32 totsize,uint32 cpdsize,uint8 fwmode);	   //�ļ�����
//uint8 exf_fdcopy(uint8(*fcpymsg)(uint8*pname,uint8 pct,uint8 mode),uint8 *psrc,uint8 *pdst,uint32 *totsize,uint32 *cpdsize,uint8 fwmode);//�ļ��и���

#endif


