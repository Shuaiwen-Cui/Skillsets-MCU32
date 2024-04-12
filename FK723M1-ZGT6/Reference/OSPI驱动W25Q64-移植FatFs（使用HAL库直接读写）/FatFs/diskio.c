/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "ospi_w25q64.h"

#define SD_Card		0	//SD卡 
#define SPI_Flash		1	//SPI Flash


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv) {
		
		case SD_Card :
		return stat;

		case SPI_Flash :
			if(W25Qxx_FLASH_ID == OSPI_W25Qxx_ReadID())
			{
			  stat &= ~STA_NOINIT;
			}	
			else
			{
			  stat = STA_NOINIT;;
			}			
		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv) {
		
		case SD_Card :
		return stat;

		case SPI_Flash :
			OSPI_W25Qxx_Init();
			stat &= ~STA_NOINIT;			
		return stat;

	}
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	switch (pdrv) {
		
		case SD_Card:
		return RES_PARERR;			

		case SPI_Flash :
			OSPI_W25Qxx_ReadBuffer(buff, sector <<12, count<<12);	// 以4K字节为单位		
		return RES_OK;
	}
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	uint32_t write_addr; 
	switch (pdrv) {
		
		case SD_Card:						
		return RES_PARERR;
					
		case SPI_Flash :
			write_addr = sector<<12; 			// 以4K字节为单位		
			OSPI_W25Qxx_SectorErase(write_addr);	
			OSPI_W25Qxx_WriteBuffer((uint8_t *)buff,write_addr,count<<12);
		return RES_OK;
	}
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	switch (pdrv) {
		case SD_Card :
		return  RES_PARERR;

		case SPI_Flash :
			switch (cmd) {
				case GET_SECTOR_COUNT:
					*(DWORD * )buff = 2048;		// 总的扇区数
				break;
				
				case GET_SECTOR_SIZE :
					*(WORD * )buff = 4096;		// 定义一个扇区大小为4K
				break;
				
				case GET_BLOCK_SIZE :
					*(DWORD * )buff = 65536;	// 定义一个块大小为64K
				break;        
			}	
		return RES_OK;
	}
	return RES_PARERR;
}
/*-----------------------------------------------------------------------*/
/* 时间获取函数，在这里不使用，返回0                                      */
/*-----------------------------------------------------------------------*/
DWORD get_fattime(void) {

	return 0;
}
