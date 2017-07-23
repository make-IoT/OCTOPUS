
#include "EPD_drive.h"
#include "EPD_drive_gpio.h"
#include "Display_Lib.h"

/**************-------------------------------------------**************
              Register table
**************-------------------------------------------**************/
#define EPD1X54 1
#ifdef EPD2X9
  #define xDot 128
  #define yDot 296
  #define DELAYTIME 1500
#elif  EPD02X13
  #define xDot 128
  #define yDot 250
  #define DELAYTIME 4000
#elif  EPD1X54
  #define xDot 200
  #define yDot 200
  #define DELAYTIME 1500
#endif
  static unsigned char GDOControl[]={0x01,(yDot-1)%256,(yDot-1)/256,0x00}; //for 1.54inch
  static unsigned char softstart[]={0x0c,0xd7,0xd6,0x9d};
  static unsigned char VCOMVol[] = {0x2c,0xa8}; // VCOM 7c
  static unsigned char DummyLine[] = {0x3a,0x1a}; // 4 dummy line per gate
  static unsigned char Gatetime[] = {0x3b,0x08};  // 2us per line
  static unsigned char RamDataEntryMode[] = {0x11,0x01};  // Ram data entry mode

//Write LUT register  
#ifdef EPD02X13
  static unsigned char LUTDefault_full[] = {0x32,0x22,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x01,0x00,0x00,0x00,0x00};
  static unsigned char LUTDefault_part[] = {0x32,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#else
  static unsigned char LUTDefault_full[] = {0x32,0x02,0x02,0x01,0x11,0x12,0x12,0x22,0x22,0x66,0x69,0x69,0x59,0x58,0x99,0x99,0x88,0x00,0x00,0x00,0x00,0xF8,0xB4,0x13,0x51,0x35,0x51,0x51,0x19,0x01,0x00};
  static unsigned char LUTDefault_part[] = {0x32,0x10,0x18,0x18,0x08,0x18,0x18,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x14,0x44,0x12,0x00,0x00,0x00,0x00,0x00,0x00};
#endif


/***********************************************************************************************************************
			------------------------------------------------------------------------
			|\\\																///|
			|\\\						Drive layer					///|
			------------------------------------------------------------------------
***********************************************************************************************************************/
/*******************************************************************************
Constructor
*******************************************************************************/
WaveShare_EPD::WaveShare_EPD(){
	
}
/*******************************************************************************
function：
			read busy
*******************************************************************************/
unsigned char WaveShare_EPD::ReadBusy(void)
{
  unsigned long i=0;
  for(i=0;i<400;i++){
	//	println("isEPD_BUSY = %d\r\n",isEPD_CS);
      if(isEPD_BUSY==EPD_BUSY_LEVEL) {
				Serial.println("Busy is Low \r\n");
      	return 1;
      }
	  driver_delay_xms(10);
  }
  return 0;
}
/*******************************************************************************
function：
		write command
*******************************************************************************/
void WaveShare_EPD::EPD_WriteCMD(unsigned char command)
{        
  EPD_CS_0;	
	EPD_DC_0;		// command write
	SPI_Write(command);
	EPD_CS_1;
}
/*******************************************************************************
function：
		write command and data
*******************************************************************************/
void WaveShare_EPD::EPD_WriteCMD_p1(unsigned char command,unsigned char para)
{
	ReadBusy();	     
  EPD_CS_0;		
	EPD_DC_0;		// command write
	SPI_Write(command);
	EPD_DC_1;		// command write
	SPI_Write(para);
  EPD_CS_1;	
}
/*******************************************************************************
function：
		Configure the power supply
*******************************************************************************/
void WaveShare_EPD::EPD_POWERON(void)
{
	EPD_WriteCMD_p1(0x22,0xc0);
	EPD_WriteCMD(0x20);
	//EPD_WriteCMD(0xff);
}

/*******************************************************************************
function：
		The first byte is written with the command value
		Remove the command value, 
		the address after a shift, 
		the length of less than one byte	
*******************************************************************************/
void WaveShare_EPD::EPD_Write(unsigned char *value, unsigned char datalen)
{
	unsigned char i = 0;
	unsigned char *ptemp;
	ptemp = value;
	
	EPD_CS_0;
	EPD_DC_0;		// When DC is 0, write command 
	SPI_Write(*ptemp);	//The first byte is written with the command value
	ptemp++;
	EPD_DC_1;		// When DC is 1, write data
	Serial.println("send data  :"); 
	for(i= 0;i<datalen-1;i++){	// sub the data
		SPI_Write(*ptemp);
		ptemp++;
	}
	EPD_CS_1;
}
/*******************************************************************************
Function: Write the display buffer
Parameters: 
		XSize x the direction of the direction of 128 points, adjusted to an 
		integer multiple of 8 times YSize y direction quantity Dispbuff displays 
		the data storage location. The data must be arranged in a correct manner
********************************************************************************/
void WaveShare_EPD::EPD_WriteDispRam(unsigned char XSize,unsigned int YSize,
							unsigned char *Dispbuff)
{
	unsigned char i = 0,j = 0;
	if(XSize%8 != 0){
		XSize = XSize+(8-XSize%8);
	}
	XSize = XSize/8;

	ReadBusy();	
	EPD_CS_0;	
	EPD_DC_0;		//command write
	SPI_Write(0x24);
	EPD_DC_1;		//data write
	for(i=0;i<YSize;i++){
		for(j=0;j<XSize;j++){
			SPI_Write(*Dispbuff);
			Dispbuff++;
		}
	}
	EPD_CS_1;
}

/*******************************************************************************
Function: Write the display buffer to write a certain area to the same display.
Parameters: XSize x the direction of the direction of 128 points,adjusted to 
			an integer multiple of 8 times YSize y direction quantity  Dispdata 
			display data.
********************************************************************************/
void WaveShare_EPD::EPD_WriteDispRamMono(unsigned char XSize,unsigned int YSize,
							unsigned char dispdata)
{
  unsigned char i = 0,j = 0;
	if(XSize%8 != 0){
		XSize = XSize+(8-XSize%8);
	}
	XSize = XSize/8;
	
	ReadBusy();	    
	EPD_CS_0;
	EPD_DC_0;		// command write
	SPI_Write(0x24);
	EPD_DC_1;		// data write
	for(i=0;i<YSize;i++){
		for(j=0;j<XSize;j++){
		 SPI_Write(dispdata);
		}
	}
	EPD_CS_1;
}

/********************************************************************************
Set RAM X and Y -address Start / End position
********************************************************************************/
void WaveShare_EPD::EPD_SetRamArea(unsigned char Xstart,unsigned char Xend,
						unsigned char Ystart,unsigned char Ystart1,unsigned char Yend,unsigned char Yend1)
{
  unsigned char RamAreaX[3];	// X start and end
	unsigned char RamAreaY[5]; 	// Y start and end
	RamAreaX[0] = 0x44;	// command
	RamAreaX[1] = Xstart;
	RamAreaX[2] = Xend;
	RamAreaY[0] = 0x45;	// command
	RamAreaY[1] = Ystart;
	RamAreaY[2] = Ystart1;
	RamAreaY[3] = Yend;
  RamAreaY[4] = Yend1;
	EPD_Write(RamAreaX, sizeof(RamAreaX));
	EPD_Write(RamAreaY, sizeof(RamAreaY));
}

/********************************************************************************
Set RAM X and Y -address counter
********************************************************************************/
void WaveShare_EPD::EPD_SetRamPointer(unsigned char addrX,unsigned char addrY,unsigned char addrY1)
{
  unsigned char RamPointerX[2];	// default (0,0)
	unsigned char RamPointerY[3];
	//Set RAM X address counter
	RamPointerX[0] = 0x4e;
	RamPointerX[1] = addrX;
	//Set RAM Y address counter
	RamPointerY[0] = 0x4f;
	RamPointerY[1] = addrY;
	RamPointerY[2] = addrY1;
	
	EPD_Write(RamPointerX, sizeof(RamPointerX));
	EPD_Write(RamPointerY, sizeof(RamPointerY));
}

/********************************************************************************
1.Set RAM X and Y -address Start / End position
2.Set RAM X and Y -address counter
********************************************************************************/
void WaveShare_EPD::EPD_part_display(unsigned char RAM_XST,unsigned char RAM_XEND,unsigned char RAM_YST,unsigned char RAM_YST1,unsigned char RAM_YEND,unsigned char RAM_YEND1)
{    
	EPD_SetRamArea(RAM_XST,RAM_XEND,RAM_YST,RAM_YST1,RAM_YEND,RAM_YEND1);  	/*set w h*/
    EPD_SetRamPointer (RAM_XST,RAM_YST,RAM_YST1);		    /*set orginal*/
}

//=========================functions============================
/*******************************************************************************
Initialize the register
********************************************************************************/
void WaveShare_EPD::EPD_Init(void)
{
	//1.reset driver
	EPD_RST_0;		// Module reset
	driver_delay_xms(100);
	EPD_RST_1;
	driver_delay_xms(100);
	
	//2. set register
	Serial.println("***********set register Start**********");
	EPD_Write(GDOControl, sizeof(GDOControl));	// Pannel configuration, Gate selection
	EPD_Write(softstart, sizeof(softstart));	// X decrease, Y decrease
	EPD_Write(VCOMVol, sizeof(VCOMVol));		// VCOM setting
	EPD_Write(DummyLine, sizeof(DummyLine));	// dummy line per gate
	EPD_Write(Gatetime, sizeof(Gatetime));		// Gage time setting
	EPD_Write(RamDataEntryMode, sizeof(RamDataEntryMode));	// X increase, Y decrease
	EPD_SetRamArea(0x00,(xDot-1)/8,(yDot-1)%256,(yDot-1)/256,0x00,0x00);	// X-source area,Y-gage area
	EPD_SetRamPointer(0x00,(yDot-1)%256,(yDot-1)/256);	// set ram
	Serial.println("***********set register  end**********");
}

/********************************************************************************
Display data updates
********************************************************************************/
void WaveShare_EPD::EPD_Update(void)
{
	EPD_WriteCMD_p1(0x22,0xc7);
	EPD_WriteCMD(0x20);
	EPD_WriteCMD(0xff);
}
void WaveShare_EPD::EPD_Update_Part(void)
{
	EPD_WriteCMD_p1(0x22,0x04);
	EPD_WriteCMD(0x20);
	EPD_WriteCMD(0xff);
}
/*******************************************************************************
write the waveform to the dirver's ram
********************************************************************************/
void WaveShare_EPD::EPD_WirteLUT(unsigned char *LUTvalue,unsigned char Size)
{	
	EPD_Write(LUTvalue, Size);
}

/*******************************************************************************
Full screen initialization
********************************************************************************/
void WaveShare_EPD::EPD_init_Full(void)
{		
	EPD_Init();			// Reset and set register 
  EPD_WirteLUT((unsigned char *)LUTDefault_full,sizeof(LUTDefault_full));
		
	EPD_POWERON();
    //driver_delay_xms(100000); 		
}

/*******************************************************************************
Part screen initialization
********************************************************************************/
void WaveShare_EPD::EPD_init_Part(void)
{		
	EPD_Init();			// display
	EPD_WirteLUT((unsigned char *)LUTDefault_part,sizeof(LUTDefault_part));
	EPD_POWERON();        		
}
/********************************************************************************
parameter:
	Label  :
       		=1 Displays the contents of the DisBuffer
	   		=0 Displays the contents of the first byte in DisBuffer,
********************************************************************************/
void WaveShare_EPD::EPD_Dis_Full(unsigned char *DisBuffer,unsigned char Label)
{
    EPD_SetRamPointer(0x00,(yDot-1)%256,(yDot-1)/256);	// set ram
	Serial.println(">>>>>>------start send display data!!---------<<<<<<<");
	if(Label == 0){
		EPD_WriteDispRamMono(xDot, yDot, 0xff);	// white	
	}else{
		EPD_WriteDispRam(xDot, yDot, (unsigned char *)DisBuffer);	// white
	}	
	EPD_Update();	
	
}

/********************************************************************************
parameter: 
		xStart :   X direction Start coordinates
		xEnd   :   X direction end coordinates
		yStart :   Y direction Start coordinates
		yEnd   :   Y direction end coordinates
		DisBuffer : Display content
		Label  :
       		=1 Displays the contents of the DisBuffer
	   		=0 Displays the contents of the first byte in DisBuffer,
********************************************************************************/
void WaveShare_EPD::EPD_Dis_Part(unsigned char xStart,unsigned char xEnd,unsigned long yStart,unsigned long yEnd,unsigned char *DisBuffer,unsigned char Label)
{
	Serial.println(">>>>>>------start send display data!!---------<<<<<<<");
	if(Label==0){// black
		EPD_part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);
		EPD_WriteDispRamMono(xEnd-xStart, yEnd-yStart+1, DisBuffer[0]);
 		EPD_Update_Part();
		driver_delay_xms(500);
		EPD_part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);	
		EPD_WriteDispRamMono(xEnd-xStart, yEnd-yStart+1, DisBuffer[0]);	
	}else{// show 
		EPD_part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);		
		EPD_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
		EPD_Update_Part();
		driver_delay_xms(500);
		EPD_part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);
		EPD_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
	}
}

/***********************************************************************************************************************
			------------------------------------------------------------------------
			|\\\																///|
			|\\\						App layer								///|
			------------------------------------------------------------------------
***********************************************************************************************************************/
/********************************************************************************
		clear full screen
********************************************************************************/
void WaveShare_EPD::Dis_Clear_full(void)
{
	unsigned char m;
	//init
	Serial.println("full init");
	EPD_init_Full();
	driver_delay_xms(DELAYTIME);

	//Clear screen
	Serial.println("full clear\t\n");
	m=0xff;
	EPD_Dis_Full((unsigned char *)&m,0);  //all white
	driver_delay_xms(DELAYTIME);
}
/********************************************************************************
		clear part screen
********************************************************************************/
void WaveShare_EPD::Dis_Clear_part(void)
{
	unsigned char m;
	//init
	EPD_init_Part();
	driver_delay_xms(DELAYTIME);

	//Clear screen
	m=0xff;
	EPD_Dis_Part(0,xDot-1,0,yDot-1,(unsigned char *)&m,0);	 //all white
	driver_delay_xms(DELAYTIME);
}

/********************************************************************************
parameter: 
		xStart :   X direction Start coordinates
		xEnd   :   X direction end coordinates
		yStart :   Y direction Start coordinates
		yEnd   :   Y direction end coordinates
		DisBuffer : Display content
********************************************************************************/
void WaveShare_EPD::Dis_pic(unsigned char xStart,unsigned char xEnd,unsigned long yStart,unsigned long yEnd,unsigned char *DisBuffer)
{
	EPD_Dis_Part(xStart,xEnd,yStart,yEnd,(unsigned char *)DisBuffer,1);
}

/********************************************************************************
funtion : Select the character size
parameter :
	acsii : char data 
	size : char len
	mode : char mode
	next : char len
Remarks:
********************************************************************************/
 void WaveShare_EPD::Dis_Char(char acsii,char size,char mode,char next,unsigned char *buffer)
 {
	unsigned char i ;
	char temp;
	unsigned char ch = acsii - ' ';
	for(i = 0;i<size;i++){
		switch(size){
			case 12:
				if(mode)temp=Font1206[ch][i];
				else temp = ~Font1206[ch][i];
				buffer[size*next+i] = temp; 
				break;
			case 16:		 
				if(mode)temp=Font1608[ch][i];
				else temp = ~Font1608[ch][i];
				buffer[size*next+i] = temp; 
				break;
		}	
	}
// return buffer;
 }	 
/********************************************************************************
funtion : write string
parameter :
	x : x start address
	y : y start address
	pString : Display data
	Size : char len
Remarks:
********************************************************************************/
 void WaveShare_EPD::Dis_String(unsigned char x,unsigned char y, const char *pString,unsigned int  Size)
{
	unsigned char len = 0;
	unsigned char x_addr = x;
	unsigned char y_addr = y;
  unsigned char buffer[1500];
  int mode = 0;
  unsigned char i = 0;
  char temp;
	//1.Remove the character and determine the character size
    while (*pString != '\0') {  
        if (x > (xDot- Size / 2)) {
			x = 0;
			y += Size;
			if (y > (yDot- Size)) {
				y = x = 0;
			}
		}        
     //   Dis_Char(*pString, Size, 0,len,buffer);
        unsigned char ch = *pString - ' ';
         Serial.println(*pString);
      for(i = 0;i<Size;i++){
        switch(Size){
          case 12:
            if(mode)temp=Font1206[ch][i];
            else temp = ~Font1206[ch][i];
            Serial.print(temp,HEX);
            Serial.print( " ");
            buffer[Size*len+i] = temp; 
            break;
          case 16:     
            if(mode)temp=Font1608[ch][i];
            else temp = ~Font1608[ch][i];
            buffer[Size*len+i] = temp; 
            break;
        } 
      }
       Serial.println( " ");
       
        x += Size / 2;
        pString ++;	//The next character of the address
		    len++;		//Calculate the current number for the first few characters
       
    }
	//2.show
	switch(Size)
	{
		case 12:
			EPD_Dis_Part(y_addr+1,y_addr+Size/2,yDot-(Size*len)-x_addr+1,yDot-x_addr,(unsigned char *)buffer,1);
			break;
		case 16:
			EPD_Dis_Part(y_addr+1,y_addr+Size/2,yDot-(Size*len)-x_addr+1,yDot-x_addr,(unsigned char *)buffer,1);
			break;
	}
}  

/********************************************************************************
funtion : Drawing pic
parameter :
	xStart : x start address
	yStart : y start address
	DisBuffer : Display data
	xSize : Displays the x length of the image
	ySize : Displays the y length of the image
Remarks:
	The sample image is 32 * 32
********************************************************************************/
void WaveShare_EPD::Dis_Drawing(unsigned char xStart,unsigned long yStart,unsigned char *DisBuffer,unsigned char xSize,unsigned char ySize)
{
	unsigned char x_addr = xStart*8;
	unsigned char y_addr = yStart*8;
	EPD_Dis_Part(y_addr,y_addr+xSize-1,yDot-ySize-x_addr,yDot-x_addr-1,(unsigned char *)DisBuffer,1);
}

/********************************************************************************
funtion : show Progress
parameter :
	progress_len : Progress bar length	
********************************************************************************/
void WaveShare_EPD::Dis_Progress(unsigned char progress_len)
{
	int x,y,z;
	int pheight_pix = 2;
	int pWidth_pix = 16;
  unsigned char buffer[5000];
	const unsigned char *phead ,*pzero,*pstart,*pspare,*pfull,*pend;
	//1.Initialize the progress bar length and place it in the center of the lower end of the display
	y = 0;
	for(z= 0;z < progress_len;z++){
		phead = progress_head;
		pspare = progress_Spare;
		pend = progress_end;
		for(x = 0;x <pWidth_pix*pheight_pix;x++){
			if(z == 0){
				buffer[y] = *phead;
				phead++;
				y++;
			}else if(z == progress_len -1){
				buffer[y] = *pend;
				pend++;
				y++;
			}else{
				buffer[y] = *pspare;
				pspare++;
				y++;
			}
		}
	}
	EPD_Dis_Part(xDot-xDot/10-1,xDot-xDot/10+8,(yDot-16*progress_len)/2-1,(yDot-16*progress_len)/2-1+16*progress_len,(unsigned char *)buffer,1);
	//2.Load progress bar
	y =0;
	for(z= 0;z < progress_len;z++){
		pstart = progress_start;
		pzero = progress_zero;
		pfull = progress_full;
		for(x = 0;x <pWidth_pix*pheight_pix;x++){
			if(z == 0){
				buffer[y] = *pzero;
				pzero++;
				y++;
			}else if(z == progress_len-1){
				buffer[y] = *pfull;
				pfull++;
				y++;
			}else{
				buffer[y] = *pstart;
				pstart++;
				y++;
			}
		}
		EPD_Dis_Part(xDot-xDot/10-1,xDot-xDot/10+8,(yDot-16*progress_len)/2-1,(yDot-16*progress_len)/2-1+16*progress_len,(unsigned char *)buffer,1);
	}
}
/********************************************************************************
funtion : show time 
parameter :
	hour : The number of hours
	min : The number of minute
	sec : The number of sec
Remarks:
********************************************************************************/
void WaveShare_EPD::Dis_showtime(unsigned int hour,unsigned int minute,unsigned int sec)
{
	int x,z;
	char temp;
	char len = 16;
  unsigned char buffer[5000];
	//1.Display format01:34:67      
	for(z= 0;z < 8;z++){
		//2.Load every bit
		for(x = 0;x <len;x++){
			switch(z){
				case 0 :
					temp = ~Font1608[16+hour/10][x];
					buffer[z*len+x] = temp;
					break;
				case 1 :
					temp = ~Font1608[16+hour%10][x];
					buffer[z*len+x] = temp;
					break;
				case 2 ://:
					temp = ~Font1608[26][x];
					buffer[z*len+x] = temp;
					break;
				case 3 :
					temp = ~Font1608[16+minute/10][x];
					buffer[z*len+x] = temp;
					break;
				case 4 :
					temp = ~Font1608[16+minute%10][x];
					buffer[z*len+x] = temp;
					break;
				case 5 ://:
					temp = ~Font1608[26][x];
					buffer[z*len+x] = temp;
					break;
				case 6 :
					temp = ~Font1608[16+sec/10][x];
					buffer[z*len+x] = temp;
					break;
				case 7 :
					temp = ~Font1608[16+sec%10][x];
					buffer[z*len+x] = temp;
					break;
				default: 
					break;
			}
		}    
	}
	EPD_Dis_Part(xDot/10-1,xDot/10+8,(yDot-16*4)/2,(yDot-16*4)/2-1+16*4,(unsigned char *)buffer,1);
}
/***********************************************************
						end file
***********************************************************/

