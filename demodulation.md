`#include <stdlib.h>
#include "DSP28_Device.h"
#include <string.h>

static int32 Threshold_Value = 0;
static int32 Threshold_Value_Decision = 0;
#define synCodeNum  7
int32 tempConvX[ DEMODU_BUFFER_NUM ];
int32 tempConvY[ DEMODU_BUFFER_NUM + 10 ];  //最大10阶，记得修改开机清除buf大小
int32 filterResult[DEMODU_PRO_NUM + Interval+1];
#define MAX_DECISION_SIZE ( (1 + 2 + RS_ENCODE_COUNT*nn + 2 + 2) * 8 ) //1个同部位 2个帧头 2*nn个编码 2个帧尾  2个保留
Uint8 tempCode[   DEMODU_BUFFER_NUM/Interval  + synCodeNum]; //DEMODU_BUFFER_NUM/( (MPPSK_N * ADC_RATE)/PWM_RATE )

/*
*@function		DemodulateSlove
*@brief				demodulate data
*@parameter		1.memoryFlag	a flag determines which buffer can be processd
*@retval			none
*/

``#if IS_FLASH
#pragma CODE_SECTION( DemodulateSlove, "ramfuncs" )
#endif``
void DemodulateSlove()
{
    int32 *filterResultPtr= &filterResult[Interval] ; //An array to hold the responses of filters
    Uint8 count = 0;

    if(!memoryFlag)
    {
        memoryFlag = ~memoryFlag;
        adcshift = 0;
    }
    else
    {
        memoryFlag = ~memoryFlag;
        adcshift = 8;
    }
    //BandPass
    BandPassFilter(adcresult,filterResultPtr,DEMODU_BUFFER_NUM);

    for(count = 0;count<DEMODU_BUFFER_NUM;count++)
    {
    		filterResultPtr[count] = filterResultPtr[count]*filterResultPtr[count];
    }

    //lowPass
    LowPass(filterResultPtr,DEMODU_BUFFER_NUM);
    //decision
    Decision(filterResultPtr,DEMODU_BUFFER_NUM,tempCode);

    return;
}



/**@file:     BandPass250k
*@brief:		This funciton implement ImpulseFilter to filter dataVector and every 4 data will be merged
*@paramter: 1.srcVector		an address of an Array which stores the original data which need to be process
*						2.destVector	an address of an Array which stores the data which has been processed
*						3.num 				the number of the original data
*@retval    none*/

//#if IS_FLASH
//#pragma CODE_SECTION( BandPassFilter, "ramfuncs" )
//#endif
//static void BandPassFilter(Uint8* srcVector,int32* destVector,Uint8 num)
//{
//    Uint8 shiftNum = 15;
//    static int32 B[5] = { 4108, 0, -8215, 0, 4108};
//    static int32 A[4] = { 0, 27356, 0, 15273};
//    static int32 xPre[4] = {0,0,0,0};//int32
//    static int32 yPre[4] = {0,0,0,0,};//int32
//
//    Uint32 count = 0,tempCount = 0;
//    //convolution input signal
//    tempConvX[0] = ( (srcVector[0]>>adcshift )& 0x00FF) * B[0]  + xPre[2] * B[2]+xPre[0] * B[4];
//    tempConvX[1] = ( (srcVector[1]>>adcshift )& 0x00FF) * B[0]  + xPre[3] * B[2]+xPre[1] * B[4];
//    tempConvX[2] = ( (srcVector[2]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[0]>>adcshift )& 0x00FF) * B[2] + xPre[2] * B[4];
//    tempConvX[3] = ( (srcVector[3]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[1]>>adcshift )& 0x00FF) * B[2] + xPre[3] * B[4];
//
//    for(count = 4 ; count < num ; count++)
//        tempConvX[count] = ( (srcVector[count]>>adcshift )& 0x00FF) * B[0]
//                           + ( (srcVector[count - 2]>>adcshift )& 0x00FF)  * B[2]+( (srcVector[count - 4]>>adcshift )& 0x00FF)  * B[4];
//    //save  xInfo
//    xPre[0] = ( (srcVector[num -4]>>adcshift )& 0x00FF);
//    xPre[1] = ( (srcVector[num -3]>>adcshift )& 0x00FF);
//    xPre[2] = ( (srcVector[num -2]>>adcshift )& 0x00FF);
//    xPre[3] = ( (srcVector[num -1]>>adcshift )& 0x00FF);
//
//    //sub feedback
//
//    tempConvY[0] = yPre[0];
//    tempConvY[1] = yPre[1];
//    tempConvY[2] = yPre[2];
//    tempConvY[3] = yPre[3];
//
//    for(count = 0 ; count < num ; count++)
//    {
//        tempCount = count+4;
//        tempConvY[tempCount] = tempConvY[count]*A[3]+tempConvY[count+2]*A[1];
//        tempConvY[tempCount] = (tempConvX[count] - tempConvY[tempCount]) >> (shiftNum);
//        destVector[count] = tempConvY[tempCount] ;
//    }
//    //save  yInfo
//    yPre[0] = tempConvY[num];
//    yPre[1] = tempConvY[num+1];
//    yPre[2] = tempConvY[num+2];
//    yPre[3] = tempConvY[num+3];
//
//}

#if IS_FLASH
#pragma CODE_SECTION( BandPassFilter, "ramfuncs" )
#endif
static void BandPassFilter(Uint8* srcVector,int32* destVector,Uint8 num)
{
    Uint8 shiftNum = 20;
    static int32 B[7]={44431, 0, -38473, 0, 38473, 0, -44431};
    static int32 A[6] = { 0, 1826462, 0, 1228869, 0, 285174 };

//    Uint8 shiftNum = 15;
//    static int32 B[7]={1388, 0, -1202,  0, 1202, 0, -1388};
//    static int32 A[6] = {0, 57077, 0, 38402,  0, 8912};

    static int32 xPre[6] = {0,0,0,0,0,0};//int32
    static int32 yPre[6] = {0,0,0,0,0,0};//int32

    Uint32 count = 0,tempCount = 0;
    //convolution input signal
    tempConvX[0] = ( (srcVector[0]>>adcshift )& 0x00FF) * B[0]  + xPre[4] * B[2]+xPre[2] * B[4]+xPre[0] * B[6];
    tempConvX[1] = ( (srcVector[1]>>adcshift )& 0x00FF) * B[0]  + xPre[5] * B[2]+xPre[3] * B[4]+xPre[1] * B[6];
    tempConvX[2] = ( (srcVector[2]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[0]>>adcshift )& 0x00FF)* B[2]   + xPre[4] * B[4]  +xPre[2] * B[6];
    tempConvX[3] = ( (srcVector[3]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[1]>>adcshift )& 0x00FF)* B[2]   + xPre[5] * B[4]  +xPre[3] * B[6];
    tempConvX[4] = ( (srcVector[4]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[2]>>adcshift )& 0x00FF)* B[2] +( (srcVector[0]>>adcshift )& 0x00FF) * B[4]+ xPre[4] * B[6];
    tempConvX[5] = ( (srcVector[5]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[3]>>adcshift )& 0x00FF)* B[2] +( (srcVector[1]>>adcshift )& 0x00FF) * B[4]+ xPre[5] * B[6];

    for(count = 6 ; count < num ; count++)
        tempConvX[count] = ( (srcVector[count]>>adcshift )& 0x00FF) * B[0]  + ( (srcVector[count - 2 ]>>adcshift )& 0x00FF)* B[2]
                           + ( (srcVector[count - 4]>>adcshift )& 0x00FF)  * B[4]+( (srcVector[count - 6]>>adcshift )& 0x00FF)  * B[6];
    //save  xInfo
    xPre[0] = ( (srcVector[num -6]>>adcshift )& 0x00FF);
    xPre[1] = ( (srcVector[num -5]>>adcshift )& 0x00FF);
    xPre[2] = ( (srcVector[num -4]>>adcshift )& 0x00FF);
    xPre[3] = ( (srcVector[num -3]>>adcshift )& 0x00FF);
    xPre[4] = ( (srcVector[num -2]>>adcshift )& 0x00FF);
    xPre[5] = ( (srcVector[num -1]>>adcshift )& 0x00FF);


    //sub feedback

    tempConvY[0] = yPre[0];
    tempConvY[1] = yPre[1];
    tempConvY[2] = yPre[2];
    tempConvY[3] = yPre[3];
    tempConvY[4] = yPre[4];
    tempConvY[5] = yPre[5];

    for(count = 0 ; count < num ; count++)
    {
        tempCount = count+6;
        tempConvY[tempCount] = tempConvY[count]*A[5]+tempConvY[count+2]*A[3]+tempConvY[count+4]*A[1];
        tempConvY[tempCount] = (tempConvX[count] - tempConvY[tempCount]) >> (shiftNum);
        destVector[count] = tempConvY[tempCount] ;
    }
    //save  yInfo
    yPre[0] = tempConvY[num];
    yPre[1] = tempConvY[num+1];
    yPre[2] = tempConvY[num+2];
    yPre[3] = tempConvY[num+3];
    yPre[4] = tempConvY[num+4];
    yPre[5] = tempConvY[num+5];
}


#if IS_FLASH
#pragma CODE_SECTION( LowPass, "ramfuncs" )
#endif
static void LowPass(int32* srcVector,Uint8 num)
{
    Uint8 shiftNum = 15;
   // static int32 B[6]= {42300,-61200,40800,40800,-61200,42300};
//    static int32 B[6]= {0, 2, 5, 5, 2, 0};
//    static int32 A[5] = {-150072, 280134, -266159, 128641, -25297};
    static int32 B[6]= {-4230,6120,-4080,-4080,6120,-4230};
    static int32 A[5] = {-114862,166642,-123782,46886,-7213};

    static int32 x2Pre[5] = {0,0,0,0,0};
    static int32 y2Pre[5] = {0,0,0,0,0};
    Uint32 count = 0,tempCount = 0;
    //convolution input signal
    tempConvX[0] = x2Pre[4]*B[1]+ x2Pre[3] * B[2]+x2Pre[2] * B[3]+ x2Pre[1] * B[4];
    tempConvX[1] = srcVector[0] *B[1] + x2Pre[4] * B[2] + x2Pre[3] * B[3]+ x2Pre[2] * B[4];
    tempConvX[2] = srcVector[1] *B[1] + srcVector[0] * B[2] + x2Pre[4] * B[3]+ x2Pre[3] * B[4];
    tempConvX[3] = srcVector[2] *B[1] + srcVector[1] * B[2] + srcVector[0] * B[3]+ x2Pre[4] * B[4];
    tempConvX[4] =  srcVector[3] *B[1] + srcVector[2] * B[2] + srcVector[1] * B[3]+ srcVector[0] * B[4];
    for(count = 5 ; count < num ; count++)
        tempConvX[count] =  srcVector[count - 1]*B[1]
                           + srcVector[count - 2] * B[2] + srcVector[count - 3] * B[3]+ srcVector[count - 4] * B[4];
    //save  xInfo
    x2Pre[0] = srcVector[num - 5];
    x2Pre[1] = srcVector[num - 4];
    x2Pre[2] = srcVector[num - 3];
    x2Pre[3] = srcVector[num - 2];
    x2Pre[4] = srcVector[num - 1];
    //sub feedback
    tempConvY[0] = y2Pre[0];
    tempConvY[1] = y2Pre[1];
    tempConvY[2] = y2Pre[2];
    tempConvY[3] = y2Pre[3];
    tempConvY[4] = y2Pre[4];
    for(count = 0 ; count < num ; count++)
    {
        tempCount = count+5;
        tempConvY[tempCount] = tempConvY[count]*A[4]+tempConvY[count+1]*A[3]+tempConvY[count+2]*A[2]+tempConvY[count+3]*A[1]+tempConvY[count+4]*A[0];
        srcVector[count] = tempConvY[tempCount] = (tempConvX[count] - tempConvY[tempCount]) >> (shiftNum);
    }
    //save yInfo
    y2Pre[0] = tempConvY[num];
    y2Pre[1] = tempConvY[num+1];
    y2Pre[2] = tempConvY[num+2];
    y2Pre[3] = tempConvY[num+3];
    y2Pre[4] = tempConvY[num+4];
}



/*
*@brief: An function is to search the maximum value in srcVector. This function will be called by the function Decision;
*/
#if IS_FLASH
#pragma CODE_SECTION( searchMaxIndex, "ramfuncs" )
#endif
static Uint8 searchMaxIndex(int32* srcVector,Uint8 num)
{
    Uint8 maxIndex = 0,count=0;
    int32 max = srcVector[0];
    for(count = 1; count<num; count++)
    {
        if(srcVector[count] > max)
        {
            max = srcVector[count];
            maxIndex = count;
        }
    }
    return maxIndex+1;   //
}

#if IS_FLASH
#pragma CODE_SECTION( Find_SynIndex, "ramfuncs" )
#endif
static int8 Find_SynIndex(int32* srcVector,Uint8 num)
{
    Uint8 synindex = 0,count=0,i=0;
    int32 temp_maxvalue=0,maxvalue=0;

    for(count = 0; count< num- ( MPPSK_K )*ADC_RATE/PWM_RATE + 1 -1; count++)  //56
    {
        for( i=0; i < ( MPPSK_K )*ADC_RATE/PWM_RATE + 1; i++ )   //23
        {
        	temp_maxvalue += srcVector[count + i];
        }
       if(temp_maxvalue > maxvalue)
    	 {
    	   synindex = count;
    	   maxvalue = temp_maxvalue;
    	 }
           temp_maxvalue = 0;
    }

    //	temp_maxvalue = srcVector[synindex + ( MPPSK_K )*ADC_RATE/PWM_RATE/2 + 2];


    if( maxvalue <= Threshold_Value )
    {
    	//Threshold_Value = maxvalue;//无调制
    	return -1 * 2* Interval  ;
    }
    else
    {
    	Threshold_Value_Decision = maxvalue;//有调制
        return searchMaxIndex( &srcVector[synindex], ( MPPSK_K )*ADC_RATE/PWM_RATE + 1 ) + synindex;
    }

}

//#if IS_FLASH
//#pragma CODE_SECTION( Decision_Syn, "ramfuncs" )
//#endif
//static Uint8 Decision_Syn(int32* srcVector,int8 max )
//{
//    int8 count=0,count1=0;
//    int32 max_0 = 0;
//    int32 max_1 = 0;
//
//
//    for(count = max - MPPSK_K*ADC_RATE/PWM_RATE/2; count<=max + MPPSK_K*ADC_RATE/PWM_RATE/2; count++)
//    {
//        max_0 += srcVector[count] ;  //
//    }
////    for(count1 = max + (MPPSK_K+MPPSK_RG)*ADC_RATE/PWM_RATE - MPPSK_K*ADC_RATE/PWM_RATE/2; count1<=max  + (MPPSK_K+MPPSK_RG)*ADC_RATE/PWM_RATE + MPPSK_K*ADC_RATE/PWM_RATE/2; count1++)
////    {
////        max_1 += srcVector[count1] ;
////    }
//   // if(  max_0 - max_1> 50*(MPPSK_K*ADC_RATE/PWM_RATE + 1) )
//    if(  max_0 > Threshold_Value)
//    {
//    	Threshold_Value_Decision = max_0;
//    	 return 0;//zhen
//
//    }
//    else
//    {
//
//    	 return 1;
//    }
//}

#if IS_FLASH
#pragma CODE_SECTION( DsearchMaxIndex, "ramfuncs" )
#endif
static Uint8 DsearchMaxIndex(int32* srcVector,int8 *max )
{
    int8 maxIndex = 0,count=0;
    int32 max_0 = 0;


    for(count = *max + Interval-DECISION_SIZE; count<=*max + Interval+DECISION_SIZE; count++) //76-84     72-88
    {
        max_0 += srcVector[count] ;
    }

    if( max_0 >  (Threshold_Value_Decision + Threshold_Value )/6)//100*(2*DECISION_SIZE+1) )//Threshold_Value_Decision )
    {
    	Threshold_Value_Decision = max_0;
        maxIndex =1;
       // Threshold_Value_Decision = (max_0 + Threshold_Value)/2;
        //*max=*max + Interval;
       *max=*max + Interval - DECISION_SIZE - 1 + ( searchMaxIndex(&srcVector[*max + Interval-DECISION_SIZE -1],2*DECISION_SIZE+1) );
    }
    else
    {
    	//Threshold_Value = max_0;
        maxIndex =0;
       // Threshold_Value = max_0;
        *max=*max + Interval;
        //*max=*max + Interval - DECISION_SIZE - 1 + ( searchMaxIndex(&srcVector[*max + Interval + (MPPSK_K+MPPSK_RG)*ADC_RATE/PWM_RATE - DECISION_SIZE -1],2*DECISION_SIZE+1) );
    }

    return maxIndex;   //
}

Uint8 HEAD[13] = {0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x01,0x01,0x00,0x01,0x00,0x01};
#if IS_FLASH
#pragma CODE_SECTION( Find_Frame, "ramfuncs" )
#endif
 Uint8* Find_Frame(Uint8* srcVector)
{
    Uint8 synindex = 0,count=0,i=0;
    Uint8 temp_maxvalue=0,maxvalue=0;

    for(count = 0; count< 11 - synCodeNum + 1 ; count++)  //56
    {
        for( i=0; i < 13; i++ )   //23
        {
        	temp_maxvalue += HEAD[i]*srcVector[count + i];
        }
        if(temp_maxvalue==9)
        {
        	return &srcVector[count];
        }
       if(temp_maxvalue > maxvalue)
    	 {
    	   synindex = count;
    	   maxvalue = temp_maxvalue;
    	 }
           temp_maxvalue = 0;
    }
    if(maxvalue>=7)
    {
    	  return &srcVector[synindex];
    }
    else
    {
    	return NULL;
    }
}



Uint8 VALUE_BUF[RS_ENCODE_COUNT*nn] = {0};
Uint8 VALUE_COUNT = 0;
#if IS_FLASH
#pragma CODE_SECTION( Conversion_To_Recvbuf, "ramfuncs" )
#endif
void Conversion_To_Recvbuf( Uint8 *buf, Uint8 count )
{
    Uint8 i = 0;
    Uint8 VALUE_BUF_pre_decode[RS_ENCODE_COUNT*kk]= {0};
    memset( VALUE_BUF, 0, RS_ENCODE_COUNT*nn*sizeof(Uint8));
    memset( VALUE_BUF_pre_decode, 0, RS_ENCODE_COUNT*kk*sizeof(Uint8));
    VALUE_COUNT = count / 8;
    for( ; i < VALUE_COUNT;  i++ )
    {
        VALUE_BUF[ i ] = ( buf[ i*8 ] << 7 ) & 0x80;
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (  ( buf[ i*8 + 1 ] << 6 ) & 0x40 );
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (  ( buf[ i*8 + 2 ] << 5 ) & 0x20 );
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (  ( buf[ i*8 + 3 ] << 4 ) & 0x10 );
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (  ( buf[ i*8 + 4 ] << 3 ) & 0x08 );
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (  ( buf[ i*8 + 5 ] << 2 ) & 0x04 );
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (  ( buf[ i*8 + 6 ] << 1 ) & 0x02 );
        VALUE_BUF[ i ] = VALUE_BUF[ i ] | (    buf[ i*8 + 7 ]  & 0x01 );
    }

    for( i = 0; i< RS_ENCODE_COUNT; i++ )
    {
        decode_rs_8( (unsigned char *)VALUE_BUF + nn*i, (unsigned char *)VALUE_BUF_pre_decode + kk*i );
    }


    for( i=0; i < RS_ENCODE_COUNT*kk; i++ )   //buf160-176
    {
        VALUE_BUF[ i ] = VALUE_BUF_pre_decode[i];
    }

}

Uint8 ACK_BUF[RS_ENCODE_COUNT*kk + 2] = {0};
#if IS_FLASH
#pragma CODE_SECTION( Get_Ackbuf, "ramfuncs" )
#endif
void Get_Ackbuf( Uint8 *buf )
{
    Uint8 i = 0;
    memset( ACK_BUF, 0, ( RS_ENCODE_COUNT*kk + 2 ) * sizeof(Uint8) );
    ACK_BUF[0] = 0x00FF;
    for( i = 1; i < RS_ENCODE_COUNT*kk; i++ )
    {
        ACK_BUF[i] = buf[i-1];
    }
    ACK_BUF[RS_ENCODE_COUNT*kk] = CheckSum(buf);
    ACK_BUF[RS_ENCODE_COUNT*kk + 1] = 0x002A;
}

#if IS_FLASH
#pragma CODE_SECTION( UartSendData, "ramfuncs" )
#endif
void UartSendData(Uint8 *BUF,Uint8 COUNT )
{
    int i=0;
    while( i != COUNT )
    {
        if(ScibTX_Ready()==1)
        {
            ScibRegs.SCITXBUF=BUF[i++];
        }
    }
}

//Uint8 HEAD1[16] = {0x01,0x00,0x01,0x00,0x00,0x01,0x00,0x01,0x01,0x00,0x01,0x00,0x00,0x01,0x00,0x01};
//Uint8 TAIL[16] = {0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x01,0x00};
//
//
//#if IS_FLASH
//#pragma CODE_SECTION( mystrstr, "ramfuncs" )
//#endif
//Uint8* mystrstr(  Uint8*source,  Uint8 *destin, int16 source_len )
//{
//
//    Uint8 *position = source;
//    Uint8 *head = destin;
//    int16 count = source_len;
//
//    while( source_len > 0 )
//    {
//        position = source;
//        destin = head;
//
//        while( ( destin-head != 13 ) && source_len-- && *destin++ == *source++ );
//
//        if( destin-head == 13 && *( destin-1 ) == *( source-1 ) && source_len >= 0 )
//        {
//            return position;
//        }
//
//        source = position + 1 ;
//        source_len = --count;
//    }
//
//    return NULL;
//}

void Reverse_Address( Uint8 *buf, Uint8 count )
{
    Uint8 temp = 0;
    int8 i = 0;

    for( i = 0; i < (count/2); i++ )
    {
        temp = buf[i];
        buf[i]=buf[count-1-i];
        buf[count-1-i] = temp;
    }
}

#if IS_FLASH
#pragma CODE_SECTION( Protocol_Analysis, "ramfuncs" )
#endif
void Protocol_Analysis()
{
    Uint8 i = 0;
 if( VALUE_BUF[RS_ENCODE_COUNT*kk - 1 ] != CheckSum(VALUE_BUF) )
          {
          	return;
          }
    if( ( VALUE_BUF[1] >> 7 )  == 0x0000  )  //控制域  最高位为0 下行 1上行
    {
        if( ( ( VALUE_BUF[2] >>3 ) & 0x0007 ) == 0x0000 )               //判断是否有中继
        {
            if( VALUE_BUF[4] == LOCAL_ADDRESS )          //没有中继，判断目标地址是否为本机
            {
                if( ( VALUE_BUF[1] & 0x001F )  == 0x0002 )     //判断命令字 02 查询
                {
                  //  VALUE_BUF[ 3 + 1 + 2 + 1 ] = DS18B20_GetTemp();            //数据段的第三个字节为温度
                	VALUE_BUF[ 3 + 1 + 2 + 1 ] = rand()%11 + 20;
                    VALUE_BUF[1] = ( VALUE_BUF[1] & 0x007F ) | 0x0080;         //将模式改为上行
                    Reverse_Address( &VALUE_BUF[3], 2 );
                }

                Get_Ackbuf(VALUE_BUF);
                recv_queue_reset();

                for(i=0; i<(RS_ENCODE_COUNT*kk+2); i++)
                {
                    recv_queue_push(ACK_BUF[i]);
                }
            }
        }
        else
        {
            if( VALUE_BUF[ 3 + (  VALUE_BUF[2] & 0x0007 ) + 1 ] == LOCAL_ADDRESS )  //存在中继判断当前中继是否为本机
            {
                if( (  VALUE_BUF[2] &0x0007 )  == ( ( VALUE_BUF[2] >>3 )&0x0007 ) ) //本机为目标主机
                {
                    if( ( VALUE_BUF[1] & 0x001F )  == 0x0002 )     //判断命令字 02 查询
                    {
                       // VALUE_BUF[ 3 + ( ( VALUE_BUF[2] >>3 )&0x0007 ) + 1 + 2 + 1 ] = DS18B20_GetTemp(); //数据段的第三个字节为温度
                    	VALUE_BUF[ 3 + 1 + 2 + 1 ] = rand()%11 + 20;
                        VALUE_BUF[1] = ( VALUE_BUF[1] & 0x007F ) | 0x0080;         //将模式改为上行
                        Reverse_Address( &VALUE_BUF[3], ( ( VALUE_BUF[2] >>3 ) & 0x0007 ) + 2 ); //反转源 中继 目的 地址
                        VALUE_BUF[2] = VALUE_BUF[2] & 0x00F8;        //清除中继级数
                    }
                }
                else                                                                //继续中继
                {
                    VALUE_BUF[2] =  ( VALUE_BUF[2] &  0x00F8 ) | ( (  VALUE_BUF[2] & 0x0007 ) + 1 ); //当前中继级数加1
                }

                Get_Ackbuf(VALUE_BUF);
                recv_queue_reset();
                for(i=0; i<(RS_ENCODE_COUNT*kk+2); i++)
                {
                    recv_queue_push(ACK_BUF[i]);
                }

            }
        }
    }
    else                 //上行
    {
        if( ( ( VALUE_BUF[2] >>3 ) & 0x0007 ) == 0x0000 )               //判断是否有中继
        {
            if( VALUE_BUF[4] == LOCAL_ADDRESS )          //没有中继，判断目标地址是否为本机(根节点)
            {
                Get_Ackbuf(VALUE_BUF);
                UartSendData( ACK_BUF, 2+RS_ENCODE_COUNT*kk );
            }
        }
        else              //有中继
        {
            if( VALUE_BUF[ 3 + (  VALUE_BUF[2] & 0x0007 ) + 1 ] == LOCAL_ADDRESS )  //存在中继，判断当前中继是否为本机（主机）
            {
                Get_Ackbuf(VALUE_BUF);
                UartSendData( ACK_BUF, 2+RS_ENCODE_COUNT*kk );
            }
            else       //继续中继
            {
                VALUE_BUF[2] =  ( VALUE_BUF[2] &  0x00F8 ) | ( (  VALUE_BUF[2] & 0x0007 ) + 1 ); //当前中继级数加1
                Get_Ackbuf(VALUE_BUF);
                recv_queue_reset();
                for(i=0; i<(RS_ENCODE_COUNT*kk+2); i++)
                {
                    recv_queue_push(ACK_BUF[i]);
                }
            }
        }
    }
}

#if IS_FLASH
#pragma CODE_SECTION( Check, "ramfuncs" )
#endif
static int RecvCount = 0;
static int TotalRightCount = 0;
//位同步丢失
Uint8 LostRate[2] = {0xAA, 0};	//统计丢包率
//协议解析过了打印：位同步、帧同步收到，数据全对
Uint8 TotalRightRate[2] = {0xBB, 0 }; //统计完全正确的包比例
//位同步、帧同步都受到
Uint8 Perr1[5] = {0xCC, 0, 0, 0, 0};		//统计收包时的解码前误码个数
Uint8 Perr2[5] = {0xDD, 0, 0, 0, 0};		//统计收包时的解码后误码个数
void Check(Uint8 *RecvFlag, Uint8 *buf)
{
	static int Psum1 = 0;	//统计收包时的解码前误码个数
	static int Psum2 = 0;	//统计收包时的解码后误码个数
	static int Tsum1 = 0;	//统计全部的解码前误码个数
	unsigned int Demon1=0;
	int i=0, TotalRightFlag = 1;
	Uint8 xor=0;
	static int Uart_flag = 1;
	int start = 4;		//目的地址在DATA_BUF中的索引
	if(RecvFlag)
	{
		RecvCount++;
		Uart_flag = 1;
		//start = 3+((DATA_BUF[2]>>3) & 0x0007)+2-1;		//目的地址在DATA_BUF中的索引
		//部分解码前误比特率
		for(i=0; i<8*RS_ENCODE_COUNT*nn; i++)
		{
			if( ( (8*(start+3)-1 < i) &&(i < 8*(start+3+1)))  | ((8*(start+9)-1 < i) && (i < 8*(start+9+1))))	//除去温度和校验位
				continue;
			else
			{
				Psum1 += (buf[i]&0x01) ^ (SEND_BUF[24+i]&0x01);
				if((buf[i]&0x01) ^ (SEND_BUF[24+i]&0x01))
					Demon1 = 0;
			}

		}
		//部分解码后误码率
		Reverse_Address( &DATA_BUF[3], 2 );
		for(i=0; i<RS_ENCODE_COUNT*kk; i++)
		{
			if((i-1)*(i-start-3)*(i-start-9)==0)			//除去温度和校验位
			{
				continue;
			}
			else
			{
				xor = (VALUE_BUF[i]^DATA_BUF[i])&0x00ff;
				if(xor)
				{
					TotalRightFlag = 0;
				}
				while(xor)//统计xor二进制表示中“1”的个数
				{
					xor &= (xor-1);
					Psum2++;
				}
			}
		}
		//全部解码前误比特率
		Tsum1 = Tsum1 + Psum1;
		if(TotalRightFlag == 1)
			TotalRightCount ++;
	}

	if(RecvCount%10 == 0 && Uart_flag == 1)
	{
		Uart_flag = 0;
//		if(RecvCount==0)
//		{
//			Perr1[1] = 100;
//			Perr2[1] = 100;
//		}
//		else
//		{
//			//Perr1[1] = 1000*Psum1/(8*RS_ENCODE_COUNT*(nn-2)*RecvCount);//计算百分数数值
//			//Perr2[1] = 1000*Psum2/(8*RS_ENCODE_COUNT*(kk-3)*RecvCount);//计算百分数数值
//        }
		Perr1[1] = (Psum1&0xff00)>>8;
		Perr1[2] = (Psum1&0x00ff);
		Demon1 = 8*RS_ENCODE_COUNT*(nn-2)*RecvCount;
		Perr1[3] = (Demon1&0xff00)>>8;
		Perr1[4] = (Demon1&0x00ff);

		Perr2[1] = (Psum2&0xff00)>>8;
		Perr2[2] = (Psum2&0x00ff);
		Demon1 = 8*RS_ENCODE_COUNT*(kk-2)*RecvCount;
		Perr2[3] = (Demon1&0xff00)>>8;
		Perr2[4] = (Demon1&0x00ff);

		//Perr3[1] = 100*Tsum1/(8*RS_ENCODE_COUNT*nn*(RecvCount+LostCount));//计算百分数数值
	    LostRate[1] = 100*RecvCount/SendCount;
	    //RightRate[1] = 100*RecvCount/SendCount;
		TotalRightRate[1] = 100*TotalRightCount/SendCount;

		UartSendData( LostRate, 2 );
		//UartSendData( RightRate, 2 );
	    UartSendData( TotalRightRate, 2 );
	   // UartSendData( Perr3, 2 );
		UartSendData( Perr1, 5 );
		UartSendData( Perr2, 5 );


	}

}

Uint8 MidInterval = Interval/2;

int8 errorInterval = (MPPSK_RG*ADC_RATE/PWM_RATE);//rg



#if IS_FLASH
#pragma CODE_SECTION( Decision, "ramfuncs" )
#endif
static int8 Decision(int32* srcVector,Uint8 num, Uint8* codeVector)
{
    int8 maxIndex = 0;
    Uint8 codeNum = 0;
    static Uint8 bitCount = 0,isSynchronized = 0;
    static int8 StartIndex = 0;
    static int32 preData[Interval];
    static Uint8 RECV_BUF[MAX_DECISION_SIZE];
    static Uint8 preDataNum = 0;
    static Uint8 RECV_COUNT =0;
    static int8 sync_index=0;
	
    Uint8 count = 0,temp_maxvalue = 0,open = 1,i = 0;
    int8  count1 = 0,windowStartIndex = -1,indexMax = 0,indexMax1 = 0;
    //merge
    for(count1 = preDataNum - 1; count1 >= 0; count1-- )
        *(--srcVector) = preData[count1];//放入前一次未处理的数
    num = num + preDataNum;
    indexMax = num - Interval;
    indexMax1 = num - 1;
    // search synchronization
    while((!isSynchronized)&&( windowStartIndex < indexMax))
    {
		if(open)
    	{
         	for( i=0; i < 7* (MPPSK_K*ADC_RATE/PWM_RATE + 1); i++ )
    	    {
    		  temp_maxvalue += srcVector[i];
    	    }
    		Threshold_Value = temp_maxvalue/7;
    		open = 0;
    	 }
        //search the index of the max
			maxIndex = Find_SynIndex(&srcVector[windowStartIndex+1],Interval);//找最大值位置
			if( (  ( maxIndex >= 1 ) && ( maxIndex - sync_index) >= ((-1)*errorInterval)) && ( (maxIndex - sync_index) <= errorInterval) )//36-44之间
			{
				bitCount ++;
			}
			else
			{
				if( bitCount >= 2 )
				{
					maxIndex = sync_index;
					bitCount ++;
				}
				else
				{
					bitCount=1;
				}

			}
			windowStartIndex = windowStartIndex + Interval;    //???
			sync_index = maxIndex;

			if(bitCount == synCodeNum )
			{
//	        	if( Decision_Syn(&srcVector[windowStartIndex - Interval -MPPSK_K*ADC_RATE/PWM_RATE],maxIndex + MPPSK_K*ADC_RATE/PWM_RATE ) )
//	        	{
//	        	     bitCount=1;
//	        	}
//	            else
				{
					//Threshold_Value_Decision = Threshold_Value;
					isSynchronized = 1;
					StartIndex = windowStartIndex -Interval + maxIndex;
				}
			}
      }

    if( isSynchronized )
    {
        windowStartIndex = StartIndex;
    }

    while(isSynchronized &&( windowStartIndex < indexMax1 ))
    {
        //max algorithm
        //search the index of the max
        if( windowStartIndex + Interval + DECISION_SIZE > indexMax1  )
        {
            if(  indexMax1  - windowStartIndex <  Interval -DECISION_SIZE ) //1-75
            {
                StartIndex =  (-1)*( indexMax1  - windowStartIndex);//
                windowStartIndex = indexMax1 ;
            }
            else
            {
                StartIndex = DECISION_SIZE - Interval;
                windowStartIndex = windowStartIndex + Interval-DECISION_SIZE-1;
            }
            break;
        }
        else
        {
            if( !DsearchMaxIndex(srcVector,&windowStartIndex) )
            {
                codeVector[codeNum++ ] = 0;
            }
            else
            {
                codeVector[codeNum++ ] = 1;
            }
        }
    }
    for( count = windowStartIndex + 1,preDataNum=0 ; count < num && preDataNum < Interval ; count++,preDataNum++)
    {
        preData[preDataNum] = srcVector[count];
    }

    if( codeNum > 0 )
    {

        if( ( RECV_COUNT + codeNum ) > MAX_DECISION_SIZE)
        {
           // Uint8 * tail = mystrstr( RECV_BUF, TAIL, RECV_COUNT );
           // Uint8 * head = mystrstr( RECV_BUF, HEAD, RECV_COUNT );

        	Uint8 * head = Find_Frame(RECV_BUF);
            if( head )
            {
                Conversion_To_Recvbuf( head + 13, RS_ENCODE_COUNT*nn*8);//RS_ENCODE_COUNT*nn*8
               // UartSendData(VALUE_BUF + 5,8);
                Protocol_Analysis();
//                if(Send_Flag)
//                {
//                	Check(head, head+13);
//                	Send_Flag = 0;
//                }


            }
            memset( VALUE_BUF, 0, RS_ENCODE_COUNT*nn *sizeof(Uint8));
            memset( RECV_BUF, 0, MAX_DECISION_SIZE*sizeof(Uint8));
            RECV_COUNT = 0;
            isSynchronized = 0;  //add by zxf
            bitCount = 0;        //add by zxf
            StartIndex = 0;
            Threshold_Value = 0;
			open = 1;

        }
        else
        {
            memcpy( RECV_BUF + RECV_COUNT, codeVector, codeNum );
            RECV_COUNT = RECV_COUNT + codeNum;
        }


    }
    return codeNum;
}
`
