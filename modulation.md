`#include <stdlib.h>
#include <string.h>
#include "DSP28_Device.h"`

`static RECV_QUEUE recv_que = {0,0,0};

 Uint8 SEND_COUNT = 0;

 Uint8 Send_Cmd_Flag = 0;

 Uint8 DATA_BUF[RS_ENCODE_COUNT*kk] = {0};

 unsigned char  SEND_BUF_pre_encode[RS_ENCODE_COUNT*nn]= {0};

 Uint8 SEND_BUF[(2 + 1 + 2 + RS_ENCODE_COUNT*nn + 2 ) * 8] = {0};

void recv_queue_reset()
{

    recv_que.head = recv_que.tail = 0;
}

void recv_queue_push(Uint8 data)
{

    {
        Uint8 pos = (recv_que.head+1)%RECV_QUEUE_MAX_SIZE;
        if(pos!=recv_que.tail)
            recv_que.data[recv_que.head] = data;
        recv_que.head = pos;
    }
}

static void recv_queue_pop(Uint8* _data)
{

    if(recv_que.tail!=recv_que.head)//
    {
        *_data = recv_que.data[recv_que.tail];
        recv_que.tail = (recv_que.tail+1)%RECV_QUEUE_MAX_SIZE;
    }
}

static Uint8 recv_queue_size()
{

    return ((recv_que.head+RECV_QUEUE_MAX_SIZE-recv_que.tail)%RECV_QUEUE_MAX_SIZE);
}

void Conversion_To_Sendbuf()
{

    int i = 0;
    memset( (void *)SEND_BUF, 0, (2 + 1 + 2 + RS_ENCODE_COUNT*nn + 2 ) * 8);

	//    for( i = 0; i < 16; i++)    //2
	//    {
	//        SEND_BUF[i] = 0x0000;
	//    }
	//    SEND_BUF[15] = 0x0001;      //防止连续8个以上的0


    SEND_BUF[0] = 0x0001;      //3
    SEND_BUF[1] = 0x0001;
    SEND_BUF[2] = 0x0001;
    SEND_BUF[3] = 0x0001;
    SEND_BUF[4] = 0x0001;
    SEND_BUF[5] = 0x0001;
    SEND_BUF[6] = 0x0001;
    SEND_BUF[7] = 0x0001;

    SEND_BUF[8] =  0x0000;    //4
    SEND_BUF[9] =  0x0000;
    SEND_BUF[10] =  0x0000;

    SEND_BUF[11] =  0x0001;
    SEND_BUF[12] =  0x0001;
    SEND_BUF[13] =  0x0001;
    SEND_BUF[14] =  0x0001;
    SEND_BUF[15] =  0x0001;

    SEND_BUF[16] =  0x0000;   //5
    SEND_BUF[17] =  0x0000;
    SEND_BUF[18] =  0x0001;
    SEND_BUF[19] =  0x0001;
    SEND_BUF[20] =  0x0000;
    SEND_BUF[21] =  0x0001;
    SEND_BUF[22] =  0x0000;
    SEND_BUF[23] =  0x0001;

    for( i = 0; i < RS_ENCODE_COUNT; i++ )
    {
        encode_rs_8(SEND_BUF_pre_encode+nn*i, (unsigned char *)DATA_BUF + kk*i, bb);
    }


    for( i = 24; i < (RS_ENCODE_COUNT*nn)*8 + 24; i = i + 8  )
    {
        SEND_BUF[i]   =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>7 & 0x01;
        SEND_BUF[i+1] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>6 & 0x01;
        SEND_BUF[i+2] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>5 & 0x01;
        SEND_BUF[i+3] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>4 & 0x01;
        SEND_BUF[i+4] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>3 & 0x01;
        SEND_BUF[i+5] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>2 & 0x01;
        SEND_BUF[i+6] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ]>>1 & 0x01;
        SEND_BUF[i+7] =  SEND_BUF_pre_encode[ ( i - 24 )/8 ] & 0x01;
    }

    SEND_COUNT = ( 1 + 2 + RS_ENCODE_COUNT*nn ) * 8;
}

Uint8 CheckSum(Uint8 *buf)
{

    Uint8 i=0;
    Uint8 sum=0;
    for(i=0; i<=RS_ENCODE_COUNT*kk-2; i++)
        sum+=buf[i];
    sum=sum&0x00ff;
    return sum;
}

void recv_queue_get_data(void)
{

	//memset( SEND_BUF, 0, (2 + 1 + 2 + RS_ENCODE_COUNT*nn + 2 ) * 8*sizeof(Uint8) );
    static int recv_cmd_pos = 0;
    Uint8 _data = 0;

    while( recv_queue_size() > 0 )
    {
        //
        recv_queue_pop( &_data );

        _data = _data & 0x00ff;

        if( recv_cmd_pos == 0 && _data!=0x00FF )//
        {
            continue;
        }

        if( recv_cmd_pos >= 1 && recv_cmd_pos <= RS_ENCODE_COUNT*kk - 1 )
        {
            DATA_BUF[ recv_cmd_pos -1 ] = _data;
        }

        if( recv_cmd_pos == RS_ENCODE_COUNT*kk )//
        {
            if( _data != CheckSum(DATA_BUF) )
            {
                memset( DATA_BUF, 0, RS_ENCODE_COUNT*kk );
                recv_cmd_pos = 0;
                continue;
            }
            else
            {
                DATA_BUF[recv_cmd_pos-1] = _data;
                Conversion_To_Sendbuf();     //发送
               // memset( DATA_BUF, 0, RS_ENCODE_COUNT*kk );
                recv_cmd_pos = 0;
		#if IS_FULL
		//                EALLOW;
		//                GpioMuxRegs.GPAMUX.bit.PWM1_GPIOA0=1;   //设置PWM1引脚
		//                EDIS;
		//              EvaRegs.ACTR.bit.CMP1ACT=2;//设定引脚PWM1~6的动作属性
		//              EvaRegs.GPTCONA.bit.T1PIN=2;//高有效
                EvaRegs.T1CON.bit.TENABLE=1; //开启T1中断，发送PWM波
                EvaRegs.T2CON.bit.TENABLE=0;//关闭T2（ADC）采样，等待PWM波发送完

                Flag = 0;  //终止当前的解调
		#endif
               // DSP28x_usDelay(100);
                Send_Cmd_Flag = 1;
                return;
            }
        }
        recv_cmd_pos++;
    }
    return;//
}

/**
    	 * @brief  将二进制编码转换为调制波形输出
    	 * @param  1.codeArray        码组
    	 *         2.codeNum          码组个数
    	 * @retval none
    	 */
Uint8 TIMER_PERIOD= T1PR_VALUE;

//Uint8 MODULATE_DUTY= T1CMP_VALUE;

Uint8 MODULATE_CMPR= (T1CMP_VALUE+1)/2;

Uint8 SendCount = 0;

Uint8 Send_Flag = 0;

void ModuLateOutput(  Uint8* codeArray,  Uint8* codeNumPtr)
{

    static Uint8 carrierCount = 0;
    static Uint8 curBit = 0;
	//   static Uint8 nextBit = 0;
    static Uint8 codeCount = 0;

    //改回正确的周期
    TIMER_SetPeriodVal( TIMER_PERIOD + 1);
    if(carrierCount == 0)
    {
        //码组发送结束
        if(codeCount == (*codeNumPtr))
        {
            codeCount = 0;
            //		    	(*codeNumPtr) = 0;
            //				Modulate_StopCarrier();
            Send_Cmd_Flag = 0;
            Send_Flag = 1;
            SendCount++;
	#if IS_FULL
	#if IS_SEND_LOOP
		    Send_Cmd_Flag = 1;
	#else
	//             EALLOW;
	//             GpioMuxRegs.GPAMUX.bit.PWM1_GPIOA0=0;   //设置PWM1引脚,1为功能引脚
	//             GpioMuxRegs.GPADIR.bit.GPIOA0 = 1;
	//             GpioDataRegs.GPACLEAR.bit.GPIOA0 = 1;
	//             EDIS;
	//           EvaRegs.GPTCONA.bit.T1PIN=0;//高有效
	//           EvaRegs.ACTR.bit.CMP1ACT=0;//设定引脚PWM1~6的动作属性
             EvaRegs.T1CON.bit.TENABLE=0;//码组发送结束
             DSP28x_usDelay(250000);
             EvaRegs.T2CON.bit.TENABLE=1;
	#endif
	#endif
            return;
        }
        //开始发送
        //Modulate_StartCarrier();
        //EvaRegs.T1CON.bit.TENABLE=1;//使能T1计数，pwm
        curBit = codeArray[codeCount++];
	//        nextBit = codeArray[codeCount % (*codeNumPtr)];
    }


     if(curBit)
    {
        if(carrierCount<MPPSK_K)
        {
            TIMER_SetPeriodVal( MODULATE_CMPR );
        }
    }
	//    else if(curBit)
	//    {
	//        if(carrierCount>=(MPPSK_K+MPPSK_RG)&&carrierCount<(2*MPPSK_K+MPPSK_RG))
	//        {
	//            TIMER_SetPeriodVal( MODULATE_CMPR );
	//        }
	//    }
	//    else if( !curBit && (carrierCount == MPPSK_ZERO_END))  //0码元，相位反转
	//    {
	//        //下个周期相位突变，减少半个周期
	//        TIMER_SetPeriodVal( MODULATE_DUTY - 1 );       //???
	//    }
	//    else if( curBit && (carrierCount == MPPSK_ONE_START)) //1码元，相位反转
	//    {
	//        //下个周期相位突变，+半个周期
	//        TIMER_SetPeriodVal( TIMER_PERIOD + MODULATE_DUTY  );
	//    }
	//    else if( curBit && (carrierCount == MPPSK_ONE_END))
	//    {
	//        //下个周期相位突变，减少半个周期
	//        TIMER_SetPeriodVal( MODULATE_DUTY - 1 );
	//    }
	//    else if( (carrierCount == MPPSK_ZERO_ADD) && !nextBit )
	//    {
	//        //最后一个周期， 如果下一个编码是0, 增加半个周期，这样总体上周期并没有改变
	//        TIMER_SetPeriodVal( TIMER_PERIOD + MODULATE_DUTY );
	//    }
    carrierCount = (carrierCount + 1) % MPPSK_N;
}`
