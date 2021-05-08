
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
*
* Modified for EECE 433 Lab 3
* 05/05/2021 Dylan Huntsman
*
*Finds frequency data for
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/******************************************************************************************
 * Module Defines
 *****************************************************************************************/
#define FFT_LENGTH          DSP_SAMPLES_PER_BLOCK
//Supported Lengths: 32, 64, 128, 256, 512, 1024, 2048
                                //Must be <= AUDIO_SAMPLES_PER_BLOCK unless zero padded

#define SAMPLE_RATE_HZ      48000
#define FREQ_CONST          44739 //20.833 * 2^31
#define FREQ_NORM(x)        x*SAMPLE_RATE_HZ/FFT_LENGTH
/******************************************************************************************
 * Private variables
 *****************************************************************************************/
static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];
static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;
static q31_t inBuffCopy[DSP_SAMPLES_PER_BLOCK];
static q31_t fftResultLeft[DSP_SAMPLES_PER_BLOCK*2];
static q31_t fftResultMagLeft[DSP_SAMPLES_PER_BLOCK];
static q31_t fftResultMod[DSP_SAMPLES_PER_BLOCK*2];
static q31_t fftResultMagMod[DSP_SAMPLES_PER_BLOCK];
static q31_t cosq[DSP_SAMPLES_PER_BLOCK];
static q31_t qbuff[DSP_SAMPLES_PER_BLOCK];
static q31_t modq[DSP_SAMPLES_PER_BLOCK];
static float32_t floatbuf[DSP_SAMPLES_PER_BLOCK];
static float32_t cosfloat[DSP_SAMPLES_PER_BLOCK];
static float32_t fftResultFloat[DSP_SAMPLES_PER_BLOCK*2];
static float32_t fftResultMagFloat[DSP_SAMPLES_PER_BLOCK];
static void  dspTask(void *p_arg);
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};
/******************************************************************************************
 * Private fft instances
 *****************************************************************************************/
static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Left;
static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Mod;
static arm_rfft_fast_instance_f32 arm_rfft_sR_f32_len128Mod;
typedef enum {STARTUP, COMPLETE} MATH_STATES;
static MATH_STATES MathState;
typedef struct {
    q31_t maxvalue[1];
    uint32_t index[1];
} DSP_FREQ_BIN;
typedef struct {
    float32_t maxvalue[1];
    uint32_t index[1];
} DSP_FLOAT_FREQ_BIN;

static DSP_FREQ_BIN LEFT_BINS;
static DSP_FREQ_BIN MOD_BINS;
static DSP_FLOAT_FREQ_BIN FLOAT_BINS;
/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    SPProcessInit();
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();


}

void SPProcessInit(void){

//init Real FFT instance
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Left,DSP_SAMPLES_PER_BLOCK, 0, 1);
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Mod, DSP_SAMPLES_PER_BLOCK, 0, 1);
    arm_rfft_fast_init_f32(&arm_rfft_sR_f32_len128Mod, (uint16_t)DSP_SAMPLES_PER_BLOCK);
}

/*******************************************************************************************
* dspTask
*******************************************************************************************/
static void dspTask(void *p_arg){

    OS_ERR os_err;
    INT8U buffer_index;
    INT16U i;
    float32_t w = 9375*2*PI;
    (void)p_arg;
    OSTimeDly(3000,OS_OPT_TIME_PERIODIC,&os_err);
    while(1){

        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
        // The following code implements a pass through
        dspOutBuffer[DSP_LEFT_CH][buffer_index] = dspInBuffer[DSP_LEFT_CH][buffer_index]; //Left Channel
        dspOutBuffer[DSP_RIGHT_CH][buffer_index] = dspInBuffer[DSP_RIGHT_CH][buffer_index]; //Right Channel

        //Only generates cosine signal once for speed
        //Copies dspInBuffer values into 3 separate arrays for each part of the lab. arm_rfft_x functions modify source buffers
        if(MathState == STARTUP){
            for (i = 0; i < DSP_SAMPLES_PER_BLOCK; i++){
                cosfloat[i] = arm_cos_f32(w*i/SAMPLE_RATE_HZ)/5;
                floatbuf[i] = (((float32_t) dspInBuffer[DSP_LEFT_CH][buffer_index].samples[i])/(1<<12))*cosfloat[i];
                inBuffCopy[i] = dspInBuffer[DSP_LEFT_CH][buffer_index].samples[i];
                qbuff[i] = dspInBuffer[DSP_LEFT_CH][buffer_index].samples[i];
            }
            arm_float_to_q31(&cosfloat[0], &cosq[0], DSP_SAMPLES_PER_BLOCK);
            MathState = COMPLETE;
        } else {
            for (i = 0; i < DSP_SAMPLES_PER_BLOCK; i++){
                floatbuf[i] = (((float32_t) dspInBuffer[DSP_LEFT_CH][buffer_index].samples[i]/(1<<12)))*cosfloat[i];
                qbuff[i] = dspInBuffer[DSP_LEFT_CH][buffer_index].samples[i];
                inBuffCopy[i] = dspInBuffer[DSP_LEFT_CH][buffer_index].samples[i];
            }
        }

        //Parts 2 and 3 called here. If your computer can't run them all at once, comment them out to improve performance.
        dspPart2();
        dspPart3();
        dspPart3Q31();

        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}


/*******************************************************************************************
* dspPart2 - calculates the frequency that is being input into the codec board
* Parameters: None
* Returns: None
*******************************************************************************************/

void dspPart2(void){
    //Left Channel
        arm_rfft_q31(&arm_rfft_sR_q31_len128Left, &inBuffCopy[0],
                &fftResultLeft[0]);
        arm_cmplx_mag_q31(&fftResultLeft[0], &fftResultMagLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_max_q31(&fftResultMagLeft[0], DSP_SAMPLES_PER_BLOCK, &LEFT_BINS.maxvalue[0], &LEFT_BINS.index[0]);
        LEFT_BINS.index[0] = FREQ_NORM(LEFT_BINS.index[0]);

}

/*******************************************************************************************
* dspPart3 - calculates the modulated frequency that is being input into the codec board and
*            modulated by a 9375 Hz cosine using floating point.
* Parameters: None
* Returns: None
*******************************************************************************************/

void dspPart3(void){
    arm_rfft_fast_f32(&arm_rfft_sR_f32_len128Mod, &floatbuf[0], &fftResultFloat[0], 0);
    arm_cmplx_mag_f32(&fftResultFloat[0], &fftResultMagFloat[0], DSP_SAMPLES_PER_BLOCK);
    arm_max_f32(&fftResultMagFloat[0], DSP_SAMPLES_PER_BLOCK, &FLOAT_BINS.maxvalue[0], &FLOAT_BINS.index[0]);
    FLOAT_BINS.index[0] = FREQ_NORM(FLOAT_BINS.index[0]);
    FLOAT_BINS.maxvalue[0] = (FLOAT_BINS.maxvalue[0]/DSP_SAMPLES_PER_BLOCK)/(1<<31);

}

/*******************************************************************************************
* dspPart3Q31 - calculates the modulated frequency that is being input into the codec board
*               and modulated by a 9375 Hz cosine using fixed point.
* Parameters: None
* Returns: None
*******************************************************************************************/

void dspPart3Q31(void){

    arm_mult_q31(&cosq[0], &qbuff[0], &modq[0], DSP_SAMPLES_PER_BLOCK);
    arm_rfft_q31(&arm_rfft_sR_q31_len128Mod, &modq[0], &fftResultMod[0]);
    arm_cmplx_mag_q31(&fftResultMod[0], &fftResultMagMod[0],DSP_SAMPLES_PER_BLOCK);
    arm_max_q31(&fftResultMagMod[0], DSP_SAMPLES_PER_BLOCK, &MOD_BINS.maxvalue[0], &MOD_BINS.index[0]);
    MOD_BINS.index[0] = FREQ_NORM(MOD_BINS.index[0]);

}

/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){

    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];

}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){

    return dspParams.ssize;

}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){

    return dspParams.srate;

}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){

    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];

}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}


