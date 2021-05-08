/*****************************************************************************************************
* Module definition against multiple inclusion
*****************************************************************************************************/

/*****************************************************************************************************
* Include files
*****************************************************************************************************/

/*****************************************************************************************************
* Declaration of project wide TYPES
*****************************************************************************************************/

/*****************************************************************************************************
* Definition of project wide VARIABLES
*****************************************************************************************************/

/*****************************************************************************************************
* Definition of project wide MACROS / #DEFINE-CONSTANTS
*****************************************************************************************************/
#define FIFO_TX_WM          4       //I2S transmit FIFO watermark
#define FIFO_RX_WM          4       //I2S receive FIFO watermark

#define I2S_TX_ENABLE()	    (I2S0->TCSR |= I2S_TCSR_TE_MASK)
#define I2S_TX_DISABLE()	(I2S0->TCSR &= ~I2S_TCSR_TE_MASK)

#define I2S_RX_ENABLE()	    (I2S0->RCSR |= I2S_RCSR_RE_MASK)
#define I2S_RX_DISABLE()	(I2S0->RCSR &= ~I2S_RCSR_RE_MASK)

/*****************************************************************************************************
* Declaration of project wide FUNCTIONS
*****************************************************************************************************/

/*****************************************************************************************************
* Declaration of module wide FUNCTIONs - NOT for use in other modules
*****************************************************************************************************/
void I2SInit(INT8U size_code);
void I2SWordSizeSet(INT8U size_code);
