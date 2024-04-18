/******************************************************************************
* File Name: main.c
*
* Version: 1.11
*
* Description: This is the source code for the ADC and UART code example.
*
* Related Document: CE195277_ADC_and_UART.pdf
*
* Hardware Dependency: See CE195277_ADC_and_UART.pdf
*
* Note:
* EDITED BY: Andreas BOTH
* DATE: 2024-04-18
* COURSE ID:tx00db04
* DESCRIPTION:  Added a I2C and SPI bus interface to the corresponding sensors
*               Outputs the data in JSON format
*
*******************************************************************************
* Copyright (2018-2020), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/

#include <project.h>
#include "stdio.h"

/* Project Defines */
#define FALSE                   0
#define TRUE                    1
#define TRANSMIT_BUFFER_SIZE    50
#define I2C_SLAVE_ADDRESS       0b1001010
#define I2C_WRITE               0
#define I2C_READ                1
#define I2C_COMMAND_READ_TEMP    0x00
#define SPI_OFFSET_CORRECTION   142
#define SPI_DISCARD_BITS_MASK   0x1fff
#define SPI_ENABLE_CS           0x00
#define SPI_DISABLE_CS          0x01

/*Global Variables*/
/*used to store the sum of the input over the duration of one second
64 Bit value that way you can add 16 Bit into it about 2^48 times*/
volatile uint64 gInputSum = 0;

volatile uint32 gAverageData = 0;
volatile uint8 gSendAverage = 0; //Flag set every second
volatile uint16 gSpiRxBuffer =0;

/*******************************************************************************
* Function Name: main
********************************************************************************
*
*
*******************************************************************************/
int main()
{
    /* Variable to store ADC result */
    uint16 temperatureIntPart;      //stores the integer value of the temperature
    uint16 temperatureDecimalPart;  //stores the decimal part of the temperature
    
    /*Vaeiable to store SPI result*/
    uint16  temperatureSPIIntPart=0;
    uint16  temperatureSPIDecimalPart=0;
    
    /* Transmit Buffer */
    char TransmitBuffer[TRANSMIT_BUFFER_SIZE];
    uint16 TransmitDummy[2] = {0,1};
    
    /* Start the components */
    ADC_DelSig_1_Start();
    UART_1_Start();
    I2C_Start();
    SPI_Start();
    
    
    isr_eoc_getValue_ClearPending();
    isr_eoc_getValue_Start();
    
    isr_averageData_ClearPending();
    isr_averageData_Start();
    

    
    CyGlobalIntEnable
    /* Initialize Variables */
    gSendAverage = FALSE;
    
    /* Start the ADC conversion */
    ADC_DelSig_1_StartConvert();
    
    /* Send message to verify COM port is connected properly */
    UART_1_PutString("COM Port Open\r\n");
    int x;
    uint16 spiADCValue = 0;
    SPI_SS_Write(0x01);
    int SPI_flag=0;
    for(;;)
    {         
            if(gSendAverage)
            {
             /* Format ADC result for transmition */
             CyGlobalIntDisable
            //I2C
            I2C_MasterSendStart(I2C_SLAVE_ADDRESS,I2C_WRITE);   //initiates transmission with I2C slave
            I2C_MasterWriteByte(I2C_COMMAND_READ_TEMP);         //writes command byte of the I2C device to return temp
            I2C_MasterSendRestart(I2C_SLAVE_ADDRESS,I2C_READ);  //innitiates read
            x= I2C_MasterReadByte(I2C_ACK_DATA);                //read and ACK
            I2C_MasterSendStop();                               //stop transmission
            
            //SPI
            SPI_ClearRxBuffer();                                //clear all buffers
            SPI_ClearTxBuffer();
            SPI_ClearFIFO();
            SPI_SS_Write(SPI_ENABLE_CS);                        //enable external ADC and start conversion
            SPI_PutArray(TransmitDummy,2);                      //transmit dummy data to start SCKL output
            gSpiRxBuffer= SPI_ReadRxData();                     //reads bus data
            SPI_SS_Write(SPI_DISABLE_CS);                       //disables external ADC
            gSpiRxBuffer = gSpiRxBuffer&SPI_DISCARD_BITS_MASK;  //removes the first 4 bits of the Int value since 
                                                                //they do not contain a defined state 
                                                                //(see datasheet MCP3201)
            spiADCValue = gSpiRxBuffer-SPI_OFFSET_CORRECTION;
            
            
            /* format output to JSON*/
            temperatureIntPart     = gAverageData / 10;     //drops the decimal part of the temperature
            temperatureDecimalPart = gAverageData % 10;     //drops the integer part of the temperature
            temperatureSPIIntPart = spiADCValue / 10;       //drops the decimal part of the temperature
            temperatureSPIDecimalPart = spiADCValue % 10;   //drops the integer part of the temperature
            
            sprintf(TransmitBuffer, "{\n\"Voltage [mV]\" : %lu,\r\n \"Temperature [C]\" : %hu.%hu, \r\n \"I2C [C]\" : %i,\r\n \"SPI [C]\":%i.%i\r\n}\r\n",gAverageData, temperatureIntPart,temperatureDecimalPart,x,temperatureSPIIntPart,temperatureSPIDecimalPart);
             
            
            
             /* Send out the data */
             UART_1_PutString(TransmitBuffer);
           
             
            CyGlobalIntEnable
            
            
            /* Reset the sendAverage flag */
            gSendAverage = FALSE;
        }        
    }
}


/* [] END OF FILE */
