/*******************************************************************************
* File Name: delete_this.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_delete_this_H) /* Pins delete_this_H */
#define CY_PINS_delete_this_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "delete_this_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 delete_this__PORT == 15 && ((delete_this__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    delete_this_Write(uint8 value);
void    delete_this_SetDriveMode(uint8 mode);
uint8   delete_this_ReadDataReg(void);
uint8   delete_this_Read(void);
void    delete_this_SetInterruptMode(uint16 position, uint16 mode);
uint8   delete_this_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the delete_this_SetDriveMode() function.
     *  @{
     */
        #define delete_this_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define delete_this_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define delete_this_DM_RES_UP          PIN_DM_RES_UP
        #define delete_this_DM_RES_DWN         PIN_DM_RES_DWN
        #define delete_this_DM_OD_LO           PIN_DM_OD_LO
        #define delete_this_DM_OD_HI           PIN_DM_OD_HI
        #define delete_this_DM_STRONG          PIN_DM_STRONG
        #define delete_this_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define delete_this_MASK               delete_this__MASK
#define delete_this_SHIFT              delete_this__SHIFT
#define delete_this_WIDTH              1u

/* Interrupt constants */
#if defined(delete_this__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in delete_this_SetInterruptMode() function.
     *  @{
     */
        #define delete_this_INTR_NONE      (uint16)(0x0000u)
        #define delete_this_INTR_RISING    (uint16)(0x0001u)
        #define delete_this_INTR_FALLING   (uint16)(0x0002u)
        #define delete_this_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define delete_this_INTR_MASK      (0x01u) 
#endif /* (delete_this__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define delete_this_PS                     (* (reg8 *) delete_this__PS)
/* Data Register */
#define delete_this_DR                     (* (reg8 *) delete_this__DR)
/* Port Number */
#define delete_this_PRT_NUM                (* (reg8 *) delete_this__PRT) 
/* Connect to Analog Globals */                                                  
#define delete_this_AG                     (* (reg8 *) delete_this__AG)                       
/* Analog MUX bux enable */
#define delete_this_AMUX                   (* (reg8 *) delete_this__AMUX) 
/* Bidirectional Enable */                                                        
#define delete_this_BIE                    (* (reg8 *) delete_this__BIE)
/* Bit-mask for Aliased Register Access */
#define delete_this_BIT_MASK               (* (reg8 *) delete_this__BIT_MASK)
/* Bypass Enable */
#define delete_this_BYP                    (* (reg8 *) delete_this__BYP)
/* Port wide control signals */                                                   
#define delete_this_CTL                    (* (reg8 *) delete_this__CTL)
/* Drive Modes */
#define delete_this_DM0                    (* (reg8 *) delete_this__DM0) 
#define delete_this_DM1                    (* (reg8 *) delete_this__DM1)
#define delete_this_DM2                    (* (reg8 *) delete_this__DM2) 
/* Input Buffer Disable Override */
#define delete_this_INP_DIS                (* (reg8 *) delete_this__INP_DIS)
/* LCD Common or Segment Drive */
#define delete_this_LCD_COM_SEG            (* (reg8 *) delete_this__LCD_COM_SEG)
/* Enable Segment LCD */
#define delete_this_LCD_EN                 (* (reg8 *) delete_this__LCD_EN)
/* Slew Rate Control */
#define delete_this_SLW                    (* (reg8 *) delete_this__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define delete_this_PRTDSI__CAPS_SEL       (* (reg8 *) delete_this__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define delete_this_PRTDSI__DBL_SYNC_IN    (* (reg8 *) delete_this__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define delete_this_PRTDSI__OE_SEL0        (* (reg8 *) delete_this__PRTDSI__OE_SEL0) 
#define delete_this_PRTDSI__OE_SEL1        (* (reg8 *) delete_this__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define delete_this_PRTDSI__OUT_SEL0       (* (reg8 *) delete_this__PRTDSI__OUT_SEL0) 
#define delete_this_PRTDSI__OUT_SEL1       (* (reg8 *) delete_this__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define delete_this_PRTDSI__SYNC_OUT       (* (reg8 *) delete_this__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(delete_this__SIO_CFG)
    #define delete_this_SIO_HYST_EN        (* (reg8 *) delete_this__SIO_HYST_EN)
    #define delete_this_SIO_REG_HIFREQ     (* (reg8 *) delete_this__SIO_REG_HIFREQ)
    #define delete_this_SIO_CFG            (* (reg8 *) delete_this__SIO_CFG)
    #define delete_this_SIO_DIFF           (* (reg8 *) delete_this__SIO_DIFF)
#endif /* (delete_this__SIO_CFG) */

/* Interrupt Registers */
#if defined(delete_this__INTSTAT)
    #define delete_this_INTSTAT            (* (reg8 *) delete_this__INTSTAT)
    #define delete_this_SNAP               (* (reg8 *) delete_this__SNAP)
    
	#define delete_this_0_INTTYPE_REG 		(* (reg8 *) delete_this__0__INTTYPE)
#endif /* (delete_this__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_delete_this_H */


/* [] END OF FILE */
