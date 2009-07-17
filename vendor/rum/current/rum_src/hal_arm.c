/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/*
  $Id: hal_arm.c,v 1.2.4.1 2009/06/05 23:26:38 mvidales Exp $
*/

#include <string.h>
#include "hal_arm.h"
#include "at86rf23x_registermap.h"
#include "rum_types.h"
#include "system.h"
#include "arm_timer.h"  // Some register definitions here..

/**
   @defgroup arm ARM-specific platform documentation
   @{

   This section contains all of documentation of the ARM-specific
   parts of RUM.  Many of these functions duplicate the functionality
   of equivalently-named functions written for the AVR controllers.

   The SAM7X (ARM) version of RUM only implements the coordinator
   node.  It uses uTasker RTOS - See www.utasker.com for more information.
*/

/*============================ MACROS & DEFINES =============================*/

/** @name Macros defined for the radio transceiver's access modes.
 *
 * These functions are implemented as macros since they are used very often and
 * we want to remove the function call overhead.
 *
 * @{
 */
#define HAL_DUMMY_READ         (0x00) //!< Dummy value for the SPI.

#define HAL_TRX_CMD_RW         (0xC0) //!< Register Write (short mode).
#define HAL_TRX_CMD_RR         (0x80) //!< Register Read (short mode).
#define HAL_TRX_CMD_FW         (0x60) //!< Frame Transmit Mode (long mode).
#define HAL_TRX_CMD_FR         (0x20) //!< Frame Receive Mode (long mode).
#define HAL_TRX_CMD_SW         (0x40) //!< SRAM Write.
#define HAL_TRX_CMD_SR         (0x00) //!< SRAM Read.
#define HAL_TRX_CMD_RADDRM     (0x7F) //!< Register Address Mask.
/** @} */
#define HAL_CALCULATED_CRC_OK   (0) //!< CRC calculated over the frame including the CRC field should be 0.

extern u8 rx_mode;
extern u8 rssi_val;


/*============================ PROTOTYPES ===================================*/

void macEdCallback(void);
static void radioIsrHandler(void);

/*============================ IMPLEMENTATION ===============================*/

/**
    @brief  The SAM7X main clock, in uTasker, is running at 48MHz. This
            function provides a low precision way of spinning in a delay
            loop.

    @param  (us) The number of us to spend spinning in the loop.
*/
void delay_us(int us)
{
    volatile int delay;
    delay = ((48*us)/16);
    while (delay--) {};
}

/**
   @name ARM HAL Functions
   @{
*/
/**
 *   @brief     ARM manual SPI function to read/write from SPI1 but does not
 *              pull the SPI Select line - software must pull the SPI1 select
 *              line separately.
 *
 *              This function is relocated into SRAM.
 *
 */
u8 spiReadWrite(u8 val)
{
    /* Transmission is started by writing the transmit data. */
    SPI_TDR_1 = val;

    /* Wait for receiver data register full. */
    while((SPI_SR_1 & SPI_RDRF) == 0);

    /* Read data. */
    val = SPI_RDR_1;

    return val;
}

/**
 *  @brief      Initialize ARM SPI1. SPI Select line is under "manual" control
 *              and not controlled by the peripheral (SPI) device.
 *
 */
volatile u32 debug_spi;
int spiInit(void)
{
    // Before anything else, reset the SPI.
    SPI_CR_1 = SPI_SWRST;

    // Enable either A or B peripheral of PIOA.
    PIO_BSR_A = SPI_RF_PINS;

    // Disable PIO control and Enable manual control for sel pin and peripheral
    // control for miso, mosi and sck.
    PIO_PDR_A = SPI_RF_PINS;

    // Enable manual control of the sel pin.
    PIO_PER_A = SPI_CS0_1B;

    // Set SEL line as an output pin.
    PIO_OER_A = SPI_CS0_1B;

    // Set the line drive to high for CS.
    PIO_SODR_A = SPI_CS0_1B;

    // Enable SPI clock.
    PMC_PCER = (PMC_PCSR | SPI1);

    // SPI reset - AGAIN! Then enable!
    SPI_CR_1 = SPI_SWRST;
    SPI_CR_1 = SPIEN;

    // Set SPI to fixed peripheral at SPI0_CS2 or SPI1_CS0, fault detection
    // disabled, master mode.
    SPI_MR_1 = ((SPI_RF_CS_MR << SPI_PCS_LSB) | MODFDIS | SPI_MSTR);

    // Data is sampled on the falling edge of the SPCK.
    // SPI baud is empirically set to 6.5MHz.
    SPI_CSR0_1 = ((0x08 << SPI_SCBR_LSB) | SPI_8_BITS | SPI_NCPHA);

    return 0;
}

/** @brief      This function initializes the Hardware Abstraction Layer.
 *
 */
void hal_init(void)
{
    // IO Specific Initialization.

    //Enable SLP_TR as output.
    PIO_OER_A = SLPTRPIN;

    // Set the SLP_TR pin low.
    PIO_CODR_A = SLPTRPIN;

    //Enable RST as output.
    PIO_OER_A = RSTPIN;

    // Set reset high since the radio reset is active low.
    PIO_SODR_A = RSTPIN;

    // Set the IRQ0 (PA30) pin to disable the pull up resistor.
    //    PIO_RF_PUDR = IRQPIN;

    // Initialize the IRQ.
    // IRQ0 (PA30) is an external interrupt and it is not necessary to assign
    // the IO to the interrupt function as the PIO Controller has no effect on
    // inputs and the interrupt lines (FIQ or IRQ's) are used only as inputs.

    // Set ISR the vector. Cast to int to eliminate compiler warning.
    AIC_SVR30 = (u32)radioIsrHandler;

    // Initialize edge triggered with defined priority.
    AIC_SMR30 = 0x67;

    // Clear the ISR.
    AIC_ICCR = AIC_IRQ0;

    // Enable the IRQ
    AIC_IECR = AIC_IRQ0;

    // Set up for SPI manual only mode.
    spiInit();
}

/** @brief  This function reads data from one of the radio transceiver's registers.
 *          This function is relocated to SRAM.
 *
 *  @param  address Register address to read from. See datasheet for register
 *          map.
 *
 *  @see    Look at the at86rf23x_registermap.h file for register address
 *          definitions.
 *
 *  @returns The actual value of the read register.
 *
 */
u8 hal_register_read(u8 address)
{
    u8 dummy = 0;

    //Add the register read command to the register address.
    address &= HAL_TRX_CMD_RADDRM;
    address |= HAL_TRX_CMD_RR;

    // Send the address to read.
    fnInUserInterrupt(1); // entering;
    uDisable_Interrupt() ;

    // Pull SEL line.
    //PIO_CODR_A = _BV(SPI_RF_NPCS);
    PIO_CODR_A = SPI_CS0_1B;

    // Transmission is started by writing the transmit data.
    SPI_TDR_1 = address;

    // Wait for receiver data register full.
    while((SPI_SR_1 & SPI_RDRF) == 0);

    // Dummy read.
    dummy = SPI_RDR_1;

    // Read the data by sending a dummy byte.

    // Transmission is started by writing the transmit data.
    SPI_TDR_1 = HAL_DUMMY_READ;

    // Wait for receiver data register full.
    while((SPI_SR_1 & SPI_RDRF) == 0);

    // Release SEL line.
    PIO_SODR_A = SPI_CS0_1B;

    // Read data.
    address = SPI_RDR_1;

    uEnable_Interrupt();
    fnInUserInterrupt(0); // leaving

    return address;
}

/** @brief  This function writes a new value to one of the radio transceiver's
 *          registers. This function is relocated to SRAM.
 *
 *  @see Look at the at86rf23x_registermap.h file for register address definitions.
 *
 *  @param  address Address of register to write.
 *  @param  value   Value to write.
 *
 */
void hal_register_write(u8 address, u8 value)
{
    // Add the Register Write command to the address.
    address = HAL_TRX_CMD_RW | (HAL_TRX_CMD_RADDRM & address);

    // Send the address to write.
    fnInUserInterrupt(1); // entering;
    uDisable_Interrupt() ;

    // Pull SEL line.
    PIO_CODR_A = SPI_CS0_1B;

    // Transmission is started by writing the transmit data.
    SPI_TDR_1 = address;

    // Wait for receiver data register full.
    while((SPI_SR_1 & SPI_RDRF) == 0);

    // Dummy read.
    SPI_RDR_1;

    // Write the data to the address.

    // Transmission is started by writing the transmit data.
    SPI_TDR_1 = value;

    // Wait for receiver data register full.
    while((SPI_SR_1 & SPI_RDRF) == 0);

    // Release SEL line.
    PIO_SODR_A = SPI_CS0_1B;

    // Dummy read.
    SPI_RDR_1;

    uEnable_Interrupt();
    fnInUserInterrupt(0); // leaving

    if ((address & ~HAL_TRX_CMD_RW) == RG_TRX_STATE)
    {
        // set rx_mode flag based on mode we're changing to
        value &= 0x1f;   // Mask for TRX_STATE register
        if (value == RX_ON ||
            value == RX_AACK_ON)
            rx_mode = 1;
        else
            rx_mode = 0;
    }
    // Return data not necessary...
}

/** @brief  This function reads the value of a specific subregister.
 *          This function is relocated to SRAM.
 *
 *  @see Look at the at86rf23x_registermap.h file for register and subregister
 *       definitions.
 *
 *  @param  address  Main register's address.
 *  @param  mask  Bit mask of the subregister.
 *  @param  position   Bit position of the subregister
 *  @retval Value of the read subregister.
 *
 */
u8 hal_subregister_read(u8 address, u8 mask, u8 position)
{
    //Read current register value and mask out subregister.
    u8 register_value = hal_register_read(address);
    register_value &= mask;
    register_value >>= position; //Align subregister value.

    return register_value;
}

/** @brief  This function writes a new value to one of the radio transceiver's
 *          subregisters. This function is relocated to SRAM.
 *
 *  @see Look at the at86rf23x_registermap.h file for register and subregister
 *       definitions.
 *
 *  @param  address  Main register's address.
 *  @param  mask  Bit mask of the subregister.
 *  @param  position  Bit position of the subregister
 *  @param  value  Value to write into the subregister.
 *
 */
void hal_subregister_write(u8 address, u8 mask, u8 position,
                            u8 value)
{
    //Read current register value and mask area outside the subregister.
    u8 register_value = hal_register_read(address);
    register_value &= ~mask;

    //Start preparing the new subregister value. shift in place and mask.
    value <<= position;
    value &= mask;

    value |= register_value; //Set the new subregister value.

    //Write the modified register value.
    hal_register_write(address, value);
}

/** @brief  This function will upload a frame from the radio transceiver's frame
 *          buffer. This function is relocated to SRAM.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *
 */
void hal_frame_read(void)
{
    u8 len;
    u8 frame_length;
    u8 tempData;
    u8 *rx_data=mac_buffer_rx;
    rx_frame_t *rx_frame=(rx_frame_t*)mac_buffer_rx;

    fnInUserInterrupt(1); // entering;
    uDisable_Interrupt();

    // Pull SEL line.
    PIO_CODR_A = SPI_CS0_1B;

    // Send frame read command.
    tempData = spiReadWrite(HAL_TRX_CMD_FR);

    // Read frame length.
    frame_length = spiReadWrite(HAL_DUMMY_READ);

    len = frame_length;

    // Check for correct frame length.
    if ((frame_length >= HAL_MIN_FRAME_LENGTH) && (frame_length <= HAL_MAX_FRAME_LENGTH))
    {
        *rx_data++ = frame_length; //Store frame length.
        do
        {
            tempData = spiReadWrite(HAL_DUMMY_READ); // dummy write
            *rx_data++ = tempData;

        } while (--frame_length > 0);

        // Read LQI value for this frame.
        rx_frame->lqi = spiReadWrite(HAL_DUMMY_READ);

        // Release SEL line.
        PIO_SODR_A = SPI_CS0_1B;
    }
    else
    {
        // Release SEL line.
        PIO_SODR_A = SPI_CS0_1B;

        if (rx_frame)
        {
            rx_frame->length = 0;
            rx_frame->lqi    = 0;
            rx_frame->crc    = 0;
        }
    }

    // A dummy read to ensure SPI1 RDRF flag is reset.
    SPI_RDR_1;

    uEnable_Interrupt();
    fnInUserInterrupt(0); // leaving
}

/** @brief  This function will download a frame to the radio transceiver's frame
 *          buffer. This function is relocated to SRAM.
 *
 *  @param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  @param  length          Length of data. The maximum length is 127 bytes.
 *
 */
void hal_frame_write(u8 *write_buffer, u8 length)
{
    length &= HAL_TRX_CMD_RADDRM; //Truncate length to maximum frame length.

    fnInUserInterrupt(1); // entering;
    uDisable_Interrupt() ;

    //Toggle the SLP_TR pin to initiate the frame transmission.
    hal_set_slptr_high();
    hal_set_slptr_low();

    // Pull SEL line.
    PIO_CODR_A = SPI_CS0_1B;

    // SEND FRAME WRITE COMMAND AND FRAME LENGTH.
    spiReadWrite(HAL_TRX_CMD_FW);
    spiReadWrite(length);

    //Download to the Frame Buffer.
    do
    {
        spiReadWrite(*write_buffer++);
        --length;

    } while (length > 0);

     // Release SEL line.
     PIO_SODR_A = SPI_CS0_1B;

    // Dummy read.
    SPI_RDR_1;

    uEnable_Interrupt();
    fnInUserInterrupt(0); // leaving
}

/**
 *  @brief  This is the radio ISR handler.
 *
 *          The dedicated ARM IRQ0 line is used for this interrupt. It is set
 *          for a positive edge triggered event with priority 7 - The highest
 *          priority.
 *
 *          This function is relocated to SRAM.
 *
 *          The NUT OS is notified via the "NutEventPostFromIrq" which is the
 *          only NUT OS supported OS call to be made from an IRQ.
 *
 */

u8 frame_length;
u8 interrupt_source;
static void radioIsrHandler(void)
{
    frame_length = 0;
    interrupt_source=0;

    interrupt_source = hal_register_read(RG_IRQ_STATUS);

    if (interrupt_source & HAL_TRX_END_MASK)
    {
        // Process the event.
        radioTrxEndEvent();
    }

    // Energy detect event
    if (interrupt_source & HAL_ED_READY_MASK)
        macEdCallback();

    // Handle the incomming interrupt. Prioritized.
    if ((interrupt_source & HAL_RX_START_MASK))
    {
        // Read Frame length and call rx_start callback.
        // Pull SEL line.
        PIO_CODR_A = SPI_CS0_1B;

        // Send frame read command.
        spiReadWrite(HAL_TRX_CMD_FR);

        // Read frame length.
        frame_length = spiReadWrite(HAL_DUMMY_READ);

        // Release SEL line.
        PIO_SODR_A = SPI_CS0_1B;

        radioRxStartEvent(frame_length);
    }
    else if (interrupt_source & HAL_TRX_UR_MASK)
        ;
    else if (interrupt_source & HAL_PLL_UNLOCK_MASK)
        ;
    else if (interrupt_source & HAL_PLL_LOCK_MASK)
        ;
    else if (interrupt_source & HAL_BAT_LOW_MASK)
    {
        //Disable BAT_LOW interrupt to prevent interrupt storm. The interrupt
        //will continously be signaled when the supply voltage is less than the
        //user defined voltage threshold.
        u8 trx_isr_mask = hal_register_read(RG_IRQ_MASK);
        trx_isr_mask &= ~HAL_BAT_LOW_MASK;
        hal_register_write(RG_IRQ_MASK, trx_isr_mask);
    }
    else
        ; // unknown ISR
}

/**
    @brief Need to way to set the MAC address of the ARM based PAN Coord.
    Should be stored in NVM somewhere. This is a simple example to load MAC
    address.

    @param p Buffer to copy MAC address to.
*/
void halGetMacAddr(u8 *p)
{
    u8 pa[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
    memcpy(p, pa, 8);
}

/** @} */
/** @} */

