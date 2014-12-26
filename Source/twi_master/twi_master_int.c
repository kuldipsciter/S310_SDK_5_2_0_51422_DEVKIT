/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "twi_master_int.h"
#if(TWI_USE_SOC_API == 1)
#include "ble.h"
#include "nrf_soc.h"
#endif
#include "nrf_delay.h"
#include "nrf_gpio.h"

/* Max cycles approximately to wait on RXDREADY and TXDREADY event, this is optimum way instead of using timers, this is not power aware, negetive side is this is not power aware */
#define MAX_TIMEOUT_LOOPS             (10000UL)        /*!< MAX while loops to wait for RXD/TXD event */

static uint8_t twi_pinselect_scl, twi_pinselect_sda; 
volatile uint8_t tx_bytes_to_send, rx_bytes_to_receive;
volatile uint8_t *tx_data_ptr, *rx_data_ptr;
volatile bool twi_operation_complete, twi_ack_received, twi_blocking_enabled;

void TWI_INTERRUPT()
{
    if(TWI->EVENTS_TXDSENT)
    {
        TWI->EVENTS_TXDSENT = 0;
        if(tx_bytes_to_send)
        {
            TWI->TXD = *tx_data_ptr++;  
            tx_bytes_to_send--;
        }
        else 
        {
            if(rx_bytes_to_receive == 0)
            {
                TWI->TASKS_STOP = 1; 
            }
            else
            {
#if(TWI_USE_SOC_API == 1)
                sd_ppi_channel_enable_set(1 << TWI_PPI_CH0);
#else
                NRF_PPI->CHENSET = (1 << TWI_PPI_CH0);
#endif
                TWI->TASKS_STARTRX = 1;
            }
        }
    }
    if(TWI->EVENTS_STOPPED)
    {
        TWI->EVENTS_STOPPED = 0;
        twi_operation_complete = true;
        twi_ack_received = true;
    }
    if(TWI->EVENTS_RXDREADY)
    {
        TWI->EVENTS_RXDREADY = 0;
        *rx_data_ptr++ = TWI->RXD;
#if(TWI_USE_SOC_API == 1)
        if (--rx_bytes_to_receive == 1)
        {
            sd_ppi_channel_assign(TWI_PPI_CH0, &TWI->EVENTS_BB, &TWI->TASKS_STOP);  
        }
#else
        if (--rx_bytes_to_receive == 1)
        {
            NRF_PPI->CH[TWI_PPI_CH0].TEP = (uint32_t)&TWI->TASKS_STOP;
        }
#endif
        if(rx_bytes_to_receive > 0)
        {
            TWI->TASKS_RESUME = 1;
        }
    }
    if(TWI->EVENTS_ERROR)
    {
        TWI->EVENTS_ERROR = 0;
        twi_operation_complete = true;
        twi_ack_received = false;
    }
}

bool twi_master_write(uint8_t address, uint8_t *tx_data, uint8_t tx_data_length)
{
    if (tx_data_length == 0)
    {    
        return false;
    }
		
    if(!twi_blocking_enabled)
    {
        // If an operation is already underway, delay operation
        while(twi_operation_complete == false);
    }

#if(TWI_USE_SOC_API == 1)
    sd_ppi_channel_enable_clr(1 << TWI_PPI_CH0);
#else   
    NRF_PPI->CHENCLR = (1 << TWI_PPI_CH0);
#endif
    
    TWI->ADDRESS = address;
    tx_data_ptr = tx_data;
    tx_bytes_to_send = tx_data_length - 1;
    rx_bytes_to_receive = 0;
    TWI->TXD = *tx_data_ptr++;
    TWI->TASKS_STARTTX = 1;
    twi_operation_complete = false;
    
    if(twi_blocking_enabled)
    {
        while(twi_operation_complete == false);
        return twi_ack_received;
    }
    return true;
}


bool twi_master_write_read(uint8_t address, uint8_t *tx_data, uint8_t tx_data_length, uint8_t *rx_data, uint8_t rx_data_length)
{
    if (tx_data_length == 0 || rx_data_length == 0)
    {    
        return false;
    }

    if(!twi_blocking_enabled)
    {
        // If an operation is already underway, delay operation
        while(twi_operation_complete == false);
    }
    
    TWI->ADDRESS = address;
    tx_data_ptr = tx_data;
    tx_bytes_to_send = tx_data_length - 1;
    rx_data_ptr = rx_data;
    rx_bytes_to_receive = rx_data_length;
       
#if(TWI_USE_SOC_API == 1)
    if (rx_bytes_to_receive == 1)
    {
        sd_ppi_channel_assign(TWI_PPI_CH0, &TWI->EVENTS_BB, &TWI->TASKS_STOP);
    }
    else
    {
        sd_ppi_channel_assign(TWI_PPI_CH0, &TWI->EVENTS_BB, &TWI->TASKS_SUSPEND);
    }
    sd_ppi_channel_enable_clr(1 << TWI_PPI_CH0);        
#else
    if (rx_bytes_to_receive == 1)
    {
        NRF_PPI->CH[TWI_PPI_CH0].TEP = (uint32_t)&TWI->TASKS_STOP;
    }
    else
    {
        NRF_PPI->CH[TWI_PPI_CH0].TEP = (uint32_t)&TWI->TASKS_SUSPEND;
    }
    NRF_PPI->CHENCLR = (1 << TWI_PPI_CH0);
#endif
    
    TWI->TXD = *tx_data_ptr++;
    TWI->TASKS_STARTTX = 1;
    twi_operation_complete = false;
    
    if(twi_blocking_enabled)
    {
        while(twi_operation_complete == false);
        return twi_ack_received;
    }
    return true;
}

bool twi_master_read(uint8_t address, uint8_t *rx_data, uint8_t rx_data_length)
{
    if(rx_data_length == 0)
    {
        return false;
    }

    if(!twi_blocking_enabled)
    {
        // If an operation is already underway, delay operation
        while(twi_operation_complete == false);
    }
    
    TWI->ADDRESS = address;
    rx_data_ptr = rx_data;
    rx_bytes_to_receive = rx_data_length;
    
#if(TWI_USE_SOC_API == 1)
    if (rx_bytes_to_receive == 1)
    {
        sd_ppi_channel_assign(TWI_PPI_CH0, &TWI->EVENTS_BB, &TWI->TASKS_STOP);
    }
    else
    {
        sd_ppi_channel_assign(TWI_PPI_CH0, &TWI->EVENTS_BB, &TWI->TASKS_SUSPEND);
    }
    sd_ppi_channel_enable_set(1 << TWI_PPI_CH0);
#else
    if (rx_bytes_to_receive == 1)
    {
        NRF_PPI->CH[TWI_PPI_CH0].TEP = (uint32_t)&TWI->TASKS_STOP;
    }
    else
    {
        NRF_PPI->CH[TWI_PPI_CH0].TEP = (uint32_t)&TWI->TASKS_SUSPEND;
    }
    NRF_PPI->CHENSET = (1 << TWI_PPI_CH0);
#endif
    
    TWI->TASKS_STARTRX = 1;
    twi_operation_complete = false;
    
    if(twi_blocking_enabled)
    {
        while(twi_operation_complete == false);
        return twi_ack_received;
    }
    return true;
}

/**
 * Detects stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi_master_clear_bus(void)
{
    bool bus_clear;

    TWI_SDA_HIGH();
    TWI_SCL_HIGH();
    TWI_DELAY();

    if (TWI_SDA_READ() == 1 && TWI_SCL_READ() == 1)
    {
        bus_clear = true;
    }
    else
    {
        uint_fast8_t i;
        bus_clear = false;

        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 for slave to respond) to SCL line and wait for SDA come high
        for (i=18; i--;)
        {
            TWI_SCL_LOW();
            TWI_DELAY();
            TWI_SCL_HIGH();
            TWI_DELAY();

            if (TWI_SDA_READ() == 1)
            {
                bus_clear = true;
                break;
            }
        }
    }
    return bus_clear;
}

bool twi_wait_for_completion(void)
{
    while(twi_operation_complete == false);
    return twi_ack_received;    
}

bool twi_master_init(twi_config_t *cfg)
{
    twi_operation_complete = true;
    twi_ack_received = true;
    
    twi_pinselect_scl = cfg->pinselect_scl;
    twi_pinselect_sda = cfg->pinselect_sda;
    
    twi_blocking_enabled = (cfg->blocking_mode == TWI_BLOCKING_ENABLED ? 1 : 0);
          
    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is 
       disabled, these pins must be configured in the GPIO peripheral.
    */ 
    NRF_GPIO->PIN_CNF[twi_pinselect_scl] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

    NRF_GPIO->PIN_CNF[twi_pinselect_sda] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos); 
    
    TWI->EVENTS_RXDREADY = 0;
    TWI->EVENTS_TXDSENT = 0;
    TWI->PSELSCL = twi_pinselect_scl;
    TWI->PSELSDA = twi_pinselect_sda;
    
    switch(cfg->frequency)
    {
        case TWI_FREQ_100KHZ:
            TWI->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K250 << TWI_FREQUENCY_FREQUENCY_Pos;
            break;
        case TWI_FREQ_400KHZ:
            TWI->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos;
            break;
    }
#if(TWI_USE_SOC_API == 1)
    sd_ppi_channel_assign(TWI_PPI_CH0, &TWI->EVENTS_BB, &TWI->TASKS_SUSPEND);
    sd_ppi_channel_enable_clr(1 << TWI_PPI_CH0);
    sd_nvic_SetPriority(TWI_INTERRUPT_NO, TWI_IRQ_PRIORITY_SD);
    sd_nvic_EnableIRQ(TWI_INTERRUPT_NO);
#else
    NRF_PPI->CH[TWI_PPI_CH0].EEP = (uint32_t)&TWI->EVENTS_BB;
    NRF_PPI->CH[TWI_PPI_CH0].TEP = (uint32_t)&TWI->TASKS_SUSPEND;
    NRF_PPI->CHENCLR = (1 << TWI_PPI_CH0);
    NVIC_SetPriority(TWI_INTERRUPT_NO, TWI_IRQ_PRIORITY_NO_SD);
    NVIC_EnableIRQ(TWI_INTERRUPT_NO);
#endif
    TWI->INTENSET = TWI_INTENSET_TXDSENT_Msk | TWI_INTENSET_STOPPED_Msk | TWI_INTENSET_ERROR_Msk | TWI_INTENSET_RXDREADY_Msk;
    TWI->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
    return twi_master_clear_bus();
}

/*
bool twi_master_transfer(uint8_t address, uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
    bool transfer_succeeded = true;
    if (data_length > 0 && twi_master_clear_bus())
    {
        NRF_TWI1->ADDRESS = (address >> 1);

        if ((address & TWI_READ_BIT) != 0)
        {
            transfer_succeeded = twi_master_read(data, data_length, issue_stop_condition);
        }
        else
        {
            transfer_succeeded = twi_master_write(data, data_length, issue_stop_condition);
        }
    }
    return transfer_succeeded;
}
*/
