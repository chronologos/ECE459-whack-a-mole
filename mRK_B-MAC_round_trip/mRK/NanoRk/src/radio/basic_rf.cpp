/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Paul M. Gurniak
*******************************************************************************/

#include "basic_rf.h"
#include "nrk_timer.h"
#include "bmac.h"

#define MRF_FSM_RESET()     do{ mrf_write_short(RFCTL, 0x04); mrf_write_short(RFCTL, 0x00); nrk_spin_wait_us(200);} while(0)

extern void mrfIsrCallback();

/**
 *  This is a callback if you require immediate response to a packet
 */
RF_RX_INFO *rf_rx_callback (RF_RX_INFO * pRRI)
{
    // Any code here gets called the instant a packet is received from the interrupt
	mrfIsrCallback();
  return pRRI;
}

/*******************************************************************************************************
 *******************************************************************************************************
 **************************                  Initialization                   **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//-------------------------------------------------------------------------------------------------------
// The RF settings structure:
volatile RF_SETTINGS rfSettings;
volatile uint8_t rx_ready;
volatile uint8_t auto_ack_enable, security_enable;

//-------------------------------------------------------------------------------------------------------

void rf_power_down(void)
{
    mrf_write_short(SOFTRST, 0x04);      // Reset power management circuitry
    mrf_write_short(SLPACK, 0x80);      // Set into sleep mode
}

void rf_power_up(void)
{
    // Cycle REGWAKE bit 6 (remember to leave IMMWAKE set)
    mrf_write_short(WAKECON, 0xC0);           
    mrf_write_short(WAKECON, 0x80);
    nrk_spin_wait_us(2*1000);     // 2ms to stabilize main oscillator
}

void rf_tx_power(uint8_t pwr)
{   
    // TODO: Paul Gurniak, check this power conversion
    mrf_write_long(RFCON3, (31 - pwr & 0x1F) << 3);
}

uint8_t rf_security_last_pkt_status(void)
{
    // TODO: Paul Gurniak
    return 0;
}

void rf_security_set_ctr_counter(uint8_t *counter)
{
    // TODO: Paul Gurniak
}

void rf_security_set_key(uint8_t *key)
{
    // TODO: Paul Gurniak
}

void rf_security_enable()
{
    // TODO: Paul Gurniak
}

void rf_security_disable(void)
{
    // TODO: Paul Gurniak
}

void rf_set_channel(uint8_t channel)
{
  //// mrf_write_long(RFCON0, (channel & 0x0F) | 0x03);   // Set channel, leave RFOPT bits at recommended
		//mrf_write_long(RFCON0, (0xC0 | 0x03));   // Set channel, leave RFOPT bits at recommended
	  mrf_write_long(RFCON0, ((channel-11 << 4) | 0x03));   // Vuk
	
    MRF_FSM_RESET();
}


void rf_addr_decode_enable(void) // Vuk: This function doesn't work right and should be used. Use rf_set_rx_mode()
{
    uint8_t rxmcr = mrf_read_short(RXMCR);
    mrf_write_short(RXMCR, rxmcr | (1 << 0));   // Set to ignore address field
}


void rf_addr_decode_disable(void) // Vuk: This function doesn't work right and should be used. Use rf_set_rx_mode()
{
    uint8_t rxmcr = mrf_read_short(RXMCR);
    mrf_write_short(RXMCR, rxmcr & ~(1 << 0));    // Set to use address field
}


void rf_auto_ack_enable()
{
    uint8_t rxmcr = mrf_read_short(RXMCR);
    mrf_write_short(RXMCR, rxmcr & ~(1 << 5));       // Enable auto-ack
    auto_ack_enable = 1;
}


void rf_auto_ack_disable()
{
    uint8_t rxmcr = mrf_read_short(RXMCR);
    mrf_write_short(RXMCR, rxmcr | (1 << 5));       // Disable auto-ack
    auto_ack_enable = 0;
}


void rf_addr_decode_set_my_mac(uint16_t my_mac)
{
    // Program address
    mrf_write_short(SADRL, my_mac & 0xFF);
    mrf_write_short(SADRH, my_mac >> 8);
    rfSettings.myAddr = my_mac;
}




/*******************************************************************************************************
 *******************************************************************************************************
 **************************                 Packet reception                  **************************
 *******************************************************************************************************
 *******************************************************************************************************/


// Vuk: Function sets reception mode
// If rxMode == 0 : NORMAL MODE (packets with valid CRC and 802.15.4 compliance check are accepted)
// If rxMode == 1 : PROMISCUOUS MODE (only CRC valid packets are accepted)
// If rxMode == 2 : ERROR MODE (all packets are accepted)
void rf_set_rx_mode(uint8_t rxMode)
{
	uint8_t newRXMCR = mrf_read_short(RXMCR) & rxMode;
	mrf_write_short(RXMCR, newRXMCR);
}

void rf_set_rx(RF_RX_INFO *pRRI, uint8_t channel)
{
    rf_flush_rx_fifo();
    rf_set_channel(channel);
    rfSettings.pRxInfo = pRRI;
		MRF_FSM_RESET();
}


void rf_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr)
{
    mrf_init(channel);
    // rf_set_channel(channel); // Vuk: this is set in radio initialization
    mrf_set_addr(myAddr);
    mrf_set_panid(panId);
    
    rfSettings.myAddr = myAddr;
    rfSettings.panId = panId;
    rfSettings.txSeqNumber = 1;
    rfSettings.pRxInfo = pRRI;
    
	
  // rf_tx_power(20); // Vuk: this is set in radio initialization
	rf_set_rx_mode(2); // Vuk: 2 for ERROR reception mode

	
	// rf_auto_ack_enable(); // Vuk: No need to do this
    auto_ack_enable = 0;
    security_enable = 0;
    
	
	// Vuk: Flush FIFO buffer and clear interrupts just as an extra measure
    mrf_read_long(RXFIFO);
		mrf_read_short(INTSTAT);
	
    NVIC_EnableIRQ(EINT3_IRQn);             // Enable the EXT3 interrupt
    
    // TODO: Paul Gurniak, check that our init settings match expected
}


void rf_rx_on(void)
{
    //mrf_power_on();
    //rf_flush_rx_fifo();
    mrf_write_short(BBREG1, 0x00);
    rfSettings.receiveOn = true;
    rx_ready = 1;
}

void rf_polling_rx_on(void)
{
    mrf_write_short(BBREG1, 0x00);
    rf_flush_rx_fifo();
    rfSettings.receiveOn = true;
    rx_ready = 1;
    // TODO: Paul Gurniak, not sure if there is anything different between these?
}

void rf_rx_off(void)
{
    //mrf_power_off();
		
    mrf_write_short(BBREG1, 0x04);
    rfSettings.receiveOn  = false;
    rx_ready = 0;
}


void rf_flush_rx_fifo(void)
{
    // Reading the first byte resets the RX FIFO
    mrf_read_long(RXFIFO);
}

void rf_test_mode(void)
{
    mrf_write_long(0x22F, 0x08 | 5);    // Put into single-tone test mode
}

void rf_data_mode(void)
{
		mrf_write_long(0x22F, 0x08);        // Put back into default mode
}

void rf_rx_set_serial(void)
{
    // TODO: Paul Gurniak, not sure we can do this with our radio?
}

void rf_tx_set_serial(void)
{
    // TODO: Paul Gurniak, same as above
}

void rf_set_preamble_length(uint8_t length)
{
    // TODO: Paul Gurniak, pretty sure this is stuck at 3 symbol periods for our radio?
}

void rf_carrier_on(void)
{
    // Force RF FSM to Tx state to give channel activity
    mrf_write_short(RFCTL, 0x06);
    nrk_spin_wait_us(192);
    mrf_write_short(RFCTL, 0x02);
    nrk_spin_wait_us(192);
}

void rf_carrier_off(void)
{
    // Reset RF FSM to normal operation
    mrf_write_short(RFCTL, 0x04);
    nrk_spin_wait_us(192);
    mrf_write_short(RFCTL, 0x00);
    nrk_spin_wait_us(192);
}

/*******************************************************************************************************
 *******************************************************************************************************
 **************************          Packet Reception & Transmission          **************************
 *******************************************************************************************************
 *******************************************************************************************************/

volatile uint8_t rx_data_ready, tx_status_ready;

uint8_t rf_rx_check_fifop()
{
    // This function is deprecated and should not be used
    return rx_data_ready;
}

uint8_t rssi_cca_thold;

void rf_set_cca_thresh(int8_t t)
{
    // Input is in -dBm
    // MRF24J40 RSSI is an integer, 255 ~= -40dBm, 0 ~= -90dBm
		if(t >= -40) {
			rssi_cca_thold = 255;
		}
		else if(t <= -90) {
			rssi_cca_thold = 0;
		}
		else {
			rssi_cca_thold = (uint8_t)(((90 - (int32_t)(-1*t))*255)/50);
		}
}

void rf_set_cca_mode(uint8_t mode)
{
    uint8_t tmp = mrf_read_short(BBREG2);
    tmp = (tmp & 0x3F) | ((mode & 0x3) << 6);
    mrf_write_short(BBREG2, tmp);
}

uint8_t rf_rx_check_cca()
{
		// Should be 1 when channel is clear
		return (mrf_rssi() < rssi_cca_thold);
}

int8_t rf_rx_packet(void)
{
    int8_t tmp;
    if(rx_data_ready > 0) {
        tmp = rx_data_ready;
        rx_data_ready = 0;
        return tmp;
    }   
    return 0;
}

int8_t rf_polling_rx_packet(void)
{
	// TODO: Paul Gurniak, not sure what this function originally did differently?
	return rf_rx_packet();
}

uint8_t rf_tx_tdma_packet(RF_TX_INFO *pRTI, uint16_t slot_start_time, uint16_t tx_guard_time)
{
    // TODO (later): Paul Gurniak
    return 0;
}

extern RF_RX_INFO rfRxInfo;
uint8_t rf_tx_packet(RF_TX_INFO *pRTI)
{
    uint16_t frameControlField;
    uint8_t packetLength;
    uint8_t success, i;
    // Note: checksum is automatic in HW
	
		LPC_GPIO1->FIOPIN ^= 1<<31;
    
    packetLength = pRTI->length + RF_PACKET_OVERHEAD_SIZE;
    
    frameControlField = RF_FCF_NOACK;
    if(auto_ack_enable || pRTI->ackRequest) frameControlField |= RF_ACK_BM;
    if(security_enable) frameControlField |= 0x0000; // TODO: Paul Gurniak, add security
	
    mrf_write_long(TXNFIFO, RF_PACKET_OVERHEAD_SIZE);    // No checksum, overhead is all header
    mrf_write_long(TXNFIFO+1, packetLength);
    
    // Write header bytes
    mrf_write_long(TXNFIFO+2, frameControlField & 0xFF);
    mrf_write_long(TXNFIFO+3, frameControlField >> 8);
    mrf_write_long(TXNFIFO+4, rfSettings.txSeqNumber);
    mrf_write_long(TXNFIFO+5, rfSettings.panId & 0xFF);
    mrf_write_long(TXNFIFO+6, rfSettings.panId >> 8);
    mrf_write_long(TXNFIFO+7, pRTI->destAddr & 0xFF);
    mrf_write_long(TXNFIFO+8, pRTI->destAddr >> 8);
    mrf_write_long(TXNFIFO+9, rfSettings.myAddr & 0xFF);
    mrf_write_long(TXNFIFO+10, rfSettings.myAddr >> 8);
    
    // Write payload
    for(i = 0; i < pRTI->length; i++) {
        mrf_write_long(TXNFIFO+11+i, pRTI->pPayload[i]); // pPayload is the user defined part of data packet right ? // YES
    }
    
    if(pRTI->cca)
			{
        uint8_t cnt = 0;
        if(!rfSettings.receiveOn) {
            rf_rx_on();
        }
				while(!rf_rx_check_cca()) {
					cnt++;
					if(cnt > 100) {
						return FALSE;
					}
					// halWait(100); // Vuk: This function is not implemented, substituted with spin_wait that works
					nrk_spin_wait_us(100);
				}
    }
    
    tx_status_ready = 0;
		
		if(auto_ack_enable)
			mrf_write_short(BBREG1, 0x00); // Vuk: RXDECINV has to be low in order for the transmitter to be able to decode the acknowledgement packet off air
    mrf_write_short(TXNCON, auto_ack_enable ? 0x05 : 0x01);  // Send contents of TXNFIFO as packet

    // Wait for Tx to finish (max 30ms)
    uint16_t cnt = 0;
		while(!tx_status_ready)
		{
			cnt++;
			nrk_spin_wait_us(100);
			if(cnt>300)
			{					
				NVIC_DisableIRQ(EINT3_IRQn);
								
				// Vuk: If no Tx interrupt occured, reset radio
				uint8_t autoAckEnabled = auto_ack_enable; // Vuk: Make sure the acknowledge setting is maintained
				rf_init(&bmac_rfRxInfo, RADIO_CHANNEL, 0xFFFF, MAC_ADDR); //RX struct, channel, panId, myAddr
				
				if (autoAckEnabled)
					rf_auto_ack_enable();
				else
					rf_auto_ack_disable();

				//mrf_write_short(RXFLUSH, 0x01);
				//mrf_read_short(INTSTAT);
				nrk_led_toggle(RED_LED);

				return 0;
			}
		}
    
    if(auto_ack_enable)
			mrf_write_short(BBREG1, 0x04); // Vuk: To keep things consistent, disable off-air packet decoding during Tx function
		
    success = 1;
    if(auto_ack_enable || pRTI->ackRequest) {
        success = !(mrf_read_short(TXSTAT) & 0x01);
    }
    if (success)
			nrk_led_toggle(GREEN_LED);
    
		//printf("tx_pkt success = %d\r\n",success);
		// Transmission works fine : Checked by Madhur, success = 1only when the receiver is enabled
		
    // Increment sequence, return result
    rfSettings.txSeqNumber++;
    return success;
}

void rf_parse_rx_packet(void)
{
    uint16_t frameControlField;
    uint8_t length, i;
    
    // Disable Rx while parsing packet
    rf_rx_off();
    
    length = mrf_read_long(RXFIFO);
		//printf("\nrx_length=%d\r\n",length);
    
    if((length - RF_PACKET_OVERHEAD_SIZE - 2) <= rfSettings.pRxInfo->max_length)
			{
        // Note: 2 bytes for footer are included in Rx length
        rfSettings.pRxInfo->length = length - RF_PACKET_OVERHEAD_SIZE - 2;
        frameControlField = mrf_read_long(RXFIFO+1);
        frameControlField |= (mrf_read_long(RXFIFO+2) << 8);
        rfSettings.pRxInfo->ackRequest = !!(frameControlField & RF_FCF_ACK_BM);
        rfSettings.pRxInfo->seqNumber = mrf_read_long(RXFIFO+3);
        rfSettings.pRxInfo->srcAddr = mrf_read_long(RXFIFO+8);
        rfSettings.pRxInfo->srcAddr |= (mrf_read_long(RXFIFO+9) << 8);
    
        for(i = 0; i < rfSettings.pRxInfo->length; i++) {
            rfSettings.pRxInfo->pPayload[i] = mrf_read_long(RXFIFO+10+i);
        }
    
        rfSettings.pRxInfo->rssi = mrf_read_long(RXFIFO+1+length+1);
        rx_data_ready = 1;
    }
    
    // Re-enable Rx when done parsing
    rf_rx_on();
    
}

extern "C" void EINT3_IRQHandler(void)
{
	uint8_t flags;

	//LPC_GPIO1->FIOPIN ^= 1<<31;

	
	LPC_GPIOINT->IO2IntClr |= (1 << 4);     // Clear mbed interrupt flag
	
	flags = mrf_read_short(INTSTAT);        // Read radio interrupt flags

		
	if(flags & 0x01)
	{
		if (LPC_GPIO0->FIOPIN & 1 << 10)
			LPC_GPIO0->FIOCLR |= 1 << 10;
		else
			LPC_GPIO0->FIOSET |= 1 << 10;
		
		tx_status_ready = 1;
	}
	if(flags & 0x08)
	{
			rf_parse_rx_packet();
			rfSettings.pRxInfo = rf_rx_callback(rfSettings.pRxInfo);
		
		// Vuk: Just in case (MRF Silicon Errata)
    mrf_write_short(RXFLUSH, 0x01);
	}
}

