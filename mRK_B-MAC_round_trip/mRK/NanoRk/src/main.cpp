#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <string.h>
#include "mbed.h"
#include "basic_rf.h"
#include "bmac.h"
#include <vector>

#define NODE3

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

nrk_task_type RPRINT_TASK;
NRK_STK rprint_task_stack[NRK_APP_STACKSIZE];

void nrk_create_taskset ();

char tx_buf[RF_MAX_PAYLOAD_SIZE];
char rx_buf[RF_MAX_PAYLOAD_SIZE];

int MOLE_APPEARANCES = 10;
int SCALE_BY = 50;

uint16_t node_buf[10] = {0x0052, 0x0051, 0x0051, 0x0052, 0x0053, 0x0051, 0x0052, 0x0053, 0x0051, 0x0052};
uint8_t time = 0x05; 

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

static uint16_t rxContentSuccess = 0;
static uint16_t mrfRxISRcallbackCounter = 0;
static uint16_t sentPackets = 0;
static uint8_t clearToTx = 1;

Serial pc(USBTX, USBRX);

// Count number of detected packets and toggle a pin for each packet
void mrfIsrCallback() {		
        mrfRxISRcallbackCounter++;
}

int main(void) {	
        nrk_setup_ports();
        nrk_init();
        bmac_task_config();
        nrk_create_taskset();
        nrk_start();
        return 0;
}

template<int C>
void rprint_task () {
        pc.printf("node %d initialized", C);
        while (1) {
                pc.printf("node %d, %d sent, %d successful\n", C, mrfRxISRcallbackCounter, rxContentSuccess);
                nrk_wait_until_next_period ();
        }
}

void master_rx_task () {
        uint8_t i, len, rssi;
        int8_t val;
        char *local_rx_buf;
        nrk_time_t check_period;
        // init bmac
        bmac_init(RADIO_CHANNEL);
        bmac_auto_ack_disable();
        check_period.secs=0;
        check_period.nano_secs=20*NANOS_PER_MS;
        val=bmac_set_rx_check_rate(check_period);
        bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 
        bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

        while (1) {
                nrk_led_toggle (RED_LED);
                // Wait until an RX packet is received
                val = bmac_wait_until_rx_pkt();
                // Get the RX packet 
                local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
                rxContentSuccess++;
                int time_left = (rx_buf[0] << 24) | (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
                calcScore(time_left); // TODO(iantay)
                // Change something to make sure the buffer isn't the same if no buffer fill from Rx occurs
                rx_buf[1] = '0';
                // Release the RX buffer so future packets can arrive 
                bmac_rx_pkt_release ();
                // this is necessary
                nrk_wait_until_next_period ();
        }
}

void slave_rx_task () {
        uint8_t i, len, rssi;
        int8_t val;
        char *local_rx_buf;
        nrk_time_t check_period;
        // init bmac
        bmac_init(RADIO_CHANNEL);
        bmac_auto_ack_disable();
        check_period.secs=0;
        check_period.nano_secs=20*NANOS_PER_MS;
        val=bmac_set_rx_check_rate(check_period);
        bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 
        bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

        while (1) {
                nrk_led_toggle (RED_LED);
                // Wait until an RX packet is received
                val = bmac_wait_until_rx_pkt ();
                // Get the RX packet 
                local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
                rxContentSuccess++;
                int time = rx_buf[0]
                // TODO(iantay) set timer
                // Change something to make sure the buffer isn't the same if no buffer fill from Rx occurs
                rx_buf[1] = '0';
                // Release the RX buffer so future packets can arrive 
                bmac_rx_pkt_release ();
                // this is necessary
                nrk_wait_until_next_period ();
        }
}

void slave_tx_task () {
        // Wait until the rx_task starts up bmac
        // This should be called by all tasks using bmac that
        // do not call bmac_init()...
        while (!bmac_started ()) {
                nrk_wait_until_next_period ();
        }

        if (!mole_active) { // TODO(iantay) add this var
                nrk_wait_until_next_period ();
        }
        while (1) {	
                // if Photodiode covered, check timer
                // take curr_time - timer_time
                // transmit
                // reset timer and stop timer
                bmac_addr_decode_dest_mac(0x0050);  // 0xFFFF is broadcast
                strncpy(tx_buf, &time, 4);
                val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
                while (val==NRK_ERROR){
                        wait_ms(50); //TODO(iantay)
                        val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
                }
                nrk_wait_until_next_period();
        }
}
void master_tx_task () {
        // Wait until the rx_task starts up bmac
        // This should be called by all tasks using bmac that
        // do not call bmac_init()...
        while (!bmac_started ()) {
                nrk_wait_until_next_period ();
        }

        int to_send = 0; // stop if to_send == MOLE_APPEARANCES
        bmac_addr_decode_set_my_mac(0x0050); 
        bmac_auto_ack_enable();

        if (!clearToTx) {
                nrk_wait_until_next_period ();
        }
        while (to_send < MOLE_APPEARANCES) {
                while (1) {	
                        clearToTx = 0;
                        bmac_addr_decode_dest_mac(node_buf[++to_send]);  // 0xFFFF is broadcast
                        // For blocking transmits, use the following function call.
                        // For this there is no need to register
                        strncpy(tx_buf, &time, 1);
                        val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
                        while (val==NRK_ERROR){
                                wait_ms(50); //TODO(iantay)
                                val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
                        }
                        sentPackets++;
                        nrk_wait_until_next_period();
                }
        }
}

void nrk_create_taskset () {
        #ifdef NODE0
        RX_TASK.task = master_rx_task;
        #endif
        #if defined(NODE2) || defined(NODE3) || defined(NODE1)
        RX_TASK.task = slave_rx_task;
        #endif
        nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
        RX_TASK.prio = 2;
        RX_TASK.FirstActivation = TRUE;
        RX_TASK.Type = BASIC_TASK;
        RX_TASK.SchType = PREEMPTIVE;
        RX_TASK.period.secs = 0;
        RX_TASK.period.nano_secs = 200*NANOS_PER_MS;
        RX_TASK.cpu_reserve.secs = 0;
        RX_TASK.cpu_reserve.nano_secs = 0;
        RX_TASK.offset.secs = 0;
        RX_TASK.offset.nano_secs = 0;
        nrk_activate_task (&RX_TASK);

        // RPRINT only for master
        RPRINT_TASK.task = rprint_task<0>;
        nrk_task_set_stk( &RPRINT_TASK, rprint_task_stack, NRK_APP_STACKSIZE);
        RPRINT_TASK.prio = 2;
        RPRINT_TASK.FirstActivation = TRUE;
        RPRINT_TASK.Type = BASIC_TASK;
        RPRINT_TASK.SchType = PREEMPTIVE;
        RPRINT_TASK.period.secs = 0;
        RPRINT_TASK.period.nano_secs = 2000*NANOS_PER_MS;
        RPRINT_TASK.cpu_reserve.secs = 0;
        RPRINT_TASK.cpu_reserve.nano_secs = 0;
        RPRINT_TASK.offset.secs = 0;
        RPRINT_TASK.offset.nano_secs = 0;
        #ifdef NODE0
        nrk_activate_task (&RPRINT_TASK);
        #endif

        #ifdef NODE0
        TX_TASK.task = master_tx_task;
        #endif
        #if defined(NODE2) || defined(NODE3)
        TX_TASK.task = slave_tx_task;
        #endif
        nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
        TX_TASK.prio = 2;
        TX_TASK.FirstActivation = TRUE;
        TX_TASK.Type = BASIC_TASK;
        TX_TASK.SchType = PREEMPTIVE;
        TX_TASK.period.secs = 0;
        TX_TASK.period.nano_secs = 200*NANOS_PER_MS;
        TX_TASK.cpu_reserve.secs = 0;
        TX_TASK.cpu_reserve.nano_secs = 0;
        TX_TASK.offset.secs = 0;
        TX_TASK.offset.nano_secs = 0;
        #ifdef NODE0
        nrk_activate_task (&TX_TASK);
        #endif
}

