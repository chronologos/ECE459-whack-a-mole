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
void master_rx_task (void);
void slave_rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void master_tx_task (void);
void slave_tx_task (void);


//nrk_task_type RPRINT_TASK;
//NRK_STK rprint_task_stack[NRK_APP_STACKSIZE];

void nrk_create_taskset ();

char tx_buf[RF_MAX_PAYLOAD_SIZE];
char rx_buf[RF_MAX_PAYLOAD_SIZE];

int MOLE_APPEARANCES = 10;
int SCALE_BY = 50;

uint16_t node_buf[10] = {0x0052, 0x0051, 0x0051, 0x0052, 0x0053, 0x0051, 0x0052, 0x0053, 0x0051, 0x0052};
char master_period_in_s = 0x05; 
int8_t slave_period_in_s;
int master_to_send = 0; // stop if to_send == MOLE_APPEARANCES
int master_waiting_for_slave = 0;
int cumscore = 0; // only meaningful for master
int mole_active = 0; // only meaningful for slaves
Timer slave_mole_timer;
 
RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

static uint16_t rxContentSuccess = 0;
static uint16_t mrfRxISRcallbackCounter = 0;
static uint16_t sentPackets = 0;
static uint8_t clearToTx = 1;

Serial pc(USBTX, USBRX);
DigitalIn photodiode(p30);

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

int calcScore(int max_period, int time_elapsed){
	return max_period-time_elapsed;
}
void master_rx_task () {
        uint8_t i, len, rssi;
        int8_t val;
        char *local_rx_buf;
        nrk_time_t check_period;
        // init bmac
        bmac_init(RADIO_CHANNEL);
        bmac_auto_ack_enable();
        check_period.secs=0;
        check_period.nano_secs=20*NANOS_PER_MS; // TODO(iantay) what relationship with task period?
        val=bmac_set_rx_check_rate(check_period);
        bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 
        bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

        while (1) {
                if (bmac_rx_pkt_ready()==0) {
									nrk_wait_until_next_period();
								} else {
									nrk_led_toggle (RED_LED);
									clearToTx = 1;
									local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
									pc.printf("message from %d \n", rfRxInfo.srcAddr);
									rxContentSuccess++;
									int time_left_in_us = (rx_buf[0] << 24) | (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
									int score = calcScore(master_period_in_s, time_left_in_us);
									pc.printf("score for this mole is %d \n", score);
									cumscore += score;
									// Change something to make sure the buffer isn't the same if no buffer fill from Rx occurs
									rx_buf[1] = '0';
									// Release the RX buffer so future packets can arrive 
									bmac_rx_pkt_release ();
									// this is necessary
									nrk_wait_until_next_period ();
								}
                
        }
}

void slave_rx_task() {
				#ifdef NODE1
				//bmac_rfTxInfo.destAddr = 0x0051;
				bmac_rfRxInfo.srcAddr = 0x0051;
				#endif
		    #ifdef NODE2
				//bmac_rfTxInfo.destAddr = 0x0052;
				bmac_rfRxInfo.srcAddr = 0x0052;
				#endif
		    #ifdef NODE3
				//bmac_rfTxInfo.destAddr = 0x0053;
				bmac_rfRxInfo.srcAddr = 0x0053; 
				#endif
				pc.printf("slave rx initialized!\n");
        uint8_t i, len, rssi;
        int8_t val;
        char *local_rx_buf;
        nrk_time_t check_period;
        bmac_init(RADIO_CHANNEL);
        bmac_auto_ack_disable();
        check_period.secs=0;
        check_period.nano_secs=20*NANOS_PER_MS;
        val=bmac_set_rx_check_rate(check_period);
        bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 
        bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

        while (1) {
				  if (!bmac_rx_pkt_ready ()){
						nrk_wait_until_next_period ();
					} else {
						pc.printf("slave got msg to activate mole.\n");
						mole_active = 1;
						nrk_led_set(RED_LED);
						local_rx_buf = bmac_rx_pkt_get (&len, &rssi);
						rxContentSuccess++;
						slave_period_in_s = (int8_t) rx_buf[0];
						slave_mole_timer.reset();
						slave_mole_timer.start();
						// Change something to make sure the buffer isn't the same if no buffer fill from Rx occurs
						rx_buf[1] = '0';
						// Release the RX buffer so future packets can arrive 
						bmac_rx_pkt_release ();
						// this is necessary
						nrk_wait_until_next_period ();
					}
					
        }
}

void slave_tx_task () {
				pc.printf("slave tx initialized!\n");
				wait_ms(5000);
	      #ifdef NODE1
				bmac_rfTxInfo.destAddr = 0x0050; 
				#endif
		    #ifdef NODE2
				bmac_rfTxInfo.destAddr = 0x0050; 
				#endif
		    #ifdef NODE3
				bmac_rfTxInfo.destAddr = 0x0050; 
				#endif
        bmac_auto_ack_enable();
        // Wait until the rx_task starts up bmac
        // This should be called by all tasks using bmac that
        // do not call bmac_init()...
				while (1) {	
					while (!bmac_started ()) {
									nrk_wait_until_next_period ();
					}

					if (!mole_active) {
									nrk_wait_until_next_period ();
					}
					else {
						int time_elapsed_in_us = slave_mole_timer.read_us();
						
						// CASE 1: MOLE TIMES OUT
						if (time_elapsed_in_us >= ((int) slave_period_in_s) * 1000000){
							pc.printf("mole timed out \n");
							nrk_led_clr(RED_LED);
							mole_active = 0;
							slave_mole_timer.stop();
							slave_mole_timer.reset();
							bmac_rfTxInfo.destAddr = (0x0050);  // 0xFFFF is broadcast
							strncpy(tx_buf, (char *) &slave_period_in_s, 4); // send max time
							int8_t val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
							while (val==NRK_ERROR){
								wait_ms(50);
								int8_t val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
							}
							nrk_wait_until_next_period ();
						}
						
						// CASE 2: MOLE IS COVERED
						if (photodiode == 1) { // 1 is covered
									pc.printf("mole covered \n");
									nrk_led_clr(RED_LED);
									mole_active = 0;
									slave_mole_timer.stop();
									slave_mole_timer.reset();
									bmac_rfTxInfo.destAddr = (0x0050);  // 0xFFFF is broadcast
									strncpy(tx_buf, (char *) &time_elapsed_in_us, 4);
									int8_t val = bmac_tx_pkt(tx_buf, strlen(tx_buf));	
									while (val==NRK_ERROR){
													wait_ms(50); //TODO(iantay) retransmit code may not be good...
													val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	
									}
									nrk_wait_until_next_period();
					}
				}
			}
}
void master_tx_task () {
				pc.printf("master initialized\n");
				wait_ms(1000);
				int8_t val;
        // Wait until the rx_task starts up bmac
        // This should be called by all tasks using bmac that
        // do not call bmac_init()...
        while (!bmac_started ()) {
                nrk_wait_until_next_period ();
        }
				nrk_time_t check_period;
				check_period.secs=0;
				check_period.nano_secs=20*NANOS_PER_MS;
				val=bmac_set_rx_check_rate(check_period);
        bmac_rfRxInfo.srcAddr = (0x0050);
        //bmac_auto_ack_enable
				bmac_auto_ack_disable();
				while (1) {	

					if (!clearToTx) {
									nrk_wait_until_next_period ();
					}
					if (!master_waiting_for_slave && master_to_send < MOLE_APPEARANCES) {
													master_waiting_for_slave = 1;
													clearToTx = 0;
													pc.printf("Master sending mole %d to node %d \n", master_to_send, node_buf[master_to_send]);
													//bmac_rfTxInfo.destAddr = (node_buf[master_to_send++]);  // 0xFFFF is broadcast
													bmac_rfTxInfo.destAddr = 0x0051;
													if (val != NRK_OK){
														pc.printf("Master could not set dest bmac");
													}
													// For blocking transmits, use the following function call.
													// For this there is no need to register
													strncpy(tx_buf, &master_period_in_s, 1);
													val = bmac_tx_pkt(tx_buf, 1);	
													wait_ms(100); //TODO(iantay)
													while (val==NRK_ERROR){
																	pc.printf("retrying mole send \n");
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
        RX_TASK.period.nano_secs = 50*NANOS_PER_MS;
        RX_TASK.cpu_reserve.secs = 0;
        RX_TASK.cpu_reserve.nano_secs = 0;
        RX_TASK.offset.secs = 0;
        RX_TASK.offset.nano_secs = 0;
        nrk_activate_task (&RX_TASK);

        // RPRINT only for master DISABLED!
        //RPRINT_TASK.task = rprint_task<0>;
//        nrk_task_set_stk( &RPRINT_TASK, rprint_task_stack, NRK_APP_STACKSIZE);
//        RPRINT_TASK.prio = 2;
//        RPRINT_TASK.FirstActivation = TRUE;
//        RPRINT_TASK.Type = BASIC_TASK;
//        RPRINT_TASK.SchType = PREEMPTIVE;
//        RPRINT_TASK.period.secs = 0;
//        RPRINT_TASK.period.nano_secs = 2000*NANOS_PER_MS;
//        RPRINT_TASK.cpu_reserve.secs = 0;
//        RPRINT_TASK.cpu_reserve.nano_secs = 0;
//        RPRINT_TASK.offset.secs = 0;
//        RPRINT_TASK.offset.nano_secs = 0;
        #ifdef NODE0
        //nrk_activate_task (&RPRINT_TASK);
        #endif

        #ifdef NODE0
        TX_TASK.task = master_tx_task;
        #endif
        #if defined(NODE2) || defined(NODE3) || defined(NODE1)
				pc.printf("slave nodes are defined\n");
        TX_TASK.task = slave_tx_task;
        #endif
        nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
        TX_TASK.prio = 2;
        TX_TASK.FirstActivation = TRUE;
        TX_TASK.Type = BASIC_TASK;
        TX_TASK.SchType = PREEMPTIVE;
        TX_TASK.period.secs = 0;
				#if defined(NODE2) || defined(NODE3) || defined(NODE1)
        TX_TASK.period.nano_secs = 50*NANOS_PER_MS; // slave polling frequency needs to be high
				#endif
        #ifdef NODE0
        TX_TASK.period.nano_secs = 200*NANOS_PER_MS;
        #endif
        TX_TASK.cpu_reserve.secs = 0;
        TX_TASK.cpu_reserve.nano_secs = 0;
        TX_TASK.offset.secs = 0;
        TX_TASK.offset.nano_secs = 0;
        nrk_activate_task (&TX_TASK);
}

