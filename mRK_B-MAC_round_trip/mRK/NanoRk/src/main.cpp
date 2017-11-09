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

typedef struct {
	uint16_t send_to;
	char msg[50];
} relay_msg;

std::vector<relay_msg> relay_msg_buf;

RF_RX_INFO rfRxInfo;
RF_TX_INFO rfTxInfo;

// Counter for number of detected packets that have valid data
static uint16_t rxContentSuccess = 0;
// Counter for number of detected packets from the preamble perspective, regardless of contents
static uint16_t mrfRxISRcallbackCounter = 0;
// Counter for number of sent packets
static uint16_t sentPackets = 0;

static uint8_t clearToTx = 0;

Serial pc(USBTX, USBRX);

// Count number of detected packets and toggle a pin for each packet
void mrfIsrCallback()
{		
	mrfRxISRcallbackCounter++;
}

int main(void)
{	
	nrk_setup_ports();
	
	nrk_init();
	bmac_task_config();
	nrk_create_taskset();
  nrk_start();
	return 0;
}

template<int C>
void rprint_task ()
{
	pc.printf("node %d initialized", C);
	while (1) {
		pc.printf("node %d, %d sent, %d successful\n", C, mrfRxISRcallbackCounter, rxContentSuccess);
		nrk_wait_until_next_period ();
	}
}

void rx_task ()
{
  uint8_t i, len, rssi;
  int8_t val;
	char *local_rx_buf;
  nrk_time_t check_period;
  //printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  // init bmac
  bmac_init (RADIO_CHANNEL);
	bmac_auto_ack_disable();
	#ifdef NODE1
	clearToTx = 1;
	#endif
	
	
  // Enable AES 128 bit encryption
  // When encryption is active, messages from plaintext
  // source will still be received. 
	
	// Commented out by MB
  //bmac_encryption_set_key(aes_key,16);
  //bmac_encryption_enable();
	// bmac_encryption_disable();


  // By default the RX check rate is 200ms
  // below shows how to change that
  check_period.secs=0;
  check_period.nano_secs=20*NANOS_PER_MS;
  val=bmac_set_rx_check_rate(check_period);

  // The default Clear Channel Assement RSSI threshold.
  // Setting this value higher means that you will only trigger
  // receive with a very strong signal.  Setting this lower means
  // bmac will try to receive fainter packets.  If the value is set
  // too high or too low performance will suffer greatly.
   bmac_set_cca_thresh(DEFAULT_BMAC_CCA); 


  // This sets the next RX buffer.
  // This can be called at anytime before releaseing the packet
  // if you wish to do a zero-copy buffer switch
  bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);

	
  while (1)
	{
    //nrk_led_toggle (RED_LED);
    // Wait until an RX packet is received
    val = bmac_wait_until_rx_pkt ();

		
		// Get the RX packet 
		local_rx_buf = bmac_rx_pkt_get (&len, &rssi);

		//if( bmac_rx_pkt_is_encrypted()==1 ) nrk_kprintf( PSTR( "Packet Encrypted\r\n" ));
		//printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
		//for (i = 0; i < len; i++)
		//  printf ("%c", rx_buf[i]);
		//printf ("]\r\n");
		//nrk_led_clr (ORANGE_LED);
	 
		
		// Vuk: Rx content test
		if (strncmp(rx_buf, "This is a test", 46) == 0)
		{
			rxContentSuccess++;
			clearToTx = 1;
			#ifdef NODE2
			if (bmac_rfRxInfo.srcAddr == 0x0051){
				pc.printf("node2 redirecting to node 3\n");
				relay_msg rm;
				rm.send_to = 0x0053;
				strcpy(rm.msg, rx_buf);
				relay_msg_buf.push_back(rm);
			}
			if (bmac_rfRxInfo.srcAddr == 0x0053){
				pc.printf("node2 redirecting to node 1\n");

				relay_msg rm;
				rm.send_to = 0x0051;
				strcpy(rm.msg, rx_buf);
				relay_msg_buf.push_back(rm);

			}
			#endif
			
			#ifdef NODE3
			strcpy(tx_buf, rx_buf);
			#endif
		}
		// Change something to make sure the buffer isn't the same if no buffer fill from Rx occurs
		rx_buf[1] = '0';
		
		// Release the RX buffer so future packets can arrive 
		bmac_rx_pkt_release ();
		
		// this is necessary
    nrk_wait_until_next_period ();

  }
}

uint8_t ctr_cnt[4];

void tx_task ()
{
  uint8_t j, i, val, len, cnt;
  int8_t v;
  nrk_sig_t tx_done_signal;
  nrk_sig_mask_t ret;
  nrk_time_t r_period;
	
	// printf("tx_task PID=%d\r\n", nrk_get_pid ());
	
  // Wait until the rx_task starts up bmac
  // This should be called by all tasks using bmac that
  // do not call bmac_init()...
  while (!bmac_started ())
    nrk_wait_until_next_period ();

	nrk_time_t check_period;
	check_period.secs=0;
  check_period.nano_secs=20*NANOS_PER_MS;
  val=bmac_set_rx_check_rate(check_period);

  // Sample of using Reservations on TX packets
  // This example allows 2 packets to be sent every 5 seconds
  // r_period.secs=5;
  // r_period.nano_secs=0;
  // v=bmac_tx_reserve_set( &r_period, 2 );
  // if(v==NRK_ERROR) nrk_kprintf( PSTR("Error setting b-mac tx reservation (is NRK_MAX_RESERVES defined?)\r\n" ));


  // Get and register the tx_done_signal if you want to
  // do non-blocking transmits
  tx_done_signal = bmac_get_tx_done_signal ();
  nrk_signal_register (tx_done_signal);

  ctr_cnt[0]=0; ctr_cnt[1]=0; ctr_cnt[2]=0; ctr_cnt[3]=0;
  cnt = 0;
	
	int packetsToTx = 1000;
	#ifdef NODE1
	bmac_addr_decode_set_my_mac(0x0051); 
	#endif
	#ifdef NODE2
	bmac_addr_decode_set_my_mac(0x0052); 
	#endif
	#ifdef NODE3
	bmac_addr_decode_set_my_mac(0x0053); 
	#endif
	
	#ifdef NODE1
  while (packetsToTx != 0)
	#endif

	#if defined(NODE2) || defined(NODE3)
  while (1)
	#endif
	{
		//nrk_led_toggle(BLUE_LED);
    // Build a TX packet
    //sprintf (tx_buf, "This is a test %d", cnt);
    //nrk_led_set (BLUE_LED);
		
		if (clearToTx == 1)
		{
			
			// Auto ACK is an energy efficient link layer ACK on packets
			// If Auto ACK is enabled, then bmac_tx_pkt() will return failure
			// if no ACK was received. In a broadcast domain, the ACK's will
			// typically collide.  To avoid this, one can use address decoding. 
			// The functions are as follows:
			// bmac_auto_ack_enable();
			//	 bmac_auto_ack_disable();

			// Address decoding is a way of preventing the radio from receiving
			// packets that are not address to a particular node.  This will 
			// supress ACK packets from nodes that should not automatically ACK.
			// The functions are as follows:
			#ifdef NODE1
			sprintf (tx_buf, "This is a test");
			bmac_addr_decode_dest_mac(0x0052);  // 0xFFFF is broadcast
			#endif

	/*
			 ctr_cnt[0]=cnt; 
			 if(ctr_cnt[0]==255) ctr_cnt[1]++; 
			 if(ctr_cnt[1]==255) ctr_cnt[2]++; 
			 if(ctr_cnt[2]==255) ctr_cnt[3]++; 
			 // You need to increase the ctr on each packet to make the 
			 // stream cipher not repeat.
			 bmac_encryption_set_ctr_counter(&ctr_cnt,4);

	*/  // For blocking transmits, use the following function call.
			// For this there is no need to register
			#ifdef NODE2
			clearToTx = 0;
			if (relay_msg_buf.size() > 0){
				relay_msg rm = relay_msg_buf.back();
				pc.printf("cheatprint %d %s", rm.send_to, rm.msg);
				relay_msg_buf.pop_back();
				strcpy(tx_buf, rm.msg);
				bmac_addr_decode_dest_mac(rm.send_to);  // 0xFFFF is broadcast
			}
			#endif
			
			#ifdef NODE3
			bmac_addr_decode_dest_mac(0x0052);  // 0xFFFF is broadcast
			if (strlen(tx_buf) == 0){
			   nrk_wait_until_next_period();
			}
			#endif
			
			// bmac_addr_decode_enable();
			// bmac_addr_decode_disable();
			val=bmac_tx_pkt(tx_buf, strlen(tx_buf));	

			if(val==NRK_OK) cnt++;
			else ;//printf("NO ack or Reserve Violated! \r\n");

			// This function shows how to transmit packets in a
			// non-blocking manner  
			// val = bmac_tx_pkt_nonblocking(tx_buf, strlen (tx_buf));
			// printf ("Tx packet enqueued\r\n");
			// This functions waits on the tx_done_signal
			//ret = nrk_event_wait (SIG(tx_done_signal));

			// Just check to be sure signal is okay
			//if(ret & SIG(tx_done_signal) == 0 ) 
			//printf ("TX done signal error\r\n");
		 
			// If you want to see your remaining reservation
			// printf( "reserve=%d ",bmac_tx_reserve_get() );
			
			// Task gets control again after TX complete
			//printf("Tx task sent data!\r\n");
			//nrk_led_clr (BLUE_LED);
			//printf("tx_task PID=%d\r\n", nrk_get_pid ());
			
			packetsToTx--;
			sentPackets++;
		}
    nrk_wait_until_next_period ();
  }
}

void nrk_create_taskset ()
{
	// Activate both tasks on both nodes
	// Choose node by macro defined on top of this file
  // Tx task runs mode often on NODE2 to make sure all received packets are returned promptly, witout blocking the receiving buffer

  RX_TASK.task = rx_task;
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
	
	RPRINT_TASK.task = rprint_task<3>; // CHANGE THIS FOR EVERY NODE
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
	nrk_activate_task (&RPRINT_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 0;
	#ifdef NODE1
  TX_TASK.period.nano_secs = 200*NANOS_PER_MS;
  #endif
	#if defined(NODE2) || defined(NODE3)
  TX_TASK.period.nano_secs = 100*NANOS_PER_MS;
  #endif
	TX_TASK.cpu_reserve.secs = 0;
  TX_TASK.cpu_reserve.nano_secs = 0;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
	nrk_activate_task (&TX_TASK);
}