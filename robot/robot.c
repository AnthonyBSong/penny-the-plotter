/**
 * Copyright (c) 2022 Andrew McDonnell
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Standard libraries
#include <string.h>
#include <stdlib.h>
#include <pico/multicore.h>
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "stdio.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "pt_cornell_rp2040_v1_4.h"

// ======================================
// udp constants
#define UDP_PORT 5555
#define UDP_MSG_LEN_MAX 32
#define UDP_TARGET "10.120.64.63" // my laptop addresss
#define UDP_INTERVAL_MS 10

// =======================================
// necessary to connect to wireless
// !!! Do NOT post this info !!!
#define WIFI_SSID "xxx"
#define WIFI_PASSWORD "xxx"

// =======================================
// protothreads and thread communication
char recv_data[UDP_MSG_LEN_MAX];

// payload to led blink
int blink_time ;
// interthread communicaitqoin
struct pt_sem new_udp_recv_s, new_udp_send_s ;

//==================================================
// UDP async receive callback setup
// NOTE that udpecho_raw_recv is triggered by a signal
// directly from the LWIP package -- not from your code
// this callback juswt copies out the packet string
// and sets a "new data" flag
// This runs in an ISR -- KEEP IT SHORT!!!

#if LWIP_UDP

static struct udp_pcb *udpecho_raw_pcb;
struct pbuf *p ;

static void
udpecho_raw_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                 const ip_addr_t *addr, u16_t port)
{
  LWIP_UNUSED_ARG(arg);

  if (p != NULL) {
    //printf("p payload in call back: = %s\n", p->payload);
    memcpy(recv_data, p->payload, UDP_MSG_LEN_MAX);
    // can signal from an ISR -- BUT NEVER wait in an ISR
    PT_SEM_SIGNAL(pt, &new_udp_recv_s) ;
    
    /* free the pbuf */
    pbuf_free(p);
  }
  else printf("NULL pt in callback");
}

// ===================================
// Define the recv callback 
void 
udpecho_raw_init(void)
{
  udpecho_raw_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_RAM);

  if (udpecho_raw_pcb != NULL) {
    err_t err;
    // netif_ip4_addr returns the picow ip address
    err = udp_bind(udpecho_raw_pcb, netif_ip4_addr(netif_list), UDP_PORT); //DHCP addr

    if (err == ERR_OK) {
      udp_recv(udpecho_raw_pcb, udpecho_raw_recv, NULL);
      //printf("Set up recv callback\n");
    } else {
      printf("bind error");
    }
  } else {
    printf("udpecho_raw_pcb error");
  }
}

#endif /* LWIP_UDP */
// end recv setup

// =======================================
// UDP send thead
// for now just bounces back the recv packet

static PT_THREAD (protothread_udp_send(struct pt *pt))
 { PT_BEGIN(pt);
    static struct udp_pcb* pcb;
    pcb = udp_new();
    pcb->remote_port = UDP_PORT ;
    pcb->local_port = UDP_PORT ;

    static ip_addr_t addr;
    ipaddr_aton(UDP_TARGET, &addr);

    static int counter = 0;
    
    while (true) {
        
        PT_SEM_WAIT(pt, &new_udp_send_s) ;

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_RAM);
        char *req = (char *)p->payload;
        memset(req, 0, UDP_MSG_LEN_MAX+1);
        //
        sprintf(req, "blink time = %d mSec", blink_time);
        err_t er = udp_sendto(pcb, p, &addr, UDP_PORT); //port
       
        pbuf_free(p);
        if (er != ERR_OK) {
            printf("Failed to send UDP packet! error=%d", er);
        } else {
            printf("Sent packet %d\n", counter);
            counter++;
        }

        // Note in practice for this simple UDP transmitter,
        // the end result for both background and poll is the same
    
#if PICO_CYW43_ARCH_POLL
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        sleep_ms(BEACON_INTERVAL_MS);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.

        PT_YIELD_usec(UDP_INTERVAL_MS*1000);
#endif
    }

    PT_END(pt);
}

// ==================================================
// udp recv processing
// ==================================================
static PT_THREAD (protothread_udp_recv(struct pt *pt))
{
    PT_BEGIN(pt);
    
     // data structure for interval timer
     PT_INTERVAL_INIT() ;

      while(1) {
        // wait for new packet
        // MUST be an integer format number!!
        PT_SEM_WAIT(pt, &new_udp_recv_s) ;

        // process packet and signal udp send thread
        sscanf(recv_data, "%d", &blink_time) ;  
       // tell send threead 
        PT_SEM_SIGNAL(pt, &new_udp_send_s) ;

        PT_YIELD_INTERVAL(10) ;
        //
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // blink thread

// ==================================================
// toggle cyw43 LED  
// this is really just a test of multitasking
// compatability with LWIP
// ==================================================
static PT_THREAD (protothread_toggle_cyw43(struct pt *pt))
{
    PT_BEGIN(pt);
    static bool LED_state = false ;
    //
     // data structure for interval timer
     PT_INTERVAL_INIT() ;
     // set some default blink time
     blink_time = 100 ;
     // echo the default time to udp connection
      PT_SEM_SIGNAL(pt, &new_udp_send_s) ;

      while(1) {
        //
        LED_state = !LED_state ;
        // the onboard LED is attached to the wifi module
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
        // blink time is modifed by the udp recv thread
        PT_YIELD_INTERVAL(blink_time*1000) ;
        //
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // blink thread


// ====================================================
int main() {
  // =======================
  // init the serial
    stdio_init_all();

  // =======================
  // init the wifi network
    if (cyw43_arch_init()) {
        printf(pt_serial_out_buffer, "failed to initialise cyw43\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 50000)) {
        printf("failed to connect: cyw43 arch wifi timeout.\n");
        return 1;
    } else {
      // optional print addr
        printf("Connected: picoW IP addr: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    }

    //============================
    // UDP recenve ISR routines
    udpecho_raw_init();

      //========================================
  // start core 1 threads -- none here
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // === config threads ========================
  // for core 0

  // init the thread control semaphores
  PT_SEM_INIT(&new_udp_send_s, 0) ;
  PT_SEM_INIT(&new_udp_recv_s, 0) ;

  //printf("Starting threads\n") ;
  pt_add_thread(protothread_udp_send);
  pt_add_thread(protothread_udp_recv);
  pt_add_thread(protothread_toggle_cyw43) ;
  //pt_add_thread(protothread_serial) ;
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;

    cyw43_arch_deinit();
    return 0;
}