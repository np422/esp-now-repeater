#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_system.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "../../esp-now-gw/main/esp_msg_types.h"

#define QUEUE_SIZE     20

const uint8_t my_mac[ESP_NOW_ETH_ALEN] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x01 };
const uint8_t gw_mac[ESP_NOW_ETH_ALEN] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x00 };

const char* PROG = "repeater";
static xQueueHandle recv_queue;
int wifi_retry_num = 0;

#define MAX_TERMINALS  20

struct esp_now_terminal_t {
    uint8_t peer_mac[ESP_NOW_ETH_ALEN];
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];
    esp_now_peer_type_t peer_type;
    uint8_t peer_tag[ESP_NOW_TAGLEN];
};
typedef struct esp_now_terminal_t esp_now_terminal_t;
static esp_now_terminal_t terminals[MAX_TERMINALS];
static SemaphoreHandle_t terminals_mutex;
static int npeers;

#define TAG "repeater-00"

char* get_msg_type_name(esp_now_message_type_t type) {
    switch(type) {
    case REGISTER:
        return "REGISTER";
        break;
    case CONFIRM:
        return "CONFIRM";
        break;
    case BUTTON_PRESSED:
        return "BUTTON_PRESSED";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}

char* get_type_name(esp_now_peer_type_t type) {
    switch(type) {
    case GW:
        return "GW";
        break;
    case SENSOR:
        return "SENSOR";
        break;
    case BUTTON:
        return "BUTTON";
        break;
    case RELAY:
        return "RELAY";
        break;
    case REPEATER:
        return "REPEATER";
        break;
    default:
        return "UNKNOWN";
    }
}

uint8_t *get_dest_mac_from_tag(uint8_t *tag) {
    xSemaphoreTake(terminals_mutex, portMAX_DELAY);
    for (int i=0 ; i<npeers ; i++) {
        if (strcmp((char *) tag, (char *) terminals[i].peer_tag) == 0) {
            xSemaphoreGive(terminals_mutex);
            return terminals[i].dest_mac;
        }
    }
    xSemaphoreGive(terminals_mutex);
    return NULL;
}

void dump_esp_msg(esp_now_message_t *msg) {
    ESP_LOGI(PROG, "##################");
    ESP_LOGI(PROG, "# ESP-NOW message ");
    ESP_LOGI(PROG, "##################");
    ESP_LOGI(PROG, "Length: %u", msg->len);
    ESP_LOGI(PROG, "Type: %s", get_msg_type_name(msg->type));
    switch(msg->type) {
    case REGISTER:
        ESP_LOGI(PROG, "Dumping register message");
        register_message_t *rmsg;
        rmsg = (register_message_t *) msg->message;
        ESP_LOGI(PROG, "Peer type: %s",get_type_name(rmsg->type));
        ESP_LOGI(PROG, "Peer tag: %s", rmsg->tag);
        break;
    case CONFIRM:
        ESP_LOGI(PROG, "TBD...");
        break;
    case BUTTON_PRESSED:
        ESP_LOGI(PROG, "Dumping button pressed message");
        button_press_message_t *bmsg;
        bmsg = (button_press_message_t *) msg->message;
        ESP_LOGI(PROG, "Peer tag: %s", bmsg->sender_tag);
        for (int i = 0 ; i<MAX_BUTTONS ; i++) {
            ESP_LOGI(PROG, "Button %i: %i", i, bmsg->buttons_pressed[i]);
        }
        break;
    default:
        ESP_LOGI(PROG, "TBD ... UNKNOWN");
        break;
    }
    ESP_LOGI(PROG, "");
}

void dump_esp_queue_msg(esp_now_queue_message_t *msg) {
    ESP_LOGI(PROG, "##################");
    ESP_LOGI(PROG, "# ESP-NOW Queue message ");
    ESP_LOGI(PROG, "##################");
    ESP_LOGI(PROG, "Mac address: %x:%x:%x %x:%x:%x", msg->mac[0], msg->mac[1], msg->mac[2], msg->mac[3], msg->mac[4], msg->mac[5]);
    ESP_LOGI(PROG, "Dumping esp_now message ...");
    ESP_LOGI(PROG, "");
    dump_esp_msg(& (msg->esp_msg));
}

void add_or_update_terminal(uint8_t *peer_mac, uint8_t *dest_mac, esp_now_peer_type_t peer_type, uint8_t *peer_tag) {
    int i;
    bool found = false;
    xSemaphoreTake(terminals_mutex, portMAX_DELAY);
    esp_now_peer_info_t peer;
    memset(&peer,0,sizeof(peer));
    peer.channel = 10;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, dest_mac, ESP_NOW_ETH_ALEN);
    esp_now_peer_info_t pi;
    if (esp_now_get_peer(dest_mac,&pi) == ESP_ERR_ESPNOW_NOT_FOUND) {
        ESP_LOGI(PROG, "Added esp peer");
        esp_now_add_peer(&peer);
    } else {
        ESP_LOGI(PROG, "Modified esp peer");
        esp_now_mod_peer(&peer);
    }
    for (i=0;i<=npeers;i++) {
        if (strcmp((char *)terminals[i].peer_tag, (char *) peer_tag) == 0) {
            ESP_LOGI(PROG, "Updated existing peer");
            found = true;
            memcpy(terminals[i].peer_mac, peer_mac, ESP_NOW_ETH_ALEN);
            memcpy(terminals[i].dest_mac, dest_mac, ESP_NOW_ETH_ALEN);
            terminals[i].peer_type = peer_type;
        }
    }
    if (!found) {
        ESP_LOGI(PROG, "Added new peer");
        memcpy(terminals[npeers].peer_mac, peer_mac, ESP_NOW_ETH_ALEN);
        memcpy(terminals[npeers].dest_mac, dest_mac, ESP_NOW_ETH_ALEN);
        terminals[npeers].peer_type = peer_type;
        strcpy((char *) terminals[npeers].peer_tag, (char *) peer_tag);
        npeers++;
    }
    for (i=0;i<npeers;i++) {
        ESP_LOGI(PROG, "Peer no %i",i);
        ESP_LOGI(PROG, "Peer mac %x:%x:%x:%x:%x:%x", terminals[i].peer_mac[0], terminals[i].peer_mac[1], terminals[i].peer_mac[2], terminals[i].peer_mac[3], terminals[i].peer_mac[4], terminals[i].peer_mac[5]);
        ESP_LOGI(PROG, "Dest mac %x:%x:%x:%x:%x:%x", terminals[i].dest_mac[0], terminals[i].dest_mac[1], terminals[i].dest_mac[2], terminals[i].dest_mac[3], terminals[i].dest_mac[4], terminals[i].dest_mac[5]);
        ESP_LOGI(PROG, "Peer type: %s", get_type_name(terminals[i].peer_type));
        ESP_LOGI(PROG, "Peer tag: %s",  terminals[i].peer_tag);
    }
    xSemaphoreGive(terminals_mutex);
}

void send_reg_msg() {
    esp_now_message_t msg;
    const TickType_t delay = 100 / portTICK_PERIOD_MS;
    register_message_t *my_reg_msg;

    my_reg_msg = (register_message_t *) msg.message;
    my_reg_msg->type = REPEATER;
    strcpy((char *) msg.dest_tag, "gw");
    strcpy((char *) my_reg_msg->tag, TAG);
    memcpy((void *) my_reg_msg->mac, my_mac, ESP_NOW_ETH_ALEN);

    // setup esp_now_message to send
    msg.len = sizeof(register_message_t);
    msg.type=REGISTER;

    ESP_LOGI(PROG,"Sending message of type %u and size %u to gw", msg.type, msg.len);
    esp_now_send(gw_mac, (const uint8_t *) &msg, sizeof(esp_now_message_t));
    vTaskDelay(delay);
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS ) {
        ESP_LOGI(PROG, "ESP_NOW_SEND callback, status SUCCESS ");
    } else {
        ESP_LOGI(PROG, "ESP_NOW_SEND callback, status FAILURE");
    }
}

void generic_send_esp_msg(esp_now_message_t *msg) {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint8_t *p;
    const TickType_t delay = 100 / portTICK_PERIOD_MS;

    if (strcmp((char *) msg->dest_tag, "gw") == 0) {
        strcpy((char *) mac, (char *) gw_mac);
        ESP_LOGI(PROG, "Sending msg to default route");
    } else if ((p = get_dest_mac_from_tag((uint8_t *) msg->dest_tag)) == NULL ) {
        ESP_LOGI(PROG, "Dest tag not in terminals table, can't send msg to %s", msg->dest_tag);
        return;
    } else {
        ESP_LOGI(PROG, "Found dest mac in terminals table");
        strcpy((char *) mac, (char *) p);
    }
    ESP_LOGI(PROG, "Sending message of type %u to mac address %x:%x:%x %x:%x:%x ", msg->type, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_now_send(mac, (const uint8_t *) msg, sizeof(esp_now_message_t));
    vTaskDelay(delay);
}


void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    esp_now_queue_message_t queue_msg;
    BaseType_t enqueue_retval;

    // Setup the queue message
    if ( len > sizeof(esp_now_message_t) ) {
        ESP_LOGI(PROG, "Recieved to big message, discarding");
        return;
    }
    memcpy(&(queue_msg.esp_msg), data, len);
    memcpy(queue_msg.mac, mac_addr, ESP_NOW_ETH_ALEN);
    dump_esp_queue_msg(&queue_msg);
    ESP_LOGI(PROG, "Enqueuing message with esp-msg length %i",queue_msg.esp_msg.len );
    enqueue_retval = xQueueSend(recv_queue, (void *) &queue_msg, ( TickType_t ) 0);
    ESP_LOGI(PROG, "Enqueued message");
    if ( enqueue_retval == pdTRUE ) {
        ESP_LOGI(PROG, "ESP_NOW_RECV callback, enqueued message sucessfully");
    } else {
        ESP_LOGI(PROG, "ESP_NOW_RECV callback, failed to enqueue message");
    }
    ESP_LOGI(PROG, "RECV_CALLBACK finished");
}

void recv_task(void *foo) {
    esp_now_queue_message_t qmsg;
    for(;;) {
        ESP_LOGI(PROG, "Starting to pull queue");
        if( xQueueReceive( recv_queue, &( qmsg ),  ( TickType_t ) 10000 / portTICK_PERIOD_MS) == pdPASS ) {
            ESP_LOGI(PROG, "Dequeued message");
            dump_esp_queue_msg(&qmsg);
            switch (qmsg.esp_msg.type) {
            case REGISTER:
                ESP_LOGI(PROG, "Dequeued register msg");
                register_message_t *reg_msg;
                reg_msg = (register_message_t *) qmsg.esp_msg.message;
                add_or_update_terminal(reg_msg->mac, qmsg.mac, reg_msg->type, reg_msg->tag);
                break;
            case CONFIRM:
                if ( strcmp((char *) qmsg.esp_msg.dest_tag, TAG) == 0) {
                    ESP_LOGI(PROG, "Received my confirm message");
                } else {
                    ESP_LOGI(PROG, "Received other terminals confirm message, retransmitting");
                    generic_send_esp_msg(&(qmsg.esp_msg));
                }
                break;
            default:
                generic_send_esp_msg(&(qmsg.esp_msg));
                break;
            }
        } else {
            ESP_LOGI(PROG, "Failedpulling message");
        }
    }
}



void esp_init(void) {

    const TickType_t delay = 10000 / portTICK_PERIOD_MS;

    esp_now_init();
    esp_now_register_send_cb(send_cb);
    esp_now_register_recv_cb(recv_cb);

    // Add gw-peer
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 10;
    peer->ifidx = WIFI_IF_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, gw_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);
    vTaskDelay(delay);
    send_reg_msg();
}

void periodic_timer_callback(void* arg) {
    send_reg_msg();
}
void timer_init() {
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    // Run once every minute
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 60 * 1000 * 1000));
}

void app_main(void)
{
    uint8_t read_mac[ESP_NOW_ETH_ALEN];
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK(esp_base_mac_addr_set(my_mac));
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(10,1));
    ESP_ERROR_CHECK(esp_read_mac(read_mac, WIFI_IF_STA));
    ESP_LOGI(PROG, "read mac-address %x:%x:%x %x:%x:%x", read_mac[0], read_mac[1],read_mac[2],read_mac[3],read_mac[4],read_mac[5]);
    esp_init();
    timer_init();

    terminals_mutex = xSemaphoreCreateMutex();
    recv_queue =xQueueCreate(10, sizeof(esp_now_queue_message_t));
    xTaskCreate(recv_task, "esp_now_recv_task", 64*1024, NULL, 1, NULL);
}
