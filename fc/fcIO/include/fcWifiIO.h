#ifndef FC_FCIO_INCLUDE_FCWIFIIO_H_
#define FC_FCIO_INCLUDE_FCWIFIIO_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif_net_stack.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_http_server.h"
#include "fcLogger.h"
#include "mathUtil.h"

uint8_t startHttpServer(void);
uint8_t startUDPServer(void);

#endif /* FC_FCIO_INCLUDE_FCWIFIIO_H_ */
