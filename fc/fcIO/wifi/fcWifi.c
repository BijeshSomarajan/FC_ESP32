#include "fcWifi.h"

// WiFi parameters
#define FC_WIFI_CONNECTED_BIT BIT0
#define FC_WIFI_FAIL_BIT      BIT1

/*STA Settings */
#define FC_ESP_WIFI_STA_SSID    "brhs-wifi"
#define FC_ESP_WIFI_STA_PASSWD  "passw0rd"
#define FC_ESP_WIFI_STA_MAXIMUM_RETRY     5
#define FC_ESP_WIFI_STA_CONN_TIMEOUT  200

/* AP Settings */
#define FC_ESP_WIFI_AP_SSID     "brhs-wifi"
#define FC_ESP_WIFI_AP_PASSWD   "passw0rd"
#define FC_ESP_WIFI_AP_MAX_STA_CON    1
#define FC_ESP_WIFI_AP_CHANNEL 1
#define FC_ESP_WIFI_AP_CONN_TIMEOUT  200

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

char wifiDebugBuf[256];

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		logString("WIFI : STA Disconnected\n");
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, FC_WIFI_CONNECTED_BIT);
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t*) event_data;
		sprintf(wifiDebugBuf, "WIFI : STA Connected , Got IP : "IPSTR"\n", IP2STR(&event->ip_info.ip));
		logString(wifiDebugBuf);
		xEventGroupSetBits(wifi_event_group, FC_WIFI_CONNECTED_BIT);
	}
}

static uint8_t initialise_wifi(void) {
	esp_err_t err = esp_netif_init();
	if (err == ESP_OK) {
		logString("WIFI : esp_netif_init Success\n");
	} else {
		logString("WIFI : esp_netif_init Failed\n");
	}

	wifi_event_group = xEventGroupCreate();

	err = esp_event_loop_create_default();
	if (err == ESP_OK) {
		logString("WIFI : esp_event_loop_create_default Success\n");
	} else {
		logString("WIFI : esp_event_loop_create_default Failed\n");
		return 0;
	}

	esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	//cfg.ampdu_rx_enable = 0; //Generally, the AMPDU should be enabled. Disabling AMPDU is usually for debugging purposes.

	err = esp_wifi_init(&cfg);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_init Success\n");
	} else {
		logString("WIFI : esp_wifi_init Failed\n");
		return 0;
	}

	err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, NULL);
	if (err == ESP_OK) {
		logString("WIFI : esp_event_handler_register , STA_DISCONNECTED , Success\n");
	} else {
		logString("WIFI : esp_event_handler_register , STA_DISCONNECTED ,Failed\n");
		return 0;
	}

	err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);
	if (err == ESP_OK) {
		logString("WIFI : esp_event_handler_register , IP_EVENT_STA_GOT_IP Success\n");
	} else {
		logString("WIFI : esp_event_handler_register , IP_EVENT_STA_GOT_IP Failed\n");
		return 0;
	}

	err = esp_wifi_set_storage(WIFI_STORAGE_FLASH);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_storage , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_storage , Failed\n");
		return 0;
	}

	err = esp_wifi_set_mode(WIFI_MODE_NULL);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_mode , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_mode , Failed\n");
		return 0;
	}

	err = esp_wifi_start();
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_start , Success\n");
	} else {
		logString("WIFI : esp_wifi_start , Failed\n");
		return 0;
	}
	return 1;
}

#if FC_WIFI_MODE == FC_WIFI_MODE_STA
int8_t wifi_sta(int timeout_ms) {
	wifi_config_t wifi_sta_config = { .sta = { .ssid = FC_ESP_WIFI_STA_SSID, .password = FC_ESP_WIFI_STA_PASSWD, .scan_method = WIFI_ALL_CHANNEL_SCAN, .failure_retry_cnt = FC_ESP_WIFI_STA_MAXIMUM_RETRY, .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, } };

	esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);

	if (err == ESP_OK) {
		logString("WIFI : wifi_set_mode(WIFI_MODE_STA) , Success\n");
	} else {
		logString("WIFI : wifi_set_mode(WIFI_MODE_STA) , Failed\n");
		return 0;
	}

	err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_STA) , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_STA) , Failed\n");
		return 0;
	}

	err = esp_wifi_connect();
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_connect , Success\n");
	} else {
		logString("WIFI : esp_wifi_connect , Failed\n");
		return 0;
	}

	int bits = xEventGroupWaitBits(wifi_event_group, FC_WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, timeout_ms / portTICK_PERIOD_MS);
	if (bits) {
		logString("WIFI : WIFI_MODE_STA Connected\n");
	} else {
		logString("WIFI : WIFI_MODE_STA Can't Connect\n");
	}

	return (bits & FC_WIFI_CONNECTED_BIT) != 0;
}
#endif

#if FC_WIFI_MODE == FC_WIFI_MODE_APSTA

uint8_t wifi_apsta(int timeout_ms) {
	wifi_config_t ap_config = { .ap = { .ssid = FC_ESP_WIFI_AP_SSID, .password = FC_ESP_WIFI_AP_PASSWD, .authmode = WIFI_AUTH_WPA_WPA2_PSK, .ssid_len = strlen(FC_ESP_WIFI_AP_SSID), .max_connection = FC_ESP_WIFI_AP_MAX_STA_CON, .channel = FC_ESP_WIFI_AP_CHANNEL, } };
	if (strlen(FC_ESP_WIFI_AP_PASSWD) == 0) {
		ap_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	wifi_config_t sta_config = { .sta = { .ssid = FC_ESP_WIFI_STA_SSID, .password = FC_ESP_WIFI_STA_PASSWD, .scan_method = WIFI_ALL_CHANNEL_SCAN, .failure_retry_cnt = FC_ESP_WIFI_STA_MAXIMUM_RETRY, .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, } };

	esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_mode(WIFI_MODE_APSTA) , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_mode(WIFI_MODE_APSTA) , Failed\n");
		return 0;
	}

	err = esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_AP) , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_AP) , Failed\n");
		return 0;
	}

	err = esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_STA) , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_STA) , Failed\n");
		return 0;
	}

	err = esp_wifi_start();
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_start() , Success\n");
	} else {
		logString("WIFI : esp_wifi_start() , Failed\n");
		return 0;
	}

	err = esp_wifi_connect();
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_connect() , Success\n");
	} else {
		logString("WIFI : esp_wifi_connect() , Failed\n");
		return 0;
	}

	int bits = xEventGroupWaitBits(wifi_event_group, FC_WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, timeout_ms / portTICK_PERIOD_MS);
	if (bits) {
		sprintf(wifiDebugBuf, "WIFI : STA connected , SSID: %s\n", FC_ESP_WIFI_STA_SSID);
		logString(wifiDebugBuf);
	} else {
		sprintf(wifiDebugBuf, "WIFI : STA Can't connect , SSID: %s\n", FC_ESP_WIFI_STA_SSID);
		logString(wifiDebugBuf);
	}

	return (bits & FC_WIFI_CONNECTED_BIT) != 0;
}
#endif

#if FC_WIFI_MODE == FC_WIFI_MODE_AP
uint8_t wifi_ap(void) {
	wifi_config_t ap_config = { .ap = { .ssid = FC_ESP_WIFI_AP_SSID, .password = FC_ESP_WIFI_AP_PASSWD, .authmode = WIFI_AUTH_WPA_WPA2_PSK, .ssid_len = strlen(FC_ESP_WIFI_AP_SSID), .max_connection = FC_ESP_WIFI_AP_MAX_STA_CON, .channel = FC_ESP_WIFI_AP_CHANNEL, } };
	if (strlen(FC_ESP_WIFI_AP_PASSWD) == 0) {
		ap_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_mode(WIFI_MODE_AP) , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_mode(WIFI_MODE_AP) , Failed\n");
		return 0;
	}

	esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

	err = esp_netif_dhcps_stop(ap_netif);
	if (err == ESP_OK) {
		logString("WIFI : esp_netif_dhcps_stop(ap_netif) , Success\n");
	} else {
		logString("WIFI : esp_netif_dhcps_stop(ap_netif) , Failed\n");
		return 0;
	}

	esp_netif_ip_info_t ip_info;
	IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
	IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
	IP4_ADDR(&ip_info.netmask, 255, 255, 0, 0);

	err = esp_netif_set_ip_info(ap_netif, &ip_info);
	if (err == ESP_OK) {
		logString("WIFI : sp_netif_set_ip_info , Success\n");
	} else {
		logString("WIFI : sp_netif_set_ip_info , Failed\n");
		return 0;
	}

	err = esp_netif_dhcps_start(ap_netif);
	if (err == ESP_OK) {
		logString("WIFI : esp_netif_dhcps_start(ap_netif) , Success\n");
	} else {
		logString("WIFI : esp_netif_dhcps_start(ap_netif) , Failed\n");
		return 0;
	}

	err = esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config);
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_AP) , Success\n");
	} else {
		logString("WIFI : esp_wifi_set_config(ESP_IF_WIFI_AP) , Failed\n");
		return 0;
	}

	err = esp_wifi_start();
	if (err == ESP_OK) {
		logString("WIFI : esp_wifi_start() , Success\n");
	} else {
		logString("WIFI : esp_wifi_start() , Failed\n");
		return 0;
	}
	return 1;
}
#endif

uint8_t initWifi() {
	if (initialise_wifi()) {
		logString("WIFI : initialise_wifi , Success\n");
	} else {
		logString("WIFI : initialise_wifi , Failed\n");
		return 0;
	}

#if FC_WIFI_MODE == FC_WIFI_MODE_STA
	 if (wifi_sta(FC_ESP_WIFI_STA_CONN_TIMEOUT * 1000)) {
	 logString("WIFI : wifi_sta , Success\n");
	 } else {
	 logString("WIFI : wifi_sta , Failed\n");
	 return 0;
	 }
#endif

#if FC_WIFI_MODE == FC_WIFI_MODE_APSTA
	 if (wifi_apsta(FC_ESP_WIFI_AP_CONN_TIMEOUT * 1000)) {
	 logString("WIFI : wifi_apsta , Success\n");
	 } else {
	 logString("WIFI : wifi_apsta , Failed\n");
	 return 0;
	 }
#endif

#if FC_WIFI_MODE == FC_WIFI_MODE_AP
	if (wifi_ap()) {
		logString("WIFI : wifi_ap , Success\n");
	} else {
		logString("WIFI : wifi_ap , Failed\n");
		return 0;
	}
#endif
	return 1;
}





