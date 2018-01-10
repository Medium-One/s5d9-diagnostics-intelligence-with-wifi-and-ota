/***********************************************************************************************************************
 * Copyright [2015] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
 *
 * The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
 * and/or its licensors ("Renesas") and subject to statutory and contractual protections.
 *
 * Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
 * display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
 * purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
 * SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
 * INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
 * be subject to different terms.
 **********************************************************************************************************************/
#include "system_thread.h"
#include "fx_stdio.h"
#include "qspi_filex_io_driver.h"
#include "r_qspi_subsector_erase.h"
#include "nxd_dns.h"
#include "nxd_dhcp_client.h"
#include "led.h"
#include "helpers.h"


extern TX_THREAD net_thread;
extern app_config_t app_config;


void SetMacAddress(nx_mac_address_t *p_mac_config);
int network_setup(FX_MEDIA * pMedia);
void R_BSP_WarmStart (bsp_warm_start_event_t event);
ssp_err_t R_IOPORT_PinCfg (ioport_port_pin_t pin, uint32_t cfg);

static UCHAR    g_qspi_mem[QSPI_SUB_SECTOR_SIZE] BSP_ALIGN_VARIABLE(8);
FX_MEDIA g_qspi_media;
char g_link_code[7];

#define IP_STACK_SIZE 2048
#define ARP_SIZE 512
#define LINK_UP_TIMEOUT 500
#define DHCP_RESOLVE_TIMEOUT 1000
#define WIFI_ASSOCIATE_TIMEOUT 1000

NX_IP g_http_ip;
static CHAR mem_ip_stack[IP_STACK_SIZE]  __attribute__ ((aligned(4)));
static char mem_arp[ARP_SIZE]  __attribute__ ((aligned(4)));
NX_DNS g_dns_client;
NX_DHCP g_dhcp;
VOID nx_ether_driver_eth0(NX_IP_DRIVER *);
VOID g_sf_el_nx0(NX_IP_DRIVER * p_driver);
int qcom_set_tx_power(uint8_t device_id, uint32_t dbm);
static sf_wifi_provisioning_t prov;


void R_BSP_WarmStart (bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_PRE_C == event)
    {
        /* C runtime environment has not been setup so you cannot use globals. System clocks and pins are not setup. */

    }
    else if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment, system clocks, and pins are all setup. */
        /*
         * Don't let USB enumerate until the drive is initialized with all information available
         *
         */
        ssp_feature_t       ssp_feature = {{(ssp_ip_t) 0U}};
        ssp_feature.unit = (uint32_t) SSP_IP_UNIT_USBFS;
        R_BSP_ModuleStop(&ssp_feature);
        R_IOPORT_PinCfg(IOPORT_PORT_04_PIN_07,IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_NMOS_ENABLE);
    }
    else
    {
        /* Do nothing */
    }
}

void SetMacAddress(nx_mac_address_t *p_mac_config)
{
    //  REA's Vendor MAC range: 00:30:55:xx:xx:xx
    fmi_unique_id_t id;
    g_fmi.p_api->uniqueIdGet(&id);

    p_mac_config->nx_mac_address_h = 0x0030;
    p_mac_config->nx_mac_address_l = ((0x55000000) | (id.unique_id[0] &(0x00FFFFFF)));

}

static int parse_wifi_credentials(char * ssid, size_t ssid_size, char * passkey, size_t pass_size) {
    FILE * file;
    ULONG status;
    int ret = 0;

    file = fopen("wifi.txt", "r");
    if (file == NULL)
        return -3;
    if (get_line(file, ssid, ssid_size, 0)) {
            ret = -1;
            goto end;
    }
    if (get_line(file, passkey, pass_size, 1)) {
            ret = -2;
            goto end;
    }

end:
    fclose(file);
    return ret;
}

static void wifi_change(sf_wifi_callback_args_t * p_args) {
    switch (p_args->event == SF_WIFI_EVENT_AP_DISCONNECT) {
        case SF_WIFI_EVENT_AP_DISCONNECT:
//            tx_semaphore_ceiling_put(&g_wifi_changed, 1);
            set_led(4, 1);
            break;
        case SF_WIFI_EVENT_AP_CONNECT:
            set_led(4, 0);
            break;
    }
}

/*
 * extracts credentials from the file "m1config.txt".
 * the expected structure is:
 *      <api key>
 *      <mqtt project id>
 *      <mqtt registration user id> <- optional
 *      <registration user password> <- required if <mqtt registration user id> present
 * Returns:
 *      - -1 if "m1config.txt" does not exist
 *      - -2 if the first line cannot be read or encounters EOF
 *      - -3 if the second line cannot be read or encounters EOF
 *      - -4 if the third line cannot be read or encounters EOF
 *      - -5 if the last line cannot be read
 */
static int extract_credentials_from_config(project_credentials_t * project, user_credentials_t * user) {
    int ret = 0;
    FILE * config_file = fopen("m1config.txt", "r");
    if (!config_file)
        return -1;
    if (get_line(config_file, project->apikey, sizeof(project->apikey), 0)) {
        ret = -2;
        goto end;
    }
    if (get_line(config_file, project->proj_id, sizeof(project->proj_id), 1)) {
        ret = -3;
        goto end;
    }
    if (get_line(config_file, user->user_id, sizeof(user->user_id), 0)) {
        ret = -4;
        goto end;
    }
    if (get_line(config_file, user->password, sizeof(user->password), 1)) {
        ret = -5;
        goto end;
    }
end:
    fclose(config_file);
    return ret;
}

static void cfprint(FILE * fp_file, const char * str) {
    fwrite(str, strlen(str), 1, fp_file);
}

int network_setup(FX_MEDIA * pMedia)
{
    FX_FILE file;
    CHAR * interface_name;
    ULONG  status, actual_status_bits;
    ULONG dns_ip[3];
    ULONG dns_ip_size;
    ULONG ip_address;
    ULONG ip_mask;
    ULONG mtu_size;
    ULONG mac_high;
    ULONG mac_low;
    char msg[80];
    NX_IP * p_nx_ip_connection = NULL;


    status = nx_ip_create(&g_http_ip, "HTTP IP Instance",
                          IP_ADDRESS(0,0,0,0), IP_ADDRESS(255, 255, 255, 0),
                          &g_http_packet_pool, nx_ether_driver_eth0,
                          mem_ip_stack, sizeof(mem_ip_stack), 2);
    /** Wait for init to finish. ??*/
    status = nx_ip_interface_status_check(&g_http_ip, 0, NX_IP_LINK_ENABLED, &actual_status_bits, 1000);
    if (status == NX_SUCCESS) {
        status = nx_ip_fragment_enable(&g_http_ip);

        status = nx_arp_enable(&g_http_ip, mem_arp, sizeof(mem_arp));

        status = nx_tcp_enable(&g_http_ip);

        status = nx_udp_enable(&g_http_ip);

        status = nx_icmp_enable(&g_http_ip);

        /* Wait till the IP thread and driver have initialized the system. */
        // TODO: this always returns success...
        status = nx_ip_status_check(&g_http_ip, NX_IP_LINK_ENABLED, &actual_status_bits, LINK_UP_TIMEOUT);

        if (status == NX_SUCCESS) {
            // enter ethernet mode
            status = nx_dhcp_create(&g_dhcp, &g_http_ip, "dhcp");
            status = nx_dhcp_start(&g_dhcp);
            status = nx_ip_status_check(&g_http_ip, NX_IP_ADDRESS_RESOLVED, &actual_status_bits, DHCP_RESOLVE_TIMEOUT);
            if (status != NX_SUCCESS) {
                status = nx_dhcp_stop(&g_dhcp);
                status = nx_dhcp_delete(&g_dhcp);
            } else {
                p_nx_ip_connection = &g_http_ip;
            }
        }
    }
    ioport_level_t wifi_detect;
    R_IOPORT_PinCfg(IOPORT_PORT_07_PIN_08, (IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW));
    tx_thread_sleep(1);
    R_IOPORT_PinCfg(IOPORT_PORT_07_PIN_08, IOPORT_CFG_PORT_DIRECTION_INPUT);
    tx_thread_sleep(1);
    R_IOPORT_PinRead(IOPORT_PORT_07_PIN_08, &wifi_detect);
    R_IOPORT_PinCfg(IOPORT_PORT_07_PIN_08, IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_IRQ_ENABLE | IOPORT_CFG_PULLUP_ENABLE);

    if ((p_nx_ip_connection == NULL) && (wifi_detect == IOPORT_LEVEL_HIGH)) {
        // enter wifi mode
        status = nx_ip_delete(&g_http_ip);
        status = nx_ip_create(&g_http_ip, "HTTP IP Instance",
                              IP_ADDRESS(0,0,0,0), IP_ADDRESS(255, 255, 255, 0),
                              &g_http_packet_pool, g_sf_el_nx0,
                              mem_ip_stack, sizeof(mem_ip_stack), 2);
        /** Wait for init to finish. */
        status = nx_ip_interface_status_check(&g_http_ip, 0, NX_IP_LINK_ENABLED, &actual_status_bits, 100);

        status = nx_ip_fragment_enable(&g_http_ip);

        status = nx_arp_enable(&g_http_ip, mem_arp, sizeof(mem_arp));

        status = nx_tcp_enable(&g_http_ip);

        status = nx_udp_enable(&g_http_ip);

        status = nx_icmp_enable(&g_http_ip);

        status = nx_ip_status_check(&g_http_ip, NX_IP_LINK_ENABLED, &actual_status_bits, LINK_UP_TIMEOUT);
        if ((status == NX_SUCCESS) && !parse_wifi_credentials(prov.ssid, sizeof(prov.ssid), prov.key, sizeof(prov.key))) {
            p_nx_ip_connection = &g_http_ip;
            prov.mode = SF_WIFI_INTERFACE_MODE_CLIENT;
            prov.encryption = SF_WIFI_ENCRYPTION_TYPE_AUTO;
            prov.security = SF_WIFI_SECURITY_TYPE_WPA2;
            prov.p_callback = wifi_change;
            ULONG start = tx_time_get();
            do {
                status = g_sf_wifi0.p_api->provisioningSet(g_sf_wifi0.p_ctrl, &prov);
            } while ((status != SSP_SUCCESS) && ((tx_time_get() - start)  < WIFI_ASSOCIATE_TIMEOUT));
            if (status != SSP_SUCCESS)
                p_nx_ip_connection = NULL;
            else {
                status = nx_dhcp_create(&g_dhcp, &g_http_ip, "dhcp");
                status = nx_dhcp_start(&g_dhcp);
                do {
                    status = nx_ip_status_check(&g_http_ip, NX_IP_ADDRESS_RESOLVED, &actual_status_bits, NX_WAIT_FOREVER);
                } while (actual_status_bits != NX_IP_ADDRESS_RESOLVED);
            }
        }
    }
    if (p_nx_ip_connection == NULL) {
       // no working network connections, die
       return -1;
    }

    status = nx_dns_create(&g_dns_client, &g_http_ip, (UCHAR *)"Netx DNS");
    if (status) __BKPT();

    status = nx_ip_address_get(&g_http_ip, &ip_address, &ip_mask );
    if (status) __BKPT();

    fx_file_delete(pMedia, "ipconfig.txt");
    status = fx_media_flush(pMedia);
    if (status)
           __BKPT();

    status = fx_file_create(pMedia, "ipconfig.txt");
    if (status) __BKPT();

    status = fx_file_open(pMedia, &file, "ipconfig.txt", FX_OPEN_FOR_WRITE);
    if (status) __BKPT();

    sprintf(msg, "IP Address: %d.%d.%d.%d\r\n", (int)(ip_address>>24), (int)(ip_address>>16)&0xFF, (int)(ip_address>>8)&0xFF, (int)(ip_address)&0xFF );
    status = fx_file_write(&file, msg, strlen(msg));
    if (status) __BKPT();

    sprintf(msg, "IP Mask: %d.%d.%d.%d\r\n", (int)(ip_mask>>24), (int)(ip_mask>>16)&0xFF, (int)(ip_mask>>8)&0xFF, (int)(ip_mask)&0xFF );
    status = fx_file_write(&file, msg, strlen(msg));
    if (status) __BKPT();

    dns_ip_size = sizeof(dns_ip);
    status = nx_dhcp_user_option_retrieve(&g_dhcp, NX_DHCP_OPTION_DNS_SVR, (UCHAR *)dns_ip, (UINT *)&dns_ip_size);
    if (status == NX_DHCP_DEST_TO_SMALL) {
        dns_ip[0] = 0x08080808UL;
        status = nx_dns_server_add(&g_dns_client, dns_ip[0]);
        if ((status != NX_SUCCESS) && (status != NX_DNS_DUPLICATE_ENTRY)) __BKPT();

        sprintf((char *)msg, "DNS Address 1: %d.%d.%d.%d\r\n", (int)(dns_ip[0]>>24), (int)(dns_ip[0]>>16)&0xFF, (int)(dns_ip[0]>>8)&0xFF, (int)(dns_ip[0])&0xFF );

        status = fx_file_write(&file, msg, strlen(msg));
        if (status) __BKPT();
    } else {
        if (status) __BKPT();

        status = nx_dns_server_add(&g_dns_client, dns_ip[0]);
        if (status) __BKPT();

        sprintf((char *)msg, "DNS Address 1: %d.%d.%d.%d\r\n", (int)(dns_ip[0]>>24), (int)(dns_ip[0]>>16)&0xFF, (int)(dns_ip[0]>>8)&0xFF, (int)(dns_ip[0])&0xFF );

        status = fx_file_write(&file, msg, strlen(msg));
        if (status) __BKPT();

        if (dns_ip_size > 4)
        {
            status = nx_dns_server_add(&g_dns_client, dns_ip[1]);
            if (status) __BKPT();

            sprintf((char *)msg, "DNS Address 2: %d.%d.%d.%d\r\n", (int)(dns_ip[1]>>24), (int)(dns_ip[1]>>16)&0xFF, (int)(dns_ip[1]>>8)&0xFF, (int)(dns_ip[1])&0xFF );
            status = fx_file_write(&file, msg, strlen(msg));
            if (status) __BKPT();
        }
    }

    status = nx_ip_interface_info_get(&g_http_ip, 0, &interface_name, &ip_address, &ip_mask, &mtu_size, &mac_high, &mac_low);
    if (status) __BKPT();

    sprintf(msg, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",(int) (mac_high>>8), (int) (mac_high&0xFF), (int) (mac_low>>24), (int) (mac_low>>16&0xFF),(int) (mac_low>>8&0xFF),(int) (mac_low&0xFF));
    status = fx_file_write(&file, msg, strlen(msg));
    if (status) __BKPT();

    status = fx_file_close(&file);
    if (status)
           __BKPT();
    status = fx_media_flush(pMedia);
    if (status)
           __BKPT();

    return 0;
}
void  usb_init(void);
/* System Thread entry function */
void system_thread_entry(void)
{
    UINT status;
    ULONG actual_status_bits;

    status = fx_media_open(&g_qspi_media, (CHAR *) "QSPI Media", _fx_qspi_driver, (void *) &g_qspi, g_qspi_mem, QSPI_SUB_SECTOR_SIZE);
    if (FX_SUCCESS == status)
    {
        _fx_stdio_initialize();
        _fx_stdio_drive_register('A', &g_qspi_media);
    }
    if (network_setup(&g_qspi_media))
        set_led(4,  1);

    g_sce_0.p_api->open(g_sce_0.p_ctrl, g_sce_0.p_cfg);
    g_sce_trng.p_api->open(g_sce_trng.p_ctrl, g_sce_trng.p_cfg);
    uint32_t rand = 0;
    g_sce_trng.p_api->read(g_sce_trng.p_ctrl, &rand, 1);
    g_sce_trng.p_api->close(g_sce_trng.p_ctrl);
    g_sce_0.p_api->close(g_sce_0.p_ctrl);
    FILE * f = fopen("link_code.txt", "w");
    for (int i = 0; i < 3; i++) {
        sprintf(g_link_code + (i * 2), "%02X", ((uint8_t *)&rand)[i]);
    }
    fwrite(g_link_code, 7, 1, f);
    fclose(f);
    fx_media_flush(&g_qspi_media);

    memset(&app_config, 0, sizeof(app_config));

    // credentials from m1config.txt
    extract_credentials_from_config(&app_config.project, &app_config.registration);

    FILE * f_manual_creds;

    // optional override via m1user.txt
    if ((f_manual_creds = fopen("m1user.txt", "r"))) {
        if (!get_line(f_manual_creds, app_config.device.user_id, sizeof(app_config.device.user_id), 0))
            get_line(f_manual_creds, app_config.device.password, sizeof(app_config.device.password), 1);
        fclose(f_manual_creds);
    }

    fx_media_flush(&g_qspi_media);

    // no more QSPI activity

    tx_thread_resume(&net_thread);

    /*
     * Make the drive available to the host by enabling enumeration on VBUS
     */
    R_IOPORT_PinCfg(IOPORT_PORT_04_PIN_07,(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS));

    while (1)
    {
        tx_semaphore_get(&g_usb_qspi_active, 200);
        fx_media_flush(&g_qspi_media);
        status = tx_semaphore_get(&g_wifi_changed, 1);
#if 0
        if (status == TX_SUCCESS) {
            do {
                status = g_sf_wifi0.p_api->provisioningSet(g_sf_wifi0.p_ctrl, &prov);
            } while (status != SSP_SUCCESS);
            status = nx_dhcp_force_renew(&g_dhcp);
            do {
                status = nx_ip_status_check(&g_http_ip, NX_IP_ADDRESS_RESOLVED, &actual_status_bits, NX_WAIT_FOREVER);
            } while (actual_status_bits != NX_IP_ADDRESS_RESOLVED);
        }
#endif
    }
}
