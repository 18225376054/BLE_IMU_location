/***************************************************************************//**
 * @file
 * @brief Silabs AoA locator application.
 *
 * AoA locator application for Silicon labs proprietary implementation.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include "system.h"
#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "sl_ncp_evt_filter_common.h"
#include "app_log.h"
#include "app_assert.h"
#include "uart.h"
#include "app.h"
#include "conn.h"
#include "aoa_util.h"
#include "app_config.h"

// Antenna switching pattern
static const uint8_t antenna_array[AOA_NUM_ARRAY_ELEMENTS] = SWITCHING_PATTERN;

/**************************************************************************//**
 * Connection specific Bluetooth event handler.
 *****************************************************************************/
void app_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  uint8_t user_data[SL_NCP_EVT_FILTER_CMD_ADD_LEN];
  uint32_t event;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Config the NCP on the target.
      // Filter out the scan response event
      user_data[0] = SL_NCP_EVT_FILTER_CMD_ADD_ID;
      event = sl_bt_evt_scanner_scan_report_id;
      memcpy(&user_data[1], &event, SL_NCP_EVT_FILTER_CMD_ADD_LEN - 1);

      sc = sl_bt_user_manage_event_filter(SL_NCP_EVT_FILTER_CMD_ADD_LEN,
                                          user_data);

      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to enable filtering on the target\n",
                 (int)sc);

      // Set passive scanning on 1Mb PHY
      sc = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, SCAN_PASSIVE);

      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to set scanner mode\n",
                 (int)sc);

      // Set scan interval and scan window
      sc = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to set scanner timing\n",
                 (int)sc);

      // Start scanning - looking for locators (AoD)
      sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_observation);

      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to start scanner\n",
                 (int)sc);

      app_log("Start scanning...\n\n\n");

      // Start Silabs CTE
      sc = sl_bt_cte_receiver_enable_silabs_cte(CTE_SLOT_DURATION,
                                                CTE_COUNT,
                                                sizeof(antenna_array),
                                                antenna_array);

      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to enable Silabs CTE\n",
                 (int)sc);
      break;

    case sl_bt_evt_cte_receiver_silabs_iq_report_id:
    {
      conn_properties_t *locator;
      aoa_iq_report_t iq_report;

#if 0
      app_log("\r\n\n");
      app_log("APP_MODE = silabs \r\n");
      app_log("Address 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x \r\n", evt->data.evt_cte_receiver_silabs_iq_report.address.addr[5],
                    evt->data.evt_cte_receiver_silabs_iq_report.address.addr[4],
                    evt->data.evt_cte_receiver_silabs_iq_report.address.addr[3],
                    evt->data.evt_cte_receiver_silabs_iq_report.address.addr[2],
                    evt->data.evt_cte_receiver_silabs_iq_report.address.addr[1],
                    evt->data.evt_cte_receiver_silabs_iq_report.address.addr[0]);
      app_log("RSSI: %d \r\n", evt->data.evt_cte_receiver_silabs_iq_report.rssi);
      app_log("IQ Sample: length %d \r\n", evt->data.evt_cte_receiver_silabs_iq_report.samples.len);
      for(int x=0;x<evt->data.evt_cte_receiver_silabs_iq_report.samples.len;x++)
      {
        app_log("%d ", evt->data.evt_cte_receiver_silabs_iq_report.samples.data[x]);
        // app_log("%d ", (int8_t)evt->data.evt_cte_receiver_silabs_iq_report.samples.data[x]);
      }
      app_log("\r\n");
#endif

      if (evt->data.evt_cte_receiver_silabs_iq_report.samples.len == 0) {
        // Nothing to be processed.
        break;
      }

#if 0 //Cheng
      // Check if the tag is allowlisted.
      if (SL_STATUS_NOT_FOUND == aoa_allowlist_find(evt->data.evt_cte_receiver_silabs_iq_report.address.addr)) {
        if (verbose_level > 0 ) {
          app_log("Tag is not on the allowlist, ignoring.\n");
        }
        break;
      }
#endif

      // Check if the locator is whitelisted.
      aoa_id_t local_id;
      aoa_address_to_id(evt->data.evt_cte_receiver_silabs_iq_report.address.addr,
                        evt->data.evt_cte_receiver_silabs_iq_report.address_type,
                        local_id);
      if (SL_STATUS_NOT_FOUND == aoa_allowlist_find(local_id)) {
        if (verbose_level > 0 ) {
          app_log("locator is not on the whitelist, ignoring.\n");
        }
        break;
      }

      // Look for this locator.
      locator = get_connection_by_address(&evt->data.evt_cte_receiver_silabs_iq_report.address);
      // Check if it is a new locator
      if (locator == NULL) {
        // Connection handle parameter unused.
        locator = add_connection(0,
                             &evt->data.evt_cte_receiver_silabs_iq_report.address,
                             evt->data.evt_cte_receiver_silabs_iq_report.address_type);
        // Check if we have enough space for hte new locator.
        if (locator == NULL) {
          app_log("Too many locators in the system.\n");
          // Don't continue the process. This will save us CPU time.
          break;
        }
      }

      // Convert event to common IQ report format.
      iq_report.channel = evt->data.evt_cte_receiver_silabs_iq_report.channel;
      iq_report.rssi = evt->data.evt_cte_receiver_silabs_iq_report.rssi;
      iq_report.event_counter = evt->data.evt_cte_receiver_silabs_iq_report.packet_counter;
      iq_report.length = evt->data.evt_cte_receiver_silabs_iq_report.samples.len;
      iq_report.samples = (int8_t *)evt->data.evt_cte_receiver_silabs_iq_report.samples.data;

      app_on_iq_report(locator, &iq_report);
    }
    break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
