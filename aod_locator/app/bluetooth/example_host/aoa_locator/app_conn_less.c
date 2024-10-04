/***************************************************************************//**
 * @file
 * @brief Connectionless AoA locator application.
 *
 * AoA locator application for connection less implementation.
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
#include "app_log.h"
#include "app_assert.h"


#include "conn.h"
#include "app.h"
#include "aoa_util.h"
#include "app_config.h"

// UUID defined by Bluetooth SIG
static const uint8_t cte_service[] = { 0x4A, 0x18 };

// Antenna switching pattern
static const uint8_t antenna_array[AOA_NUM_ARRAY_ELEMENTS] = SWITCHING_PATTERN;
// static uint8_t antenna_array[33]={0} ;



/**************************************************************************//**
 * Connection specific Bluetooth event handler.
 *****************************************************************************/
void app_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
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

      // Start scanning - looking for tags
      sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_observation);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to start scanner\n",
                 (int)sc);

      app_log("Start scanning...\n");
      break;

    case sl_bt_evt_scanner_scan_report_id:
    {
      // Check if the tag is allowlisted
      if (SL_STATUS_NOT_FOUND == aoa_allowlist_find(evt->data.evt_scanner_scan_report.address.addr)) {
        if (verbose_level > 0 ) {
          app_log("Tag is not on the allowlist, ignoring.\n");
        }
        break;
      }
      // Parse extended advertisement packets
      if (evt->data.evt_scanner_scan_report.packet_type & 0x80) {
        // If a CTE service is found...
        uint16_t sync_handle;
        conn_properties_t *tag;

        if (find_service_in_advertisement(evt->data.evt_scanner_scan_report.data.data,
                                          evt->data.evt_scanner_scan_report.data.len,
                                          cte_service,
                                          sizeof(cte_service))) {
          // ...then sync on the periodic advertisement
          sc = sl_bt_sync_open(evt->data.evt_scanner_scan_report.address,
                               evt->data.evt_scanner_scan_report.address_type,
                               evt->data.evt_scanner_scan_report.adv_sid,
                               &sync_handle);
          app_assert(sc == SL_STATUS_OK,
                     "[E: 0x%04x] Failed to synchronize to tag\n",
                     (int)sc);
          tag = get_connection_by_handle(sync_handle);
          if (tag == NULL) {
            add_connection(sync_handle,
                           &evt->data.evt_scanner_scan_report.address,
                           evt->data.evt_scanner_scan_report.address_type);
          }
        }
      }
      break;
    }
    case sl_bt_evt_sync_opened_id:

      // Stop scanning
      sc = sl_bt_scanner_stop();
      app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                 "[E: 0x%04x] Failed to stop scanning\n",
                 (int)sc);

      app_log("Synced on tag\n");

      sc = sl_bt_cte_receiver_enable_connectionless_cte(evt->data.evt_sync_opened.sync,
                                                        CTE_SLOT_DURATION,
                                                        CTE_COUNT,
                                                        sizeof(antenna_array),
                                                        antenna_array);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to enable CTE\n",
                 (int)sc);
      // Start scanning again for new devices
      sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
      app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                 "[E: 0x%04x] Failed to start scanning\n",
                 (int)sc);
      break;

    case sl_bt_evt_sync_closed_id:
      app_log("Sync lost\n");
      remove_connection(evt->data.evt_cte_receiver_connectionless_iq_report.sync);
      // start scanning again to find new devices
      sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
      app_assert(sc == SL_STATUS_OK || sc == SL_STATUS_INVALID_STATE,
                 "[E: 0x%04x] Failed to start scanning\n",
                 (int)sc);
      break;

    case sl_bt_evt_cte_receiver_connectionless_iq_report_id:
    {
      conn_properties_t *tag;
      aoa_iq_report_t iq_report;

      if (evt->data.evt_cte_receiver_connectionless_iq_report.samples.len == 0) {
        // Nothing to be processed.
        break;
      }

      // Check if asset tag is known.
      tag = get_connection_by_handle(evt->data.evt_cte_receiver_connectionless_iq_report.sync);
      if (tag == NULL) {
        app_log("Unkown tag.\n");
        break;
      }

      // Convert event to common IQ report format.
      iq_report.channel = evt->data.evt_cte_receiver_connectionless_iq_report.channel;
      iq_report.rssi = evt->data.evt_cte_receiver_connectionless_iq_report.rssi;
      iq_report.event_counter = evt->data.evt_cte_receiver_connectionless_iq_report.event_counter;
      iq_report.length = evt->data.evt_cte_receiver_connectionless_iq_report.samples.len;
      iq_report.samples = (int8_t *)evt->data.evt_cte_receiver_connectionless_iq_report.samples.data;

      app_on_iq_report(tag, &iq_report);
    }
    break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
