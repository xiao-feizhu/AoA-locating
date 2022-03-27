/***************************************************************************//**
 * @file
 * @brief AoA multilocator.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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

#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include "app_log.h"
#include "app_assert.h"
#include "mqtt.h"
#include "aoa_util.h"
#include "aoa_config.h"
#include "aoa_parse.h"
#include "sl_rtl_clib_api.h"
#include "app_config.h"
#include "app.h"

// Check if the configuration is valid
#if MAX_NUM_SEQUENCE_IDS > MAX_SEQUENCE_DIFF
#warning MAX_NUM_SEQUENCE_IDS > MAX_SEQUENCE_DIFF, please check the configuration.
#endif

// -----------------------------------------------------------------------------
// Private macros

#define INVALID_IDX              UINT32_MAX

#define CHECK_ERROR(x)           if ((x) != SL_RTL_ERROR_SUCCESS) return (x)

#define USAGE                    "\nUsage: %s -c <config> [-m <address>[:<port>]]\n"

#define ROUND_DIV(num, den)      (((num) + ((den) / 2)) / (den))

enum axis_list {
  AXIS_X,
  AXIS_Y,
  AXIS_Z,
  AXIS_COUNT
};

// -----------------------------------------------------------------------------
// Private types

typedef struct {
  aoa_id_t id;
  struct sl_rtl_loc_locator_item item;
} aoa_locator_t;

typedef struct {
  int32_t sequence;
  int32_t num_angles;
  aoa_angle_t angles[MAX_NUM_LOCATORS];
  bool has_angle[MAX_NUM_LOCATORS];
} aoa_correlated_angles_t;

typedef struct {
  aoa_id_t id;
  uint32_t loc_id[MAX_NUM_LOCATORS]; // assigned by RTL lib
  sl_rtl_loc_libitem loc;
  sl_rtl_util_libitem filter[AXIS_COUNT];
  aoa_correlated_angles_t correlated_angles[MAX_NUM_SEQUENCE_IDS];
  aoa_position_t position;
  int32_t oldest_sequence;
} aoa_asset_tag_t;

// -----------------------------------------------------------------------------
// Private variables

static mqtt_handle_t mqtt_handle = MQTT_DEFAULT_HANDLE;

static aoa_locator_t locator_list[MAX_NUM_LOCATORS];
static aoa_asset_tag_t asset_tag_list[MAX_NUM_TAGS];

static uint32_t locator_count = 0;
static uint32_t asset_tag_count = 0;

static aoa_id_t multilocator_id = "";

static uint32_t expected_angles_count[MAX_NUM_SEQUENCE_IDS];

// -----------------------------------------------------------------------------
// Private function declarations

static void parse_config(char *filename);
static void on_message(mqtt_handle_t *handle, const char *topic, const char *payload);
static void subscribe_angle(aoa_locator_t *loc);
static void publish_position(aoa_asset_tag_t *tag);
static enum sl_rtl_error_code run_estimation(aoa_asset_tag_t *tag, uint32_t slot);
static enum sl_rtl_error_code init_asset_tag(aoa_asset_tag_t *tag, aoa_id_t id);
static uint32_t find_asset_tag(aoa_id_t id);
static uint32_t find_locator(aoa_id_t id);
static void init_expected_angle_counts(void);
static void init_correlated_angle_data(aoa_correlated_angles_t* angle);
static void add_angle_data_to_tag(aoa_asset_tag_t* tag, uint32_t loc_idx, aoa_angle_t* angle);
static int32_t find_angle_slot(aoa_correlated_angles_t* slots, int32_t sequence);
static void update_slots(aoa_asset_tag_t* tag, aoa_angle_t* angle, int32_t slot, uint32_t loc_idx);
static void push_completed_angle_data(aoa_asset_tag_t* tag, int32_t check_idx_from);
static int32_t sequence_diff(uint32_t old_seq, uint32_t new_seq);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(int argc, char* argv[])
{
  mqtt_status_t rc;
  int opt;
  char *port_str = NULL;
  char *config_file = NULL;

  // Parse command line arguments.
  while ((opt = getopt(argc, argv, "c:m:h")) != -1) {
    switch (opt) {
      // Configuration file.
      case 'c':
        config_file = optarg;
        break;

      // MQTT broker connection parameters.
      case 'm':
        mqtt_handle.host = strtok(optarg, ":");
        port_str = strtok(NULL, ":");
        if (port_str != NULL) {
          mqtt_handle.port = atoi(port_str);
        }
        break;

      // Help.
      case 'h':
        app_log(USAGE, argv[0]);
        exit(EXIT_SUCCESS);

      // Illegal option.
      default:
        app_log(USAGE, argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  // Configuration file is mandatory.
  if (config_file == NULL) {
    app_log(USAGE, argv[0]);
    exit(EXIT_FAILURE);
  }

  parse_config(config_file);
  init_expected_angle_counts();

  mqtt_handle.on_message = on_message;
  mqtt_handle.client_id = multilocator_id;

  rc = mqtt_init(&mqtt_handle);
  app_assert(rc == MQTT_SUCCESS, "MQTT init failed.\n");

  for (uint32_t i = 0; i < locator_count; i++) {
    subscribe_angle(&locator_list[i]);
  }

  app_log("\nPress Crtl+C to quit\n\n");
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  mqtt_status_t rc;
  rc = mqtt_step(&mqtt_handle);
  app_assert(rc == MQTT_SUCCESS, "MQTT step failed.\n");
}

/**************************************************************************//**
 * Application Deinit.
 *****************************************************************************/
void app_deinit(void)
{
  mqtt_deinit(&mqtt_handle);
}

/**************************************************************************//**
 * Configuration file parser.
 *****************************************************************************/
static void parse_config(char *filename)
{
  sl_status_t sc;
  char *buffer;
  aoa_locator_t *loc;

  buffer = load_file(filename);
  app_assert(buffer != NULL, "Failed to load file: %s\n", filename);

  sc = aoa_parse_init(buffer);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_parse_init failed\n",
             (int)sc);

  sc = aoa_parse_multilocator(multilocator_id);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_parse_multilocator failed\n",
             (int)sc);

  do {
    loc = &locator_list[locator_count];
    sc = aoa_parse_locator(loc->id, &loc->item);
    if (sc == SL_STATUS_OK) {
      app_log("Locator added: id: %s, coordinate: %f %f %f, orientation: %f %f %f\n",
              loc->id,
              loc->item.coordinate_x,
              loc->item.coordinate_y,
              loc->item.coordinate_z,
              loc->item.orientation_x_axis_degrees,
              loc->item.orientation_y_axis_degrees,
              loc->item.orientation_z_axis_degrees);
      ++locator_count;
    } else {
      app_assert(sc == SL_STATUS_NOT_FOUND,
                 "[E: 0x%04x] aoa_parse_locator failed\n",
                 (int)sc);
    }
  } while ((locator_count < MAX_NUM_LOCATORS) && (sc == SL_STATUS_OK));

  app_log("Locator count: %d\n", locator_count);

  sc = aoa_parse_deinit();
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_parse_deinit failed\n",
             (int)sc);

  free(buffer);
}

/**************************************************************************//**
 * MQTT message arrived callback.
 *****************************************************************************/
static void on_message(mqtt_handle_t *handle, const char *topic, const char *payload)
{
  int result;
  aoa_id_t loc_id, tag_id;
  uint32_t loc_idx, tag_idx;
  aoa_asset_tag_t *tag;
  aoa_angle_t angle;
  enum sl_rtl_error_code sc;

  (void)handle;

  // Parse topic.
  result = sscanf(topic, AOA_TOPIC_ANGLE_SCAN, loc_id, tag_id);
  app_assert(result == 2, "Failed to parse angle topic: %d.\n", result);

  // Find locator.
  loc_idx = find_locator(loc_id);
  app_assert(loc_idx != INVALID_IDX, "Failed to find locator %s.\n", loc_id);

  // Find asset tag.
  tag_idx = find_asset_tag(tag_id);

  if (tag_idx == INVALID_IDX) {
    if (asset_tag_count < MAX_NUM_TAGS) {
      // Add new tag
      sc = init_asset_tag(&asset_tag_list[asset_tag_count], tag_id);
      app_assert(sc == SL_RTL_ERROR_SUCCESS,
                 "[E: 0x%04x] Failed to init asset tag %s.\n", sc, tag_id);
      app_log("New tag added (%d): %s\n", asset_tag_count, tag_id);
      tag_idx = asset_tag_count++;
    } else {
      app_log("Warning! Maximum number of asset tags reached: %d\n", asset_tag_count);
      // No further procesing possible.
      return;
    }
  }

  // Create shortcut.
  tag = &asset_tag_list[tag_idx];

  // Parse payload.
  aoa_string_to_angle((char *)payload, &angle);
  add_angle_data_to_tag(tag, loc_idx, &angle);
}

/**************************************************************************//**
 * Subscribe for angle data published by a given locator.
 *****************************************************************************/
static void subscribe_angle(aoa_locator_t *loc)
{
  const char topic_template[] = AOA_TOPIC_ANGLE_PRINT;
  char topic[sizeof(topic_template) + sizeof(aoa_id_t) + 1];
  mqtt_status_t rc;

  snprintf(topic, sizeof(topic), topic_template, loc->id, "#");

  app_log("Subscribing to topic '%s'.\n", topic);

  rc = mqtt_subscribe(&mqtt_handle, topic);
  app_assert(rc == MQTT_SUCCESS, "Failed to subscribe to topic '%s'.\n", topic);
}

/**************************************************************************//**
 * Publish position of a given tag.
 *****************************************************************************/
static void publish_position(aoa_asset_tag_t *tag)
{
  mqtt_status_t rc;
  char *payload;
  const char topic_template[] = AOA_TOPIC_POSITION_PRINT;
  char topic[sizeof(topic_template) + sizeof(aoa_id_t) + sizeof(aoa_id_t)];

  // Compile topic.
  snprintf(topic, sizeof(topic), topic_template, multilocator_id, tag->id);

  // Compile payload.
  aoa_position_to_string(&tag->position, &payload);

  rc = mqtt_publish(&mqtt_handle, topic, payload);
  app_assert(rc == MQTT_SUCCESS, "Failed to publish to topic '%s'.\n", topic);

  // Clean up.
  free(payload);
}

/**************************************************************************//**
 * Run position estimation algorithm for a given asset tag.
 *****************************************************************************/
static enum sl_rtl_error_code run_estimation(aoa_asset_tag_t *tag, uint32_t slot)
{
  enum sl_rtl_error_code sc;
  float time_step;

  // Feed measurement values into RTL lib.
  for (uint32_t i = 0; i < locator_count; i++) {
    if (tag->correlated_angles[slot].has_angle[i]) {
      sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                              tag->loc_id[i],
                                              SL_RTL_LOC_LOCATOR_MEASUREMENT_AZIMUTH,
                                              tag->correlated_angles[slot].angles[i].azimuth);
      CHECK_ERROR(sc);

      sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                              tag->loc_id[i],
                                              SL_RTL_LOC_LOCATOR_MEASUREMENT_ELEVATION,
                                              tag->correlated_angles[slot].angles[i].elevation);
      CHECK_ERROR(sc);

      // Feeding RSSI distance measurement to the RTL library improves location
      // accuracy when the measured distance is reasonably correct.
      // If the received signal strength of the incoming signal is altered for any
      // other reason than the distance between the TX and RX itself, it will lead
      // to incorrect measurement and it will lead to incorrect position estimates.
      // For this reason the RSSI distance usage is disabled by default in the
      // multilocator case.
      // Single locator mode however always requires the distance measurement in
      // addition to the angle, please note the if-condition below.
      // In case the distance estimation should be used in the  multilocator case,
      // you can enable it by commenting out the condition.
      if (locator_count == 1) {
        sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                                tag->loc_id[i],
                                                SL_RTL_LOC_LOCATOR_MEASUREMENT_DISTANCE,
                                                tag->correlated_angles[slot].angles[i].distance);
        CHECK_ERROR(sc);
      }
    }
  }

  // Estimate the time step based on the sequence number.
  if (sequence_diff(tag->correlated_angles[slot].sequence,
                    tag->oldest_sequence) == INT_MAX) {
    time_step = ESTIMATION_INTERVAL_SEC;
  } else {
    time_step = abs(sequence_diff(tag->correlated_angles[slot].sequence,
                                  tag->oldest_sequence)) * ESTIMATION_INTERVAL_SEC;
  }

  // Process new measurements, time step given in seconds.
  sc = sl_rtl_loc_process(&tag->loc, time_step);
  tag->oldest_sequence = tag->correlated_angles[slot].sequence;

  CHECK_ERROR(sc);

  // Get results from the estimator.
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_X, &tag->position.x);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_Y, &tag->position.y);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_Z, &tag->position.z);
  CHECK_ERROR(sc);

  // Apply filter on the result.
  sc = sl_rtl_util_filter(&tag->filter[AXIS_X], tag->position.x, &tag->position.x);
  CHECK_ERROR(sc);
  sc = sl_rtl_util_filter(&tag->filter[AXIS_Y], tag->position.y, &tag->position.y);
  CHECK_ERROR(sc);
  sc = sl_rtl_util_filter(&tag->filter[AXIS_Z], tag->position.z, &tag->position.z);
  CHECK_ERROR(sc);

  // Clear measurements.
  sc = sl_rtl_loc_clear_measurements(&tag->loc);
  CHECK_ERROR(sc);

  return SL_RTL_ERROR_SUCCESS;
}

/**************************************************************************//**
 * Initialise a new asset tag.
 *****************************************************************************/
static enum sl_rtl_error_code init_asset_tag(aoa_asset_tag_t *tag, aoa_id_t id)
{
  enum sl_rtl_error_code sc;

  aoa_id_copy(tag->id, id);

  // Initialize RTL library
  sc = sl_rtl_loc_init(&tag->loc);
  CHECK_ERROR(sc);

  // Select estimation mode.
  sc = sl_rtl_loc_set_mode(&tag->loc, ESTIMATION_MODE);
  CHECK_ERROR(sc);

  // Provide locator configurations to the position estimator.
  for (uint32_t i = 0; i < locator_count; i++) {
    sc = sl_rtl_loc_add_locator(&tag->loc, &locator_list[i].item, &tag->loc_id[i]);
    CHECK_ERROR(sc);
  }
  for (uint32_t i = 0; i < MAX_NUM_SEQUENCE_IDS; i++) {
    init_correlated_angle_data(&tag->correlated_angles[i]);
  }

  // Create position estimator.
  sc = sl_rtl_loc_create_position_estimator(&tag->loc);
  CHECK_ERROR(sc);

  // Initialize util functions.
  for (enum axis_list i = 0; i < AXIS_COUNT; i++) {
    sc = sl_rtl_util_init(&tag->filter[i]);
    CHECK_ERROR(sc);
    // Set position filtering parameter for every axis.
    sc = sl_rtl_util_set_parameter(&tag->filter[i],
                                   SL_RTL_UTIL_PARAMETER_AMOUNT_OF_FILTERING,
                                   FILTERING_AMOUNT);
    CHECK_ERROR(sc);
  }

  return SL_RTL_ERROR_SUCCESS;
}

/**************************************************************************//**
 * Find asset tag in the local list based on its ID.
 *****************************************************************************/
static uint32_t find_asset_tag(aoa_id_t id)
{
  uint32_t retval = INVALID_IDX;

  for (uint32_t i = 0; (i < asset_tag_count) && (retval == INVALID_IDX); i++) {
    if (aoa_id_compare(asset_tag_list[i].id, id) == 0) {
      retval = i;
    }
  }
  return retval;
}

/**************************************************************************//**
 * Find locator in the local list based on its ID.
 *****************************************************************************/
static uint32_t find_locator(aoa_id_t id)
{
  uint32_t retval = INVALID_IDX;

  for (uint32_t i = 0; (i < locator_count) && (retval == INVALID_IDX); i++) {
    if (aoa_id_compare(locator_list[i].id, id) == 0) {
      retval = i;
    }
  }
  return retval;
}

/**************************************************************************//**
 * Initialize expected angle counts.
 *
 * The expected number of angles depends on the slot index. A slot with smaller
 * index (i.e. with more recent sequence number) expects more angles, while a
 * slot with higher index (i.e. with older sequence number) requires less
 * angles.
 * This function implements a linear connection, thus the first slot expects
 * an angle from all locators, while the last slot needs only 2 locators.
 *****************************************************************************/
static void init_expected_angle_counts(void)
{
  uint32_t coeff = (locator_count < 2) ? 0 : (locator_count - 2);
  for (int i = 0; i < MAX_NUM_SEQUENCE_IDS; i++) {
    expected_angles_count[i] = locator_count - ROUND_DIV(i * coeff, MAX_NUM_SEQUENCE_IDS - 1);
  }
}

/**************************************************************************//**
 * Initialize angle data in a slot.
 *****************************************************************************/
static void init_correlated_angle_data(aoa_correlated_angles_t* angle)
{
  angle->num_angles = 0;
  angle->sequence = -1;
  memset(angle->has_angle, 0, MAX_NUM_LOCATORS);
}

/**************************************************************************//**
 * Add angle data to asset tag.
 *****************************************************************************/
static void add_angle_data_to_tag(aoa_asset_tag_t* tag, uint32_t loc_idx, aoa_angle_t* angle)
{
  // Drop stored data that are considered too old.
  for (int i = 0; i < MAX_NUM_SEQUENCE_IDS; i++) {
    if (sequence_diff(tag->correlated_angles[i].sequence, angle->sequence) == INT_MAX) {
      for (int k = i; k < MAX_NUM_SEQUENCE_IDS; k++) {
        init_correlated_angle_data(&tag->correlated_angles[k]);
      }
      break;
    }
  }
  // Find the correct slot.
  int32_t angle_slot = find_angle_slot(tag->correlated_angles, angle->sequence);
  // Update with / insert new data
  update_slots(tag, angle, angle_slot, loc_idx);
  // Push if some of the slots have completed
  push_completed_angle_data(tag, angle_slot);
}

/**************************************************************************//**
 * Find angle slot.
 *****************************************************************************/
static int32_t find_angle_slot(aoa_correlated_angles_t* slots, int32_t sequence)
{
  for (uint32_t i = 0; i < MAX_NUM_SEQUENCE_IDS; i++) {
    if (sequence == slots[i].sequence) {
      return i;
    }
  }
  return 0;
}

/**************************************************************************//**
 * Update slots.
 *****************************************************************************/
static void update_slots(aoa_asset_tag_t* tag, aoa_angle_t* angle, int32_t slot, uint32_t loc_idx)
{
  if (slot >= 0) {
    // If the selected slot has not matched with the angle then insert it.
    if (tag->correlated_angles[slot].sequence != angle->sequence) {
      for (int i = MAX_NUM_SEQUENCE_IDS - 1; i > slot; i--) {
        tag->correlated_angles[i] = tag->correlated_angles[i - 1];
      }
      init_correlated_angle_data(&tag->correlated_angles[slot]);
      tag->correlated_angles[slot].sequence = angle->sequence;
    }
    tag->correlated_angles[slot].angles[loc_idx] = *angle;
    tag->correlated_angles[slot].has_angle[loc_idx] = true;
    tag->correlated_angles[slot].num_angles++;
  }
}

/**************************************************************************//**
 * Push completed angle data to the estimatorand publish to MQTT.
 *****************************************************************************/
static void push_completed_angle_data(aoa_asset_tag_t* tag, int32_t check_idx_from)
{
  int32_t last_updated_index = MAX_NUM_SEQUENCE_IDS;
  // Check start from the oldest slots to keep in order and complete as many
  // slots as possible.
  if (check_idx_from >= 0) {
    for (int i = MAX_NUM_SEQUENCE_IDS - 1; i >= check_idx_from; i--) {
      if (tag->correlated_angles[i].num_angles == expected_angles_count[i]) {
        enum sl_rtl_error_code sc = run_estimation(tag, i);
        app_assert(sc == SL_RTL_ERROR_SUCCESS,
                   "[E: 0x%04x] Position estimation failed for %s.\n", sc, tag->id);
        publish_position(tag);
        tag->oldest_sequence = tag->correlated_angles[i].sequence;
        last_updated_index = i;
      }
    }
    for (; last_updated_index < MAX_NUM_SEQUENCE_IDS; last_updated_index++) {
      init_correlated_angle_data(&tag->correlated_angles[last_updated_index]);
    }
  }
}

/**************************************************************************//**
 * Calculate the difference of 2 sequence numbers.
 *
 * @note The function has unsigned arguments, thus invalid (negative) sequences
 *       will return INT_MAX.
 *****************************************************************************/
static int32_t sequence_diff(uint32_t old_seq, uint32_t new_seq)
{
  // Sequence is 16 bit long, so if the new sequence is smaller than the old
  // one, it might overflowed.
  int32_t result = old_seq <= new_seq ? new_seq - old_seq : -(65536 + new_seq - old_seq);
  if (abs(result) > MAX_SEQUENCE_DIFF) {
    result = INT_MAX;
  }
  return result;
}
