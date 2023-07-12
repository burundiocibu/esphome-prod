/*
  hre.hpp: Badger HR-E Water Meter Encoder interface

  Copyright (C) 2022  Jon Little

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*********************************************************************************************\
 * HR-E LCD Water meter register interface
 *
 * https://www.badgermeter.com/business-lines/utility/high-resolution-lcd-encoders-hr-e-lcd/
 * Source: Jon Little, https://github.com/burundiocibu/particle/blob/master/water_meter/src/HRE_Reader.cpp
 *
 * This code marches the bits out the data line as ASCII characters with the form
 *    KG44?Q45484=0444444V;RB000000022;IB018435683
 * where the RB...; is the miligalons used
 *
 * Note that this sensor takes a _long_ time to read. 62 bits * 4 ms/bit for the
 * sync sequence plus 46 bytes * 40 ms/byte = 2088 ms minimum. If we aren't alligned
 * to the sync sequence, it could be almost twice that.
 * To keep from bogging the kernel down, we read 8 bits at a time on the 50 ms callback.
 * It will take seconds to discover if the device is there.
 *
 * In lieu of an actual schematic to describe the electrical interface, here is a description:
 *
 * hre_clock_pin: drives the power/clock for the water meter through a 1k resister to
 *   the base of a pnp transistor
 * hre_data_pin: is the data and has a 1 k pulldown
 *
 * The pnp transitor has the collector connected to the power/clock and is pulled up
 * to +5 via a 1 k resistor.
 * The emitter is connected to ground
 *
 * This code was originally included in the most excelent Tasmota project.
 * Subsequently the author ported it to a ESPHome Custom Sensor Component.
 *
\*********************************************************************************************/

#include "esphome.h"

static const char *TAG = "hre"; // log tag

class HRE : public esphome::PollingComponent
{
   public:
   enum hre_states
   {
      hre_sync,    // Start search for sync sequence
      hre_syncing, // Searching for sync sequence
      hre_read,    // Start reading data block
      hre_reading, // Reading data
      hre_sleep,   // Start sleeping
      hre_sleeping // pausing before reading again
   };

   HRE(uint8_t _clock, uint8_t _data) :
      esphome::PollingComponent(15000),
      clock(_clock), data(_data)
   {};

   esphome::sensor::Sensor *usage_sensor = new Sensor();
   esphome::sensor::Sensor *rate_sensor = new Sensor();

   void setup() override
   {
      ESP_LOGD(TAG, "Setting up...");
      hre_read_errors = 0;
      hre_good = false;

      pinMode(clock, OUTPUT);
      pinMode(data, INPUT);

      // Note that the level shifter inverts this line and we want to leave it
      // high when not being read.
      digitalWrite(clock, LOW);

      hre_state = hre_sync;
   }

   void loop() override
   {
      hreEvery50ms();
   }

   void update() override
   {
      if (!hre_good)
      {
         ESP_LOGD(TAG, "hre not good...");
         return;
      }

      ESP_LOGD(TAG, "hre good, updating %f", uptime());
      usage_sensor->publish_state(hre_usage);
      rate_sensor->publish_state(hre_rate);
   }

   private:

   uint8_t clock,data;
   hre_states hre_state = hre_sync;

   float hre_usage = 0; // total water usage, in gal
   float hre_rate = 0;  // flow rate, in gal/min
   float hre_usage_time = 0; // uptime associated with hre_usage and hre_rate

   int hre_read_errors = 0; // total number of read errors since boot
   bool hre_good =  false;

   // returns uptime in seconds
   float uptime()
   {
      static uint32_t last_millis = 0;
      static uint32_t wraps = 0;
      uint32_t ms = millis();
      if (ms < last_millis)
         wraps++;
      return ms / 1000.0 + wraps * 4294967.0; // 4294967 = 2^32/1000
   }

   // The settling times here were determined using a single unit hooked to a scope
   int hreReadBit()
   {
      digitalWrite(clock, HIGH);
      delay(1);
      int bit = digitalRead(data);
      digitalWrite(clock, LOW);
      delay(1);
      return bit;
   }

   // With the times in the HreReadBit routine, a characer will take
   // 20 ms plus io time.
   char hreReadChar(int &parity_errors)
   {
      // start bit
      hreReadBit();

      unsigned ch=0;
      int sum=0;
      for (uint32_t i=0; i<7; i++)
      {
         int b = hreReadBit();
         ch |= b << i;
         sum += b;
      }

      // parity
      if ( (sum & 0x1) != hreReadBit())
         parity_errors++;

      // stop bit
      hreReadBit();

      return ch;
   }

   void hreEvery50ms(void)
   {
      static int sync_counter = 0;   // Number of sync bit reads
      static int sync_run = 0; // Number of consecutive '1's read

      static float curr_start = 0; // uptime when entered hre_reading for current read
      static int read_counter = 0; // number of bytes in the current read
      static int read_sum = 0; // sum of all bytes read
      static int parity_errors = 0; // Number of parity errors in current read
      static char buff[46];  // 8 char and a term
      static float sleep_start = 0;

      static char ch;
      static size_t i;

      switch (hre_state)
      {
         case hre_sync:
            sync_run = 0;
            sync_counter = 0;
            hre_state = hre_syncing;
            ESP_LOGD(TAG, "hre_state:hre_syncing %f", uptime());
            break;

         case hre_syncing:
            // Find the header, a string of 62 '1's
            // Since each bit taks 2 ms, we just read 20 bits at a time
            for (uint32_t i=0; i<20; i++)
            {
               if (hreReadBit())
                  sync_run++;
               else
                  sync_run = 0;
               if (sync_run == 62)
               {
                  hre_state = hre_read;
                  ESP_LOGD(TAG, "hre_state:hre_read %f", uptime());
                  break;
               }
               sync_counter++;
            }
            // If the meter doesn't get in sync within 1000 bits, give up for now
            if (sync_counter > 1000)
            {
               hre_state = hre_sleep;
               ESP_LOGD(TAG, "hre_state:hre_sleep %f", uptime());
               ESP_LOGE(TAG, "sync error");
            }
            break;

            // Start reading the data block
         case hre_read:
            ESP_LOGD(TAG, "sync_run:%d, sync_counter:%d", sync_run, sync_counter);
            read_counter = 0;
            parity_errors = 0;
            read_sum = 0;
            curr_start = uptime();
            memset(buff, 0, sizeof(buff));
            hre_state = hre_reading;
            ESP_LOGD(TAG, "hre_state:hre_reading %f", uptime());
            // So this is intended to fall through to the hre_reading section.
            // it seems that if there is much of a delay between getting the sync
            // bits and starting the read, the HRE won't output the message we
            // are looking for...

         case hre_reading:
            // Read two characters at a time...
            buff[read_counter++] = hreReadChar(parity_errors);
            read_sum += buff[read_counter-1];
            buff[read_counter++] = hreReadChar(parity_errors);
            read_sum += buff[read_counter-1];

            if (read_counter == 46)
            {
               buff[read_counter-1] = '\000';
               ESP_LOGD(TAG, "pe:%d, re:%d, sum:%d", parity_errors, hre_read_errors, read_sum);
               ESP_LOGD(TAG, "read buff: %s", buff);
               if (read_sum == 0)
               {
                  ESP_LOGW(TAG, "null buffer %f", uptime());
               }
               else if (parity_errors == 0)
               {
                  float curr_usage;
                  curr_usage = 0.01f * atol(buff+24); // useage in gal
                  if (hre_usage_time)
                  {
                     double dt = 1.666e-2 * (curr_start - hre_usage_time); // dt in minutes
                     hre_rate = (curr_usage - hre_usage)/dt; // gallons/min
                  }
                  hre_usage = curr_usage;
                  hre_usage_time = curr_start;
                  hre_good = true;
               }
               else
               {
                  ESP_LOGD(TAG, "parity error %f", buff, uptime());
                  hre_read_errors++;
               }
               hre_state = hre_sleep;
               ESP_LOGD(TAG, "hre_state:hre_sleep %f", uptime());
            }
            break;

         case hre_sleep:
            sleep_start = uptime();
            hre_state = hre_sleeping;
            ESP_LOGD(TAG, "hre_state:hre_sleeping %f", uptime());
            break;

         case hre_sleeping:
            // If there isn't some delay between readings, rate calculations aren't as accurate.
            if (uptime() - sleep_start >= 15)
            {
               hre_state = hre_sync;
               ESP_LOGD(TAG, "hre_state:hre_sync %f", uptime());
            }
            break;
      }
   }
};