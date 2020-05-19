/*

    GPS Clock
    Copyright (C) 2016 Nicholas W. Sayer

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
    
  */

// Fuse settings: lfuse=0xe2, hfuse=0xdf, efuse=0xff

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

// UBRR?_VALUE macros defined here are used below in serial initialization in main()
#define F_CPU (8000000UL)
#define BAUD 9600
#include <util/setbaud.h>

/* EEPROM:

0 timezone
1 DST
2 start hour
3 end hour

*/

#define EE_TIMEZONE ((void*)0)
#define EE_DST_MODE ((void*)1)
#define EE_START_HOUR ((void*)2)
#define EE_END_HOUR ((void*)3)

/* Hardware:

PB3: !RESET
PB2: CH4
PB1: CH3
PB0: CH2
PA7: PPS
PA6:
PA5:
PA4:
PA3: CH1
PA2: TX
PA1: RX
PA0: CH0

*/

// The millisecond timer. 8 MHz divided by 256 is 31.25 kHz. So we
// count to 32 once, and 31 3 times.

#define TICK_BASE_CYCLE (31)
#define TICK_CYCLE_LENGTH (4)
#define TICK_NUM_LONG (1)

// This is the timer frequency - we're aiming for a millisecond timer
#define F_TICK (1000UL)

// One second beats
#define BEAT_TIME (F_TICK)

// How long to energize the solenoid? 166 ms.
#define SOLENOID_ON (25UL)

// These are return values from the DST detector routine.
// DST is not in effect all day
#define DST_NO 0
// DST is in effect all day
#define DST_YES 1
// DST begins at 0200
#define DST_BEGINS 2
// DST ends 0300 - that is, at 0200 pre-correction.
#define DST_ENDS 3

// The possible values for dst_mode
#define DST_OFF 0
#define DST_US 1
#define DST_EU 2
#define DST_AU 3
#define DST_NZ 4
#define DST_MODE_MAX DST_NZ

uint8_t hour, minute, second;
uint8_t tz_hour, dst_mode;
uint8_t start_hour, end_hour;

uint32_t song_start;
int8_t song_pos;
const uint8_t *song;
size_t song_length;

volatile uint8_t new_second;
volatile uint32_t ticks;

uint8_t gps_locked;
uint16_t utc_ref_year;
uint8_t utc_ref_mon;
uint8_t utc_ref_day;

// Serial buffer stuff
#define RX_BUF_LEN (96)
#define TX_BUF_LEN (24)

volatile uint8_t rx_buf[RX_BUF_LEN];
volatile uint8_t rx_str_len;
volatile uint8_t tx_buf[TX_BUF_LEN];
volatile uint16_t tx_buf_head, tx_buf_tail;
volatile uint8_t nmea_ready;

const unsigned char month_tweak[] PROGMEM = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

static inline unsigned char first_sunday(unsigned char m, unsigned int y) {
	// first, what's the day-of-week for the first day of whatever month?
	// From http://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week
	y -= m < 3;
	unsigned char month_tweak_val = pgm_read_byte(&(month_tweak[m - 1]));
	unsigned char dow = (y + y/4 - y/100 + y/400 + month_tweak_val + 1) % 7;

	// If the 1st is a Sunday, then the answer is 1. Otherwise, we count
	// up until we find a Sunday.
	return (dow == 0)?1:(8 - dow);
}

static inline unsigned char calculateDSTAU(const unsigned char d, const unsigned char m, const unsigned int y) {
        // DST is in effect between the first Sunday in October and the first Sunday in April
        unsigned char change_day;
        switch(m) {
                case 1: // November through March
                case 2:
                case 3:
                case 11:
                case 12:
                        return DST_YES;
                case 4: // April
                        change_day = first_sunday(m, y);
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                case 5: // April through September
                case 6:
                case 7:
                case 8:
                case 9:
                        return DST_NO;
                case 10: // October
                        change_day = first_sunday(m, y);
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static inline unsigned char calculateDSTNZ(const unsigned char d, const unsigned char m, const unsigned int y) {
        // DST is in effect between the last Sunday in September and the first Sunday in April
        unsigned char change_day;
        switch(m) {
                case 1: // October through March
                case 2:
                case 3:
                case 10:
                case 11:
                case 12:
                        return DST_YES;
                case 4: // April
                        change_day = first_sunday(m, y);
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                case 5: // April through August
                case 6:
                case 7:
                case 8:
                        return DST_NO;
                case 9: // September
                        change_day = first_sunday(m, y);
                        while(change_day + 7 <= 30) change_day += 7; // last Sunday
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static inline unsigned char calculateDSTEU(const unsigned char d, const unsigned char m, const unsigned int y) {
        // DST is in effect between the last Sunday in March and the last Sunday in October
        unsigned char change_day;
        switch(m) {
                case 1: // November through February
                case 2:
                case 11:
                case 12:
                        return DST_NO;
                case 3: // March
                        change_day = first_sunday(m, y);
                        while(change_day + 7 <= 31) change_day += 7; // last Sunday
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                case 4: // April through September
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                        return DST_YES;
                case 10: // October
                        change_day = first_sunday(m, y);
                        while(change_day + 7 <= 31) change_day += 7; // last Sunday
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static inline unsigned char calculateDSTUS(const unsigned char d, const unsigned char m, const unsigned int y) {
	// DST is in effect between the 2nd Sunday in March and the first Sunday in November
	// The return values here are that DST is in effect, or it isn't, or it's beginning
	// for the year today or it's ending today.
	unsigned char change_day;
	switch(m) {
		case 1: // December through February
		case 2:
		case 12:
			return DST_NO;
		case 3: // March
			change_day = first_sunday(m, y) + 7; // second Sunday.
			if (d < change_day) return DST_NO;
			else if (d == change_day) return DST_BEGINS;
			else return DST_YES;
			break;
		case 4: // April through October
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			return DST_YES;
		case 11: // November
			change_day = first_sunday(m, y);
			if (d < change_day) return DST_YES;
			else if (d == change_day) return DST_ENDS;
			else return DST_NO;
			break;
		default: // This is impossible, since m can only be between 1 and 12.
			return 255;
	}
}
static inline unsigned char calculateDST(const unsigned char d, const unsigned char m, const unsigned int y) {
        switch(dst_mode) {
                case DST_US:
                        return calculateDSTUS(d, m, y);
                case DST_EU:
                        return calculateDSTEU(d, m, y);
                case DST_AU:
                        return calculateDSTAU(d, m, y);
                case DST_NZ:
                        return calculateDSTNZ(d, m, y);
                default: // off - should never happen
                        return DST_NO;
        }
}

static inline void startLeapCheck();

static inline void handle_time(char h, unsigned char m, unsigned char s, unsigned char dst_flags) {
	// What we get is the current second. We have to increment it
	// to represent the *next* second.
	s++;
	// Note that this also handles leap-seconds. We wind up pinning to 0
	// twice.
	if (s >= 60) { s = 0; m++; }
	if (m >= 60) { m = 0; h++; }
	if (h >= 24) { h = 0; }

	// Move to local standard time.
	h += tz_hour;
	while (h >= 24) h -= 24;
	while (h < 0) h += 24;

	if (dst_mode != DST_OFF) {
		unsigned char dst_offset = 0;
		// For Europe, decisions are at 0100. Everywhere else it's 0200.
		unsigned char decision_hour = (dst_mode == DST_EU)?1:2;
		switch(dst_flags) {
			case DST_NO: dst_offset = 0; break; // do nothing
			case DST_YES: dst_offset = 1; break; // add one hour
			case DST_BEGINS:
				dst_offset = (h >= decision_hour)?1:0; // offset becomes 1 at 0200 (0100 EU)
				break;
                        case DST_ENDS:
				// The *summer time* hour has to be the decision hour,
				// and we haven't yet made 'h' the summer time hour,
				// so compare it to one less than the decision hour.
                                dst_offset = (h >= (decision_hour - 1))?0:1; // offset becomes 0 at 0200 (daylight) (0100 EU)
				break;
		}
		h += dst_offset;
		if (h >= 24) h -= 24;
	}

	// Every hour, check to see if the leap second value in the receiver is out-of-date
	unsigned char doLeapCheck = (m == 30 && s == 0);

	hour = h;
	minute = m;
	second = s;

	if (doLeapCheck) startLeapCheck();
}

static inline void tx_char(const unsigned char c);
static inline void write_msg(const unsigned char *msg, const size_t length) {
	for(int i = 0; i < length; i++) {
		tx_char(msg[i]);
	}
 }
 
const unsigned char PROGMEM leap_check_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x64, 0x20, 0x44, 0x0d, 0x0a };
static inline void startLeapCheck(void) {
	// Ask for the time message. We expect a 0x64-0x8e in response
	unsigned char msg[sizeof(leap_check_msg)];
	memcpy_P(msg, leap_check_msg, sizeof(leap_check_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM leap_update_msg[] = { 0xa0, 0xa1, 0x00, 0x04, 0x64, 0x1f, 0x00, 0x01, 0x7a, 0x0d, 0x0a };
static inline void updateLeapDefault(const unsigned char leap_offset) {
	// This is a set leap-second default message. It will write the given
	// offset to flash.
	unsigned char msg[sizeof(leap_update_msg)];
	memcpy_P(msg, leap_update_msg, sizeof(leap_update_msg));
	msg[6] = leap_offset;
	msg[8] ^= leap_offset; // fix the checksum
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM get_utc_ref_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x64, 0x16, 0x72, 0x0d, 0x0a };
static inline void startUTCReferenceFetch() {
	// This is a request for UTC reference date message. We expect a 0x64-0x8a in response
	unsigned char msg[sizeof(get_utc_ref_msg)];
	memcpy_P(msg, get_utc_ref_msg, sizeof(get_utc_ref_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM utc_ref_msg[] = { 0xa0, 0xa1, 0x00, 0x08, 0x64, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x71, 0x0d, 0x0a };
static inline void updateUTCReference(const unsigned int y, const unsigned char mon, const unsigned char d) {
	// This sets the UTC reference date, which controls the boundaries of the GPS week window
	unsigned char msg[sizeof(utc_ref_msg)];
	memcpy_P(msg, utc_ref_msg, sizeof(utc_ref_msg));
	msg[7] = (unsigned char)(y >> 8);
	msg[8] = (unsigned char)y;
	msg[9] = mon;
	msg[10] = d;
	for(int i = 7; i <= 10; i++) msg[12] ^= msg[i]; // fix checksum
	write_msg(msg, sizeof(msg));
}

static const char *skip_commas(const char *ptr, const int num) {
	for(int i = 0; i < num; i++) {
		ptr = strchr(ptr, ',');
		if (ptr == NULL) return NULL; // not enough commas
		ptr++; // skip over it
	}
	return ptr;
}

const char hexes[] PROGMEM = "0123456789abcdef";

static uint8_t hexChar(uint8_t c) {
	if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
	const char* outP = strchr_P(hexes, c);
	if (outP == NULL) return 0;
	return (uint8_t)(outP - hexes);
}

static inline void handleGPS() {
	uint16_t str_len = rx_str_len; // rx_str_len is where the \0 was written.

	if (str_len >= 3 && rx_buf[0] == 0xa0 && rx_buf[1] == 0xa1) { // binary protocol message
		uint16_t payloadLength = (((uint16_t)rx_buf[2]) << 8) | rx_buf[3];
		if (str_len != payloadLength + 7) return; // the A0, A1 bytes, length and checksum are added
		uint16_t checksum = 0;
		for(int i = 0; i < payloadLength; i++) checksum ^= rx_buf[i + 4];
		if (checksum != rx_buf[payloadLength + 4]) return; // checksum mismatch
		if (rx_buf[4] == 0x64 && rx_buf[5] == 0x8a) {
			utc_ref_year = (rx_buf[3 + 4] << 8) | rx_buf[3 + 5];
			utc_ref_mon = rx_buf[3 + 6];
			utc_ref_day = rx_buf[3 + 7];
		} else if (rx_buf[4] == 0x64 && rx_buf[5] == 0x8e) {
			if (!(rx_buf[15 + 3] & (1 << 2))) return; // GPS leap seconds invalid
			if (rx_buf[13 + 3] == rx_buf[14 + 3]) return; // Current and default agree
			updateLeapDefault(rx_buf[14 + 3]);
		} else {
			return; // unknown binary protocol message
		}
	}

	if (str_len < 9) return; // No sentence is shorter than $GPGGA*xx
	// First, check the checksum of the sentence
	uint8_t checksum = 0;
	int i;
	for(i = 1; i < str_len; i++) {
		if (rx_buf[i] == '*') break;
		checksum ^= rx_buf[i];
	}
	if (i > str_len - 3) {
		return; // there has to be room for the "*" and checksum.
	}
	i++; // skip the *
	uint8_t sent_checksum = (hexChar(rx_buf[i]) << 4) | hexChar(rx_buf[i + 1]);
	if (sent_checksum != checksum) {
		return; // bad checksum.
	}
  
	const char *ptr = (char *)rx_buf;
	if (!strncmp_P(ptr, PSTR("$GPRMC"), 6)) {
		// $GPRMC,172313.000,A,xxxx.xxxx,N,xxxxx.xxxx,W,0.01,180.80,260516,,,D*74\x0d\x0a
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		char h = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		uint8_t min = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		uint8_t s = (ptr[4] - '0') * 10 + (ptr[5] - '0');
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		gps_locked = *ptr == 'A'; // A = AOK
		ptr = skip_commas(ptr, 7);
		if (ptr == NULL) return; // not enough commas
		uint8_t d = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		uint8_t mon = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		uint8_t y = (ptr[4] - '0') * 10 + (ptr[5] - '0');

		// We must turn the two digit year into the actual A.D. year number.
		// As time goes forward, we can keep a record of how far time has gotten,
		// and assume that time will always go forwards. If we see a date ostensibly
		// in the past, then it "must" mean that we've actually wrapped and need to
		// add 100 years. We keep this "reference" date in sync with the GPS receiver,
		// as it uses the reference date to control the GPS week rollover window.
  		y += 2000;
		while (y < utc_ref_year) y += 100; // If it's in the "past," assume time wrapped on us.
  
		if (utc_ref_year != 0 && y != utc_ref_year) {
			// Once a year, we should update the refence date in the receiver. If we're running on New Years,
			// then that's probably when it will happen, but anytime is really ok. We just don't want to do
			// it a lot for fear of burning the flash out in the GPS receiver.
  			updateUTCReference(y, mon, d);
			utc_ref_year = y;
			utc_ref_mon = mon;
			utc_ref_day = d;
		}

		// The problem is that our D/M/Y is UTC, but DST decisions are made in the local
		// timezone. We can adjust the day against standard time midnight, and
		// that will be good enough. Don't worry that this can result in d being either 0
		// or past the last day of the month. Those will still be more or less than the "decision day"
		// for DST, which is all that really matters.
		if (h + tz_hour < 0) d--;
		if (h + tz_hour > 23) d++;
		uint8_t dst_flags = calculateDST(d, mon, y);
		handle_time(h, min, s, dst_flags);
	}
}

ISR(USART0_RX_vect) {
	uint8_t rx_char = UDR0;
 
	if (nmea_ready) return; // ignore serial until current buffer handled 
	if (rx_str_len == 0 && !(rx_char == '$' || rx_char == 0xa0)) return; // wait for a "$" or A0 to start the line.

	rx_buf[rx_str_len] = rx_char;

	if (++rx_str_len == RX_BUF_LEN) {
		// The string is too long. Start over.
		rx_str_len = 0;
	}

	// If it's an ASCII message, then it's ended with a CRLF.
	// If it's a binary message, then it's ended when it's the correct length
	if ( (rx_buf[0] == '$' && (rx_char == 0x0d || rx_char == 0x0a)) ||
		(rx_buf[0] == 0xa0 && rx_str_len >= 4 && rx_str_len >= ((rx_buf[2] << 8) + rx_buf[3] + 7)) ) {
		rx_buf[rx_str_len] = 0; // null terminate
		nmea_ready = 1; // Mark it as ready
		return;
	}

}

ISR(USART0_UDRE_vect) {
	if (tx_buf_head == tx_buf_tail) {
		// the transmit queue is empty.
		UCSR0B &= ~_BV(UDRIE0); // disable the TX interrupt
		return;
	}
	UDR0 = tx_buf[tx_buf_tail];
	if (++tx_buf_tail == TX_BUF_LEN) tx_buf_tail = 0; // point to the next char
}

// If the TX buffer fills up, then this method will block, which should be avoided.
static inline void tx_char(const uint8_t c) {
	int buf_in_use;
	do {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			buf_in_use = tx_buf_head - tx_buf_tail;
		}
		if (buf_in_use < 0) buf_in_use += TX_BUF_LEN;
		wdt_reset(); // we might be waiting a while.
	} while (buf_in_use >= TX_BUF_LEN - 2) ; // wait for room in the transmit buffer

	tx_buf[tx_buf_head] = c;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// this needs to be atomic, because an intermediate state is tx_buf_head
		// pointing *beyond* the end of the buffer.
		if (++tx_buf_head == TX_BUF_LEN) tx_buf_head = 0; // point to the next free spot in the tx buffer
	}
	UCSR0B |= _BV(UDRIE0); // enable the TX interrupt. If it was disabled, then it will trigger one now.
}

static inline uint32_t timer_value() __attribute__ ((always_inline));
static inline uint32_t timer_value() {
	uint32_t now;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		now = ticks;
	}
	return now;
}

ISR(TIMER2_COMPA_vect) {
	static uint8_t tick_cycle_pos = 0;
	if (++tick_cycle_pos >= TICK_CYCLE_LENGTH) tick_cycle_pos = 0;
	if (tick_cycle_pos >= TICK_NUM_LONG)
		OCR2A = TICK_BASE_CYCLE - 1; // the OCR register is 1 based - we want a *short* cycle
	else
		OCR2A = TICK_BASE_CYCLE; // a long cycle

	// ticks is nor allowed to equal zero
	if (++ticks == 0) ticks++;
}

ISR(PCINT0_vect) {
	if (!(PINA & _BV(7))) return; // ignore the trailing edge

	// we just need to know when the second starts in the outer loop
	new_second = 1;
}

// pins 0-3 are the 4 quarters notes, low to high. pin 4 is the hourly chime.
void write_pin(uint8_t pin, uint8_t val) {
  if (val) {
    switch(pin) {
      case 0:
        PORTA |= _BV(0); break;
      case 1:
        PORTA |= _BV(3); break;
      case 2:
        PORTB |= _BV(0); break;
      case 3:
        PORTB |= _BV(1); break;
      case 4:
        PORTB |= _BV(2); break;
    }
  } else {
    switch(pin) {
      case 0:
        PORTA &= ~_BV(0); break;
      case 1:
        PORTA &= ~_BV(3); break;
      case 2:
        PORTB &= ~_BV(0); break;
      case 3:
        PORTB &= ~_BV(1); break;
      case 4:
        PORTB &= ~_BV(2); break;
    }
  }
}

void do_chime(uint8_t note) {
	write_pin(note, 1); // turn solenoid on
	for(uint32_t now = timer_value(); timer_value() - now < SOLENOID_ON; );
	write_pin(note, 0); // turn solenoid off
}

// westminster quarters.
// These should be an even number of seconds long (padding with rests where necessary
// if BEAT_TIME is a fraction of a second).
const uint8_t PROGMEM first_song[] = { 3, 2, 1, 0 };
const uint8_t PROGMEM second_song[] = { 1, 3, 2, 0, 0xff, 1, 2, 3, 1 };
const uint8_t PROGMEM third_song[] = { 3, 1, 2, 0, 0xff, 0, 2, 3, 1, 0xff, 3, 2, 1, 0 };
const uint8_t PROGMEM hour_song[] = { 1, 3, 2, 0, 0xff, 1, 2, 3, 1, 0xff, 3, 1, 2, 0, 0xff, 0, 2, 3, 1, 0xff, 0xff, 0xff };

// main() never returns.
void __ATTR_NORETURN__ main(void) {

	wdt_enable(WDTO_250MS);

	// Leave on only the parts of the chip we use.
	PRR = ~( _BV(PRUSART0) | _BV(PRTIM2));

	// pull up on the unused pins (the programming interface)
	PORTA = 0;
	PUEA = _BV(4) | _BV(5) | _BV(6);
	DDRA = _BV(0) | _BV(2) | _BV(3); // The two chime pins and the TX pin are outputs

	PORTB = 0;
	PUEB = 0; // no pull-ups
	DDRB = _BV(0) | _BV(1) | _BV(2); // all outputs

	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A = _BV(U2X0);
#else
	UCSR0A = 0;
#endif

	UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0); // RX interrupt and TX+RX enable

	// 8N1
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

	tx_buf_head = tx_buf_tail = 0;
	rx_str_len = 0;
	nmea_ready = 0;

	// millisecond timer for Timer 2.
	TCCR2B = _BV(WGM22) | _BV(CS22); // prescale by 256, CTC mode
	TIMSK2 = _BV(OCIE2A); // interrupt on compare match A
	OCR2A = TICK_BASE_CYCLE + 1; // start with a long cycle

	ticks = 1;

	// Pin change interrupt on PA7
	PCMSK0 = _BV(PCINT7); // pin change interrupt on PA7
	GIMSK = _BV(PCIE0); // enable pin change interrupt 0

	uint8_t ee_rd = eeprom_read_byte(EE_TIMEZONE);
	if (ee_rd == 0xff)
		tz_hour = -8;
	else
		tz_hour = ee_rd - 12;
	dst_mode = eeprom_read_byte(EE_DST_MODE);
	if (dst_mode > DST_MODE_MAX) dst_mode = DST_US;

	// start hour and end hour are inclusive, and are the times when the chimes will operate (24 hour time)
	start_hour = eeprom_read_byte(EE_START_HOUR);
	if (start_hour > 23) start_hour = 7;
	end_hour = eeprom_read_byte(EE_END_HOUR);
	if (end_hour > 23) end_hour = 22;

	gps_locked = 0;
	song_start = 0;

	// Turn on interrupts
	sei();

	while(1) {
		wdt_reset();
		if (nmea_ready) {
			// Do this out here so it's not in an interrupt-disabled context.
			handleGPS();
			rx_str_len = 0; // now clear the buffer
			nmea_ready = 0;
			continue;
		}
		uint32_t now = timer_value();

		if (new_second) {
			new_second = 0;
			if (!gps_locked) continue; // ignore unless locked

			{
				// We need to take into account the hour it's about to be
				uint8_t h = ((minute < 50)?hour:hour + 1) % 24;

				if (start_hour > end_hour) {
					// this means we chime *before* end and *after* start.
					// So if it's between start and end, abstain.
					if (h > start_hour && h < end_hour) continue;
				} else {
					// This means we DON'T chime if before start or
					// after end.
					if (h < start_hour || h > end_hour) continue;
				}
			}

			if (minute < 2) { // depending on how slow you chime, it might take up to 2 minutes.
				// Create AM/PM time
				uint8_t h = hour;
			        if (h == 0) { h = 12; }
				else if (h > 12) h -= 12;
				// second within the hour
				uint16_t s = second + minute * 60;
				if ((!(s % 4)) && (s / 4) < h) { // every third second, starting at 0
					do_chime(4);
				}
			}

#if 0
			// Time for a song?
			if (second == 0) {
				switch(minute) {
					case 15:
						song = first_song;
						song_length = sizeof(first_song);
						song_pos = -1;
						song_start = now; // GO!
						break;
					case 30:
						song = second_song;
						song_length = sizeof(second_song);
						song_pos = -1;
						song_start = now; // GO!
						break;
					case 45:
						song = third_song;
						song_length = sizeof(third_song);
						song_pos = -1;
						song_start = now; // GO!
						break;
				}
			}
#endif

			// The hour song is special - it has to be early so the chimes are on-time.
			if (minute == 59 && second == (60 - sizeof(hour_song) * (BEAT_TIME / 1000))) {
				song = hour_song;
				song_length = sizeof(hour_song);
				song_pos = -1;
				song_start = now; // GO!
			}
		}
		// is it time for the next song note?
		if (song_start && (now - song_start >= (song_pos + 1) * BEAT_TIME)) {
			if (++song_pos >= song_length) { // Are we done?
				song_start = 0;
				continue;
			}
			uint8_t note = pgm_read_byte(&(song[song_pos]));
			if (note <= 4) // it's not a rest
				do_chime(note);
			continue;
		}
	}
	__builtin_unreachable();
}
