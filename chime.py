#!/usr/bin/python3

# Chime clock
# Copyright 2019 Nicholas W. Sayer
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warran of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

# Run this from cron at 14, 29, 44 and 59 past the hour, for any hour
# you want to chime.
#
# example:
# 14,29,44,59 7-22 * * * $HOME/chime.py

import RPi.GPIO as GPIO
import time

# GPIO numbers for the chimes. 0-3 are low-high quarters, 4 is hour gong.
# These are the BCM numbers
channels = [17, 18, 27, 22, 23]

# 20 ms solenoid pulses
solenoid_time = 0.02

first_song = [ 3, 2, 1, 0 ]
second_song = [ 1, 3, 2, 0, 0xff, 1, 2, 3, 1 ]
third_song = [ 3, 1, 2, 0, 0xff, 0, 2, 3, 1, 0xff, 3, 2, 1, 0 ]
hour_song = [ 1, 3, 2, 0, 0xff, 1, 2, 3, 1, 0xff, 3, 1, 2, 0, 0xff, 0, 2, 3, 1, 0xff, 0xff, 0xff ]

def do_chime(chime):
	if (chime >= len(channels)):
		return
	GPIO.output(channels[chime], GPIO.HIGH)
	time.sleep(solenoid_time)
	GPIO.output(channels[chime], GPIO.LOW)

def get_time():
	global tm_hour, tm_minute, tm_second
	struct_time = time.localtime()
	tm_hour = struct_time[3]
	tm_minute = struct_time[4]
	tm_second = struct_time[5]

def quarters(song):
	start = time.time()
	song_pos = 0
	while True:
		if (song_pos > (time.time() - start)):
			time.sleep(0.1)
			continue
		do_chime(song[song_pos])
		song_pos = song_pos + 1
		if (song_pos >= len(song)):
			return

def wait_for(second):
	while True:
		get_time()
		if (second == tm_second):
			return
		wait_time = second - (tm_second + 1)
		if (wait_time < 0):
			wait_time = wait_time + 60
		elif (wait_time == 0):
			wait_time = 0.1
		time.sleep(wait_time)

GPIO.setmode(GPIO.BCM)
for pin in channels:
	GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

try:

	get_time()

	if (tm_minute == 14):
		wait_for(0)
		quarters(first_song)
	elif (tm_minute == 29):
		wait_for(0)
		quarters(second_song)
	elif (tm_minute == 44):
		wait_for(0)
		quarters(third_song)
	elif (tm_minute == 59):
		wait_for(60 - len(hour_song)) # wait for the start second
		quarters(hour_song)

		# Wait for the proper start of the minute
		get_time()
		if (tm_second > 0):
			wait_for(0)

		chime_num = 0
		start = time.time()

		while True:

			if (time.time() - start < chime_num * 3):
				time.sleep(0.1)
				continue

			do_chime(4)
			chime_num = chime_num + 1
			# make AM/PM hour
			get_time()
			hour_12 = tm_hour
			if (hour_12 == 0):
				hour_12 = 12
			elif (hour_12 > 12):
				hour_12 = hour_12 - 12
			if (chime_num >= hour_12):
				break
	
finally:
	GPIO.cleanup()
