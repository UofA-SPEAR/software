#!/usr/bin/env bash

for i in {0..8}; do
	rosrun ar_track_alvar createMarker $i -ucm -s 17
done
