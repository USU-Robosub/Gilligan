#!/bin/bash

PREV_YEAR="0"
PREV_MONTH="0"
PREV_DAY="0"
FIRST_LOOP=1
MONTHS=( Fake January February March April May June July August September October November December )

# First clear out existing thumbnails
rm *.thumb.jpg

# Loop through remaining images in reverse
for IMG in `ls *.jpg | tac`; do

	# Build Out HTML. This only works with the filename format the existing
	# photos use. I guess this should be implemented in Python or something if
	# we need to support more naming conventions
	YEAR=`echo "$IMG" | cut -c 5-8`
	MONTH=`echo "$IMG" | cut -c 9-10`
	DAY=`echo "$IMG" | cut -c 11-12`
	if [[ $PREV_YEAR != $YEAR || $PREV_MONTH != $MONTH || $PREV_DAY != $DAY ]]; then
		PREV_YEAR=$YEAR
		PREV_MONTH=$MONTH
		PREV_DAY=$DAY
		if [ $FIRST_LOOP -ne 1 ]; then
			echo "            </section>"
		else
			FIRST_LOOP=0
		fi
		echo "            <section>"
		echo "                <time datetime=\"$YEAR-$MONTH-$DAY\">$DAY ${MONTHS[$MONTH]} $YEAR</time>"
	fi
	THUMB=`echo "$IMG" | cut -d '.' -f 1`
	echo "                <a class=\"media\" href=\"pictures/$IMG\"><img src=\"pictures/$THUMB.thumb.jpg\" /></a>"

	# Create a thumbnail
	convert "$IMG" -resize 175x "$THUMB.thumb.jpg"

done

# Finish HTML
echo "            </section>"
