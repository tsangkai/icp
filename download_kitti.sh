#!/bin/bash

files=(
2011_10_03_calib.zip
2011_10_03_drive_0027
2011_10_03_drive_0034
2011_10_03_drive_0042
2011_10_03_drive_0047
2011_10_03_drive_0058)

for i in ${files[@]}; do
        if [ ${i:(-3)} != "zip" ]
        then
                shortname=$i'_sync.zip'
                fullname=$i'/'$i'_sync.zip'
        else
                shortname=$i
                fullname=$i
        fi
	echo "Downloading: "$shortname
        wget 'https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/'$fullname
        unzip -o $shortname
        rm $shortname
done