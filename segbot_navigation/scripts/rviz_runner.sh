#!/bin/bash

tmpfile=`mktemp /tmp/tempfile.XXXXXXXXX`
cp $1 $tmpfile
sed -i "s/marvin/$2/g" $tmpfile
echo $tmpfile
$OPTIRUN_LAUNCH_PREFIX rosrun rviz rviz -d $tmpfile
rm -f $tmpfile
