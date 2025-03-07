## Step 1

open /config and run the InfluxDBRemove.sh shell script first, and then run the InfluxDBSetup.sh shell script after.

## Step 2

A new terminal will appear when setting up the InfluxDB local server.
Here you have to make a few modifications. The terminal will open nano, so everyone can always edit it.

Under the area with HTTP

 * On line 247 remove "#" to enable HTTP request
 * On line 250 remove "#" to enable HTTP request - must be changed from false to true as well
 * On line 253 remove "#" to enable HTTP request - must be changed from false to true as well
 * On line 256 remove "#" to enable HTTP request

When all modifications have been made. press: ctrl+X, ctrl+y and ENTER.

When the database is running (can be seen with the logo of InfluxDB popping up.

When you create the first database an _internal table should apear.

## Step 3

A new terminal window will appear. In here you can access the database for visual purposes.

type command: influx -precision rfc3339

## Step 4

call the following commands:


USE DATABASE Kinova

SELECT * FROM jointPositions
SELECT * FROM TaskPoses
