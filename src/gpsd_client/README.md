# gpsd_client package
This package provide an interface to the gpsd daemon. It is used to get the GPS position and the RTK corrections if available.

## gpsd service parameters
To enable the gpsd service with RTK support at ENSTA Bretagne, you need to set the following parameters in the
`/etc/default/gpsd` file:

    START_DAEMON="true"
    DEVICES="/dev/ttyACM0"
    GPSD_OPTIONS="-n ntrip://centipede:centipede@caster.centipede.fr:2101/IUEM"

We use the `ntrip` protocol to get the RTK corrections from the caster server. The `centipede` service is used with IUEM reference.

To apply the changes, you need to restart the `gpsd` service:

    sudo service gpsd restart

