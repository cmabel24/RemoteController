<p>To get the two Xbees talking to each other as they would not connect I had to configure the settings. To have them both match to do this I kept track of what I inputed into one with excel as I had only one explorer dongle. This also meant that it was very challenging for me to interface with the Xbees to make sure that they worked as intended. This also on the other hand made it very simple for me to code and make sure I had the right Xbee on the dongle. With this I was able to find code for the remote controll that I had with the Xbees and got that working to print the speed. 
</p>

To generate the protobuf headers
1. Download the appropriate binary from https://jpa.kapsi.fi/nanopb/download/
2. Adjust the paths in `proto_update.sh`
3. Execute
``` bash
./proto_update.sh
```
