#!/usr/bin/env python3

import sys, time, os

from bluetooth.ble import BeaconService
from datetime import datetime


class Beacon:

    def __init__(self, data, address):
        self._uuid = data[0]
        self._major = data[1]
        self._minor = data[2]
        self._power = data[3]
        self._rssi = data[4]
        self._address = address

    def __str__(self):
        ret = "Beacon: address:{ADDR} uuid:{UUID} major:{MAJOR} " \
              "minor:{MINOR} txpower:{POWER} rssi:{RSSI}" \
              .format(ADDR=self._address, UUID=self._uuid, MAJOR=self._major,
                      MINOR=self._minor, POWER=self._power, RSSI=self._rssi)
        return ret


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: {} <addr>".format(sys.argv[0]))
        sys.exit(1)

    target = sys.argv[1]

    service = BeaconService()

    # fcleanname = "data/distance-experiment-clean-" + datetime.now().strftime('%Y-%m-%dT%H%M%S' + '.csv')
    frawname = "data/distance-experiment-raw-" + datetime.now().strftime('%Y-%m-%dT%H%M%S' + '.csv')

    # fclean = open(fcleanname, "w+") 
    fraw = open(frawname, "w+") 

    # fclean.write('timestamp,address,rssi\n')
    fraw.write('timestamp,address,rssi\n')

    values = []

    while (True): 
        devices = service.scan(2)

        for address, data in list(devices.items()):
            if (address == target):
                b = Beacon(data, address)
                print(b)

                while(True):
                    devices = service.scan(2)

                    for address, data in list(devices.items()):
                        if (address == target):
                            b = Beacon(data, address)

                            values.append(b._rssi)

                            fraw.write(datetime.now().strftime('%Y-%m-%dT%H:%M:%S') + ',' + address + ',' + str(values[-1]) + '\n')

                            # if (True):
                            #     fclean.write(datetime.now().strftime('%Y-%m-%dT%H:%M:%S') + ',' + address + ',' + str(values[-1]) + '\n')

                            if (len(values) > 5):   
                                print("rssi (rolling average): " + str(sum(values[-5:]) / 5))

                    time.sleep(1)