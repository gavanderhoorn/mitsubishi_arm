#!/usr/bin/env python
# encoding: utf-8

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor

class EchoUDP(DatagramProtocol):
    def datagramReceived(self, datagram, address):
        print "Received from address: " + str(address)
        print str(datagram)
        self.transport.write(datagram, address)
        print "Finished sending reply."

print "Starting server."
reactor.listenUDP(10000, EchoUDP())
reactor.run()
