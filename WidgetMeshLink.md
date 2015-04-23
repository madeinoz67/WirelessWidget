
---



---

# Link Assignments #

As part of the [Slotted TDMA access](WidgetMesh.md), the mesh network is made up of communication links between nodes that have been assigned to a timeslot. These link assignments are assigned by the Gateway/coordinator device.

# Link Types #
There are a few different link types.
  * Transmit
  * Receive
  * Transmit/Receive
  * Shared
  * Idle
  * Discovery (for Advertisements)
  * Join Response

## Transmit Link Type ##
unicast transmit only, the receiving node will have a Receive Link type assigned to the same slot.

## Receive Link Type ##
Unicast Receive only, the transmitting node will have a Transmit Link type assigned to the same slot.

## Transmit/Receive Link type ##
A bi-direction link between two nodes, both nodes can either Transmit or receive, Collisions can occur and a back-off mechanism may be required.

## Shared Access Link Type ##
The Shared Access link is a bi-directional link that can be assigned to many nodes, this can be used for broadcasts, or [CSMA](http://en.wikipedia.org/wiki/CSMA/CD) type access within the slot time.  Collisions can occur and a back-off mechanism may be required

## Idle Link Type ##
The link is idle and no transmission or reception will take place, the transceiver can be switched off to save power.

## Discovery Link Type ##
Used for transmitting advertisement packets or receiving solicitation requests.

## Join Response Link Type ##
Join responses from the Gateway device are sent through these links back to the requesting node.