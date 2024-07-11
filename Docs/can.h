#ifndef __CAN_H
#define __CAN_H

/*
The point of this file is to standardize CAN 

Bit: 1 Bit
Nibble: 4 Bits
GID: 8 Bits
UUID: 8 Bits
SUBID: 8 Bits 


Bit: set to zero for everything 
Nibble: this sets the priority of the messge 1 Highest, 5 Lowest
	only uses 4 different states cause why not. Also 0 is not used 
	because this allows for a block at the start that disallows for
	non-zero IDs. This is done purely for style.
GID: Global ID, this is the "usage ID" that is used to differentiate nodes, 
	each node that has same or similar function uses the same global ID
UUID: This is used ot differnetiate the different nodes that use the same
	GID
SUBID: anything that a node wants to send with its specific ID uses this
	this allows for 256 different messages, and if that is not enough 
	messages can be given a seperate UUID to increase the amount of SUBIDs
 
	
*/


#define BIT_ZERO        0

#define NIBBlE_CRITICAL 1
#define NIBBLE_HIGH     2
#define NIBBLE_NORMAL   3
#define NIBBLE_LOW      4

//ONLY SET THE GID,UUID

#define GID             0

#define UUID            0

//dont set the SUBID just add a uint8_t when sending the message

#define SUBID           0

#define ID_CRITICAL     (uint8_t)(BIT_ZERO << 28) + (uint8_t)(NIBBLE_CRITICAL << 24) + (uint8_t)(GID << 16) + (uint8_t)(UUID << 8) + (uint8_t)(SUBID)
#define ID_HIGH         (uint8_t)(BIT_ZERO << 28) + (uint8_t)(NIBBLE_HIGH << 24) + (uint8_t)(GID << 16) + (uint8_t)(UUID << 8) + (uint8_t)(SUBID)
#define ID_NORMAL       (uint8_t)(BIT_ZERO << 28) + (uint8_t)(NIBBLE_NORMAL << 24) + (uint8_t)(GID << 16) + (uint8_t)(UUID << 8) + (uint8_t)(SUBID)
#define ID_LOW          (uint8_t)(BIT_ZERO << 28) + (uint8_t)(NIBBLE_LOW << 24) + (uint8_t)(GID << 16) + (uint8_t)(UUID << 8) + (uint8_t)(SUBID)


//useful structs and unions

union rx_u {
    uint8_t data_8[8]
    uint16_t data_16[4]
    uint32_t data_32[2]
};
#endif
