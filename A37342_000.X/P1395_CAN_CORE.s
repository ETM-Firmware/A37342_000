.ifdef __dsPIC30F
        .include "p30fxxxx.inc"
.endif


.equ buffer_length, 15
.equ error_identifier, 0b0001011111111000



;; DPARKER consider disabling CAN interrupt while these functions are running so that we can overwrite read/write pointers


.global  _ETMCanBufferRowsAvailable
	;; Address of the Buffer Data Structure is in W0
	;; Uses W0,W1,W2,SR
.text
_ETMCanBufferRowsAvailable:
	MOV		[W0+0x2], W2 ; Move read_index to W2
	MOV		[W0], W1     ; Move write_index to W1
	SUB		W2, W1, W0
	DEC		W0, W0
	AND		#buffer_length, W0
Return
	



.global  _ETMCanBufferNotEmpty
	;; Address of the Buffer Data Structure is in W0
	;; Uses W0,W1,W2,SR	
.text
_ETMCanBufferNotEmpty:
	MOV		[W0+0x2], W2 ; Move read_index to W1
	MOV		[W0], W1     ; Move write_index to W1
	SUB		W1,W2,W0
	AND		#buffer_length, W0
Return

	

	
.global  _ETMCanRXMessageBuffer
	;; Uses W0,W1,W2,W3,SR
	;; Address of the Message Buffer Data Structure is in W0
	;; Address of the RX buffer is in W1
.text
_ETMCanRXMessageBuffer:
	;; Confirm there is data in RX buffer
	BTSS [W1], #7
	BRA		_ETMCanRXMessageBuffer_RX_EMPTY

	;; Increment message_write_count
	ADD		W0, #4, W3
	INC		[W3], [W3]
	
	;; Check to see if the message buffer is full
	;; WO initial points to the write_index
	MOV		[W0+0x2], W3 ; Move read index to W3
	INC             [W0], W2 ; W2 = Write Index + 1
	AND		#buffer_length, W2 ; Wrap the write index
	CP		W2, W3 ; If (write_index +1) = Read Index 
	BRA		Z, _ETMCanRXMessageBuffer_BUFFER_FULL

	;; Calculate where the data should be added
	MOV		[W0], W2
	MUL.UU 		W2,#10,W2 ; W2 is now the offset based on write index 
	ADD		W0,#8, W3 ; Move Start of data to W3
	ADD		W3,W2,W3 ; W3 is now the start address for this data row 

	;; Copy the data from the SFRs to the data buffer
	SUB		W1, #14, W2
	MOV		[W2], [W3++]   ; Copy SID to Data
	ADD		W2, #6, W2       ; W1 now points to SFR Data 1
	MOV		[W2++], [W3++] ; Copy SFR data 1 to RAM
	MOV		[W2++], [W3++] ; Copy SFR data 2 to RAM
	MOV		[W2++], [W3++] ; Copy SFR data 3 to RAM
	MOV		[W2++], [W3++] ; Copy SFR data 4 to RAM	

	;; Increment the write Index and store
	INC		[W0], W2 ; W2 = Write Index + 1
	AND		#buffer_length, W2 ; Wrap the write index
	MOV		W2, [W0]

	;; Decrement to the overwrite counter
	ADD		W0, #6, W3
	DEC		[W3], [W3]
	
_ETMCanRXMessageBuffer_BUFFER_FULL:	
	;; Increment to the overwrite counter
	ADD		W0, #6, W3
	INC		[W3], [W3]

	;; Clear the RX Buffer full status bit
	BCLR           [W1],#7
_ETMCanRXMessageBuffer_RX_EMPTY:	
	
RETURN




	

.global  _ETMCanRXMessage
	;; Uses W0,W1,W2,SR
	;; Address of the Message Data Structure is in W0
	;; Address of the RX buffer is in W1
.text
_ETMCanRXMessage:
	;; Confirm there is data in RX buffer
	BTSS [W1], #7
	BRA		_ETMCanRXMessage_RX_EMPTY
	
	;; Copy the data from the SFRs to the data buffer
	SUB		W1, #14, W2
	MOV		[W2], [W0++]   ; Copy SID to Data
	ADD		W2, #6, W2       ; W1 now points to SFR Data 1
	MOV		[W2++], [W0++] ; Copy SFR data 1 to RAM
	MOV		[W2++], [W0++] ; Copy SFR data 2 to RAM
	MOV		[W2++], [W0++] ; Copy SFR data 3 to RAM
	MOV		[W2++], [W0++] ; Copy SFR data 4 to RAM	

	;; Clear the RX Buffer full status bit
	BCLR           [W1],#7
	BRA            _ETMCanRXMessage_DONE

	
_ETMCanRXMessage_RX_EMPTY:

	;; The RX Buffer was empty, stuff the return data with error numbers
	;; Do this by setting the ID register to the error_identifier and stuffing the data with Zeros
	MOV		#error_identifier, W2
	MOV             W2, [W0++]
	MOV		#0, W2
	MOV		W2, [W0++]
	MOV		W2, [W0++]
	MOV		W2, [W0++]
	MOV		W2, [W0++]
	BRA		_ETMCanRXMessage_DONE	

_ETMCanRXMessage_DONE:
	
RETURN























	
.global  _ETMCanAddMessageToBuffer
	;; Uses W0,W1,W2,W3,SR
	;; Address of the Message Buffer Data Structure is in W0
	;; Address of the Message is in W1
.text
_ETMCanAddMessageToBuffer:
	;; Increment message_write_count
	ADD		W0, #4, W3
	INC		[W3], [W3]
	
	;; Check to see if the message buffer is full
	;; WO initial points to the write_index
	MOV		[W0+0x2], W3 ; Move read index to W3
	INC             [W0], W2 ; W2 = Write Index + 1
	AND		#buffer_length, W2 ; Wrap the write index
	CP		W2, W3 ; If (write_index +1) = Read Index 
	BRA		Z, _ETMCanAddMessageToBuffer_BUFFER_FULL

	;; Calculate where the data should be added
	MOV		[W0], W2
	MUL.UU 		W2,#10,W2 ; W2 is now the offset based on write index 
	ADD		W0,#8, W3 ; Move Start of data to W3
	ADD		W3,W2,W3 ; W3 is now the start address for this data row 

	;; Copy the data from the message to the data buffer
	MOV		[W1++], [W3++] ; Copy SID to Data
	MOV		[W1++], [W3++] ; Copy word 0 from message to message buffer
	MOV		[W1++], [W3++] ; Copy word 1 from message to message buffer
	MOV		[W1++], [W3++] ; Copy word 2 from message to message buffer
	MOV		[W1++], [W3++] ; Copy word 3 from message to message buffer

	;; Increment the write Index and store
	INC		[W0], W2 ; W2 = Write Index + 1
	AND		#buffer_length, W2 ; Wrap the write index
	MOV		W2, [W0]

	;; Decrement to the overwrite counter
	ADD		W0, #6, W3
	DEC		[W3], [W3]
	
_ETMCanAddMessageToBuffer_BUFFER_FULL:	
	;; Increment to the overwrite counter
	ADD		W0, #6, W3
	INC		[W3], [W3]
	
RETURN


	


	


	
.global  _ETMCanReadMessageFromBuffer
	;; Address of the Buffer Data Structure is in W0
	;; Address of Message (return data) is in W1
	;; Uses W0,W1,W2,W3,SR
.text
_ETMCanReadMessageFromBuffer:
	;; See if the buffer is empty (the read and write index are the same)
	MOV		[W0+0x2], W2 ; Move read_index to W2
	MOV		[W0], W3     ; Move write_index to W3
	CP		W2,W3
	BRA		NZ, _ETMCanReadMessageFromBuffer_NOT_EMPTY

	;; The Buffer was empty, stuff the return data with error numbers
	;; Do this by setting the ID register to the error_identifier and stuffing the data with Zeros
	MOV		#error_identifier, W3
	MOV             W3, [W1++]
	MOV		#0, W3
	MOV		W3, [W1++]
	MOV		W3, [W1++]
	MOV		W3, [W1++]
	MOV		W3, [W1++]
	BRA		_ETMCanReadMessageFromBuffer_DONE	

_ETMCanReadMessageFromBuffer_NOT_EMPTY:

	;; First calculate the address for the data that we want
	MUL.UU		W2, #10, W2
	ADD		W2, #8, W2
	ADD 		W2, W0, W2 ;W2 is now the base address of the data that we want to copy

	;; Copy the data from can message buffer to the message data
	MOV 		[W2++], [W1++]
	MOV 		[W2++], [W1++]
	MOV 		[W2++], [W1++]
	MOV 		[W2++], [W1++]
	MOV 		[W2++], [W1++]

	;; increment the read pointer
	MOV		[W0+0x2], W2 ; Move read_index to W2
	INC 		W2, W2
	AND		#buffer_length, W2
	MOV		W2, [W0+0x2]
	
_ETMCanReadMessageFromBuffer_DONE:	

RETURN











	


.global  _ETMCanTXMessageBuffer
	;; Address of the Buffer Data Structure is in W0
	;; Address of CxTXxCON register is in W1
	;; Uses W0,W1,W2,W3,SR
.text
_ETMCanTXMessageBuffer:
	;; See if the TX buffer is ready to accept a new message
	BTSC		[W1], #3	     ; If the bit is set we need to branch to the end
	BRA             _ETMCanTXMessageBuffer_DONE


	;; See if the buffer is empty (the read and write index are the same)
	MOV		[W0+0x2], W2 ; Move read_index to W2
	MOV		[W0], W3     ; Move write_index to W3
	CP		W2,W3
	BRA		Z, _ETMCanTXMessageBuffer_DONE

	;; If we get here, there is data in the message buffer and the transmit buffer is ready to Transmit

	;; First calculate the address for the data that we want
	MUL.UU		W2, #10, W2
	ADD		W2, #8, W2
	ADD 		W2, W0, W2 ;W2 is now the base address of the data that we want to copy

	;; Copy the data from can message buffer to the TX buffer
	SUB		W1, #14, W3
	MOV 		[W2++], [W3]
	ADD		W3, #6, W3
	MOV 		[W2++], [W3++]
	MOV 		[W2++], [W3++]
	MOV 		[W2++], [W3++]
	MOV 		[W2++], [W3++]

	;; Set the transmit bit to start transfer
	BSET		[W1], #3     ; Queue Transmission
	
	
	;; increment the read pointer
	MOV		[W0+0x2], W2 ; Move read_index to W2
	INC 		W2, W2
	AND		#buffer_length, W2
	MOV		W2, [W0+0x2]
	
_ETMCanTXMessageBuffer_DONE:	

RETURN







	
.global  _ETMCanTXMessage
	;; Address of the Message is in W0
	;; Address of CxTXxCON register is in W1
	;; Uses W0,W1,W2,SR
.text
_ETMCanTXMessage:
	;; Clear the transmit bit before we modify transmit data, this will abort ongoing transmissions
	BCLR		[W1], #3	     

	;; Copy the data from can message to the TX buffer
	SUB		W1, #14, W2
	MOV 		[W0++], [W2]
	ADD		W2, #6, W2
	MOV 		[W0++], [W2++]
	MOV 		[W0++], [W2++]
	MOV 		[W0++], [W2++]
	MOV 		[W0++], [W2++]

	;; Set the transmit bit to start transfer
	BSET		[W1], #3     ; Queue Transmission

RETURN

	





.global  _ETMCanBufferInitialize
	;; Address of the Buffer Data Structure is in W0
	;; Uses W0
.text
_ETMCanBufferInitialize:
	;; Initialize the buffer by setting read and write pointers to zero
	CLR		[W0++]
	CLR		[W0++]
	CLR		[W0++]
	CLR		[W0]	
Return
	
