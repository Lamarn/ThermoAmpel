
; REGISTER				
; Wenn LSB im register (RW Bit genannt) 0 -> Schreibzugriff, 1 -> Lesezugriff
; Slave adresse beliebig außer 0000 000
.equ SLA_W = 0b00000100
.equ SLA_R = 0b00000101
;TODO DATA als register umsetzen, um Anfrage nach Temp oder Humidity in runtime zu ändern
.equ DATA  = 0b00000101 ; DATA wäre dann 00000011 für Temperatur; 00000101 für Relative Humidity
.equ START = $08
.equ MT_SLA_ACK = $18
.equ MT_DATA_ACK = $28
.equ MR_SLA_ACK = $40
.equ MR_DATA_ACK = $50
.equ MR_DATA_NACK = $58

; Infos zu Bits
	; TWINT: TWI Interrupt Flag				| This bit is set by hardware when the TWI has finished its current job and expects application software response
	; TWSTA: TWSTA: TWI START Condition Bit | The application writes the TWSTA bit to one when it desires to become a Master on the Two-wire Serial Bus.
	; TWEN: TWI Enabled Bit					| The TWEN bit enables TWI operation and activates the TWI interface.
	

;---------Begin TWI_MASTER_TRANSMITTER-----------------
TWI_MASTER_TRANSMITTER:
	push r16

	rcall TOGGLE_RED_LED
	rcall DELAY_500MS

	; Send START Condiditon
	ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) 
	out TWCR, r16

	rcall TOGGLE_RED_LED
	rcall DELAY_500MS
	
	rcall TOGGLE_RED_LED
	
	; Wait for TWINT Flag set. This indicates that the START condition has been transmitted
	rcall wait_TWINT

	rcall TOGGLE_RED_LED
	rcall DELAY_500MS

	; Check value of TWI Status Register. Mask prescaler bits. If status different from START go to ERROR
	in r16,TWSR
	andi r16, 0xF8	; ver-und-en + in R16 speichern
	cpi r16, START	; compare immediate um branch if not equal auszuführen in nächster zeile
	brne ERROR

	; Load SLA_W into TWDR Register. Clear TWINT bit in TWCR to start transmission of address
	ldi r16, SLA_W
	out TWDR, r16 
	ldi r16, (1<<TWINT) | (1<<TWEN)
	out TWCR, r16

	; Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
	rcall wait_TWINT

	; Check value of TWI Status Register. Mask prescaler bits. If status different from MT_SLA_ACK go to ERROR
	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MT_SLA_ACK
	brne ERROR

	; Load DATA into TWDR Register. Clear TWINT bit in TWCR to start transmission of data
	ldi r16, DATA
	out TWDR, r16       
	ldi r16, (1<<TWINT) | (1<<TWEN)
	out TWCR, r16

	; Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
	rcall wait_TWINT

	; Check value of TWI Status Register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MT_DATA_ACK
	brne ERROR

	; Transmit STOP condition
	ldi r16, (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
	out TWCR, r16 

	pop r16
ret
;----------End TWI_MASTER_TRANSMITTER-----------

;---------Begin TWI_MASTER_RECEIVER-----------------
TWI_MASTER_RECEIVER:
	push r16
	;START
	ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN)
	out TWCR, r16

	rcall wait_TWINT

	; Check value of TWI Status Register. Mask prescaler bits. If status different from START go to ERROR
	in r16,TWSR
	andi r16, 0xF8	; ver-und-en + in R16 speichern
	cpi r16, START	; compare immediate um branch if not equal auszuführen in nächster zeile
	brne ERROR

	; Load SLA_R into TWDR Register. Clear TWINT bit in TWCR to start transmission of address
	ldi r16, SLA_R
	out TWDR, r16 
	ldi r16, (1<<TWINT) | (1<<TWEN)
	out TWCR, r16

	rcall wait_TWINT

	; Check value of TWI Status Register. Mask prescaler bits. If status different from MR_SLA_ACK go to ERROR
	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MR_SLA_ACK
	brne ERROR

	; Clear TWINT bit in TWCR to start transmission of data 
	ldi r16, (1<<TWINT) | (1<<TWEA) | (1<<TWEN) ; TWEA 1 um ACK zu senden
	out TWCR, r16

	; Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
	rcall wait_TWINT

	; Check value of TWI Status Register. Mask prescaler bits. If status different from MR_DATA_ACK go to ERROR
	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MR_DATA_ACK ; Bedeutet Data Byte empfangen und ACK zurückgesendet
	brne ERROR

	; Daten lesen
	in RECEIVED_DATA_H, TWDR

	;Noch ein Byte empfangen
	; senden "NOT ACK" (TWEA=0) für letztes Daten-Byte 
	ldi r16, (1<<TWINT) | (0<<TWEA)  | (1<<TWEN)
	out TWCR, r16

	rcall wait_TWINT

	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MR_DATA_NACK ; Bedeutet (letztes) Data Byte empfangen und NACK zurückgesendet
	brne ERROR

	; Daten lesen
	in RECEIVED_DATA_L, TWDR

	; TODO eventuell checken?

	; Transmit STOP condition
	ldi r16, (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
	out TWCR, r16 

	pop r16
ret
;----------End TWI_MASTER_RECEIVER-----------

;-- ATmega16A datasheet, seite 185 als referenz  

ERROR: 
	ldi r16, 0b11100000
	out PORTA, r16
	looppp:
		rjmp looppp


;-------------------------------------------------
; WARTEN, bis TWINT Flag gesetzt ist 
wait_TWINT:
	in r16, TWCR
	sbrs r16, TWINT ; SBRS Skip if Bit in Register is Set 
	rjmp wait_TWINT
ret
;-------------------------------------------------