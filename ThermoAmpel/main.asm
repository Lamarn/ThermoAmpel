;
; ThermoAmpel.asm
;
; Created: 10.09.2018 10:57:27
;
.include "m16adef.inc"

.equ SLA_W = 0b00000100
.equ SLA_R = 0b00000101
.equ DATA  = 0b00000000 ; DATA wäre dann 00000011 für Temperatur; 00000101 für Relative Humidity
.equ START = $08
.equ MT_SLA_ACK = $18
.equ MT_DATA_ACK = $28

	; TWINT: TWI Interrupt Flag				| This bit is set by hardware when the TWI has finished its current job and expects application software response
	; TWSTA: TWSTA: TWI START Condition Bit | The application writes the TWSTA bit to one when it desires to become a Master on the Two-wire Serial Bus.
	; TWEN: TWI Enabled Bit					| The TWEN bit enables TWI operation and activates the TWI interface.

; Send START Condiditon
	ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) 
	out TWCR, r16

; Wait for TWINT Flag set. This indicates that the START condition has been transmitted
wait1:
	in r16,TWCR
	sbrs r16,TWINT ; SBRS Skip if Bit in Register is Set 
	rjmp wait1

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
wait2:
	in r16,TWCR
	sbrs r16,TWINT
	rjmp wait2

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
wait3:
	in r16,TWCR
	sbrs r16,TWINT
	rjmp wait3

; Check value of TWI Status Register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MT_DATA_ACK
	brne ERROR

; Transmit STOP condition
	ldi r16, (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
	out TWCR, r16 

;-- ATmega16A datasheet, seite 185 als referenz  

ERROR: nop
