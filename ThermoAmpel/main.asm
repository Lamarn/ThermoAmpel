; REGISTER				
; Wenn LSB im register (RW Bit genannt) 0 -> Schreibzugriff, 1 -> Lesezugriff
; Slave adresse beliebig außer 0000 000

.include "m16adef.inc"

.def RECEIVED_DATA_H = R25
.def RECEIVED_DATA_L = R24

.equ SLA_W = 0b00000100
.equ SLA_R = 0b00000101

;TODO DATA als register umsetzen, um Anfrage nach Temp oder Humidity in runtime zu ändern

.equ DATA  = 0b00000000 ; DATA wäre dann 00000011 für Temperatur; 00000101 für Relative Humidity
.equ START = $08
.equ MT_SLA_ACK = $18
.equ MT_DATA_ACK = $28
.equ MR_SLA_ACK = $40
.equ MR_DATA_ACK = $50
.equ MR_DATA_NACK = $58

.equ HUMIDITY_LB_H = 0b00000100 ; LB = Lowerbound, H = Highbits
.equ HUMIDITY_LB_L = 0b00000101 ; L = Lowbits | Dec. Wert = 1029

.equ HUMIDITY_UB_H = 0b00000111 ; 
.equ HUMIDITY_UB_L = 0b00101101 ; 1837

	
.org $000
    jmp reset
.org INT0addr
	jmp int0_knopf_mitte
.org INT1addr
	jmp int1_knopf_rechts
.org INT2addr
	jmp int2_knopf_links


reset:
	; init the stack
	ldi r16,low(RAMEND) 
	out SPL,r16 
	ldi r16,high(RAMEND) 
	out SPH,r16

	; LED pins as output
	ldi r16, 0b11100000
	out DDRA, r16


	; Falling edge of INT1, INT0 generates interrupt request
	;ldi r16, (1<<ISC11) | (1<<ISC01)
	ldi r16, (1<<ISC10) | (1<<ISC00)
	out MCUCR, r16
	
	; Falling edge of INT2 generates interrupt request is by default
	
	; enable interrupts
	ldi r16, (1<<INT1) | (1<<INT0) | (1<<INT2)
	out GICR, r16
	
	; set global interrupt enable
	sei 
/*	
	ldi r16, $FF
	out DDRD, r16

	out PORTD, r16
	*/

	
	 

main:
	rjmp main



int0_knopf_mitte:
	; Toggle gelbe LED
	in r16, PORTA
	ldi r17, 0b01000000
	eor r16, r17
	out PORTA, r16
reti

int1_knopf_rechts:
	; Toggle grüne LED
	in r16, PORTA
	ldi r17, 0b10000000
	eor r16, r17
	out PORTA, r16
reti

int2_knopf_links:
	; Toggle rote LED
	in r16, PORTA
	ldi r17, 0b00100000
	eor r16, r17
	out PORTA, r16
reti

CALC_TEMPERATURE:
/* 
17 = -40.1 + 0.01*x 
x = 5710

20 = -40.1 + 0.01*x 
x = 6010

23 = -40.1 + 0.01*x
x = 6310
*/



;---------Begin CALC_HUMIDITY-----------------
CALC_HUMIDITY:
		/*Gutes RH im Wohnzimmer zwischen 40-60%
		40 = -2.0468 + 0.0367 * x - 1.5955*10^(-6) * x²
		x = 1029

		60 = -2.0468 + 0.0367 * x - 1.5955*10^(-6) * x²
		x = 1837

		70 = -2.0468 + 0.0367 * x - 1.5955*10^(-6) * x²
		x = 2167

		75 = -2.0468 + 0.0367 * x - 1.5955*10^(-6) * x²
		x = 2336

		80 = -2.0468 + 0.0367 * x - 1.5955*10^(-6) * x²
		x = 2509

		TODO: vllt ab 60 gelb und ab 70 rot? (ab 70 kann sich schimmel bilden, ab 80 fürn langen zeitraum bildet sich sicher schimmel)

		das heißt guter Wert x für RH ist: 
		1029 < x < 1837

		1029: 00000100 00000101
		1837: 00000111 00101101
		*/

		/* TODO: bsp werte entfernen*/ 
		ldi RECEIVED_DATA_H, 0b00000100
		ldi RECEIVED_DATA_L, 0b00000101

		cpi RECEIVED_DATA_H, HUMIDITY_LB_H ; Erst High bits der unteren Grenze checken
		brlo BAD_LED_STATE 				   ; Branch if Lower (Unsigned) | Wenn data < humidity -> LED ROT 
		breq maybe_bad_state
	
	continue_calc_humidity:
		ldi r16, HUMIDITY_UB_H
		cp r16, RECEIVED_DATA_H ; wenn r16 < data -> LED ROT
		brlo BAD_LED_STATE
		breq maybe_bad_state2

	continue_calc_humidity2:
		rjmp GOOD_LED_STATE
ret 

maybe_bad_state:
	cpi RECEIVED_DATA_L, HUMIDITY_LB_L ; Dann Low bits der unteren Grenze checken
	brlo BAD_LED_STATE
	rjmp continue_calc_humidity
ret

maybe_bad_state2:
	ldi r16, HUMIDITY_UB_L
	cp r16, RECEIVED_DATA_L
	brlo BAD_LED_STATE
	rjmp continue_calc_humidity2
ret
;---------End CALC_HUMIDITY-----------------


BAD_LED_STATE:
;TODO
ret

GOOD_LED_STATE:
;TODO
ret


; Infos zu Bits
	; TWINT: TWI Interrupt Flag				| This bit is set by hardware when the TWI has finished its current job and expects application software response
	; TWSTA: TWSTA: TWI START Condition Bit | The application writes the TWSTA bit to one when it desires to become a Master on the Two-wire Serial Bus.
	; TWEN: TWI Enabled Bit					| The TWEN bit enables TWI operation and activates the TWI interface.
	
;---------Begin TWI_MASTER_TRANSMITTER-----------------
TWI_MASTER_TRANSMITTER:
; Send START Condiditon
	ldi r16, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) 
	out TWCR, r16

; Wait for TWINT Flag set. This indicates that the START condition has been transmitted
/*wait1:
	in r16,TWCR
	sbrs r16,TWINT ; SBRS Skip if Bit in Register is Set 
	rjmp wait1*/
	rcall wait_TWINT
	
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
/*wait2:
	in r16,TWCR
	sbrs r16,TWINT
	rjmp wait2*/
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
/*wait3:
	in r16,TWCR
	sbrs r16,TWINT
	rjmp wait3*/
	rcall wait_TWINT

; Check value of TWI Status Register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR
	in r16,TWSR
	andi r16, 0xF8
	cpi r16, MT_DATA_ACK
	brne ERROR

; Transmit STOP condition
	ldi r16, (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
	out TWCR, r16 

ret
;----------End TWI_MASTER_TRANSMITTER-----------

;---------Begin TWI_MASTER_RECEIVER-----------------
TWI_MASTER_RECEIVER:
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


	; Transmit STOP condition
	ldi r16, (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
	out TWCR, r16 

ret
;----------End TWI_MASTER_RECEIVER-----------


;-- ATmega16A datasheet, seite 185 als referenz  

ERROR: nop


;-------------------------------------------------
; WARTEN, bis TWINT Flag gesetzt ist 
wait_TWINT:
	in r16, TWCR
	sbrs r16, TWINT ; SBRS Skip if Bit in Register is Set 
	rjmp wait_TWINT
ret
;-------------------------------------------------


/*
;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret


;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;* "dd8uH:dd8uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:	clr	drem16uL	;clear remainder Low byte
	sub	drem16uH,drem16uH;clear remainder High byte and carry
	ldi	dcnt16u,17	;init loop counter
d16u_1:	rol	dd16uL		;shift left dividend
	rol	dd16uH
	dec	dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:	rol	drem16uL	;shift dividend into remainder
	rol	drem16uH
	sub	drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	drem16uL,dv16uL	;    restore remainder
	adc	drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1
	*/ 
	
	/*	Routine zum multiplizieren
	; [ 16x16 Bit Unsigned Multiplication ]
; multiplicand: r17:r16
; multiplier  : r23:r22
; result out  : r21:r20:r19:r18

; 24 cycles
	ldi r16, 0b00110001
	ldi r17, 0b00000100

	ldi r22, 0b00110001
	ldi r23, 0b00000100
	
M_16x16:
    MUL   r17, r23              ; ah * bh
    MOVW  r21:r20, r1:r0
    MUL   r16, r22              ; al * bl
    MOVW  r19:r18, r1:r0
    MUL   r17, r22              ; ah * bl
    CLR   r22
    ADD   r19, r0
    ADC   r20, r1
    ADC   r21, r22
    MUL   r23, r16              ; bh * al
    ADD   r19, r0
    ADC   r20, r1
    ADC   r21, r22
RET

*/

/*Alte Codeblöcke*/
/* Multiplikation beispiel
	ldi mc16uL, 0b00110001
	ldi mc16uH, 0b00000100

	ldi mp16uL, 0b00110001
	ldi mp16uH, 0b00000100

	rcall mpy16u
	*/

	/*
	RH = c1 + c2 * SO + c3 * SO²
	c1: -2.0468
	c2: 0.0367
	c3: -1.5955*10^(-6)

	RH = (367 * SO - 20468 - 159,55*SO/100*SO*100) / 10000

	20468: 01001111 11110100

	**********NICHT NÖTIG*******
	*/



	/*	ldi r17, -2.0468
		ldi r18, 255
		
		;fmuls r18, r17
		;movw r18:r17,r1:r0 ; Copy result back in r23:r22
		
		ldi r22, 2.0468
		ldi r23, 127
		muls r23,r22 ; Multiply signed r23 and r22 in (1.7) format, result in (1.15) format
		movw r23:r22,r1:r0 ; Copy result back in r23:r22
		*/