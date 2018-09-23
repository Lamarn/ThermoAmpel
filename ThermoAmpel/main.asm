.include "m16adef.inc"

.def CHECKSUM = R25
.def RECEIVED_BYTE = R24
.def RECEIVED_DATA_H = R23
.def RECEIVED_DATA_L = R22
.def MODE = R21
.def MEAS_LB_H = R4 
.def MEAS_LB_L = R5
.def MEAS_UB_H = R6
.def MEAS_UB_L = R7


;TODO www.mikrocontroller.net/articles/Entprellung

.equ HUMIDITY_LB_H = 0b00000100 ; LB = Lowerbound, H = Highbits
.equ HUMIDITY_LB_L = 0b00000101 ; L = Lowbits | Dec. Wert = 1029 - 40RH

.equ HUMIDITY_UB_H = 0b00000111 ; 
.equ HUMIDITY_UB_L = 0b00101101 ; 1837 -  60RH


/*
.equ HUMIDITY_LB_H = 0b00010011; LB = Lowerbound, H = Highbits
.equ HUMIDITY_LB_L = 0b10010010 ; L = Lowbits | Dec. Wert = 5010 = 10°
*/

/*
.equ HUMIDITY_LB_H = 0b00010111; LB = Lowerbound, H = Highbits
.equ HUMIDITY_LB_L = 0b01111010 ; L = Lowbits | Dec. Wert = 6010 = 20°
*/

/*
.equ TEMPERATURE_LB_H = 0b00010111; LB = Lowerbound, H = Highbits
.equ TEMPERATURE_LB_L = 0b01111010 ; L = Lowbits | Dec. Wert = 6010 = 20°

.equ TEMPERATURE_UB_H = 0b00011010 ; 
.equ TEMPERATURE_UB_L = 0b10100110 ; 6310 = 23°
*/

.org $000
    jmp RESET
.org INT0addr
	jmp INT0_BUTTON_MIDDLE
.org INT1addr
	jmp INT1_BUTTON_RIGHT
;.org $00C
;	jmp TIMER1_INTERRUPT_500MS ; Routine nicht nötig, da Flag in DELAY_500MS manuell abgefragt wird
.org INT2addr
	jmp INT2_BUTTON_LEFT
	
RESET:
	; init the stack
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	; LED pins as output
	ldi r16, 0b11100000
	out DDRA, r16

	; Falling edge of INT1, INT0 generates interrupt request
	;ldi r16, (1<<ISC11) | (1<<ISC01)
	ldi r16, (1<<ISC11) | (1<<ISC01)
	;ldi r16, 0x00
	out MCUCR, r16
	
	; Falling edge of INT2 generates interrupt request is by default
	
	; enable interrupts
	ldi r16, (1<<INT1) | (1<<INT0) | (1<<INT2)
	out GICR, r16
	
	; set global interrupt enable
	sei

	; internen pull-up resistor für buttons aktivieren (Quelle: www.mikrocontroller.net/articles/AVR-Tutorial:_IO-Grundlagen#Pullup-Widerstand)
	ldi r16, 0x00
	out DDRD, r16 ; Pins am Port D auf Eingang setzen, eventuell konflikt mit JTAG schnittstelle? (unwichtig)

	ldi r16, 0b00001100
	out PORTD, r16 ; Pullup Widerstand aktivieren bei INT0 INT1
	
	cbi DDRB, 2
	sbi PORTB, 2 ; Das selbe für INT2

	; configure 16bit-timer

	; 16-bit counter prescaler auf /64 stellen WGM12: setzt timer zurück wenn übereinstimmung
	ldi r16, (1<<CS11) | (1<<CS10) | (1<<WGM12)
	out TCCR1B, r16

	;ldi r16, (1<<OCIE1A) ; interrupt bei nem match zwischen counter und ocr1a
	;out TIMSK, r16 ; Nicht enablen, um manuell abfragen zu können
	
	; SCL Frequency
	ldi r16, 0xFF
	out TWBR, r16

	ldi r16, (1<<TWPS0) | (1<<TWPS1)
	out TWSR, r16

	ldi r16, 0x00
	mov RECEIVED_DATA_H, r16
	mov RECEIVED_DATA_L, r16


MAIN:
	ldi MODE, 0b00000101; 0b00000011 für Temp, 0b00000101 für Humidity
	rcall SEND_SENSOR
	rcall CALC_HUMIDITY ; gerade auf temperatur eingestellt
	cont_main:
	rcall DELAY_500MS ; Maximum eine measurement pro sekunde bei 12 bit messungen
	rcall DELAY_500MS ; daher lieber 2 sekunden warten vor neuer messung
	rcall DELAY_500MS
	rcall DELAY_500MS

rjmp MAIN


TOGGLE_ALL_LED:
	rcall TOGGLE_RED_LED
	rcall TOGGLE_YELLOW_LED
	rcall TOGGLE_GREEN_LED
ret

TOGGLE_RED_LED:
	push r16
	push r17

	in r16, PINA
	ldi r17, 0b00100000
	eor r16, r17
	out PORTA, r16

	pop r17
	pop r16
ret

TOGGLE_YELLOW_LED:
	push r16
	push r17

	in r16, PINA
	ldi r17, 0b01000000
	eor r16, r17
	out PORTA, r16

	pop r17
	pop r16
ret

TOGGLE_GREEN_LED:
	push r16
	push r17

	in r16, PINA
	ldi r17, 0b10000000
	eor r16, r17
	out PORTA, r16

	pop r17
	pop r16
ret

ON_RED_LED:
	sbi PORTA, PORTA5
ret

ON_RED_YELLOW:
	sbi PORTA, PORTA6
ret

ON_RED_GREEN:
	sbi PORTA, PORTA7
ret

INT0_BUTTON_MIDDLE:
	push r16
	push r17

	cli
	; Toggle gelbe LED
	in r16, PINA
	ldi r17, 0b01000000
	eor r16, r17
	out PORTA, r16
	sei

	pop r17
	pop r16
reti

INT1_BUTTON_RIGHT:
	push r16
	push r17

	cli
	; Toggle grüne LED
	in r16, PINA
	ldi r17, 0b10000000
	eor r16, r17
	out PORTA, r16
	sei

	pop r17
	pop r16
reti

INT2_BUTTON_LEFT:
	push r16
	push r17

	cli
	; Toggle rote LED
	in r16, PINA
	ldi r17, 0b00100000
	eor r16, r17
	out PORTA, r16
	sei

	pop r17
	pop r16
reti

DELAY_500MS:
	push r16
	push r17

	; lade 31250 in compare register, weil 1/(4000000/64) * 31250 = 0.5
	ldi r17, 0b01111010
	ldi r16, 0b00010010 
	out OCR1AH, r17
	out OCR1AL, r16 

	; starte counter bei 0
	rcall RESET_16BITCOUNTER

	loop:
		in r16, TIFR
		sbrs r16, OCF1A ; SBRS Skip if Bit in Register is Set 
	rjmp loop
	
	; reset match bit
	ldi r16, (1<<OCF1A) 
	out TIFR, r16

	pop r17
	pop r16

ret
		
RESET_16BITCOUNTER:
	push r16
	push r17
	push r18

	; Schreiben in counter muss atomar geschehen, daher interrupt disablen
	; Schreiben geschieht vom High to Lowbyte, lesen andersrum
	ldi r17, 0x00
	ldi r16, 0x00
	; Save global interrupt flag
	in r18, SREG
	; Disable interrupts
	cli
	; Set TCNT 1 to r17:r16
	out TCNT1H,r17
	out TCNT1L,r16
	; Restore global interrupt flag
	out SREG,r18
	
	pop r18
	pop r17
	pop r16
ret


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
		push r16
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
		;ldi RECEIVED_DATA_H, 0b00000100
		;ldi RECEIVED_DATA_L, 0b00000101
		;cpi RECEIVED_DATA_L, HUMIDITY_UB_L
		;brlo BAD_LED_STATE ;hier
		;rcall GOOD_LED_STATE

		; cpi RECEIVED_DATA_H, 0x00
		; breq test_state ;hier

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
				
	continue_calc_humidity3:
		pop r16
		rjmp cont_main
ret 


test_state:
	rcall TOGGLE_ALL_LED
	rcall DELAY_500MS
	rcall TOGGLE_ALL_LED
	rcall DELAY_500MS
	rjmp test_state
ret

maybe_bad_state:
	cpi RECEIVED_DATA_L, HUMIDITY_LB_L ; Dann Low bits der unteren Grenze checken
	brlo BAD_LED_STATE
	rjmp continue_calc_humidity
ret

maybe_bad_state2:
	push r16
	ldi r16, HUMIDITY_UB_L
	cp r16, RECEIVED_DATA_L
	brlo BAD_LED_STATE
	rjmp continue_calc_humidity2
ret
;---------End CALC_HUMIDITY-----------------


BAD_LED_STATE:
	; Toggle rote LED
	rcall TOGGLE_RED_LED
	rcall DELAY_500MS
	rcall TOGGLE_RED_LED
	rjmp continue_calc_humidity3
ret

GOOD_LED_STATE:
	; Toggle grüne LED
	rcall TOGGLE_GREEN_LED
	rcall DELAY_500MS
	rcall TOGGLE_GREEN_LED
	rjmp continue_calc_humidity3
ret




; TWI FUNKTIONIERT NICHT, DAHER FOLGENDES:
;DELAY_11MS
; PC1 SDA (DATA Pin am Sensor), PC0 SCL(SCK)
SEND_SENSOR:
	; als output setzen
	sbi DDRC, PORTC1
	sbi DDRC, PORTC0
	rcall RESET_SEQUENCE

	; Transmission Start
	; DATA high
	sbi PORTC, PORTC1
	nop
	nop


	; SCK LOW
	cbi PORTC, PORTC0
	nop 
	nop 

	; SCK HIGH
	sbi PORTC, PORTC0
	nop 
	nop

	; lowering of the DATA line while SCK remains high
	cbi PORTC, PORTC1
	nop
	nop

	; low pulse of SCK
	cbi PORTC, PORTC0
	nop
	nop
	nop
	nop
	nop
	nop

	; SCK high again
	sbi PORTC, PORTC0
	nop
	nop

	; raising DATA again while sck is still high
	sbi PORTC, PORTC1
	nop
	nop

	; SCK low
	cbi PORTC, PORTC0

	; WRITE what you want
	; first enable writing (data als output)
	sbi DDRC, PORTC1
	nop

	rcall SEND_COMMAND ; HIER
	; disable data to read ACK
	rcall DISABLE_DATA

	nop

	; clock high
	sbi PORTC, PORTC0
	nop
	nop

	; read ACK
	in r16, PINC

	; clock low
	cbi PORTC, PORTC0

	; check ob ACK gekommen ist
	sbrc r16, PORTC1
	rcall ERROR_NO_ACK


	; zum read nötig
	rcall DISABLE_DATA
	
	; warte bis data ready ist, wenn ready ist wird data auf low gesetzt
	; eigentlich dauerts 80 ms aber so geht auch
	wait_data:
		in r16, PINC
		sbrc r16, PORTC1
	rjmp wait_data
	
	; read first high bits
	rcall READ_BYTE
	mov RECEIVED_DATA_H, RECEIVED_BYTE 
	
	; read lower bits
	rcall READ_BYTE
	mov RECEIVED_DATA_L, RECEIVED_BYTE
	
	; read checksum, kann man skippen wenn man es nicht benutzt
	rcall READ_BYTE
	mov CHECKSUM, RECEIVED_BYTE

	/*	
	rcall ENABLE_DATA
	; DATA HIGH
	sbi PORTC, PORTC1

	rcall CLOCK_HIGH_LOW
	*/
ret

/*
checksum_error:
	rcall TOGGLE_RED_LED
ret
*/
ERROR_NO_ACK:
	rcall TOGGLE_RED_LED
	rcall DELAY_500MS
	rcall TOGGLE_YELLOW_LED
	rcall DELAY_500MS
	rcall TOGGLE_GREEN_LED
	rcall DELAY_500MS
	rcall TOGGLE_ALL_LED
ret

READ_BYTE:
	push r16
	push r17
	push r18

	ldi r16, 0x08
	ldi r18, 0x00
	loop2:
		lsl r18
		; clock high
		sbi PORTC, PORTC0
		nop 
		nop
	
		in r17, PINC
		SBRC r17, PINC1
		; Wenn 1 gefunden wird +1 und nach links schieben, sonst nur nach links schieben
		inc r18

		; clock low
		cbi PORTC, PORTC0
		nop
		nop

		dec r16
	brne loop2

	mov RECEIVED_BYTE, r18

	; send ACK
	rcall ENABLE_DATA
	nop

	; data low
	cbi PORTC, PORTC1
	nop

	; clock high
	sbi PORTC, PORTC0
	nop
	nop
	nop

	; clock low
	cbi PORTC, PORTC0
	nop
	nop
	
	rcall DISABLE_DATA

	pop r18
	pop r17
	pop r16
ret

DISABLE_DATA:
	; Macht data als eingang und pull up hoch
	cbi DDRC, PORTC1
	sbi PORTC, PORTC1
ret

ENABLE_DATA:
	sbi DDRC, PORTC1
ret


ClOCK_HIGH_LOW:
	; Clock hoch und tiefsetzen für übertragung
	sbi PORTC, PORTC0
	nop
	nop
	cbi PORTC, PORTC0
	nop
	nop
ret

RESET_SEQUENCE:

	push r16

	rcall ENABLE_DATA
	nop

	; data high
	sbi PORTC, PORTC1

	; clock low for pulses
	cbi PORTC, PORTC0

	; 9x clock high - low senden
	ldi r16, 0x09
	loop_pulses:
		rcall CLOCK_HIGH_LOW
		dec r16
	brne loop_pulses


	pop r16
ret

	
SEND_HUMIDITY_REQ:
	
	; 1. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW
	
	; 2. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 3. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 4. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 5. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 6. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 7. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 8. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

ret

SEND_TEMPERATURE_REQ:
	
	; 1. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW
	
	; 2. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 3. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 4. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 5. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 6. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 7. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 8. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

ret

SEND_SOFTRESET_REQ:
	
	; 1. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW
	
	; 2. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 3. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 4. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 5. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 6. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 7. Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	; 8. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW

	;11ms delay bei softreset nötig
	rcall DELAY_500MS

ret

SEND_COMMAND:
	push r16
	push r17
	push r18

	ldi r16, 0x08
	ldi r17, 0x80

	loop_command:
		; Setze Mode auf veränderbarem register
		mov r18, MODE
		
		; maskiere bit für bit
		and r18, r17

		; wenn maskiertes byte + maskierung gleich -> dann sende 1
		cp r18, r17

		breq send_one
		brne send_zero
		cont_loop_command:
		; maskierung nach rechts schieben
		lsr r17

		dec r16
	brne loop_command

	pop r18
	pop r17
	pop r16

ret

send_one:
	; Übertragen: 1
	sbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW	
	
	rjmp cont_loop_command
ret

send_zero:
	; 3. Übertragen: 0
	cbi PORTC, PORTC1

	; Clock hoch und tiefsetzen für übertragung
	rcall CLOCK_HIGH_LOW
	
	rjmp cont_loop_command
ret
/* Für "debug" zwecke
RECEIVED_ACK:
	rcall TOGGLE_GREEN_LED
	rcall DELAY_500MS
	rcall DELAY_500MS
	rcall TOGGLE_GREEN_LED
	rcall DELAY_500MS
	rcall DELAY_500MS
	rcall TOGGLE_GREEN_LED
	rjmp main
ret

DISMISSED_ACK:
	rcall TOGGLE_RED_LED
	rcall DELAY_500MS
	rcall DELAY_500MS
	rcall TOGGLE_RED_LED
	rcall DELAY_500MS
	rcall DELAY_500MS
	rcall TOGGLE_RED_LED
	rjmp main
ret
*/ 

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