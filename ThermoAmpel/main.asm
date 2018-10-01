.include "m16adef.inc"



.def CHECKSUM = R25
.def RECEIVED_BYTE = R24
.def RECEIVED_DATA_H = R23
.def RECEIVED_DATA_L = R22
.def MODE = R21
.def MEAS_LB_H = R6
.def MEAS_LB_L = R7
.def MEAS_UB_H = R8
.def MEAS_UB_L = R9

;TODO www.mikrocontroller.net/articles/Entprellung

.equ HUMIDITY_MODE    = 0b00000101
.equ TEMPERATURE_MODE = 0b00000011
.equ SOFTRESET_MODE   = 0b00011110

/* paar werte zum besseren einstellen 

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


.cseg
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

	;damit sensor funktioniert ist nach power-up oder softresetkommando 11 ms delay nötig
	rcall DELAY_500MS


MAIN:
	ldi MODE, HUMIDITY_MODE; 0b00000011 für Temp, 0b00000101 für Humidity
	rcall LOAD_HUM_BOUNDS
	rcall SEND_SENSOR
	rcall CALC_MEAS ; gerade auf temperatur eingestellt
	cont_main:
	rcall DELAY_500MS ; Maximum eine measurement pro sekunde bei 12 bit messungen
	rcall DELAY_500MS ; 

	ldi MODE, TEMPERATURE_MODE; 0b00000011 für Temp, 0b00000101 für Humidity
	rcall LOAD_TEMP_BOUNDS
	rcall SEND_SENSOR
	rcall CALC_MEAS ; gerade auf temperatur eingestellt
	cont_main2:
	rcall DELAY_500MS
	rcall DELAY_500MS
	
rjmp MAIN


TEMPERATURE_CONVERSION:
	ldi ZL, low(2*LUT_Temperature)
	ldi ZH, high(2*LUT_Temperature) ; LUT-Adresse laden
	rcall divide_data_by_64 ; Data auf 8 Bit richten
	mov r16, RECEIVED_DATA_L 
	ldi r17, 0x00
	add ZL, r16 ; Auf den Tabelleneintrag springen
	adc ZH, r17 ; Übertrag addieren
	lpm RECEIVED_DATA_L, Z ; Ergebnis in R_D_L laden
ret


divide_data_by_64:
	; Für Temperatur
	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 (=/4)

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 /2 (=/8)

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 /2 /2 (=/16)

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 /2 /2 /2 (=/32)

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 /2 /2 /2 /2 (=/64)
ret

HUMIDITY_CONVERSION:
	ldi ZL, low(2*LUT_Humidity)
	ldi ZH, high(2*LUT_Humidity) ; LUT-Adresse laden
	rcall divide_data_by_16 ; Data auf 8 Bit richten
	mov r16, RECEIVED_DATA_L 
	ldi r17, 0x00
	add ZL, r16 ; Auf den Tabelleneintrag springen
	adc ZH, r17 ; Übertrag addieren
	lpm RECEIVED_DATA_L, Z ; Ergebnis in R_D_L laden
ret

divide_data_by_16:
	; Für Humidity
	lsr RECEIVED_DATA_H
	ror RECEIVED_DATA_L ; /2

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 (=/4)

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 /2 (=/8)

	lsr RECEIVED_DATA_H 
	ror RECEIVED_DATA_L ; /2 /2 /2 /2 (=/16)
ret

; Grenzen für Relative Luftfeuchtigkeit in die dafür vorgesehenen Register laden
LOAD_HUM_BOUNDS:
	push r16

	ldi r16, 0b00000100
	mov MEAS_LB_H, r16
	ldi r16, 0b10111001 ; L = Lowbits | Dec. Wert = 1209 - 40RH
	mov MEAS_LB_L, r16
	ldi r16, 0b00000111 ;
	mov MEAS_UB_H, r16 
	ldi r16, 0b00101101 ; 1837 -  60RH
	mov MEAS_UB_L, r16

/*	Bei dieser Einstellung sind Änderungen leichter zu sehen
	
	ldi r16, 0b00000100 ;
	mov MEAS_UB_H, r16 
	ldi r16, 0b11101101 ; 
	mov MEAS_UB_L, r16
*/
	
	pop r16
ret

; Grenzen für Temperatur in die dafür vorgesehenen Register laden
LOAD_TEMP_BOUNDS:
	push r16

	ldi r16, 0b00010111 ; LB = Lowerbound, H = Highbits
	mov MEAS_LB_H, r16
	ldi r16, 0b01111010 ; L = Lowbits | Dec. Wert = 6010 = 20°
	mov MEAS_LB_L, r16

	ldi r16, 0b00011010 ; 
	mov MEAS_UB_H, r16
	ldi r16, 0b10100110 ; 6310 = 23°
	mov MEAS_UB_L, r16

	pop r16
ret


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

ON_YELLOW_LED:
	sbi PORTA, PORTA6
ret

ON_GREEN_LED:
	sbi PORTA, PORTA7
ret

OFF_RED_LED:
	cbi PORTA, PORTA5
ret

OFF_YELLOW_LED:
	cbi PORTA, PORTA6
ret

OFF_GREEN_LED:
	cbi PORTA, PORTA7
ret

INT0_BUTTON_MIDDLE:
	cli
	push r16
	push r17

	; Toggle gelbe LED
	in r16, PINA
	ldi r17, 0b01000000
	eor r16, r17
	out PORTA, r16


	pop r17
	pop r16
	sei
reti

INT1_BUTTON_RIGHT:
	cli
	push r16
	push r17


	; Toggle grüne LED
	in r16, PINA
	ldi r17, 0b10000000
	eor r16, r17
	out PORTA, r16

	pop r17
	pop r16
	sei
reti

INT2_BUTTON_LEFT:
	cli
	push r16
	push r17

	; Toggle rote LED
	in r16, PINA
	ldi r17, 0b00100000
	eor r16, r17
	out PORTA, r16
	sei

	pop r17
	pop r16
	sei
reti

DELAY_24MS:
	push r16
	push r17

	; lade 31250 in compare register, weil 1/(4000000/64) * 1500 = 0.024
	ldi r17, 0b00000101
	ldi r16, 0b11011100
	out OCR1AH, r17
	out OCR1AL, r16 

	; starte counter bei 0
	rcall RESET_16BITCOUNTER

	loop_24:
		in r16, TIFR
		sbrs r16, OCF1A ; SBRS Skip if Bit in Register is Set 
	rjmp loop_24
	
	; reset match bit
	ldi r16, (1<<OCF1A) 
	out TIFR, r16

	pop r17
	pop r16
ret

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

; Gehört zu DELAY_500MS / Kann für andere Delay-Routinen verwendet werden
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




;############################# BEGIN CALC_MEAS ##########################################
;# Überprüft, ob vom Sender empfangene Bytes im korrekten Wertebereich sind und setzt	#
;# entsprechend der Werte die LEDS														#
;# Rote LED leuchtet:	Relative Luftfeuchtigkeit ist kleiner 40 oder größer 60			#
;# Gelbe LED leuchtet:	Temperatur ist kleiner 20°C oder größer 23°C					#
;# Grüne LED leuchtet:	Relative Luftfeuchtigkeit zwischen 40-60 und					#
;#						Temperatur zwischen 20°C - 23°C									#
;########################################################################################

CALC_MEAS:
		push r16

		cp RECEIVED_DATA_H, MEAS_LB_H ; Erst High bits der unteren Grenze checken
		brlo BAD_LED_STATE 				   ; Branch if Lower (Unsigned) | Wenn data < humidity -> LED ROT 
		breq maybe_bad_state
	
	continue_CALC_MEAS:
		mov r16, MEAS_UB_H
		cp r16, RECEIVED_DATA_H ; wenn r16 < data -> LED ROT
		brlo BAD_LED_STATE
		breq maybe_bad_state2
	continue_CALC_MEAS2:
		rjmp GOOD_LED_STATE
				
	continue_CALC_MEAS3:
		pop r16

		cpi MODE, TEMPERATURE_MODE
		breq calc_temp_mode
		rjmp cont_main
ret 

calc_temp_mode:
	jmp cont_main2

maybe_bad_state:
	cp RECEIVED_DATA_L, MEAS_LB_L ; Dann Low bits der unteren Grenze checken
	brlo BAD_LED_STATE
	rjmp continue_CALC_MEAS
ret

maybe_bad_state2:
	mov r16, MEAS_UB_L
	cp r16, RECEIVED_DATA_L
	brlo BAD_LED_STATE
	rjmp continue_CALC_MEAS2
ret

BAD_LED_STATE:
	cpi MODE, TEMPERATURE_MODE
	breq BAD_TEMPERATURE
	brne BAD_HUMIDITY
	cont_BAD_LED_STATE:
	rjmp continue_CALC_MEAS3
ret

BAD_TEMPERATURE:
	rcall OFF_GREEN_LED
	rcall ON_YELLOW_LED
	jmp cont_BAD_LED_STATE

BAD_HUMIDITY:
	rcall OFF_GREEN_LED
	rcall ON_RED_LED
	jmp cont_BAD_LED_STATE

GOOD_LED_STATE:
	push r16
	push r17

	cpi MODE, TEMPERATURE_MODE
	breq GOOD_TEMPERATURE
	brne GOOD_HUMIDITY
	cont_GOOD_LED_STATE:

	; Check if temperature and humidity is good by checking state of LED
	ldi r16, (1<<PORTA6) | (1<<PORTA5)
	in r17, PINA
	and r17, r16
	cpi r17, 0x00
	; if both good -> GREEN LED ON
	breq GOOD_TEMP_HUM
	cont_GOOD_LED_STATE2:

	pop r17
	pop r16
	rjmp continue_CALC_MEAS3
ret

GOOD_TEMPERATURE:
	rcall OFF_YELLOW_LED
	jmp cont_GOOD_LED_STATE

GOOD_HUMIDITY:
	rcall OFF_RED_LED
	jmp cont_GOOD_LED_STATE

GOOD_TEMP_HUM:
	rcall ON_GREEN_LED
	jmp cont_GOOD_LED_STATE2

;############################### End CALC_MEAS #########################################



;############################# BEGIN SEND_SENSOR ########################################
;# Kommuniziert mit dem SHT75 Sensor. Sendet je nachdem wie MODE eingestellt ist		#
;# eine Anfrage für die Temperatur (TEMPERATURE_MODE) oder Relative Luftfeuchtigkeit	#  
;# (HUMIDITY_MODE). RECEIVED_DATA_H und RECEIVED_DATA_L beinhalten die Antwort des		#
;# Sensors.																				#
;########################################################################################

; PC1 SDA (DATA Pin am Sensor), PC0 SCL(SCK)
SEND_SENSOR:
	; als output setzen
	sbi DDRC, PORTC1
	sbi DDRC, PORTC0
	; TODO nicht nach jedem mal reset_sequence starten, aber stört nicht wenn dabei ist
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

ret

ERROR_NO_ACK:
	;TODO eventuell transmission neustarten lassen wenn kein ack gekommen ist. oder einfach reset knopf drücken

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

	cpi MODE, SOFTRESET_MODE
	breq delay_for_softreset
	cont_send_command:
		

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

delay_for_softreset:
	rcall TOGGLE_YELLOW_LED
	rcall DELAY_500MS
	rcall TOGGLE_YELLOW_LED
	; TODO transstart nach softwarereset initialisieren + MODE überschreiben
	; BSP:
	; ldi MODE, HUMIDITY_MODE
	; rcall SEND_SENSOR
	; eventuell software reset bei einem interrupt?
	rjmp cont_send_command

;############################# END SEND_SENSOR #########################################


; Lookup Tables für Temperatur und Humidity
LUT_Temperatur:
.db -40,-39,-39,-38,-38,-37,-36,-36,-35,-34,-34,-33,-32,-32,-31,-31
.db -30,-29,-29,-28,-27,-27,-26,-25,-25,-24,-23,-23,-22,-22,-21,-20
.db -20,-19,-18,-18,-17,-16,-16,-15,-15,-14,-13,-13,-12,-11,-11,-10
.db -9,-9,-8,-7,-7,-6,-6,-5,-4,-4,-3,-2,-2,-1,0,0
.db 1,2,2,3,3,4,5,5,6,7,7,8,9,9,10,10
.db 11,12,12,13,14,14,15,16,16,17,18,18,19,19,20,21
.db 21,22,23,23,24,25,25,26,26,27,28,28,29,30,30,31
.db 32,32,33,34,34,35,35,36,37,37,38,39,39,40,41,41
.db 42,42,43,44,44,45,46,46,47,48,48,49,50,50,51,51
.db 52,53,53,54,55,55,56,57,57,58,58,59,60,60,61,62
.db 62,63,64,64,65,66,66,67,67,68,69,69,70,71,71,72
.db 73,73,74,74,75,76,76,77,78,78,79,80,80,81,82,82
.db 83,83,84,85,85,86,87,87,88,89,89,90,90,91,92,92
.db 93,94,94,95,96,96,97,98,98,99,99,100,101,101,102,103
.db 103,104,105,105,106,106,107,108,108,109,110,110,111,112,112,113
.db 114,114,115,115,116,117,117,118,119,119,120,121,121,122,122,123


LUT_Humidity:
.db 0,0,0,0,0,1,1,2,3,3,4,4,5,6,6,7
.db 7,8,8,9,10,10,11,11,12,12,13,14,14,15,15,16
.db 16,17,17,18,19,19,20,20,21,21,22,22,23,24,24,25
.db 25,26,26,27,27,28,28,29,30,30,31,31,32,32,33,33
.db 34,34,0x23,0x23,36,37,37,38,38,39,39,40,40,41,41,42
.db 42,43,43,44,44,45,45,46,46,47,47,48,49,49,50,50
.db 51,51,52,52,53,53,54,54,55,55,56,56,57,57,58,58
.db 59,59,60,60,61,61,62,62,63,63,64,64,64,65,65,66
.db 66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,74
.db 74,75,75,75,76,76,77,77,78,78,79,79,80,80,81,81
.db 81,82,82,83,83,84,84,85,85,86,86,86,87,87,88,88
.db 89,89,90,90,90,91,91,92,92,93,93,93,94,94,95,95
.db 96,96,96,97,97,98,98,99,99,99,100,100,100,100,100,100
.db 100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100
.db 100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100
.db 100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100