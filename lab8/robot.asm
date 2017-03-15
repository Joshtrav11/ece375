;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the RECEIVE skeleton file for Lab 8 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Josh Matteson
;*	   Date: 02/02/20017
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def 	mpr2 = r17				; Second Multi-Puporse Register
.def 	waitcnt = r18			; Wait counter
.def	ilcnt = r19				; Inner Loop Counter
.def	olcnt = r20				; Outer Loop Counter

.equ 	WTime = 100				; 1 second

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ	BotAddress = 13 ;(Enter your robot's address here (8 bits))
.equ 	Command = $0100
.equ	FreezeCount = $0140

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////
.equ	MovFwd =  (1<<EngDirR|1<<EngDirL)	;0b01100000 Move Forward Action Code
.equ	MovBck =  $00						;0b00000000 Move Backward Action Code
.equ	TurnR =   (1<<EngDirL)				;0b01000000 Turn Right Action Code
.equ	TurnL =   (1<<EngDirR)				;0b00100000 Turn Left Action Code
.equ	Halt = ($80|1<<(EngEnR-1)|1<<(EngEnL-1))	;0b10010000 Halt Action Code
.equ 	Freeze = 	0b01010101				;Command to freeze this robot
.equ 	SendFreeze = 0b11111000
.equ 	SendFreezeR = 0b011110000

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

;Should have Interrupt vectors for:
;- Right whisker
.org 	$0002 ;{IRQ0 => pin0, PORTD}
		rcall HitRight ; Call hit right function
		reti ; Return from interrupt
;- Left whisker
.org 	$0004 ;{IRQ1 => pin1, PORTD}
		rcall HitLeft ; Call hit left function
		reti ; Return from interrupt
;- USART receive
.org 	$003C ; USART1, Rx complete interrupt
		rcall USART_Receive ; Call USART_Receive
		reti ; Return from interrupt

.org	$0046					; End of Interrupt Vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
	;Stack Pointer (VERY IMPORTANT!!!!)
	LDI 	mpr, LOW(RAMEND) ; Low Byte of End SRAM Address
	OUT 	SPL, mpr ; Write byte to SPL
	LDI 	mpr, HIGH(RAMEND) ; High Byte of End SRAM Address
	OUT 	SPH, mpr ; Write byte to SPH
	; I/O Ports
	; Initialize Port B for output
	ldi		mpr, $00		; Initialize Port B Data Register
	out		PORTB, mpr		; so all Port B outputs are low	
	ldi		mpr, $FF		; Set Port B Data Direction Register
	out		DDRB, mpr		; for output
	
	; Initialize Port D for input
	ldi		mpr, $FF		; Initialize Port D Data Register
	out		PORTD, mpr		; so all Port D inputs are Tri-State
	ldi		mpr, $00		; Set Port D Data Direction Register
	out		DDRD, mpr		; for input

	;USART1
	;Set baudrate at 2400bps
	; (16 MHz / (16 * 2400 [desired baud rate])) - 1 = 416
	ldi 	mpr, low(416) ; Load low byte into UBRR1L
	sts 	UBRR1L, mpr
	ldi 	mpr, high(416) ; Load high byte into UBRR1H
	sts 	UBRR1H, mpr
	;Don't know if this is necessary, but set UCSR1A to 0b00000000
	;ldi 	mpr, 0b00000000
	;sts 	UCSR1A, mpr
	;Enable receiver and enable receive interrupts
	ldi 	mpr, (1<<RXEN1 | 1<<RXCIE1 | 1<<TXEN0) ; 
	sts		UCSR1B, mpr
	;Set frame format: 8 data bits, 2 stop bits
	ldi		mpr, (1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10 | 1<<UCSZ11) ; UCSZn1 and UCSZn0 to 
	; 1's for 8 data bits and USBSn set to 1 for 2 stopping bits
	sts 	UCSR1C, mpr
	;External Interrupts

	;Set the External Interrupt Mask
	ldi		mpr, 0b00000011 ; Enable INTO and INT1 for interrupts
	out		EIMSK, mpr
	;Set the Interrupt Sense Control to falling edge detection
	ldi		mpr, 0b00001010 ; Set INT0 and INT1 to trigger on falling edge
	sts		EICRA, mpr

	sei
	;Other
	ldi 	YL, low(Command)
	ldi 	YH, high(command)
	ldi		mpr, 0
	st		Y, mpr
	
	ldi 	ZL, low(FreezeCount)
	ldi 	ZH, high(FreezeCount)
	ldi		mpr, 0b00000011
	st		Z, mpr

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
	;TODO: ???
		rjmp	MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

USART_Receive:

		push	mpr2
		push 	mpr
		in		mpr, SREG
		push	waitcnt			; Wait counter
		push 	mpr
		push	YL
		push	YH
		push	ZL
		push	ZH

		lds		mpr, UDR1
		cpi		mpr, Freeze
		breq	FreezeRobot
		sbrc	mpr, 7
		rjmp	CheckCommand
		rjmp	CheckID

CheckID:

		ldi 	YL, low(Command)
		ldi 	YH, high(command)
		ld		mpr2, Y
		
		cpi 	mpr, BotAddress
		breq	Correct
		ldi		mpr2, 0
		st		Y, mpr2

		rjmp 	USART_Receiver_end

Correct:

		ldi		mpr2, 1
		st		Y, mpr2

		rjmp 	USART_Receiver_end

CheckCommand:

		ldi 	YL, low(Command)
		ldi 	YH, high(command)
		ld		mpr2, Y
		sbrs	mpr2, 0
		rjmp	USART_Receiver_end
		rol 	mpr
		;ldi		waitcnt, WTime	; Wait for 1 second
		;rcall	Wait
		cpi		mpr, SendFreezeR
		breq	FreezeOthers
		out		PORTB, mpr

		rjmp	USART_Receiver_end

FreezeRobot:

		; Turn off interupts for duration of function
		cli

		ldi		mpr2, Halt
		rol		mpr2
		out		PORTB, mpr2
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait

		ldi		ZL, low(FreezeCount)
		ldi		ZH, high(FreezeCount)
		ld		mpr, Z

		dec		mpr
		breq	Die
		st		Z, mpr

		; Turn interupts back on
		ldi		mpr, 0b00000011
		out 	EIFR, mpr ; Clear External Interupt Flag Register
		sei

		rjmp 	USART_Receiver_end

FreezeOthers:

		ldi		mpr, 0b01010101
		out		PORTB, mpr
		sts		UDR1, mpr
Send_Loop:
		lds		mpr, UCSR1A
		sbrs	mpr, TXC1
		rjmp	Send_Loop
Clear_Input:
		lds 	mpr, UCSR1A
		sbrs 	mpr, RXC1
		rjmp 	Clear_Input
		lds		mpr, UDR1

		rjmp	USART_Receiver_end

		
USART_Receiver_end:

		pop		ZH
		pop 	ZL
		pop 	YH
		pop		YL
		pop 	mpr
		pop		waitcnt
		out		SREG, mpr
		pop 	mpr
		pop 	mpr2
		ret

Die:
		rjmp 	Die


HitRight:							; Begin a function with a label

		push	mpr			; Save mpr register
		push	waitcnt		; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Turn off interupts for duration of function
		cli

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Turn left for a second
		ldi		mpr, TurnL	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port

		; Turn interupts back on
		ldi		mpr, 0b00000011
		out 	EIFR, mpr ; Clear External Interupt Flag Register
		sei

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		
		ret						; End a function with RET

HitLeft:							; Begin a function with a label

		push	mpr			; Save mpr register
		push	waitcnt		; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Turn off interupts for duration of function
		cli

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for a second
		rcall	Wait			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for a second
		rcall	Wait			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port

		; Turn interupts back on
		ldi		mpr, 0b00000011 ; Left and Right bumper can interrupt again
		out 	EIFR, mpr ; Clear External Interupt Flag Register
		sei

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr

		ret				; Return from subroutine

Wait:
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register
; Wait function from previous labs
Loop:	
		ldi		olcnt, 224		; load olcnt register
OLoop:	
		ldi		ilcnt, 237		; load ilcnt register
ILoop:	
		dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt		; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt		; Decrement wait 
		brne	Loop			; Continue Wait loop	

		pop		olcnt		; Restore olcnt register
		pop		ilcnt		; Restore ilcnt register
		pop		waitcnt		; Restore wait register
		ret					; Return from subroutine

;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************
