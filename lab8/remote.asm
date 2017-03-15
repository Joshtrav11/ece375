;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the TRANSMIT skeleton file for Lab 8 of ECE 375
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
.def 	waitcnt = r18			; Wait counter
.def	ilcnt = r19				; Inner Loop Counter
.def	olcnt = r20				; Outer Loop Counter

.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ 	WTime = 25				; 1 second

; Use these action codes between the remote and robot
; MSB = 1 thus:
; control signals are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwd =  ($80|1<<(EngDirR-1)|1<<(EngDirL-1))	;0b10110000 Move Forward Action Code
.equ	MovBck =  ($80|$00)								;0b10000000 Move Backward Action Code
.equ	TurnR =   ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Action Code
.equ	TurnL =   ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Action Code
.equ	Halt =    ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Action Code
.equ	Freeze =  0b11111000

.equ	SendComplete = 0b01100000

.equ	BotAddress = 13 ;(Enter your robot's address here (8 bits))
; 0b00001101

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

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
	;I/O Ports
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

	;Enable transmitter
	ldi		mpr, (1<<TXEN0)
	sts		UCSR1B, mpr
	;Set frame format: 8 data bits, 2 stop bits
	ldi		mpr, (1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10 | 1<<UPM11) ; UCSZn1 and UCSZn0 to 
	; 1's for 8 data bits and USBSn set to 1 for 2 stopping bits
	sts 	UCSR1C, mpr

	sei

	;Other

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
		in		mpr, PIND		; Get button input from Port D
		cpi		mpr, 0b11111110	; Check for first button input
		brne	NEXT1			; Continue with next check
		rcall	ForwardCommand		; Call the subroutine ForwardCommand
		rjmp	MAIN			; Loop back to main
NEXT1:	
		cpi		mpr, 0b11111101	; Check for 2nd button input 
		brne	NEXT2			; No button input, continue program
		rcall	BackwardCommand			; Call subroutine BackwardCommand
		rjmp	MAIN			; Continue through main
NEXT2:	
		cpi		mpr, 0b11101111	; Check for 5th button input 
		brne	NEXT3			; No button input, continue program
		rcall	TurnRCommand			; Call subroutine TurnRCommand
		rjmp	MAIN			; Continue through main
NEXT3:	
		cpi		mpr, 0b11011111	; Check for 6th button input 
		brne	NEXT4			; No button input, continue program
		rcall	TurnLCommand			; Call subroutine TurnLCommand
		rjmp	MAIN			; Continue through main
NEXT4:	
		cpi		mpr, 0b10111111	; Check for 7th button input 
		brne	NEXT5			; No button input, continue program
		rcall	HaltCommand		; Call subroutine HaltCommand
		rjmp	MAIN			; Continue through main
NEXT5:	
		cpi		mpr, 0b01111111	; Check for 8th button input 
		brne	MAIN			; No button input, continue program
		rcall	FreezeCommand			; Call subroutine FreezeCommand
		rjmp	MAIN			; Continue through main

;***********************************************************
;*	Functions and Subroutines
;***********************************************************
ForwardCommand:

		rcall TransmitAddress
		
		ldi		mpr, MovFwd
		out		PORTB, mpr
		sts		UDR1, mpr

		
Forward_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	Forward_Loop

		ret

BackwardCommand:

		rcall TransmitAddress

		ldi		mpr, MovBck
		out		PORTB, mpr
		sts		UDR1, mpr

Backward_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	Backward_Loop

		ret

TurnRCommand:

		rcall TransmitAddress
		
		ldi		mpr, TurnR
		out		PORTB, mpr
		sts		UDR1, mpr

TurnR_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	TurnR_Loop

		ret

TurnLCommand:

		rcall TransmitAddress
		
		ldi		mpr, TurnL
		out		PORTB, mpr
		sts		UDR1, mpr

TurnL_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	TurnL_Loop

		ret

HaltCommand:

		rcall TransmitAddress
		
		ldi		mpr, Halt
		out		PORTB, mpr
		sts		UDR1, mpr

Halt_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	Halt_Loop

		ret

;FreezeCommand:

		;ldi		mpr, 0b01010101
		;out		PORTB, mpr
		;sts		UDR1, mpr

;Freeze_Loop:
		;lds		mpr, UCSR1A
		;sbrs	mpr, TXC1
		;rjmp	Freeze_Loop

		;ldi		waitcnt, WTime	; Wait for 1 second
		;rcall	Wait

		;ret

FreezeCommand:

		rcall TransmitAddress
		
		ldi		mpr, Freeze
		out		PORTB, mpr
		sts		UDR1, mpr

Freeze_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	Freeze_Loop

		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait

		ret

TransmitAddress:

		ldi		mpr, BotAddress
		sts		UDR1, mpr

Transmit_Loop:
		lds		mpr, UCSR1A
		cpi		mpr, SendComplete
		brne	Transmit_Loop

		ret
; Wait function from previous labs
Wait:
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register

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