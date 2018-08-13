.device	ATtiny25
.include "tn25def.inc"

.def temp = r16						;рабочая переменная
.def Razr0 = r17
.def Razr1 = r18
.def count = r20
.def count2 = r21

.set		PIN_SW = PINB0					; Кнопка
.set		PORT_SW = PORTB0
.set		PIN_SENSOR = PINB1				; Датчик
.set		PORT_SENSOR = PORTB1
.set		PIN_BUZ = PINB2					; Пищалка
.set		PORT_BUZ = PORTB2
.set		PIN_LED = PINB3					; Светодиодик
.set		PORT_LED = PORTB3


; EEPROM
.eseg
			VCC_EE:		.dw	0x01b1			; Калибровочное значение
											; результат возвращаемый АЦП
											; при напряжении питания 2.7V

; RAM, Оперативка
.dseg
.org	SRAM_START

			VCC:		.byte	2			
			TIMER:		.byte	1
			TIMER_VCC:	.byte	1

; FLASH, Память программы
.cseg

.org 0x0000								; Таблица векторов прерываний
			rjmp	RESET				; External Pin, Power-on Reset, Brown-out Reset, and Watchdog Reset
.org 0x0002
			rjmp	PCINT0_ISR			; Прерывание от кнопки


.org	INT_VECTORS_SIZE				; Конец таблицы векторов прерываний


PCINT0_ISR:	
			push	r25
			ldi		r25,70
			sts		TIMER,r25
			cbi		PORTB,PIN_BUZ
			pop		r25
	
			reti

; Начало основной программы
RESET:
			in		temp,MCUSR			; Выясняем причину ресета
			sbrc	temp,WDRF			; Если по таймеру
			rjmp	END_INIT			; Пропускаем инициализацию

			cli							; иначе
			ldi		temp,low(RAMEND)	; Инициализация стека
			out		SPL,temp
			sei

							; Настраиваем порт Ввода-вывода
			ldi		temp,(1<<PIN_BUZ) | (1<<PIN_LED)
			out		DDRB,temp



			sbi		PORTB,PIN_LED
			sbi		PORTB,PIN_BUZ

			ldi		YL,low(VCC)
			ldi		YH,high(VCC)

			ldi		r17,low(VCC_EE)
			ldi		r18,high(VCC_EE)
			rcall	EEPROM_read
			st		Y+,temp

			inc		r17
			rcall	EEPROM_read
			st		Y,temp
			
			ldi		Razr1,0x0F
			ldi		Razr0,0x00
			rcall	Delay

			cbi		PORTB,PIN_BUZ

			clr		temp				; Очищаем
			sts		TIMER,temp
			sts		TIMER_VCC,temp
								
END_INIT:
			rcall	WDT_off

			ldi		temp,0b00001100				; Подготовка АЦП
			out		ADMUX,temp

			clr		temp				; Настраиваем порт Ввода-вывода
			ldi		temp,(1<<PIN_BUZ) | (1<<PIN_LED)
			out		DDRB,temp

			sei
			
			sbi		PORTB,PIN_LED

			sbi		PORTB,PIN_SENSOR
			nop

			sbis	PINB,PIN_SENSOR
			rjmp	P01
			clr		temp
			sts		TIMER,temp

P01:
			sbis	PINB,PIN_SENSOR
			rcall	PERELIV

			cbi		PORTB,PIN_SENSOR

			ldi		Razr1,0x01
			ldi		Razr0,0x00
			rcall	Delay

			ldi		temp, (1<<ADEN) | (1<<ADSC)
			out		ADCSRA,temp

T01:
			sbis	ADCSRA,ADIF
			rjmp	T01

			in		temp,ADCSRA
			sbr		temp,(1<<ADIF)
			out		ADCSRA,temp


			ldi		YL,low(VCC)
			ldi		YH,high(VCC)

			ld		r22,Y+
			ld		r23,Y

			in		r20,ADCL
			in		r21,ADCH

			clr		temp
			out		ADCSRA,temp
			out		ADMUX,temp

			cp		r20,r22
			cpc		r21,r23
			brsh	LOW_VCC
			clr		temp
			sts		TIMER_VCC,temp

			rjmp	END_PROG

LOW_VCC:
			lds		count,TIMER_VCC
			cpi		count,0
			breq	LOW00
			dec		count
			sts		TIMER_VCC,count
			rjmp	END_PROG

LOW00:
			ldi		temp,36
			sts		TIMER_VCC,temp

			ldi		count,32
			
LOW01:
			sbi		PORTB,PIN_LED
			sbi		PORTB,PIN_BUZ
			
			ldi		Razr1,4
			ldi		Razr0,0
			rcall	Delay

			cbi		PORTB,PIN_LED
			cbi		PORTB,PIN_BUZ

			ldi		Razr1,12
			ldi		Razr0,0
			rcall	Delay

			dec		count
			brne	LOW01

END_PROG:
			rjmp	BYE_BYE

; Конец программы :-)



PERELIV:
			lds		temp,TIMER
			cpi		temp,0
			breq	SOUND00		
			dec		temp
			sts		TIMER,temp
			rjmp	END_PERELIV

SOUND00:
			ldi		temp,(1<<PCIE)
			out		GIMSK,temp

			ldi		temp,(1<<PCINT0)
			out		PCMSK,temp
			
SOUND01:
			sbi		PORTB,PIN_LED
			sbi		PORTB,PIN_BUZ
			
			ldi		Razr1,0x08
			ldi		Razr0,0x00
			rcall	Delay

			cbi		PORTB,PIN_LED
			cbi		PORTB,PIN_BUZ

			ldi		Razr1,0x40
			ldi		Razr0,0x00
			rcall	Delay

			sbic	PINB,PIN_SENSOR
			rjmp	S01

			lds		temp,TIMER
			cpi		temp,0
			breq	SOUND01

S01:		clr		temp
			out		GIMSK,temp
			out		PCMSK,temp

END_PERELIV:
			ret


DELAY:
		
			subi	Razr0,1
			sbci	Razr1,0
			brcc	DELAY
			ret

EEWrite:
			SBIC	EECR,EEPE  ; Ждем готовности памяти к записи. Крутимся в цикле
			RJMP	EEWrite  ; до тех пор пока не очистится флаг EEWE
			CLI		; Затем запрещаем прерывания.

			ldi r18, (0<<EEPM1)|(0<<EEPM0)
			out EECR, r18

			OUT		EEARH,R17  ; Загружаем адрес нужной ячейки
			OUT		EEARL,R16  ; старший и младший байт адреса
			OUT		EEDR,R22  ; и сами данные, которые нам нужно загрузить
			SBI		EECR,EEMPE  ; взводим предохранитель
			SBI		EECR,EEPE  ; записываем байт
			SEI  ; разрешаем прерывания
			RET

EEPROM_read:
								; Wait for completion of previous write
			sbic	EECR,EEPE
			rjmp	EEPROM_read
								; Set up address (r18:r17) in address register
			out		EEARH,r18
			out		EEARL,r17
								; Start eeprom read by writing EERE
			sbi		EECR,EERE
								; Read data from data register
			in		r16,EEDR
			ret

WDT_off:
			wdr
			ldi		r16,(0<<WDRF)
			out		MCUSR,r16
			in		r16,WDTCR
			ori		r16,(1<<WDCE)|(1<<WDE)
			out		WDTCR,r16
			ldi		r16,(0<<WDE)
			out		WDTCR,r16
			ret

BYE_BYE:
			cbi		PORTB,PIN_LED

			wdr							; Инициализируем WD таймер
			ldi		temp,(1<<WDCE)|(1<<WDE)|(1<<WDP0)|(1<<WDP3)
			out		WDTCR,temp

			ldi		temp,(1<<SM1)|(1<<SE)
			out		MCUCR,temp

			ldi		temp,(1<<PRTIM1)|(1<<PRTIM0)|(1<<PRUSI)|(1<<PRADC)
			out		PRR,temp

			ldi		temp,0b00111111
			out		DIDR0,temp

			ldi		temp,0b10000000
			out		ACSR,temp

			clr		temp
			out		ADCSRA,temp

			sbi		PORTB,4

			sei

			sleep
