
c:\Dokumente und Einstellungen\Administrator\Lokale Einstellungen\Temp\buildee46fb344f591937b0f338e7598e16fc.tmp\ShiftReg595.ino.elf:     file format elf32-avr


Disassembly of section .text:

00000000 <__vectors>:
		if (*p == find) *p = replace;
	}
}

void String::replace(const String& find, const String& replace)
{
       0:	0c 94 e6 00 	jmp	0x1cc	; 0x1cc <__ctors_end>
       4:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
       8:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
       c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      10:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      14:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      18:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      1c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      20:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      24:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      28:	0c 94 1d 0b 	jmp	0x163a	; 0x163a <__vector_10>
	if (len == 0 || find.len == 0) return;
      2c:	0c 94 af 09 	jmp	0x135e	; 0x135e <__vector_11>
      30:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      34:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      38:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      3c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      40:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
	int diff = replace.len - find.len;
      44:	0c 94 65 01 	jmp	0x2ca	; 0x2ca <__vector_17>
      48:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      4c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
	char *readFrom = buffer;
      50:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
	char *foundAt;
	if (diff == 0) {
      54:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
      58:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      5c:	0c 94 d1 0b 	jmp	0x17a2	; 0x17a2 <__vector_23>
      60:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      64:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      68:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
			memcpy(foundAt, replace.buffer, replace.len);
      6c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      70:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      74:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      78:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
			readFrom = foundAt + replace.len;
      7c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      80:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      84:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
		}
	} else if (diff < 0) {
      88:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
			len += diff;
		}
		strcpy(writeTo, readFrom);
	} else {
		unsigned int size = len; // compute size needed for result
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
      8c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      90:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      94:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
			memcpy(foundAt, replace.buffer, replace.len);
			readFrom = foundAt + replace.len;
		}
	} else if (diff < 0) {
		char *writeTo = buffer;
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
      98:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      9c:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      a0:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      a4:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
      a8:	0c 94 0e 01 	jmp	0x21c	; 0x21c <__bad_interrupt>
			unsigned int n = foundAt - readFrom;
      ac:	f9 02       	muls	r31, r25
      ae:	fc 02       	muls	r31, r28
			memcpy(writeTo, readFrom, n);
      b0:	eb 02       	muls	r30, r27
      b2:	ef 02       	muls	r30, r31
      b4:	f5 02       	muls	r31, r21
      b6:	1d 03       	fmul	r17, r21
      b8:	1d 03       	fmul	r17, r21
			writeTo += n;
      ba:	1d 03       	fmul	r17, r21
      bc:	00 03       	mulsu	r16, r16
			memcpy(writeTo, replace.buffer, replace.len);
      be:	04 03       	mulsu	r16, r20
      c0:	08 03       	fmul	r16, r16
      c2:	0e 03       	fmul	r16, r22
      c4:	12 03       	mulsu	r17, r18
      c6:	1d 03       	fmul	r17, r21
      c8:	18 03       	fmul	r17, r16

000000ca <__trampolines_end>:
      ca:	00 00       	nop
      cc:	00 02       	muls	r16, r16
			writeTo += replace.len;
      ce:	00 09       	sbc	r16, r0
      d0:	0f 00       	.word	0x000f	; ????
      d2:	00 03       	mulsu	r16, r16
      d4:	04 01       	movw	r0, r8
      d6:	00 0c       	add	r0, r0
	...

000000e9 <digital_pin_to_bit_mask_PGM>:
			readFrom = foundAt + find.len;
			len += diff;
      e9:	04 08 02 01 10 40 80 40 10 20 40 80 40 80 08 02     .....@.@. @.@...
			buffer[len] = 0;
			memcpy(buffer + index, replace.buffer, replace.len);
			index--;
		}
	}
}
      f9:	04 01 80 40 20 10 02 01 10 80 10 20 40 40 20        ...@ ...... @@ 

00000108 <digital_pin_to_port_PGM>:
     108:	04 04 04 04 04 03 04 05 02 02 02 02 04 03 02 02     ................
			memcpy(writeTo, replace.buffer, replace.len);
			writeTo += replace.len;
			readFrom = foundAt + find.len;
			len += diff;
		}
		strcpy(writeTo, readFrom);
     118:	02 02 06 06 06 06 06 06 04 04 02 02 02 04 04        ...............

00000127 <port_to_output_PGM>:
	} else {
		unsigned int size = len; // compute size needed for result
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
     127:	00 00 00 00 25 00 28 00 2b 00 2e 00 31 00           ....%.(.+...1.

00000135 <port_to_mode_PGM>:
			readFrom = foundAt + find.len;
			size += diff;
		}
		if (size == len) return;
     135:	00 00 00 00 24 00 27 00 2a 00 2d 00 30 00           ....$.'.*.-.0.

00000143 <_cdcInterface>:
		if (size > capacity && !changeBuffer(size)) return; // XXX: tell user!
     143:	08 0b 00 02 02 02 01 00 09 04 00 00 01 02 02 00     ................
		int index = len - 1;
     153:	00 05 24 00 10 01 05 24 01 01 01 04 24 02 06 05     ..$....$....$...
		while (index >= 0 && (index = lastIndexOf(find, index)) >= 0) {
     163:	24 06 00 01 07 05 81 03 10 00 40 09 04 01 00 02     $.........@.....
			readFrom = buffer + index + find.len;
     173:	0a 00 00 00 07 05 02 02 40 00 00 07 05 83 02 40     ........@......@
	...

00000185 <USB_DeviceDescriptorB>:
			memmove(readFrom + diff, readFrom, len - (readFrom - buffer));
     185:	12 01 00 02 ef 02 01 40 41 23 36 80 00 01 01 02     .......@A#6.....
			len += diff;
     195:	03 01                                               ..

00000197 <USB_DeviceDescriptor>:
     197:	12 01 00 02 00 00 00 40 41 23 36 80 00 01 01 02     .......@A#6.....
			buffer[len] = 0;
			memcpy(buffer + index, replace.buffer, replace.len);
     1a7:	03 01                                               ..

000001a9 <STRING_MANUFACTURER>:
     1a9:	41 72 64 75 69 6e 6f 20 4c 4c 43 00                 Arduino LLC.

000001b5 <STRING_PRODUCT>:
     1b5:	41 72 64 75 69 6e 6f 20 4c 65 6f 6e 61 72 64 6f     Arduino Leonardo
	...

000001c6 <STRING_LANGUAGE>:
			index--;
		}
	}
}
     1c6:	04 03 09 04                                         ....

000001ca <__ctors_start>:
     1ca:	93 04       	cpc	r9, r3

000001cc <__ctors_end>:
     1cc:	11 24       	eor	r1, r1
     1ce:	1f be       	out	0x3f, r1	; 63
     1d0:	cf ef       	ldi	r28, 0xFF	; 255
     1d2:	da e0       	ldi	r29, 0x0A	; 10
     1d4:	de bf       	out	0x3e, r29	; 62
     1d6:	cd bf       	out	0x3d, r28	; 61

000001d8 <__do_copy_data>:
     1d8:	11 e0       	ldi	r17, 0x01	; 1
     1da:	a0 e0       	ldi	r26, 0x00	; 0
     1dc:	b1 e0       	ldi	r27, 0x01	; 1
     1de:	e2 eb       	ldi	r30, 0xB2	; 178
     1e0:	ff e1       	ldi	r31, 0x1F	; 31
     1e2:	02 c0       	rjmp	.+4      	; 0x1e8 <__do_copy_data+0x10>
     1e4:	05 90       	lpm	r0, Z+
     1e6:	0d 92       	st	X+, r0
     1e8:	a8 3f       	cpi	r26, 0xF8	; 248
     1ea:	b1 07       	cpc	r27, r17
     1ec:	d9 f7       	brne	.-10     	; 0x1e4 <__do_copy_data+0xc>

000001ee <__do_clear_bss>:
     1ee:	22 e0       	ldi	r18, 0x02	; 2
     1f0:	a8 ef       	ldi	r26, 0xF8	; 248
     1f2:	b1 e0       	ldi	r27, 0x01	; 1
     1f4:	01 c0       	rjmp	.+2      	; 0x1f8 <.do_clear_bss_start>

000001f6 <.do_clear_bss_loop>:
     1f6:	1d 92       	st	X+, r1

000001f8 <.do_clear_bss_start>:
     1f8:	a6 37       	cpi	r26, 0x76	; 118
     1fa:	b2 07       	cpc	r27, r18
     1fc:	e1 f7       	brne	.-8      	; 0x1f6 <.do_clear_bss_loop>

000001fe <__do_global_ctors>:
     1fe:	11 e0       	ldi	r17, 0x01	; 1
     200:	cc ec       	ldi	r28, 0xCC	; 204
     202:	d1 e0       	ldi	r29, 0x01	; 1
     204:	04 c0       	rjmp	.+8      	; 0x20e <__do_global_ctors+0x10>
     206:	22 97       	sbiw	r28, 0x02	; 2
     208:	fe 01       	movw	r30, r28
     20a:	0e 94 b9 0f 	call	0x1f72	; 0x1f72 <__tablejump__>
     20e:	ca 3c       	cpi	r28, 0xCA	; 202
     210:	d1 07       	cpc	r29, r17
     212:	c9 f7       	brne	.-14     	; 0x206 <__do_global_ctors+0x8>
     214:	0e 94 be 0b 	call	0x177c	; 0x177c <main>
     218:	0c 94 d7 0f 	jmp	0x1fae	; 0x1fae <_exit>

0000021c <__bad_interrupt>:
     21c:	0c 94 00 00 	jmp	0	; 0x0 <__vectors>

00000220 <_Z20configure_interruptsv>:
#define TIMER2_PRESCALE_BY_128  ((1 << CS22)               | (1 << CS20)) // 125.000 MHz /   8.0 µs
#define TIMER2_PRESCALE_BY_256  ((1 << CS22) | (1 << CS21)              ) // 62.500 KHz /   16.0 µs
#define TIMER2_PRESCALE_BY_1024 ((1 << CS22) | (1 << CS21) | (1 << CS20)) // 15.625 KHz /   64.0 µs

void configure_interrupts(void) {
  noInterrupts();
     220:	f8 94       	cli
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                          // TIMER1 CTC mode ("Clear Timer on Compare")
     222:	10 92 80 00 	sts	0x0080, r1
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | TIMER1_PRESCALE_BY_1;   // TIMER1 CTC mode, prescaling
     226:	89 e0       	ldi	r24, 0x09	; 9
     228:	80 93 81 00 	sts	0x0081, r24
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt for TIMER1
     22c:	ef e6       	ldi	r30, 0x6F	; 111
     22e:	f0 e0       	ldi	r31, 0x00	; 0
     230:	80 81       	ld	r24, Z
     232:	82 60       	ori	r24, 0x02	; 2
     234:	80 83       	st	Z, r24
  OCR1A  = (50 * 16) - 1;   // compare match register
     236:	8f e1       	ldi	r24, 0x1F	; 31
     238:	93 e0       	ldi	r25, 0x03	; 3
     23a:	90 93 89 00 	sts	0x0089, r25
     23e:	80 93 88 00 	sts	0x0088, r24

  // Timer 3 (16 bit)
  TCCR3A =  ((1 << WGM31) & 0x00) | ((1 << WGM30) & 0x00);                        // TIMER3 normal mode
     242:	10 92 90 00 	sts	0x0090, r1
  TCCR3B =  ((1 << WGM33) & 0x00) | ((1 << WGM32) & 0x00) | TIMER3_PRESCALE_BY_1; // TIMER3 normal mode @ 2MHz / 0.5µs period (overflow after 32.768 ms)
     246:	81 e0       	ldi	r24, 0x01	; 1
     248:	80 93 91 00 	sts	0x0091, r24
  TIMSK3 = 0; // no interrupt from Timer3
     24c:	10 92 71 00 	sts	0x0071, r1

/* there is no TIMER2 on the Atmega32uXX
  TCCR2 = 0 | TIMER2_PRESCALE_BY_8; // TIMER2 in normal mode @ 2MHz / 0.5µs period (overflow after 128 µs)
  TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B)); // no interrupts from TIMER2
*/ 
  interrupts();
     250:	78 94       	sei
     252:	08 95       	ret

00000254 <setup>:
#define s595_shift_LO() (PORTD =                  0); s595_SHCP_HI; //(s595_state_int <<= 1)  //  s595_SHCP_LO; s595_DS_LO; s595_SHCP_HI //    
#define s595_shift_HI() (PORTD = (1 << s595_DS_bit)); s595_SHCP_HI; //(s595_state_int <<= 1); (s595_state_int |= 1);   //  s595_SHCP_LO; s595_DS_HI; s595_SHCP_HI //   


void setup() {
  Serial.begin(57600);
     254:	40 e0       	ldi	r20, 0x00	; 0
     256:	51 ee       	ldi	r21, 0xE1	; 225
     258:	60 e0       	ldi	r22, 0x00	; 0
     25a:	70 e0       	ldi	r23, 0x00	; 0
     25c:	85 e0       	ldi	r24, 0x05	; 5
     25e:	92 e0       	ldi	r25, 0x02	; 2
     260:	0e 94 7e 04 	call	0x8fc	; 0x8fc <_ZN7Serial_5beginEm>

  DDRD |= B01011111;
     264:	8a b1       	in	r24, 0x0a	; 10
     266:	8f 65       	ori	r24, 0x5F	; 95
     268:	8a b9       	out	0x0a, r24	; 10
  
  pinMode(s595_OE_pin,   OUTPUT);  // OE (active-low)
     26a:	61 e0       	ldi	r22, 0x01	; 1
     26c:	86 e0       	ldi	r24, 0x06	; 6
     26e:	0e 94 1e 03 	call	0x63c	; 0x63c <pinMode>
  pinMode(s595_MR_pin,   OUTPUT);  // MR (active-low)
     272:	61 e0       	ldi	r22, 0x01	; 1
     274:	85 e0       	ldi	r24, 0x05	; 5
     276:	0e 94 1e 03 	call	0x63c	; 0x63c <pinMode>
  pinMode(s595_STCP_pin, OUTPUT);  // STCP ("latch-out")
     27a:	61 e0       	ldi	r22, 0x01	; 1
     27c:	84 e0       	ldi	r24, 0x04	; 4
     27e:	0e 94 1e 03 	call	0x63c	; 0x63c <pinMode>
  pinMode(s595_SHCP_pin, OUTPUT);  // SHCP ("shift-out")
     282:	61 e0       	ldi	r22, 0x01	; 1
     284:	83 e0       	ldi	r24, 0x03	; 3
     286:	0e 94 1e 03 	call	0x63c	; 0x63c <pinMode>
  pinMode(s595_DS_pin,   OUTPUT);  // DS (serial data)
     28a:	61 e0       	ldi	r22, 0x01	; 1
     28c:	82 e0       	ldi	r24, 0x02	; 2
     28e:	0e 94 1e 03 	call	0x63c	; 0x63c <pinMode>

  digitalWrite(s595_OE_pin, HIGH);
     292:	61 e0       	ldi	r22, 0x01	; 1
     294:	86 e0       	ldi	r24, 0x06	; 6
     296:	0e 94 57 03 	call	0x6ae	; 0x6ae <digitalWrite>
  digitalWrite(s595_MR_pin, LOW);
     29a:	60 e0       	ldi	r22, 0x00	; 0
     29c:	85 e0       	ldi	r24, 0x05	; 5
     29e:	0e 94 57 03 	call	0x6ae	; 0x6ae <digitalWrite>
  digitalWrite(s595_STCP_pin, LOW);
     2a2:	60 e0       	ldi	r22, 0x00	; 0
     2a4:	84 e0       	ldi	r24, 0x04	; 4
     2a6:	0e 94 57 03 	call	0x6ae	; 0x6ae <digitalWrite>
  digitalWrite(s595_SHCP_pin, LOW);
     2aa:	60 e0       	ldi	r22, 0x00	; 0
     2ac:	83 e0       	ldi	r24, 0x03	; 3
     2ae:	0e 94 57 03 	call	0x6ae	; 0x6ae <digitalWrite>
  digitalWrite(s595_DS_pin, LOW);
     2b2:	60 e0       	ldi	r22, 0x00	; 0
     2b4:	82 e0       	ldi	r24, 0x02	; 2
     2b6:	0e 94 57 03 	call	0x6ae	; 0x6ae <digitalWrite>
  
  s595_off();
     2ba:	5f 9a       	sbi	0x0b, 7	; 11
  s595_clear();
     2bc:	46 98       	cbi	0x08, 6	; 8
     2be:	46 9a       	sbi	0x08, 6	; 8
  s595_latch();
     2c0:	5c 98       	cbi	0x0b, 4	; 11
     2c2:	5c 9a       	sbi	0x0b, 4	; 11
  s595_on();
     2c4:	5f 98       	cbi	0x0b, 7	; 11

  configure_interrupts();
     2c6:	0c 94 10 01 	jmp	0x220	; 0x220 <_Z20configure_interruptsv>

000002ca <__vector_17>:
byte pwm_tick = pwm_tickCount;
byte pwm_row_index = pwm_rowCount;

byte rowBuf[(pwm_tickCount * pwm_columnCount) >> 3];

ISR(TIMER1_COMPA_vect) {
     2ca:	1f 92       	push	r1
     2cc:	0f 92       	push	r0
     2ce:	0f b6       	in	r0, 0x3f	; 63
     2d0:	0f 92       	push	r0
     2d2:	11 24       	eor	r1, r1
     2d4:	2f 93       	push	r18
     2d6:	3f 93       	push	r19
     2d8:	4f 93       	push	r20
     2da:	5f 93       	push	r21
     2dc:	8f 93       	push	r24
     2de:	9f 93       	push	r25
     2e0:	ef 93       	push	r30
     2e2:	ff 93       	push	r31
  TCNT3 = 0;
     2e4:	10 92 95 00 	sts	0x0095, r1
     2e8:	10 92 94 00 	sts	0x0094, r1
  if (!timer_enabled) {
     2ec:	80 91 fa 01 	lds	r24, 0x01FA
     2f0:	88 23       	and	r24, r24
     2f2:	09 f4       	brne	.+2      	; 0x2f6 <__vector_17+0x2c>
     2f4:	85 c0       	rjmp	.+266    	; 0x400 <pwm_clock_out+0x98>
    return;
  }
 
  s595_latch(); // latch out what we've done last time
     2f6:	5c 98       	cbi	0x0b, 4	; 11
     2f8:	5c 9a       	sbi	0x0b, 4	; 11
  s595_clear();
     2fa:	46 98       	cbi	0x08, 6	; 8
     2fc:	46 9a       	sbi	0x08, 6	; 8

  if (++pwm_tick >= pwm_tickCount) {
     2fe:	80 91 09 01 	lds	r24, 0x0109
     302:	8f 5f       	subi	r24, 0xFF	; 255
     304:	80 93 09 01 	sts	0x0109, r24
     308:	80 32       	cpi	r24, 0x20	; 32
     30a:	60 f0       	brcs	.+24     	; 0x324 <__vector_17+0x5a>
    if (++pwm_row_index >= pwm_rowCount) {
     30c:	80 91 08 01 	lds	r24, 0x0108
     310:	8f 5f       	subi	r24, 0xFF	; 255
     312:	8a 30       	cpi	r24, 0x0A	; 10
     314:	18 f4       	brcc	.+6      	; 0x31c <__vector_17+0x52>
     316:	80 93 08 01 	sts	0x0108, r24
     31a:	02 c0       	rjmp	.+4      	; 0x320 <__vector_17+0x56>
      pwm_row_index = 0;
     31c:	10 92 08 01 	sts	0x0108, r1
    }
    pwm_tick = 0;
     320:	10 92 09 01 	sts	0x0109, r1
  }

  uint16_t temp = TCNT3;
     324:	20 91 94 00 	lds	r18, 0x0094
     328:	30 91 95 00 	lds	r19, 0x0095
  byte tick = pwm_tick;

  byte* rowData = (byte*)&data_rgb[pwm_row_index];
  if (pwm_row_index == 0) {
     32c:	80 91 08 01 	lds	r24, 0x0108
     330:	81 11       	cpse	r24, r1
     332:	66 c0       	rjmp	.+204    	; 0x400 <pwm_clock_out+0x98>

00000334 <pwm_tick_start>:
        "pwm_clock_out:                                                           \n"
        "         sbi  0x0b, 0          ; 2     // SHCP hi on PORTD                   \n"
        "         dec  r25              ; 1     // next column (r25 was initialized with pwm_columnCount) \n" 
        "         brne pwm_tick_loop    ; 1/2   // repeat if r25 still > 0            \n"
        : [tick] "=&r" (tick)
    );
     334:	40 91 09 01 	lds	r20, 0x0109
     338:	98 e1       	ldi	r25, 0x18	; 24

0000033a <pwm_tick_loop>:
     33a:	81 91       	ld	r24, Z+
     33c:	48 17       	cp	r20, r24
     33e:	69 f0       	breq	.+26     	; 0x35a <rgb_XX0>

00000340 <rgb_XX1>:
     340:	81 91       	ld	r24, Z+
     342:	48 17       	cp	r20, r24
     344:	29 f0       	breq	.+10     	; 0x350 <rgb_X01>

00000346 <rgb_X11>:
     346:	81 91       	ld	r24, Z+
     348:	48 17       	cp	r20, r24
     34a:	61 f0       	breq	.+24     	; 0x364 <rgb_011>

0000034c <rgb_111>:
     34c:	7b b8       	out	0x0b, r7	; 11
     34e:	0c c0       	rjmp	.+24     	; 0x368 <pwm_clock_out>

00000350 <rgb_X01>:
     350:	81 91       	ld	r24, Z+
     352:	48 17       	cp	r20, r24
     354:	29 f0       	breq	.+10     	; 0x360 <rgb_001>

00000356 <rgb_101>:
     356:	5b b8       	out	0x0b, r5	; 11
     358:	07 c0       	rjmp	.+14     	; 0x368 <pwm_clock_out>

0000035a <rgb_XX0>:
     35a:	81 91       	ld	r24, Z+
     35c:	48 17       	cp	r20, r24
     35e:	01 f0       	breq	.+0      	; 0x360 <rgb_001>

00000360 <rgb_001>:
     360:	3b b8       	out	0x0b, r3	; 11
     362:	02 c0       	rjmp	.+4      	; 0x368 <pwm_clock_out>

00000364 <rgb_011>:
     364:	3b b8       	out	0x0b, r3	; 11
     366:	00 c0       	rjmp	.+0      	; 0x368 <pwm_clock_out>

00000368 <pwm_clock_out>:
     368:	58 9a       	sbi	0x0b, 0	; 11
     36a:	9a 95       	dec	r25
     36c:	31 f7       	brne	.-52     	; 0x33a <pwm_tick_loop>
     36e:	ea e0       	ldi	r30, 0x0A	; 10
     370:	f1 e0       	ldi	r31, 0x01	; 1

    
    byte i = pwm_columnCount;
    while (i-- != 0) {  // faster than for "(byte i = 0; i < pwm_columnCount; i++)"
     372:	99 e1       	ldi	r25, 0x19	; 25
     374:	91 50       	subi	r25, 0x01	; 1
     376:	91 f0       	breq	.+36     	; 0x39c <pwm_clock_out+0x34>
      byte out = 0;
 
      if (*rowData++ > tick) {
     378:	80 81       	ld	r24, Z
     37a:	48 17       	cp	r20, r24
     37c:	10 f0       	brcs	.+4      	; 0x382 <pwm_clock_out+0x1a>
    );

    
    byte i = pwm_columnCount;
    while (i-- != 0) {  // faster than for "(byte i = 0; i < pwm_columnCount; i++)"
      byte out = 0;
     37e:	80 e0       	ldi	r24, 0x00	; 0
     380:	01 c0       	rjmp	.+2      	; 0x384 <pwm_clock_out+0x1c>
 
      if (*rowData++ > tick) {
        out |= (1 << s595_DS_bit);  // "bank" a
     382:	82 e0       	ldi	r24, 0x02	; 2
      }
      if (*rowData++ > tick) {  // "bank" b
     384:	51 81       	ldd	r21, Z+1	; 0x01
     386:	45 17       	cp	r20, r21
     388:	08 f4       	brcc	.+2      	; 0x38c <pwm_clock_out+0x24>
        out |= (1 << s595_DSb_bit);
     38a:	84 60       	ori	r24, 0x04	; 4
      }
      if (*rowData++ > tick) {  // "bank" c
     38c:	52 81       	ldd	r21, Z+2	; 0x02
     38e:	45 17       	cp	r20, r21
     390:	08 f4       	brcc	.+2      	; 0x394 <pwm_clock_out+0x2c>
        out |= (1 << s595_DSc_bit);
     392:	88 60       	ori	r24, 0x08	; 8
      }
      PORTD = out; // also sets SHCP and STCP low
     394:	8b b9       	out	0x0b, r24	; 11
      s595_SHCP_HI;
     396:	58 9a       	sbi	0x0b, 0	; 11
     398:	33 96       	adiw	r30, 0x03	; 3
     39a:	ec cf       	rjmp	.-40     	; 0x374 <pwm_clock_out+0xc>
*/
    
    }


    if (timer_intr || ((_pwm_tick >= pwm_tick) && (_pwm_tick < pwm_tickCount - 1))) {
     39c:	40 91 fb 01 	lds	r20, 0x01FB
     3a0:	80 91 fc 01 	lds	r24, 0x01FC
     3a4:	90 91 fd 01 	lds	r25, 0x01FD
     3a8:	41 11       	cpse	r20, r1
     3aa:	0a c0       	rjmp	.+20     	; 0x3c0 <pwm_clock_out+0x58>
     3ac:	50 91 fe 01 	lds	r21, 0x01FE
     3b0:	40 91 09 01 	lds	r20, 0x0109
     3b4:	54 17       	cp	r21, r20
     3b6:	50 f0       	brcs	.+20     	; 0x3cc <pwm_clock_out+0x64>
     3b8:	50 91 fe 01 	lds	r21, 0x01FE
     3bc:	5f 31       	cpi	r21, 0x1F	; 31
     3be:	30 f4       	brcc	.+12     	; 0x3cc <pwm_clock_out+0x64>
      missed++;
     3c0:	01 96       	adiw	r24, 0x01	; 1
     3c2:	90 93 fd 01 	sts	0x01FD, r25
     3c6:	80 93 fc 01 	sts	0x01FC, r24
     3ca:	1a c0       	rjmp	.+52     	; 0x400 <pwm_clock_out+0x98>
    } else {
      _missed = missed;
     3cc:	90 93 00 02 	sts	0x0200, r25
     3d0:	80 93 ff 01 	sts	0x01FF, r24
      missed = 0;
     3d4:	10 92 fd 01 	sts	0x01FD, r1
     3d8:	10 92 fc 01 	sts	0x01FC, r1
      _pwm_tick = pwm_tick;
     3dc:	40 93 fe 01 	sts	0x01FE, r20
      _tm_1 = temp;
     3e0:	30 93 02 02 	sts	0x0202, r19
     3e4:	20 93 01 02 	sts	0x0201, r18
      _tm_0 = TCNT3 + 16; // add 16 cycles for rest of fn plus returning
     3e8:	80 91 94 00 	lds	r24, 0x0094
     3ec:	90 91 95 00 	lds	r25, 0x0095
     3f0:	40 96       	adiw	r24, 0x10	; 16
     3f2:	90 93 04 02 	sts	0x0204, r25
     3f6:	80 93 03 02 	sts	0x0203, r24
      timer_intr = true;
     3fa:	81 e0       	ldi	r24, 0x01	; 1
     3fc:	80 93 fb 01 	sts	0x01FB, r24
    }

  }


}
     400:	ff 91       	pop	r31
     402:	ef 91       	pop	r30
     404:	9f 91       	pop	r25
     406:	8f 91       	pop	r24
     408:	5f 91       	pop	r21
     40a:	4f 91       	pop	r20
     40c:	3f 91       	pop	r19
     40e:	2f 91       	pop	r18
     410:	0f 90       	pop	r0
     412:	0f be       	out	0x3f, r0	; 63
     414:	0f 90       	pop	r0
     416:	1f 90       	pop	r1
     418:	18 95       	reti

0000041a <loop>:
    }
  }
}

  
void loop() {
     41a:	cf 92       	push	r12
     41c:	df 92       	push	r13
     41e:	ff 92       	push	r15
     420:	0f 93       	push	r16
     422:	1f 93       	push	r17
     424:	cf 93       	push	r28
     426:	df 93       	push	r29
  char buf[33];

// if there's any serial available, read it:
  if (Serial) {
     428:	85 e0       	ldi	r24, 0x05	; 5
     42a:	92 e0       	ldi	r25, 0x02	; 2
     42c:	0e 94 84 04 	call	0x908	; 0x908 <_ZN7Serial_cvbEv>
     430:	81 11       	cpse	r24, r1
     432:	0d c0       	rjmp	.+26     	; 0x44e <loop+0x34>
        Serial.println("ms");
      }
    } 
  }

  if (delay_ms == 0) {
     434:	80 91 06 01 	lds	r24, 0x0106
     438:	90 91 07 01 	lds	r25, 0x0107
     43c:	89 2b       	or	r24, r25
     43e:	09 f0       	breq	.+2      	; 0x442 <loop+0x28>
     440:	b8 c0       	rjmp	.+368    	; 0x5b2 <loop+0x198>
    s595_off();
     442:	5f 9a       	sbi	0x0b, 7	; 11
    i = 0;
     444:	10 92 f9 01 	sts	0x01F9, r1
     448:	10 92 f8 01 	sts	0x01F8, r1
     44c:	b2 c0       	rjmp	.+356    	; 0x5b2 <loop+0x198>
void loop() {
  char buf[33];

// if there's any serial available, read it:
  if (Serial) {
    timer_enabled = true;
     44e:	81 e0       	ldi	r24, 0x01	; 1
     450:	80 93 fa 01 	sts	0x01FA, r24
    if (timer_intr) {
     454:	80 91 fb 01 	lds	r24, 0x01FB
     458:	88 23       	and	r24, r24
     45a:	09 f4       	brne	.+2      	; 0x45e <loop+0x44>
     45c:	a2 c0       	rjmp	.+324    	; 0x5a2 <loop+0x188>
      uint16_t a      = _tm_0;
     45e:	c0 90 03 02 	lds	r12, 0x0203
     462:	d0 90 04 02 	lds	r13, 0x0204
      uint16_t b      = _tm_1;
     466:	00 91 01 02 	lds	r16, 0x0201
     46a:	10 91 02 02 	lds	r17, 0x0202
      uint16_t missed = _missed;
     46e:	c0 91 ff 01 	lds	r28, 0x01FF
     472:	d0 91 00 02 	lds	r29, 0x0200
      uint8_t  tick   = _pwm_tick;
     476:	f0 90 fe 01 	lds	r15, 0x01FE
      timer_intr = false;
     47a:	10 92 fb 01 	sts	0x01FB, r1
      Serial.print(a);
     47e:	4a e0       	ldi	r20, 0x0A	; 10
     480:	50 e0       	ldi	r21, 0x00	; 0
     482:	b6 01       	movw	r22, r12
     484:	85 e0       	ldi	r24, 0x05	; 5
     486:	92 e0       	ldi	r25, 0x02	; 2
     488:	0e 94 9a 05 	call	0xb34	; 0xb34 <_ZN5Print5printEji>
      Serial.print(" / ");
     48c:	6d e9       	ldi	r22, 0x9D	; 157
     48e:	71 e0       	ldi	r23, 0x01	; 1
     490:	85 e0       	ldi	r24, 0x05	; 5
     492:	92 e0       	ldi	r25, 0x02	; 2
     494:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
      Serial.print(b);
     498:	4a e0       	ldi	r20, 0x0A	; 10
     49a:	50 e0       	ldi	r21, 0x00	; 0
     49c:	b8 01       	movw	r22, r16
     49e:	85 e0       	ldi	r24, 0x05	; 5
     4a0:	92 e0       	ldi	r25, 0x02	; 2
     4a2:	0e 94 9a 05 	call	0xb34	; 0xb34 <_ZN5Print5printEji>
      Serial.print(" cycles (");
     4a6:	61 ea       	ldi	r22, 0xA1	; 161
     4a8:	71 e0       	ldi	r23, 0x01	; 1
     4aa:	85 e0       	ldi	r24, 0x05	; 5
     4ac:	92 e0       	ldi	r25, 0x02	; 2
     4ae:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
      Serial.print(a * 0.0625);
     4b2:	b6 01       	movw	r22, r12
     4b4:	80 e0       	ldi	r24, 0x00	; 0
     4b6:	90 e0       	ldi	r25, 0x00	; 0
     4b8:	0e 94 78 0e 	call	0x1cf0	; 0x1cf0 <__floatunsisf>
     4bc:	20 e0       	ldi	r18, 0x00	; 0
     4be:	30 e0       	ldi	r19, 0x00	; 0
     4c0:	40 e8       	ldi	r20, 0x80	; 128
     4c2:	5d e3       	ldi	r21, 0x3D	; 61
     4c4:	0e 94 2e 0f 	call	0x1e5c	; 0x1e5c <__mulsf3>
     4c8:	ab 01       	movw	r20, r22
     4ca:	bc 01       	movw	r22, r24
     4cc:	22 e0       	ldi	r18, 0x02	; 2
     4ce:	30 e0       	ldi	r19, 0x00	; 0
     4d0:	85 e0       	ldi	r24, 0x05	; 5
     4d2:	92 e0       	ldi	r25, 0x02	; 2
     4d4:	0e 94 8b 06 	call	0xd16	; 0xd16 <_ZN5Print5printEdi>
      Serial.print(" / ");
     4d8:	6d e9       	ldi	r22, 0x9D	; 157
     4da:	71 e0       	ldi	r23, 0x01	; 1
     4dc:	85 e0       	ldi	r24, 0x05	; 5
     4de:	92 e0       	ldi	r25, 0x02	; 2
     4e0:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
      Serial.print(b * 0.0625);
     4e4:	b8 01       	movw	r22, r16
     4e6:	80 e0       	ldi	r24, 0x00	; 0
     4e8:	90 e0       	ldi	r25, 0x00	; 0
     4ea:	0e 94 78 0e 	call	0x1cf0	; 0x1cf0 <__floatunsisf>
     4ee:	20 e0       	ldi	r18, 0x00	; 0
     4f0:	30 e0       	ldi	r19, 0x00	; 0
     4f2:	40 e8       	ldi	r20, 0x80	; 128
     4f4:	5d e3       	ldi	r21, 0x3D	; 61
     4f6:	0e 94 2e 0f 	call	0x1e5c	; 0x1e5c <__mulsf3>
     4fa:	ab 01       	movw	r20, r22
     4fc:	bc 01       	movw	r22, r24
     4fe:	22 e0       	ldi	r18, 0x02	; 2
     500:	30 e0       	ldi	r19, 0x00	; 0
     502:	85 e0       	ldi	r24, 0x05	; 5
     504:	92 e0       	ldi	r25, 0x02	; 2
     506:	0e 94 8b 06 	call	0xd16	; 0xd16 <_ZN5Print5printEdi>
      Serial.print(" micros)");
     50a:	6b ea       	ldi	r22, 0xAB	; 171
     50c:	71 e0       	ldi	r23, 0x01	; 1
     50e:	85 e0       	ldi	r24, 0x05	; 5
     510:	92 e0       	ldi	r25, 0x02	; 2
     512:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
      Serial.print(", s595_int: ");
      Serial.print(ltoa(((unsigned long)1 << pwm_columnCount) | s595_state_int, buf, 2));
      Serial.print(", s595_ext: ");
      Serial.print(ltoa(((unsigned long)1 << pwm_columnCount) | s595_state_ext, buf, 2));
      */
      Serial.print(", pwm_tick: ");
     516:	64 eb       	ldi	r22, 0xB4	; 180
     518:	71 e0       	ldi	r23, 0x01	; 1
     51a:	85 e0       	ldi	r24, 0x05	; 5
     51c:	92 e0       	ldi	r25, 0x02	; 2
     51e:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
      Serial.print(tick);
     522:	4a e0       	ldi	r20, 0x0A	; 10
     524:	50 e0       	ldi	r21, 0x00	; 0
     526:	6f 2d       	mov	r22, r15
     528:	85 e0       	ldi	r24, 0x05	; 5
     52a:	92 e0       	ldi	r25, 0x02	; 2
     52c:	0e 94 93 05 	call	0xb26	; 0xb26 <_ZN5Print5printEhi>
      Serial.print(", missed: ");
     530:	61 ec       	ldi	r22, 0xC1	; 193
     532:	71 e0       	ldi	r23, 0x01	; 1
     534:	85 e0       	ldi	r24, 0x05	; 5
     536:	92 e0       	ldi	r25, 0x02	; 2
     538:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
      Serial.print(missed);
     53c:	4a e0       	ldi	r20, 0x0A	; 10
     53e:	50 e0       	ldi	r21, 0x00	; 0
     540:	be 01       	movw	r22, r28
     542:	85 e0       	ldi	r24, 0x05	; 5
     544:	92 e0       	ldi	r25, 0x02	; 2
     546:	0e 94 9a 05 	call	0xb34	; 0xb34 <_ZN5Print5printEji>
      Serial.println();
     54a:	85 e0       	ldi	r24, 0x05	; 5
     54c:	92 e0       	ldi	r25, 0x02	; 2
     54e:	0e 94 cf 04 	call	0x99e	; 0x99e <_ZN5Print7printlnEv>
     552:	27 c0       	rjmp	.+78     	; 0x5a2 <loop+0x188>
    }
    while (Serial.available() > 0) {
      delay_ms = Serial.parseInt();
     554:	41 e0       	ldi	r20, 0x01	; 1
     556:	60 e0       	ldi	r22, 0x00	; 0
     558:	70 e0       	ldi	r23, 0x00	; 0
     55a:	85 e0       	ldi	r24, 0x05	; 5
     55c:	92 e0       	ldi	r25, 0x02	; 2
     55e:	0e 94 fe 06 	call	0xdfc	; 0xdfc <_ZN6Stream8parseIntE13LookaheadModec>
     562:	70 93 07 01 	sts	0x0107, r23
     566:	60 93 06 01 	sts	0x0106, r22
  
      if (Serial.read() == '\n') {
     56a:	85 e0       	ldi	r24, 0x05	; 5
     56c:	92 e0       	ldi	r25, 0x02	; 2
     56e:	0e 94 c7 03 	call	0x78e	; 0x78e <_ZN7Serial_4readEv>
     572:	0a 97       	sbiw	r24, 0x0a	; 10
     574:	b1 f4       	brne	.+44     	; 0x5a2 <loop+0x188>
        Serial.print("delay: ");
     576:	6c ec       	ldi	r22, 0xCC	; 204
     578:	71 e0       	ldi	r23, 0x01	; 1
     57a:	85 e0       	ldi	r24, 0x05	; 5
     57c:	92 e0       	ldi	r25, 0x02	; 2
     57e:	0e 94 c6 04 	call	0x98c	; 0x98c <_ZN5Print5printEPKc>
        Serial.print(delay_ms);
     582:	60 91 06 01 	lds	r22, 0x0106
     586:	70 91 07 01 	lds	r23, 0x0107
     58a:	4a e0       	ldi	r20, 0x0A	; 10
     58c:	50 e0       	ldi	r21, 0x00	; 0
     58e:	85 e0       	ldi	r24, 0x05	; 5
     590:	92 e0       	ldi	r25, 0x02	; 2
     592:	0e 94 7e 05 	call	0xafc	; 0xafc <_ZN5Print5printEii>
        Serial.println("ms");
     596:	64 ed       	ldi	r22, 0xD4	; 212
     598:	71 e0       	ldi	r23, 0x01	; 1
     59a:	85 e0       	ldi	r24, 0x05	; 5
     59c:	92 e0       	ldi	r25, 0x02	; 2
     59e:	0e 94 d3 04 	call	0x9a6	; 0x9a6 <_ZN5Print7printlnEPKc>
      Serial.print(tick);
      Serial.print(", missed: ");
      Serial.print(missed);
      Serial.println();
    }
    while (Serial.available() > 0) {
     5a2:	85 e0       	ldi	r24, 0x05	; 5
     5a4:	92 e0       	ldi	r25, 0x02	; 2
     5a6:	0e 94 b7 03 	call	0x76e	; 0x76e <_ZN7Serial_9availableEv>
     5aa:	18 16       	cp	r1, r24
     5ac:	19 06       	cpc	r1, r25
     5ae:	94 f2       	brlt	.-92     	; 0x554 <loop+0x13a>
     5b0:	41 cf       	rjmp	.-382    	; 0x434 <loop+0x1a>
    delay(delay_ms);
    */
  }

  
}
     5b2:	df 91       	pop	r29
     5b4:	cf 91       	pop	r28
     5b6:	1f 91       	pop	r17
     5b8:	0f 91       	pop	r16
     5ba:	ff 90       	pop	r15
     5bc:	df 90       	pop	r13
     5be:	cf 90       	pop	r12
     5c0:	08 95       	ret

000005c2 <turnOffPWM>:
//
//static inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline));
//static inline void turnOffPWM(uint8_t timer)
static void turnOffPWM(uint8_t timer)
{
	switch (timer)
     5c2:	90 e0       	ldi	r25, 0x00	; 0
     5c4:	fc 01       	movw	r30, r24
     5c6:	31 97       	sbiw	r30, 0x01	; 1
     5c8:	ef 30       	cpi	r30, 0x0F	; 15
     5ca:	f1 05       	cpc	r31, r1
     5cc:	b0 f5       	brcc	.+108    	; 0x63a <turnOffPWM+0x78>
     5ce:	ea 5a       	subi	r30, 0xAA	; 170
     5d0:	ff 4f       	sbci	r31, 0xFF	; 255
     5d2:	0c 94 b7 0f 	jmp	0x1f6e	; 0x1f6e <__tablejump2__>
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
     5d6:	80 91 80 00 	lds	r24, 0x0080
     5da:	8f 77       	andi	r24, 0x7F	; 127
     5dc:	03 c0       	rjmp	.+6      	; 0x5e4 <turnOffPWM+0x22>
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
     5de:	80 91 80 00 	lds	r24, 0x0080
     5e2:	8f 7d       	andi	r24, 0xDF	; 223
     5e4:	80 93 80 00 	sts	0x0080, r24
     5e8:	08 95       	ret
		#endif
		#if defined(TCCR1A) && defined(COM1C1)
		case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
     5ea:	80 91 80 00 	lds	r24, 0x0080
     5ee:	87 7f       	andi	r24, 0xF7	; 247
     5f0:	f9 cf       	rjmp	.-14     	; 0x5e4 <turnOffPWM+0x22>
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
     5f2:	84 b5       	in	r24, 0x24	; 36
     5f4:	8f 77       	andi	r24, 0x7F	; 127
     5f6:	02 c0       	rjmp	.+4      	; 0x5fc <turnOffPWM+0x3a>
		#endif
		
		#if defined(TCCR0A) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
     5f8:	84 b5       	in	r24, 0x24	; 36
     5fa:	8f 7d       	andi	r24, 0xDF	; 223
     5fc:	84 bd       	out	0x24, r24	; 36
     5fe:	08 95       	ret
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
     600:	80 91 90 00 	lds	r24, 0x0090
     604:	8f 77       	andi	r24, 0x7F	; 127
     606:	07 c0       	rjmp	.+14     	; 0x616 <turnOffPWM+0x54>
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
     608:	80 91 90 00 	lds	r24, 0x0090
     60c:	8f 7d       	andi	r24, 0xDF	; 223
     60e:	03 c0       	rjmp	.+6      	; 0x616 <turnOffPWM+0x54>
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
     610:	80 91 90 00 	lds	r24, 0x0090
     614:	87 7f       	andi	r24, 0xF7	; 247
     616:	80 93 90 00 	sts	0x0090, r24
     61a:	08 95       	ret
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
     61c:	80 91 c0 00 	lds	r24, 0x00C0
     620:	8f 77       	andi	r24, 0x7F	; 127
     622:	03 c0       	rjmp	.+6      	; 0x62a <turnOffPWM+0x68>
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
     624:	80 91 c0 00 	lds	r24, 0x00C0
     628:	8f 7d       	andi	r24, 0xDF	; 223
     62a:	80 93 c0 00 	sts	0x00C0, r24
     62e:	08 95       	ret
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
     630:	80 91 c2 00 	lds	r24, 0x00C2
     634:	87 7f       	andi	r24, 0xF7	; 247
     636:	80 93 c2 00 	sts	0x00C2, r24
     63a:	08 95       	ret

0000063c <pinMode>:
#define ARDUINO_MAIN
#include "wiring_private.h"
#include "pins_arduino.h"

void pinMode(uint8_t pin, uint8_t mode)
{
     63c:	cf 93       	push	r28
     63e:	df 93       	push	r29
	uint8_t bit = digitalPinToBitMask(pin);
     640:	90 e0       	ldi	r25, 0x00	; 0
     642:	fc 01       	movw	r30, r24
     644:	e7 51       	subi	r30, 0x17	; 23
     646:	ff 4f       	sbci	r31, 0xFF	; 255
     648:	24 91       	lpm	r18, Z
	uint8_t port = digitalPinToPort(pin);
     64a:	fc 01       	movw	r30, r24
     64c:	e8 5f       	subi	r30, 0xF8	; 248
     64e:	fe 4f       	sbci	r31, 0xFE	; 254
     650:	84 91       	lpm	r24, Z
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;
     652:	88 23       	and	r24, r24
     654:	49 f1       	breq	.+82     	; 0x6a8 <pinMode+0x6c>

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
     656:	90 e0       	ldi	r25, 0x00	; 0
     658:	88 0f       	add	r24, r24
     65a:	99 1f       	adc	r25, r25
     65c:	fc 01       	movw	r30, r24
     65e:	eb 5c       	subi	r30, 0xCB	; 203
     660:	fe 4f       	sbci	r31, 0xFE	; 254
     662:	a5 91       	lpm	r26, Z+
     664:	b4 91       	lpm	r27, Z
	out = portOutputRegister(port);
     666:	89 5d       	subi	r24, 0xD9	; 217
     668:	9e 4f       	sbci	r25, 0xFE	; 254
     66a:	fc 01       	movw	r30, r24
     66c:	c5 91       	lpm	r28, Z+
     66e:	d4 91       	lpm	r29, Z

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
     670:	9f b7       	in	r25, 0x3f	; 63

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
     672:	61 11       	cpse	r22, r1
     674:	08 c0       	rjmp	.+16     	; 0x686 <pinMode+0x4a>
		uint8_t oldSREG = SREG;
                cli();
     676:	f8 94       	cli
		*reg &= ~bit;
     678:	8c 91       	ld	r24, X
     67a:	20 95       	com	r18
     67c:	82 23       	and	r24, r18
     67e:	8c 93       	st	X, r24
		*out &= ~bit;
     680:	88 81       	ld	r24, Y
     682:	82 23       	and	r24, r18
     684:	0a c0       	rjmp	.+20     	; 0x69a <pinMode+0x5e>
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
     686:	62 30       	cpi	r22, 0x02	; 2
     688:	51 f4       	brne	.+20     	; 0x69e <pinMode+0x62>
		uint8_t oldSREG = SREG;
                cli();
     68a:	f8 94       	cli
		*reg &= ~bit;
     68c:	8c 91       	ld	r24, X
     68e:	32 2f       	mov	r19, r18
     690:	30 95       	com	r19
     692:	83 23       	and	r24, r19
     694:	8c 93       	st	X, r24
		*out |= bit;
     696:	88 81       	ld	r24, Y
     698:	82 2b       	or	r24, r18
     69a:	88 83       	st	Y, r24
     69c:	04 c0       	rjmp	.+8      	; 0x6a6 <pinMode+0x6a>
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
     69e:	f8 94       	cli
		*reg |= bit;
     6a0:	8c 91       	ld	r24, X
     6a2:	82 2b       	or	r24, r18
     6a4:	8c 93       	st	X, r24
		SREG = oldSREG;
     6a6:	9f bf       	out	0x3f, r25	; 63
	}
}
     6a8:	df 91       	pop	r29
     6aa:	cf 91       	pop	r28
     6ac:	08 95       	ret

000006ae <digitalWrite>:
		#endif
	}
}

void digitalWrite(uint8_t pin, uint8_t val)
{
     6ae:	0f 93       	push	r16
     6b0:	1f 93       	push	r17
     6b2:	cf 93       	push	r28
     6b4:	df 93       	push	r29
     6b6:	1f 92       	push	r1
     6b8:	cd b7       	in	r28, 0x3d	; 61
     6ba:	de b7       	in	r29, 0x3e	; 62
	uint8_t timer = digitalPinToTimer(pin);
     6bc:	28 2f       	mov	r18, r24
     6be:	30 e0       	ldi	r19, 0x00	; 0
     6c0:	f9 01       	movw	r30, r18
     6c2:	e6 53       	subi	r30, 0x36	; 54
     6c4:	ff 4f       	sbci	r31, 0xFF	; 255
     6c6:	84 91       	lpm	r24, Z
	uint8_t bit = digitalPinToBitMask(pin);
     6c8:	f9 01       	movw	r30, r18
     6ca:	e7 51       	subi	r30, 0x17	; 23
     6cc:	ff 4f       	sbci	r31, 0xFF	; 255
     6ce:	14 91       	lpm	r17, Z
	uint8_t port = digitalPinToPort(pin);
     6d0:	f9 01       	movw	r30, r18
     6d2:	e8 5f       	subi	r30, 0xF8	; 248
     6d4:	fe 4f       	sbci	r31, 0xFE	; 254
     6d6:	04 91       	lpm	r16, Z
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;
     6d8:	00 23       	and	r16, r16
     6da:	c9 f0       	breq	.+50     	; 0x70e <digitalWrite+0x60>

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);
     6dc:	88 23       	and	r24, r24
     6de:	21 f0       	breq	.+8      	; 0x6e8 <digitalWrite+0x3a>
     6e0:	69 83       	std	Y+1, r22	; 0x01
     6e2:	0e 94 e1 02 	call	0x5c2	; 0x5c2 <turnOffPWM>
     6e6:	69 81       	ldd	r22, Y+1	; 0x01

	out = portOutputRegister(port);
     6e8:	e0 2f       	mov	r30, r16
     6ea:	f0 e0       	ldi	r31, 0x00	; 0
     6ec:	ee 0f       	add	r30, r30
     6ee:	ff 1f       	adc	r31, r31
     6f0:	e9 5d       	subi	r30, 0xD9	; 217
     6f2:	fe 4f       	sbci	r31, 0xFE	; 254
     6f4:	a5 91       	lpm	r26, Z+
     6f6:	b4 91       	lpm	r27, Z

	uint8_t oldSREG = SREG;
     6f8:	9f b7       	in	r25, 0x3f	; 63
	cli();
     6fa:	f8 94       	cli

	if (val == LOW) {
		*out &= ~bit;
     6fc:	8c 91       	ld	r24, X
	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
     6fe:	61 11       	cpse	r22, r1
     700:	03 c0       	rjmp	.+6      	; 0x708 <digitalWrite+0x5a>
		*out &= ~bit;
     702:	10 95       	com	r17
     704:	81 23       	and	r24, r17
     706:	01 c0       	rjmp	.+2      	; 0x70a <digitalWrite+0x5c>
	} else {
		*out |= bit;
     708:	81 2b       	or	r24, r17
     70a:	8c 93       	st	X, r24
	}

	SREG = oldSREG;
     70c:	9f bf       	out	0x3f, r25	; 63
}
     70e:	0f 90       	pop	r0
     710:	df 91       	pop	r29
     712:	cf 91       	pop	r28
     714:	1f 91       	pop	r17
     716:	0f 91       	pop	r16
     718:	08 95       	ret

0000071a <_ZN7Serial_5writeEh>:
{
	USB_Flush(CDC_TX);
}

size_t Serial_::write(uint8_t c)
{
     71a:	cf 93       	push	r28
     71c:	df 93       	push	r29
     71e:	1f 92       	push	r1
     720:	cd b7       	in	r28, 0x3d	; 61
     722:	de b7       	in	r29, 0x3e	; 62
     724:	69 83       	std	Y+1, r22	; 0x01
	return write(&c, 1);
     726:	dc 01       	movw	r26, r24
     728:	ed 91       	ld	r30, X+
     72a:	fc 91       	ld	r31, X
     72c:	02 80       	ldd	r0, Z+2	; 0x02
     72e:	f3 81       	ldd	r31, Z+3	; 0x03
     730:	e0 2d       	mov	r30, r0
     732:	41 e0       	ldi	r20, 0x01	; 1
     734:	50 e0       	ldi	r21, 0x00	; 0
     736:	be 01       	movw	r22, r28
     738:	6f 5f       	subi	r22, 0xFF	; 255
     73a:	7f 4f       	sbci	r23, 0xFF	; 255
     73c:	09 95       	icall
}
     73e:	0f 90       	pop	r0
     740:	df 91       	pop	r29
     742:	cf 91       	pop	r28
     744:	08 95       	ret

00000746 <_ZN7Serial_4peekEv>:
	}
	return USB_Available(CDC_RX);
}

int Serial_::peek(void)
{
     746:	cf 93       	push	r28
     748:	df 93       	push	r29
     74a:	ec 01       	movw	r28, r24
	if (peek_buffer < 0)
     74c:	8c 85       	ldd	r24, Y+12	; 0x0c
     74e:	9d 85       	ldd	r25, Y+13	; 0x0d
     750:	97 ff       	sbrs	r25, 7
     752:	05 c0       	rjmp	.+10     	; 0x75e <_ZN7Serial_4peekEv+0x18>
		peek_buffer = USB_Recv(CDC_RX);
     754:	82 e0       	ldi	r24, 0x02	; 2
     756:	0e 94 7f 08 	call	0x10fe	; 0x10fe <_Z8USB_Recvh>
     75a:	9d 87       	std	Y+13, r25	; 0x0d
     75c:	8c 87       	std	Y+12, r24	; 0x0c
	return peek_buffer;
}
     75e:	8c 85       	ldd	r24, Y+12	; 0x0c
     760:	9d 85       	ldd	r25, Y+13	; 0x0d
     762:	df 91       	pop	r29
     764:	cf 91       	pop	r28
     766:	08 95       	ret

00000768 <_ZN7Serial_5flushEv>:
	return USB_SendSpace(CDC_TX);
}

void Serial_::flush(void)
{
	USB_Flush(CDC_TX);
     768:	83 e0       	ldi	r24, 0x03	; 3
     76a:	0c 94 13 0b 	jmp	0x1626	; 0x1626 <_Z9USB_Flushh>

0000076e <_ZN7Serial_9availableEv>:
{
}

int Serial_::available(void)
{
	if (peek_buffer >= 0) {
     76e:	fc 01       	movw	r30, r24
     770:	84 85       	ldd	r24, Z+12	; 0x0c
     772:	95 85       	ldd	r25, Z+13	; 0x0d
     774:	97 fd       	sbrc	r25, 7
     776:	06 c0       	rjmp	.+12     	; 0x784 <_ZN7Serial_9availableEv+0x16>
		return 1 + USB_Available(CDC_RX);
     778:	82 e0       	ldi	r24, 0x02	; 2
     77a:	0e 94 27 08 	call	0x104e	; 0x104e <_Z13USB_Availableh>
     77e:	90 e0       	ldi	r25, 0x00	; 0
     780:	01 96       	adiw	r24, 0x01	; 1
     782:	08 95       	ret
	}
	return USB_Available(CDC_RX);
     784:	82 e0       	ldi	r24, 0x02	; 2
     786:	0e 94 27 08 	call	0x104e	; 0x104e <_Z13USB_Availableh>
     78a:	90 e0       	ldi	r25, 0x00	; 0
}
     78c:	08 95       	ret

0000078e <_ZN7Serial_4readEv>:
		peek_buffer = USB_Recv(CDC_RX);
	return peek_buffer;
}

int Serial_::read(void)
{
     78e:	fc 01       	movw	r30, r24
	if (peek_buffer >= 0) {
     790:	84 85       	ldd	r24, Z+12	; 0x0c
     792:	95 85       	ldd	r25, Z+13	; 0x0d
     794:	97 fd       	sbrc	r25, 7
     796:	05 c0       	rjmp	.+10     	; 0x7a2 <_ZN7Serial_4readEv+0x14>
		int c = peek_buffer;
		peek_buffer = -1;
     798:	2f ef       	ldi	r18, 0xFF	; 255
     79a:	3f ef       	ldi	r19, 0xFF	; 255
     79c:	35 87       	std	Z+13, r19	; 0x0d
     79e:	24 87       	std	Z+12, r18	; 0x0c
		return c;
     7a0:	08 95       	ret
	}
	return USB_Recv(CDC_RX);
     7a2:	82 e0       	ldi	r24, 0x02	; 2
     7a4:	0c 94 7f 08 	jmp	0x10fe	; 0x10fe <_Z8USB_Recvh>

000007a8 <_ZN7Serial_5writeEPKhj>:
{
	return write(&c, 1);
}

size_t Serial_::write(const uint8_t *buffer, size_t size)
{
     7a8:	cf 93       	push	r28
     7aa:	df 93       	push	r29
     7ac:	ec 01       	movw	r28, r24
	 the connection is closed are lost - just like with a UART. */
	
	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)
	if (_usbLineInfo.lineState > 0)	{
     7ae:	80 91 95 01 	lds	r24, 0x0195
     7b2:	88 23       	and	r24, r24
     7b4:	31 f0       	breq	.+12     	; 0x7c2 <_ZN7Serial_5writeEPKhj+0x1a>
		int r = USB_Send(CDC_TX,buffer,size);
     7b6:	83 e0       	ldi	r24, 0x03	; 3
     7b8:	0e 94 b0 08 	call	0x1160	; 0x1160 <_Z8USB_SendhPKvi>
		if (r > 0) {
     7bc:	18 16       	cp	r1, r24
     7be:	19 06       	cpc	r1, r25
     7c0:	34 f0       	brlt	.+12     	; 0x7ce <_ZN7Serial_5writeEPKhj+0x26>
  private:
    int write_error;
    size_t printNumber(unsigned long, uint8_t);
    size_t printFloat(double, uint8_t);
  protected:
    void setWriteError(int err = 1) { write_error = err; }
     7c2:	81 e0       	ldi	r24, 0x01	; 1
     7c4:	90 e0       	ldi	r25, 0x00	; 0
     7c6:	9b 83       	std	Y+3, r25	; 0x03
     7c8:	8a 83       	std	Y+2, r24	; 0x02
			setWriteError();
			return 0;
		}
	}
	setWriteError();
	return 0;
     7ca:	80 e0       	ldi	r24, 0x00	; 0
     7cc:	90 e0       	ldi	r25, 0x00	; 0
}
     7ce:	df 91       	pop	r29
     7d0:	cf 91       	pop	r28
     7d2:	08 95       	ret

000007d4 <_Z16CDC_GetInterfacePh>:
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,USB_EP_SIZE,0)
};

int CDC_GetInterface(u8* interfaceNum)
{
	interfaceNum[0] += 2;	// uses 2
     7d4:	fc 01       	movw	r30, r24
     7d6:	20 81       	ld	r18, Z
     7d8:	2e 5f       	subi	r18, 0xFE	; 254
     7da:	20 83       	st	Z, r18
	return USB_SendControl(TRANSFER_PGM,&_cdcInterface,sizeof(_cdcInterface));
     7dc:	42 e4       	ldi	r20, 0x42	; 66
     7de:	50 e0       	ldi	r21, 0x00	; 0
     7e0:	63 e4       	ldi	r22, 0x43	; 67
     7e2:	71 e0       	ldi	r23, 0x01	; 1
     7e4:	80 e8       	ldi	r24, 0x80	; 128
     7e6:	0c 94 4b 09 	jmp	0x1296	; 0x1296 <_Z15USB_SendControlhPKvi>

000007ea <_Z9CDC_SetupR8USBSetup>:
}

bool CDC_Setup(USBSetup& setup)
{
     7ea:	fc 01       	movw	r30, r24
	u8 r = setup.bRequest;
     7ec:	21 81       	ldd	r18, Z+1	; 0x01
	u8 requestType = setup.bmRequestType;
     7ee:	80 81       	ld	r24, Z

	if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
     7f0:	81 3a       	cpi	r24, 0xA1	; 161
     7f2:	59 f4       	brne	.+22     	; 0x80a <_Z9CDC_SetupR8USBSetup+0x20>
	{
		if (CDC_GET_LINE_CODING == r)
     7f4:	21 32       	cpi	r18, 0x21	; 33
     7f6:	09 f0       	breq	.+2      	; 0x7fa <_Z9CDC_SetupR8USBSetup+0x10>
     7f8:	4b c0       	rjmp	.+150    	; 0x890 <_Z9CDC_SetupR8USBSetup+0xa6>
		{
			USB_SendControl(0,(void*)&_usbLineInfo,7);
     7fa:	47 e0       	ldi	r20, 0x07	; 7
     7fc:	50 e0       	ldi	r21, 0x00	; 0
     7fe:	6e e8       	ldi	r22, 0x8E	; 142
     800:	71 e0       	ldi	r23, 0x01	; 1
     802:	80 e0       	ldi	r24, 0x00	; 0
     804:	0e 94 4b 09 	call	0x1296	; 0x1296 <_Z15USB_SendControlhPKvi>
     808:	5e c0       	rjmp	.+188    	; 0x8c6 <_Z9CDC_SetupR8USBSetup+0xdc>
			return true;
		}
	}

	if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
     80a:	81 32       	cpi	r24, 0x21	; 33
     80c:	09 f0       	breq	.+2      	; 0x810 <_Z9CDC_SetupR8USBSetup+0x26>
     80e:	40 c0       	rjmp	.+128    	; 0x890 <_Z9CDC_SetupR8USBSetup+0xa6>
	{
		if (CDC_SEND_BREAK == r)
     810:	23 32       	cpi	r18, 0x23	; 35
     812:	89 f4       	brne	.+34     	; 0x836 <_Z9CDC_SetupR8USBSetup+0x4c>
		{
			breakValue = ((uint16_t)setup.wValueH << 8) | setup.wValueL;
     814:	83 81       	ldd	r24, Z+3	; 0x03
     816:	90 e0       	ldi	r25, 0x00	; 0
     818:	98 2f       	mov	r25, r24
     81a:	88 27       	eor	r24, r24
     81c:	32 81       	ldd	r19, Z+2	; 0x02
     81e:	83 2b       	or	r24, r19
     820:	a0 e0       	ldi	r26, 0x00	; 0
     822:	b0 e0       	ldi	r27, 0x00	; 0
     824:	80 93 8a 01 	sts	0x018A, r24
     828:	90 93 8b 01 	sts	0x018B, r25
     82c:	a0 93 8c 01 	sts	0x018C, r26
     830:	b0 93 8d 01 	sts	0x018D, r27
     834:	0f c0       	rjmp	.+30     	; 0x854 <_Z9CDC_SetupR8USBSetup+0x6a>
		}

		if (CDC_SET_LINE_CODING == r)
     836:	20 32       	cpi	r18, 0x20	; 32
     838:	39 f4       	brne	.+14     	; 0x848 <_Z9CDC_SetupR8USBSetup+0x5e>
		{
			USB_RecvControl((void*)&_usbLineInfo,7);
     83a:	67 e0       	ldi	r22, 0x07	; 7
     83c:	70 e0       	ldi	r23, 0x00	; 0
     83e:	8e e8       	ldi	r24, 0x8E	; 142
     840:	91 e0       	ldi	r25, 0x01	; 1
     842:	0e 94 7c 09 	call	0x12f8	; 0x12f8 <_Z15USB_RecvControlPvi>
     846:	09 c0       	rjmp	.+18     	; 0x85a <_Z9CDC_SetupR8USBSetup+0x70>
		}

		if (CDC_SET_CONTROL_LINE_STATE == r)
     848:	22 32       	cpi	r18, 0x22	; 34
     84a:	21 f4       	brne	.+8      	; 0x854 <_Z9CDC_SetupR8USBSetup+0x6a>
		{
			_usbLineInfo.lineState = setup.wValueL;
     84c:	82 81       	ldd	r24, Z+2	; 0x02
     84e:	80 93 95 01 	sts	0x0195, r24
     852:	03 c0       	rjmp	.+6      	; 0x85a <_Z9CDC_SetupR8USBSetup+0x70>
		}

		if (CDC_SET_LINE_CODING == r || CDC_SET_CONTROL_LINE_STATE == r)
     854:	2d 7f       	andi	r18, 0xFD	; 253
     856:	20 32       	cpi	r18, 0x20	; 32
     858:	b1 f5       	brne	.+108    	; 0x8c6 <_Z9CDC_SetupR8USBSetup+0xdc>
// This is used to keep compatible with the old leonardo bootloaders.
// You are still able to set the magic key position manually to RAMEND-1 to save a few bytes for this check.
#if MAGIC_KEY_POS != (RAMEND-1)
			// For future boards save the key in the inproblematic RAMEND
			// Which is reserved for the main() return value (which will never return)
			if (_updatedLUFAbootloader) {
     85a:	80 91 55 02 	lds	r24, 0x0255
     85e:	81 11       	cpse	r24, r1
     860:	03 c0       	rjmp	.+6      	; 0x868 <_Z9CDC_SetupR8USBSetup+0x7e>
			// auto-reset into the bootloader is triggered when the port, already 
			// open at 1200 bps, is closed.  this is the signal to start the watchdog
			// with a relatively long period so it can finish housekeeping tasks
			// like servicing endpoints before the sketch ends

			uint16_t magic_key_pos = MAGIC_KEY_POS;
     862:	e0 e0       	ldi	r30, 0x00	; 0
     864:	f8 e0       	ldi	r31, 0x08	; 8
     866:	02 c0       	rjmp	.+4      	; 0x86c <_Z9CDC_SetupR8USBSetup+0x82>
#if MAGIC_KEY_POS != (RAMEND-1)
			// For future boards save the key in the inproblematic RAMEND
			// Which is reserved for the main() return value (which will never return)
			if (_updatedLUFAbootloader) {
				// horray, we got a new bootloader!
				magic_key_pos = (RAMEND-1);
     868:	ee ef       	ldi	r30, 0xFE	; 254
     86a:	fa e0       	ldi	r31, 0x0A	; 10
			}
#endif

			// We check DTR state to determine if host port is open (bit 0 of lineState).
			if (1200 == _usbLineInfo.dwDTERate && (_usbLineInfo.lineState & 0x01) == 0)
     86c:	80 91 8e 01 	lds	r24, 0x018E
     870:	90 91 8f 01 	lds	r25, 0x018F
     874:	a0 91 90 01 	lds	r26, 0x0190
     878:	b0 91 91 01 	lds	r27, 0x0191
     87c:	80 3b       	cpi	r24, 0xB0	; 176
     87e:	94 40       	sbci	r25, 0x04	; 4
     880:	a1 05       	cpc	r26, r1
     882:	b1 05       	cpc	r27, r1
     884:	11 f5       	brne	.+68     	; 0x8ca <_Z9CDC_SetupR8USBSetup+0xe0>
     886:	80 91 95 01 	lds	r24, 0x0195
     88a:	80 ff       	sbrs	r24, 0
     88c:	03 c0       	rjmp	.+6      	; 0x894 <_Z9CDC_SetupR8USBSetup+0xaa>
     88e:	1d c0       	rjmp	.+58     	; 0x8ca <_Z9CDC_SetupR8USBSetup+0xe0>
				}
			}
		}
		return true;
	}
	return false;
     890:	80 e0       	ldi	r24, 0x00	; 0
     892:	08 95       	ret
			if (1200 == _usbLineInfo.dwDTERate && (_usbLineInfo.lineState & 0x01) == 0)
			{
#if MAGIC_KEY_POS != (RAMEND-1)
				// Backup ram value if its not a newer bootloader.
				// This should avoid memory corruption at least a bit, not fully
				if (magic_key_pos != (RAMEND-1)) {
     894:	ee 3f       	cpi	r30, 0xFE	; 254
     896:	2a e0       	ldi	r18, 0x0A	; 10
     898:	f2 07       	cpc	r31, r18
     89a:	31 f0       	breq	.+12     	; 0x8a8 <_Z9CDC_SetupR8USBSetup+0xbe>
					*(uint16_t *)(RAMEND-1) = *(uint16_t *)magic_key_pos;
     89c:	80 81       	ld	r24, Z
     89e:	91 81       	ldd	r25, Z+1	; 0x01
     8a0:	90 93 ff 0a 	sts	0x0AFF, r25
     8a4:	80 93 fe 0a 	sts	0x0AFE, r24
				}
#endif
				// Store boot key
				*(uint16_t *)magic_key_pos = MAGIC_KEY;
     8a8:	87 e7       	ldi	r24, 0x77	; 119
     8aa:	97 e7       	ldi	r25, 0x77	; 119
     8ac:	91 83       	std	Z+1, r25	; 0x01
     8ae:	80 83       	st	Z, r24
				wdt_enable(WDTO_120MS);
     8b0:	2b e0       	ldi	r18, 0x0B	; 11
     8b2:	88 e1       	ldi	r24, 0x18	; 24
     8b4:	90 e0       	ldi	r25, 0x00	; 0
     8b6:	0f b6       	in	r0, 0x3f	; 63
     8b8:	f8 94       	cli
     8ba:	a8 95       	wdr
     8bc:	80 93 60 00 	sts	0x0060, r24
     8c0:	0f be       	out	0x3f, r0	; 63
     8c2:	20 93 60 00 	sts	0x0060, r18
				// Clean up RAMEND key
					*(uint16_t *)magic_key_pos = 0x0000;
				}
			}
		}
		return true;
     8c6:	81 e0       	ldi	r24, 0x01	; 1
     8c8:	08 95       	ret
				// Most OSs do some intermediate steps when configuring ports and DTR can
				// twiggle more than once before stabilizing.
				// To avoid spurious resets we set the watchdog to 250ms and eventually
				// cancel if DTR goes back high.

				wdt_disable();
     8ca:	88 e1       	ldi	r24, 0x18	; 24
     8cc:	0f b6       	in	r0, 0x3f	; 63
     8ce:	f8 94       	cli
     8d0:	80 93 60 00 	sts	0x0060, r24
     8d4:	10 92 60 00 	sts	0x0060, r1
     8d8:	0f be       	out	0x3f, r0	; 63
				wdt_reset();
     8da:	a8 95       	wdr
#if MAGIC_KEY_POS != (RAMEND-1)
				// Restore backed up (old bootloader) magic key data
				if (magic_key_pos != (RAMEND-1)) {
     8dc:	ee 3f       	cpi	r30, 0xFE	; 254
     8de:	8a e0       	ldi	r24, 0x0A	; 10
     8e0:	f8 07       	cpc	r31, r24
     8e2:	39 f0       	breq	.+14     	; 0x8f2 <_Z9CDC_SetupR8USBSetup+0x108>
					*(uint16_t *)magic_key_pos = *(uint16_t *)(RAMEND-1);
     8e4:	80 91 fe 0a 	lds	r24, 0x0AFE
     8e8:	90 91 ff 0a 	lds	r25, 0x0AFF
     8ec:	91 83       	std	Z+1, r25	; 0x01
     8ee:	80 83       	st	Z, r24
     8f0:	ea cf       	rjmp	.-44     	; 0x8c6 <_Z9CDC_SetupR8USBSetup+0xdc>
				} else
#endif
				{
				// Clean up RAMEND key
					*(uint16_t *)magic_key_pos = 0x0000;
     8f2:	10 92 ff 0a 	sts	0x0AFF, r1
     8f6:	10 92 fe 0a 	sts	0x0AFE, r1
     8fa:	e5 cf       	rjmp	.-54     	; 0x8c6 <_Z9CDC_SetupR8USBSetup+0xdc>

000008fc <_ZN7Serial_5beginEm>:
}


void Serial_::begin(unsigned long /* baud_count */)
{
	peek_buffer = -1;
     8fc:	2f ef       	ldi	r18, 0xFF	; 255
     8fe:	3f ef       	ldi	r19, 0xFF	; 255
     900:	fc 01       	movw	r30, r24
     902:	35 87       	std	Z+13, r19	; 0x0d
     904:	24 87       	std	Z+12, r18	; 0x0c
     906:	08 95       	ret

00000908 <_ZN7Serial_cvbEv>:
// to just being connected to the host).  It can be used, for example, in 
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial_::operator bool() {
     908:	cf 93       	push	r28
	bool result = false;
	if (_usbLineInfo.lineState > 0) 
     90a:	c0 91 95 01 	lds	r28, 0x0195
		result = true;
	delay(10);
     90e:	6a e0       	ldi	r22, 0x0A	; 10
     910:	70 e0       	ldi	r23, 0x00	; 0
     912:	80 e0       	ldi	r24, 0x00	; 0
     914:	90 e0       	ldi	r25, 0x00	; 0
     916:	0e 94 4a 0c 	call	0x1894	; 0x1894 <delay>
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial_::operator bool() {
	bool result = false;
	if (_usbLineInfo.lineState > 0) 
     91a:	81 e0       	ldi	r24, 0x01	; 1
     91c:	c1 11       	cpse	r28, r1
     91e:	01 c0       	rjmp	.+2      	; 0x922 <_ZN7Serial_cvbEv+0x1a>
     920:	80 e0       	ldi	r24, 0x00	; 0
		result = true;
	delay(10);
	return result;
}
     922:	cf 91       	pop	r28
     924:	08 95       	ret

00000926 <_GLOBAL__sub_I__updatedLUFAbootloader>:
  public:
    Print() : write_error(0) {}
     926:	10 92 08 02 	sts	0x0208, r1
     92a:	10 92 07 02 	sts	0x0207, r1
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;

    Stream() {_timeout=1000;}
     92e:	88 ee       	ldi	r24, 0xE8	; 232
     930:	93 e0       	ldi	r25, 0x03	; 3
     932:	a0 e0       	ldi	r26, 0x00	; 0
     934:	b0 e0       	ldi	r27, 0x00	; 0
     936:	80 93 09 02 	sts	0x0209, r24
     93a:	90 93 0a 02 	sts	0x020A, r25
     93e:	a0 93 0b 02 	sts	0x020B, r26
     942:	b0 93 0c 02 	sts	0x020C, r27
class Serial_ : public Stream
{
private:
	int peek_buffer;
public:
	Serial_() { peek_buffer = -1; };
     946:	8b ed       	ldi	r24, 0xDB	; 219
     948:	91 e0       	ldi	r25, 0x01	; 1
     94a:	90 93 06 02 	sts	0x0206, r25
     94e:	80 93 05 02 	sts	0x0205, r24
     952:	8f ef       	ldi	r24, 0xFF	; 255
     954:	9f ef       	ldi	r25, 0xFF	; 255
     956:	90 93 12 02 	sts	0x0212, r25
     95a:	80 93 11 02 	sts	0x0211, r24
     95e:	08 95       	ret

00000960 <_ZN5Print5writeEPKc>:
    int getWriteError() { return write_error; }
    void clearWriteError() { setWriteError(0); }
  
    virtual size_t write(uint8_t) = 0;
    size_t write(const char *str) {
      if (str == NULL) return 0;
     960:	61 15       	cp	r22, r1
     962:	71 05       	cpc	r23, r1
     964:	81 f0       	breq	.+32     	; 0x986 <_ZN5Print5writeEPKc+0x26>
      return write((const uint8_t *)str, strlen(str));
     966:	db 01       	movw	r26, r22
     968:	0d 90       	ld	r0, X+
     96a:	00 20       	and	r0, r0
     96c:	e9 f7       	brne	.-6      	; 0x968 <_ZN5Print5writeEPKc+0x8>
     96e:	ad 01       	movw	r20, r26
     970:	41 50       	subi	r20, 0x01	; 1
     972:	51 09       	sbc	r21, r1
     974:	46 1b       	sub	r20, r22
     976:	57 0b       	sbc	r21, r23
     978:	dc 01       	movw	r26, r24
     97a:	ed 91       	ld	r30, X+
     97c:	fc 91       	ld	r31, X
     97e:	02 80       	ldd	r0, Z+2	; 0x02
     980:	f3 81       	ldd	r31, Z+3	; 0x03
     982:	e0 2d       	mov	r30, r0
     984:	09 94       	ijmp
    }
     986:	80 e0       	ldi	r24, 0x00	; 0
     988:	90 e0       	ldi	r25, 0x00	; 0
     98a:	08 95       	ret

0000098c <_ZN5Print5printEPKc>:
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     98c:	0c 94 b0 04 	jmp	0x960	; 0x960 <_ZN5Print5writeEPKc>

00000990 <_ZN5Print5printEc>:
}

size_t Print::print(char c)
{
  return write(c);
     990:	dc 01       	movw	r26, r24
     992:	ed 91       	ld	r30, X+
     994:	fc 91       	ld	r31, X
     996:	01 90       	ld	r0, Z+
     998:	f0 81       	ld	r31, Z
     99a:	e0 2d       	mov	r30, r0
     99c:	09 94       	ijmp

0000099e <_ZN5Print7printlnEv>:
  return x.printTo(*this);
}

size_t Print::println(void)
{
  return write("\r\n");
     99e:	67 ee       	ldi	r22, 0xE7	; 231
     9a0:	71 e0       	ldi	r23, 0x01	; 1
     9a2:	0c 94 b0 04 	jmp	0x960	; 0x960 <_ZN5Print5writeEPKc>

000009a6 <_ZN5Print7printlnEPKc>:
  n += println();
  return n;
}

size_t Print::println(const char c[])
{
     9a6:	0f 93       	push	r16
     9a8:	1f 93       	push	r17
     9aa:	cf 93       	push	r28
     9ac:	df 93       	push	r29
     9ae:	ec 01       	movw	r28, r24
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     9b0:	0e 94 b0 04 	call	0x960	; 0x960 <_ZN5Print5writeEPKc>
     9b4:	8c 01       	movw	r16, r24
}

size_t Print::println(const char c[])
{
  size_t n = print(c);
  n += println();
     9b6:	ce 01       	movw	r24, r28
     9b8:	0e 94 cf 04 	call	0x99e	; 0x99e <_ZN5Print7printlnEv>
  return n;
}
     9bc:	80 0f       	add	r24, r16
     9be:	91 1f       	adc	r25, r17
     9c0:	df 91       	pop	r29
     9c2:	cf 91       	pop	r28
     9c4:	1f 91       	pop	r17
     9c6:	0f 91       	pop	r16
     9c8:	08 95       	ret

000009ca <_ZN5Print11printNumberEmh>:
}

// Private Methods /////////////////////////////////////////////////////////////

size_t Print::printNumber(unsigned long n, uint8_t base)
{
     9ca:	8f 92       	push	r8
     9cc:	9f 92       	push	r9
     9ce:	af 92       	push	r10
     9d0:	bf 92       	push	r11
     9d2:	df 92       	push	r13
     9d4:	ef 92       	push	r14
     9d6:	ff 92       	push	r15
     9d8:	0f 93       	push	r16
     9da:	1f 93       	push	r17
     9dc:	cf 93       	push	r28
     9de:	df 93       	push	r29
     9e0:	cd b7       	in	r28, 0x3d	; 61
     9e2:	de b7       	in	r29, 0x3e	; 62
     9e4:	a1 97       	sbiw	r28, 0x21	; 33
     9e6:	0f b6       	in	r0, 0x3f	; 63
     9e8:	f8 94       	cli
     9ea:	de bf       	out	0x3e, r29	; 62
     9ec:	0f be       	out	0x3f, r0	; 63
     9ee:	cd bf       	out	0x3d, r28	; 61
     9f0:	7c 01       	movw	r14, r24
     9f2:	f4 2f       	mov	r31, r20
     9f4:	d5 2e       	mov	r13, r21
     9f6:	cb 01       	movw	r24, r22
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';
     9f8:	19 a2       	std	Y+33, r1	; 0x21

  // prevent crash if called with base == 1
  if (base < 2) base = 10;
     9fa:	22 30       	cpi	r18, 0x02	; 2
     9fc:	08 f4       	brcc	.+2      	; 0xa00 <_ZN5Print11printNumberEmh+0x36>
     9fe:	2a e0       	ldi	r18, 0x0A	; 10
     a00:	8e 01       	movw	r16, r28
     a02:	0f 5d       	subi	r16, 0xDF	; 223
     a04:	1f 4f       	sbci	r17, 0xFF	; 255

  do {
    char c = n % base;
     a06:	82 2e       	mov	r8, r18
     a08:	91 2c       	mov	r9, r1
     a0a:	a1 2c       	mov	r10, r1
     a0c:	b1 2c       	mov	r11, r1
     a0e:	6f 2f       	mov	r22, r31
     a10:	7d 2d       	mov	r23, r13
     a12:	a5 01       	movw	r20, r10
     a14:	94 01       	movw	r18, r8
     a16:	0e 94 95 0f 	call	0x1f2a	; 0x1f2a <__udivmodsi4>
    n /= base;
     a1a:	f2 2f       	mov	r31, r18
     a1c:	d3 2e       	mov	r13, r19
     a1e:	ca 01       	movw	r24, r20

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
     a20:	01 50       	subi	r16, 0x01	; 1
     a22:	11 09       	sbc	r17, r1
     a24:	6a 30       	cpi	r22, 0x0A	; 10
     a26:	14 f4       	brge	.+4      	; 0xa2c <_ZN5Print11printNumberEmh+0x62>
     a28:	e0 e3       	ldi	r30, 0x30	; 48
     a2a:	01 c0       	rjmp	.+2      	; 0xa2e <_ZN5Print11printNumberEmh+0x64>
     a2c:	e7 e3       	ldi	r30, 0x37	; 55
     a2e:	e6 0f       	add	r30, r22
     a30:	d8 01       	movw	r26, r16
     a32:	ec 93       	st	X, r30
  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
     a34:	23 2b       	or	r18, r19
     a36:	24 2b       	or	r18, r20
     a38:	25 2b       	or	r18, r21
     a3a:	49 f7       	brne	.-46     	; 0xa0e <_ZN5Print11printNumberEmh+0x44>
    n /= base;

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  return write(str);
     a3c:	b8 01       	movw	r22, r16
     a3e:	c7 01       	movw	r24, r14
     a40:	0e 94 b0 04 	call	0x960	; 0x960 <_ZN5Print5writeEPKc>
}
     a44:	a1 96       	adiw	r28, 0x21	; 33
     a46:	0f b6       	in	r0, 0x3f	; 63
     a48:	f8 94       	cli
     a4a:	de bf       	out	0x3e, r29	; 62
     a4c:	0f be       	out	0x3f, r0	; 63
     a4e:	cd bf       	out	0x3d, r28	; 61
     a50:	df 91       	pop	r29
     a52:	cf 91       	pop	r28
     a54:	1f 91       	pop	r17
     a56:	0f 91       	pop	r16
     a58:	ff 90       	pop	r15
     a5a:	ef 90       	pop	r14
     a5c:	df 90       	pop	r13
     a5e:	bf 90       	pop	r11
     a60:	af 90       	pop	r10
     a62:	9f 90       	pop	r9
     a64:	8f 90       	pop	r8
     a66:	08 95       	ret

00000a68 <_ZN5Print5printEli>:
{
  return print((unsigned long) n, base);
}

size_t Print::print(long n, int base)
{
     a68:	cf 92       	push	r12
     a6a:	df 92       	push	r13
     a6c:	ef 92       	push	r14
     a6e:	ff 92       	push	r15
     a70:	0f 93       	push	r16
     a72:	1f 93       	push	r17
     a74:	cf 93       	push	r28
     a76:	df 93       	push	r29
     a78:	ec 01       	movw	r28, r24
     a7a:	6a 01       	movw	r12, r20
     a7c:	7b 01       	movw	r14, r22
  if (base == 0) {
     a7e:	21 15       	cp	r18, r1
     a80:	31 05       	cpc	r19, r1
     a82:	79 f4       	brne	.+30     	; 0xaa2 <_ZN5Print5printEli+0x3a>
    return write(n);
     a84:	e8 81       	ld	r30, Y
     a86:	f9 81       	ldd	r31, Y+1	; 0x01
     a88:	01 90       	ld	r0, Z+
     a8a:	f0 81       	ld	r31, Z
     a8c:	e0 2d       	mov	r30, r0
     a8e:	64 2f       	mov	r22, r20
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
  }
}
     a90:	df 91       	pop	r29
     a92:	cf 91       	pop	r28
     a94:	1f 91       	pop	r17
     a96:	0f 91       	pop	r16
     a98:	ff 90       	pop	r15
     a9a:	ef 90       	pop	r14
     a9c:	df 90       	pop	r13
     a9e:	cf 90       	pop	r12
}

size_t Print::print(long n, int base)
{
  if (base == 0) {
    return write(n);
     aa0:	09 94       	ijmp
  } else if (base == 10) {
     aa2:	2a 30       	cpi	r18, 0x0A	; 10
     aa4:	31 05       	cpc	r19, r1
     aa6:	e9 f4       	brne	.+58     	; 0xae2 <_ZN5Print5printEli+0x7a>
    if (n < 0) {
     aa8:	77 ff       	sbrs	r23, 7
     aaa:	1a c0       	rjmp	.+52     	; 0xae0 <_ZN5Print5printEli+0x78>
      int t = print('-');
     aac:	6d e2       	ldi	r22, 0x2D	; 45
     aae:	0e 94 c8 04 	call	0x990	; 0x990 <_ZN5Print5printEc>
     ab2:	8c 01       	movw	r16, r24
      n = -n;
     ab4:	44 27       	eor	r20, r20
     ab6:	55 27       	eor	r21, r21
     ab8:	ba 01       	movw	r22, r20
     aba:	4c 19       	sub	r20, r12
     abc:	5d 09       	sbc	r21, r13
     abe:	6e 09       	sbc	r22, r14
     ac0:	7f 09       	sbc	r23, r15
      return printNumber(n, 10) + t;
     ac2:	2a e0       	ldi	r18, 0x0A	; 10
     ac4:	ce 01       	movw	r24, r28
     ac6:	0e 94 e5 04 	call	0x9ca	; 0x9ca <_ZN5Print11printNumberEmh>
     aca:	80 0f       	add	r24, r16
     acc:	91 1f       	adc	r25, r17
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
  }
}
     ace:	df 91       	pop	r29
     ad0:	cf 91       	pop	r28
     ad2:	1f 91       	pop	r17
     ad4:	0f 91       	pop	r16
     ad6:	ff 90       	pop	r15
     ad8:	ef 90       	pop	r14
     ada:	df 90       	pop	r13
     adc:	cf 90       	pop	r12
     ade:	08 95       	ret
    if (n < 0) {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
     ae0:	2a e0       	ldi	r18, 0x0A	; 10
  } else {
    return printNumber(n, base);
     ae2:	b7 01       	movw	r22, r14
     ae4:	a6 01       	movw	r20, r12
     ae6:	ce 01       	movw	r24, r28
  }
}
     ae8:	df 91       	pop	r29
     aea:	cf 91       	pop	r28
     aec:	1f 91       	pop	r17
     aee:	0f 91       	pop	r16
     af0:	ff 90       	pop	r15
     af2:	ef 90       	pop	r14
     af4:	df 90       	pop	r13
     af6:	cf 90       	pop	r12
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
     af8:	0c 94 e5 04 	jmp	0x9ca	; 0x9ca <_ZN5Print11printNumberEmh>

00000afc <_ZN5Print5printEii>:
{
  return print((unsigned long) b, base);
}

size_t Print::print(int n, int base)
{
     afc:	9a 01       	movw	r18, r20
  return print((long) n, base);
     afe:	ab 01       	movw	r20, r22
     b00:	66 27       	eor	r22, r22
     b02:	57 fd       	sbrc	r21, 7
     b04:	60 95       	com	r22
     b06:	76 2f       	mov	r23, r22
     b08:	0c 94 34 05 	jmp	0xa68	; 0xa68 <_ZN5Print5printEli>

00000b0c <_ZN5Print5printEmi>:
  }
}

size_t Print::print(unsigned long n, int base)
{
  if (base == 0) return write(n);
     b0c:	21 15       	cp	r18, r1
     b0e:	31 05       	cpc	r19, r1
     b10:	41 f4       	brne	.+16     	; 0xb22 <_ZN5Print5printEmi+0x16>
     b12:	dc 01       	movw	r26, r24
     b14:	ed 91       	ld	r30, X+
     b16:	fc 91       	ld	r31, X
     b18:	01 90       	ld	r0, Z+
     b1a:	f0 81       	ld	r31, Z
     b1c:	e0 2d       	mov	r30, r0
     b1e:	64 2f       	mov	r22, r20
     b20:	09 94       	ijmp
  else return printNumber(n, base);
     b22:	0c 94 e5 04 	jmp	0x9ca	; 0x9ca <_ZN5Print11printNumberEmh>

00000b26 <_ZN5Print5printEhi>:
{
  return write(c);
}

size_t Print::print(unsigned char b, int base)
{
     b26:	9a 01       	movw	r18, r20
  return print((unsigned long) b, base);
     b28:	46 2f       	mov	r20, r22
     b2a:	50 e0       	ldi	r21, 0x00	; 0
     b2c:	60 e0       	ldi	r22, 0x00	; 0
     b2e:	70 e0       	ldi	r23, 0x00	; 0
     b30:	0c 94 86 05 	jmp	0xb0c	; 0xb0c <_ZN5Print5printEmi>

00000b34 <_ZN5Print5printEji>:
{
  return print((long) n, base);
}

size_t Print::print(unsigned int n, int base)
{
     b34:	9a 01       	movw	r18, r20
  return print((unsigned long) n, base);
     b36:	ab 01       	movw	r20, r22
     b38:	60 e0       	ldi	r22, 0x00	; 0
     b3a:	70 e0       	ldi	r23, 0x00	; 0
     b3c:	0c 94 86 05 	jmp	0xb0c	; 0xb0c <_ZN5Print5printEmi>

00000b40 <_ZN5Print10printFloatEdh>:

  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits) 
{ 
     b40:	4f 92       	push	r4
     b42:	5f 92       	push	r5
     b44:	6f 92       	push	r6
     b46:	7f 92       	push	r7
     b48:	8f 92       	push	r8
     b4a:	9f 92       	push	r9
     b4c:	af 92       	push	r10
     b4e:	bf 92       	push	r11
     b50:	cf 92       	push	r12
     b52:	df 92       	push	r13
     b54:	ef 92       	push	r14
     b56:	ff 92       	push	r15
     b58:	0f 93       	push	r16
     b5a:	1f 93       	push	r17
     b5c:	cf 93       	push	r28
     b5e:	df 93       	push	r29
     b60:	ec 01       	movw	r28, r24
     b62:	6a 01       	movw	r12, r20
     b64:	7b 01       	movw	r14, r22
     b66:	b2 2e       	mov	r11, r18
  size_t n = 0;
  
  if (isnan(number)) return print("nan");
     b68:	9a 01       	movw	r18, r20
     b6a:	ab 01       	movw	r20, r22
     b6c:	c7 01       	movw	r24, r14
     b6e:	b6 01       	movw	r22, r12
     b70:	0e 94 91 0f 	call	0x1f22	; 0x1f22 <__unordsf2>
     b74:	88 23       	and	r24, r24
     b76:	19 f0       	breq	.+6      	; 0xb7e <_ZN5Print10printFloatEdh+0x3e>
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     b78:	6a ee       	ldi	r22, 0xEA	; 234
     b7a:	71 e0       	ldi	r23, 0x01	; 1
     b7c:	27 c0       	rjmp	.+78     	; 0xbcc <_ZN5Print10printFloatEdh+0x8c>
size_t Print::printFloat(double number, uint8_t digits) 
{ 
  size_t n = 0;
  
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
     b7e:	26 01       	movw	r4, r12
     b80:	37 01       	movw	r6, r14
     b82:	e8 94       	clt
     b84:	77 f8       	bld	r7, 7
     b86:	2f ef       	ldi	r18, 0xFF	; 255
     b88:	3f ef       	ldi	r19, 0xFF	; 255
     b8a:	4f e7       	ldi	r20, 0x7F	; 127
     b8c:	5f e7       	ldi	r21, 0x7F	; 127
     b8e:	c3 01       	movw	r24, r6
     b90:	b2 01       	movw	r22, r4
     b92:	0e 94 91 0f 	call	0x1f22	; 0x1f22 <__unordsf2>
     b96:	81 11       	cpse	r24, r1
     b98:	0d c0       	rjmp	.+26     	; 0xbb4 <_ZN5Print10printFloatEdh+0x74>
     b9a:	2f ef       	ldi	r18, 0xFF	; 255
     b9c:	3f ef       	ldi	r19, 0xFF	; 255
     b9e:	4f e7       	ldi	r20, 0x7F	; 127
     ba0:	5f e7       	ldi	r21, 0x7F	; 127
     ba2:	c3 01       	movw	r24, r6
     ba4:	b2 01       	movw	r22, r4
     ba6:	0e 94 db 0d 	call	0x1bb6	; 0x1bb6 <__cmpsf2>
     baa:	18 16       	cp	r1, r24
     bac:	1c f4       	brge	.+6      	; 0xbb4 <_ZN5Print10printFloatEdh+0x74>
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     bae:	6e ee       	ldi	r22, 0xEE	; 238
     bb0:	71 e0       	ldi	r23, 0x01	; 1
     bb2:	0c c0       	rjmp	.+24     	; 0xbcc <_ZN5Print10printFloatEdh+0x8c>
{ 
  size_t n = 0;
  
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
     bb4:	2f ef       	ldi	r18, 0xFF	; 255
     bb6:	3f ef       	ldi	r19, 0xFF	; 255
     bb8:	4f e7       	ldi	r20, 0x7F	; 127
     bba:	5f e4       	ldi	r21, 0x4F	; 79
     bbc:	c7 01       	movw	r24, r14
     bbe:	b6 01       	movw	r22, r12
     bc0:	0e 94 2a 0f 	call	0x1e54	; 0x1e54 <__gesf2>
     bc4:	18 16       	cp	r1, r24
     bc6:	ac f4       	brge	.+42     	; 0xbf2 <_ZN5Print10printFloatEdh+0xb2>
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     bc8:	62 ef       	ldi	r22, 0xF2	; 242
     bca:	71 e0       	ldi	r23, 0x01	; 1
     bcc:	ce 01       	movw	r24, r28
    n += print(toPrint);
    remainder -= toPrint; 
  } 
  
  return n;
}
     bce:	df 91       	pop	r29
     bd0:	cf 91       	pop	r28
     bd2:	1f 91       	pop	r17
     bd4:	0f 91       	pop	r16
     bd6:	ff 90       	pop	r15
     bd8:	ef 90       	pop	r14
     bda:	df 90       	pop	r13
     bdc:	cf 90       	pop	r12
     bde:	bf 90       	pop	r11
     be0:	af 90       	pop	r10
     be2:	9f 90       	pop	r9
     be4:	8f 90       	pop	r8
     be6:	7f 90       	pop	r7
     be8:	6f 90       	pop	r6
     bea:	5f 90       	pop	r5
     bec:	4f 90       	pop	r4
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     bee:	0c 94 b0 04 	jmp	0x960	; 0x960 <_ZN5Print5writeEPKc>
  size_t n = 0;
  
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically
     bf2:	2f ef       	ldi	r18, 0xFF	; 255
     bf4:	3f ef       	ldi	r19, 0xFF	; 255
     bf6:	4f e7       	ldi	r20, 0x7F	; 127
     bf8:	5f ec       	ldi	r21, 0xCF	; 207
     bfa:	c7 01       	movw	r24, r14
     bfc:	b6 01       	movw	r22, r12
     bfe:	0e 94 db 0d 	call	0x1bb6	; 0x1bb6 <__cmpsf2>
     c02:	87 fd       	sbrc	r24, 7
     c04:	e1 cf       	rjmp	.-62     	; 0xbc8 <_ZN5Print10printFloatEdh+0x88>
  
  // Handle negative numbers
  if (number < 0.0)
     c06:	20 e0       	ldi	r18, 0x00	; 0
     c08:	30 e0       	ldi	r19, 0x00	; 0
     c0a:	a9 01       	movw	r20, r18
     c0c:	c7 01       	movw	r24, r14
     c0e:	b6 01       	movw	r22, r12
     c10:	0e 94 db 0d 	call	0x1bb6	; 0x1bb6 <__cmpsf2>
     c14:	87 ff       	sbrs	r24, 7
     c16:	0a c0       	rjmp	.+20     	; 0xc2c <_ZN5Print10printFloatEdh+0xec>
  {
     n += print('-');
     c18:	6d e2       	ldi	r22, 0x2D	; 45
     c1a:	ce 01       	movw	r24, r28
     c1c:	0e 94 c8 04 	call	0x990	; 0x990 <_ZN5Print5printEc>
     c20:	8c 01       	movw	r16, r24
     number = -number;
     c22:	f7 fa       	bst	r15, 7
     c24:	f0 94       	com	r15
     c26:	f7 f8       	bld	r15, 7
     c28:	f0 94       	com	r15
     c2a:	02 c0       	rjmp	.+4      	; 0xc30 <_ZN5Print10printFloatEdh+0xf0>
  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits) 
{ 
  size_t n = 0;
     c2c:	00 e0       	ldi	r16, 0x00	; 0
     c2e:	10 e0       	ldi	r17, 0x00	; 0
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
     c30:	a1 2c       	mov	r10, r1
     n += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
     c32:	60 e0       	ldi	r22, 0x00	; 0
     c34:	70 e0       	ldi	r23, 0x00	; 0
     c36:	80 e0       	ldi	r24, 0x00	; 0
     c38:	9f e3       	ldi	r25, 0x3F	; 63
  for (uint8_t i=0; i<digits; ++i)
     c3a:	ab 14       	cp	r10, r11
     c3c:	41 f0       	breq	.+16     	; 0xc4e <_ZN5Print10printFloatEdh+0x10e>
    rounding /= 10.0;
     c3e:	20 e0       	ldi	r18, 0x00	; 0
     c40:	30 e0       	ldi	r19, 0x00	; 0
     c42:	40 e2       	ldi	r20, 0x20	; 32
     c44:	51 e4       	ldi	r21, 0x41	; 65
     c46:	0e 94 df 0d 	call	0x1bbe	; 0x1bbe <__divsf3>
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
     c4a:	a3 94       	inc	r10
     c4c:	f6 cf       	rjmp	.-20     	; 0xc3a <_ZN5Print10printFloatEdh+0xfa>
    rounding /= 10.0;
  
  number += rounding;
     c4e:	9b 01       	movw	r18, r22
     c50:	ac 01       	movw	r20, r24
     c52:	c7 01       	movw	r24, r14
     c54:	b6 01       	movw	r22, r12
     c56:	0e 94 77 0d 	call	0x1aee	; 0x1aee <__addsf3>
     c5a:	6b 01       	movw	r12, r22
     c5c:	7c 01       	movw	r14, r24

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
     c5e:	0e 94 4c 0e 	call	0x1c98	; 0x1c98 <__fixunssfsi>
     c62:	2b 01       	movw	r4, r22
     c64:	3c 01       	movw	r6, r24
  double remainder = number - (double)int_part;
     c66:	0e 94 78 0e 	call	0x1cf0	; 0x1cf0 <__floatunsisf>
     c6a:	9b 01       	movw	r18, r22
     c6c:	ac 01       	movw	r20, r24
     c6e:	c7 01       	movw	r24, r14
     c70:	b6 01       	movw	r22, r12
     c72:	0e 94 76 0d 	call	0x1aec	; 0x1aec <__subsf3>
     c76:	6b 01       	movw	r12, r22
     c78:	7c 01       	movw	r14, r24
}

size_t Print::print(unsigned long n, int base)
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
     c7a:	2a e0       	ldi	r18, 0x0A	; 10
     c7c:	b3 01       	movw	r22, r6
     c7e:	a2 01       	movw	r20, r4
     c80:	ce 01       	movw	r24, r28
     c82:	0e 94 e5 04 	call	0x9ca	; 0x9ca <_ZN5Print11printNumberEmh>
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);
     c86:	08 0f       	add	r16, r24
     c88:	19 1f       	adc	r17, r25

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
     c8a:	bb 20       	and	r11, r11
     c8c:	39 f0       	breq	.+14     	; 0xc9c <_ZN5Print10printFloatEdh+0x15c>
  return write(s.c_str(), s.length());
}

size_t Print::print(const char str[])
{
  return write(str);
     c8e:	66 ef       	ldi	r22, 0xF6	; 246
     c90:	71 e0       	ldi	r23, 0x01	; 1
     c92:	ce 01       	movw	r24, r28
     c94:	0e 94 b0 04 	call	0x960	; 0x960 <_ZN5Print5writeEPKc>
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print("."); 
     c98:	08 0f       	add	r16, r24
     c9a:	19 1f       	adc	r17, r25
     n += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
     c9c:	7b 2c       	mov	r7, r11
  if (digits > 0) {
    n += print("."); 
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
     c9e:	77 20       	and	r7, r7
     ca0:	41 f1       	breq	.+80     	; 0xcf2 <_ZN5Print10printFloatEdh+0x1b2>
  {
    remainder *= 10.0;
     ca2:	20 e0       	ldi	r18, 0x00	; 0
     ca4:	30 e0       	ldi	r19, 0x00	; 0
     ca6:	40 e2       	ldi	r20, 0x20	; 32
     ca8:	51 e4       	ldi	r21, 0x41	; 65
     caa:	c7 01       	movw	r24, r14
     cac:	b6 01       	movw	r22, r12
     cae:	0e 94 2e 0f 	call	0x1e5c	; 0x1e5c <__mulsf3>
     cb2:	6b 01       	movw	r12, r22
     cb4:	7c 01       	movw	r14, r24
    int toPrint = int(remainder);
     cb6:	0e 94 47 0e 	call	0x1c8e	; 0x1c8e <__fixsfsi>
  return print((unsigned long) b, base);
}

size_t Print::print(int n, int base)
{
  return print((long) n, base);
     cba:	4b 01       	movw	r8, r22
     cbc:	aa 24       	eor	r10, r10
     cbe:	97 fc       	sbrc	r9, 7
     cc0:	a0 94       	com	r10
     cc2:	ba 2c       	mov	r11, r10
     cc4:	2a e0       	ldi	r18, 0x0A	; 10
     cc6:	30 e0       	ldi	r19, 0x00	; 0
     cc8:	b5 01       	movw	r22, r10
     cca:	a4 01       	movw	r20, r8
     ccc:	ce 01       	movw	r24, r28
     cce:	0e 94 34 05 	call	0xa68	; 0xa68 <_ZN5Print5printEli>
  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    n += print(toPrint);
     cd2:	08 0f       	add	r16, r24
     cd4:	19 1f       	adc	r17, r25
    remainder -= toPrint; 
     cd6:	c5 01       	movw	r24, r10
     cd8:	b4 01       	movw	r22, r8
     cda:	0e 94 7a 0e 	call	0x1cf4	; 0x1cf4 <__floatsisf>
     cde:	9b 01       	movw	r18, r22
     ce0:	ac 01       	movw	r20, r24
     ce2:	c7 01       	movw	r24, r14
     ce4:	b6 01       	movw	r22, r12
     ce6:	0e 94 76 0d 	call	0x1aec	; 0x1aec <__subsf3>
     cea:	6b 01       	movw	r12, r22
     cec:	7c 01       	movw	r14, r24
     cee:	7a 94       	dec	r7
     cf0:	d6 cf       	rjmp	.-84     	; 0xc9e <_ZN5Print10printFloatEdh+0x15e>
  } 
  
  return n;
}
     cf2:	c8 01       	movw	r24, r16
     cf4:	df 91       	pop	r29
     cf6:	cf 91       	pop	r28
     cf8:	1f 91       	pop	r17
     cfa:	0f 91       	pop	r16
     cfc:	ff 90       	pop	r15
     cfe:	ef 90       	pop	r14
     d00:	df 90       	pop	r13
     d02:	cf 90       	pop	r12
     d04:	bf 90       	pop	r11
     d06:	af 90       	pop	r10
     d08:	9f 90       	pop	r9
     d0a:	8f 90       	pop	r8
     d0c:	7f 90       	pop	r7
     d0e:	6f 90       	pop	r6
     d10:	5f 90       	pop	r5
     d12:	4f 90       	pop	r4
     d14:	08 95       	ret

00000d16 <_ZN5Print5printEdi>:
  else return printNumber(n, base);
}

size_t Print::print(double n, int digits)
{
  return printFloat(n, digits);
     d16:	0c 94 a0 05 	jmp	0xb40	; 0xb40 <_ZN5Print10printFloatEdh>

00000d1a <_ZN6Stream9timedPeekEv>:
  return -1;     // -1 indicates timeout
}

// private method to peek stream with timeout
int Stream::timedPeek()
{
     d1a:	0f 93       	push	r16
     d1c:	1f 93       	push	r17
     d1e:	cf 93       	push	r28
     d20:	df 93       	push	r29
     d22:	ec 01       	movw	r28, r24
  int c;
  _startMillis = millis();
     d24:	0e 94 1b 0c 	call	0x1836	; 0x1836 <millis>
     d28:	68 87       	std	Y+8, r22	; 0x08
     d2a:	79 87       	std	Y+9, r23	; 0x09
     d2c:	8a 87       	std	Y+10, r24	; 0x0a
     d2e:	9b 87       	std	Y+11, r25	; 0x0b
  do {
    c = peek();
     d30:	e8 81       	ld	r30, Y
     d32:	f9 81       	ldd	r31, Y+1	; 0x01
     d34:	00 84       	ldd	r0, Z+8	; 0x08
     d36:	f1 85       	ldd	r31, Z+9	; 0x09
     d38:	e0 2d       	mov	r30, r0
     d3a:	ce 01       	movw	r24, r28
     d3c:	09 95       	icall
    if (c >= 0) return c;
     d3e:	97 ff       	sbrs	r25, 7
     d40:	15 c0       	rjmp	.+42     	; 0xd6c <_ZN6Stream9timedPeekEv+0x52>
  } while(millis() - _startMillis < _timeout);
     d42:	0e 94 1b 0c 	call	0x1836	; 0x1836 <millis>
     d46:	08 85       	ldd	r16, Y+8	; 0x08
     d48:	19 85       	ldd	r17, Y+9	; 0x09
     d4a:	2a 85       	ldd	r18, Y+10	; 0x0a
     d4c:	3b 85       	ldd	r19, Y+11	; 0x0b
     d4e:	60 1b       	sub	r22, r16
     d50:	71 0b       	sbc	r23, r17
     d52:	82 0b       	sbc	r24, r18
     d54:	93 0b       	sbc	r25, r19
// private method to peek stream with timeout
int Stream::timedPeek()
{
  int c;
  _startMillis = millis();
  do {
     d56:	0c 81       	ldd	r16, Y+4	; 0x04
     d58:	1d 81       	ldd	r17, Y+5	; 0x05
     d5a:	2e 81       	ldd	r18, Y+6	; 0x06
     d5c:	3f 81       	ldd	r19, Y+7	; 0x07
     d5e:	60 17       	cp	r22, r16
     d60:	71 07       	cpc	r23, r17
     d62:	82 07       	cpc	r24, r18
     d64:	93 07       	cpc	r25, r19
     d66:	20 f3       	brcs	.-56     	; 0xd30 <_ZN6Stream9timedPeekEv+0x16>
    c = peek();
    if (c >= 0) return c;
  } while(millis() - _startMillis < _timeout);
  return -1;     // -1 indicates timeout
     d68:	8f ef       	ldi	r24, 0xFF	; 255
     d6a:	9f ef       	ldi	r25, 0xFF	; 255
}
     d6c:	df 91       	pop	r29
     d6e:	cf 91       	pop	r28
     d70:	1f 91       	pop	r17
     d72:	0f 91       	pop	r16
     d74:	08 95       	ret

00000d76 <_ZN6Stream13peekNextDigitE13LookaheadModeb>:

// returns peek of the next digit in the stream or -1 if timeout
// discards non-numeric characters
int Stream::peekNextDigit(LookaheadMode lookahead, bool detectDecimal)
{
     d76:	ff 92       	push	r15
     d78:	0f 93       	push	r16
     d7a:	1f 93       	push	r17
     d7c:	cf 93       	push	r28
     d7e:	df 93       	push	r29
     d80:	ec 01       	movw	r28, r24
     d82:	8b 01       	movw	r16, r22
     d84:	f4 2e       	mov	r15, r20
  int c;
  while (1) {
    c = timedPeek();
     d86:	ce 01       	movw	r24, r28
     d88:	0e 94 8d 06 	call	0xd1a	; 0xd1a <_ZN6Stream9timedPeekEv>
     d8c:	9c 01       	movw	r18, r24

    if( c < 0 ||
     d8e:	97 fd       	sbrc	r25, 7
     d90:	28 c0       	rjmp	.+80     	; 0xde2 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x6c>
     d92:	8d 32       	cpi	r24, 0x2D	; 45
     d94:	91 05       	cpc	r25, r1
     d96:	19 f1       	breq	.+70     	; 0xdde <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x68>
        c == '-' ||
        (c >= '0' && c <= '9') ||
     d98:	c0 97       	sbiw	r24, 0x30	; 48
  int c;
  while (1) {
    c = timedPeek();

    if( c < 0 ||
        c == '-' ||
     d9a:	0a 97       	sbiw	r24, 0x0a	; 10
     d9c:	10 f1       	brcs	.+68     	; 0xde2 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x6c>
        (c >= '0' && c <= '9') ||
     d9e:	ff 20       	and	r15, r15
     da0:	19 f0       	breq	.+6      	; 0xda8 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x32>
        (detectDecimal && c == '.')) return c;
     da2:	2e 32       	cpi	r18, 0x2E	; 46
     da4:	31 05       	cpc	r19, r1
     da6:	f9 f0       	breq	.+62     	; 0xde6 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x70>

    switch( lookahead ){
     da8:	01 30       	cpi	r16, 0x01	; 1
     daa:	11 05       	cpc	r17, r1
     dac:	f9 f0       	breq	.+62     	; 0xdec <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x76>
     dae:	02 30       	cpi	r16, 0x02	; 2
     db0:	11 05       	cpc	r17, r1
     db2:	69 f4       	brne	.+26     	; 0xdce <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x58>
        case SKIP_NONE: return -1; // Fail code.
        case SKIP_WHITESPACE:
            switch( c ){
     db4:	2d 30       	cpi	r18, 0x0D	; 13
     db6:	31 05       	cpc	r19, r1
     db8:	51 f0       	breq	.+20     	; 0xdce <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x58>
     dba:	34 f4       	brge	.+12     	; 0xdc8 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x52>
     dbc:	29 50       	subi	r18, 0x09	; 9
     dbe:	31 09       	sbc	r19, r1
     dc0:	22 30       	cpi	r18, 0x02	; 2
     dc2:	31 05       	cpc	r19, r1
     dc4:	20 f0       	brcs	.+8      	; 0xdce <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x58>
     dc6:	12 c0       	rjmp	.+36     	; 0xdec <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x76>
     dc8:	20 32       	cpi	r18, 0x20	; 32
     dca:	31 05       	cpc	r19, r1
     dcc:	79 f4       	brne	.+30     	; 0xdec <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x76>
                default: return -1; // Fail code.
            }
        case SKIP_ALL:
            break;
    }
    read();  // discard non-numeric
     dce:	e8 81       	ld	r30, Y
     dd0:	f9 81       	ldd	r31, Y+1	; 0x01
     dd2:	06 80       	ldd	r0, Z+6	; 0x06
     dd4:	f7 81       	ldd	r31, Z+7	; 0x07
     dd6:	e0 2d       	mov	r30, r0
     dd8:	ce 01       	movw	r24, r28
     dda:	09 95       	icall
  }
     ddc:	d4 cf       	rjmp	.-88     	; 0xd86 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x10>
     dde:	8d e2       	ldi	r24, 0x2D	; 45
     de0:	03 c0       	rjmp	.+6      	; 0xde8 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x72>
// discards non-numeric characters
int Stream::peekNextDigit(LookaheadMode lookahead, bool detectDecimal)
{
  int c;
  while (1) {
    c = timedPeek();
     de2:	c9 01       	movw	r24, r18
     de4:	05 c0       	rjmp	.+10     	; 0xdf0 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x7a>
     de6:	8e e2       	ldi	r24, 0x2E	; 46
     de8:	90 e0       	ldi	r25, 0x00	; 0
     dea:	02 c0       	rjmp	.+4      	; 0xdf0 <_ZN6Stream13peekNextDigitE13LookaheadModeb+0x7a>
            switch( c ){
                case ' ':
                case '\t':
                case '\r':
                case '\n': break;
                default: return -1; // Fail code.
     dec:	8f ef       	ldi	r24, 0xFF	; 255
     dee:	9f ef       	ldi	r25, 0xFF	; 255
        case SKIP_ALL:
            break;
    }
    read();  // discard non-numeric
  }
}
     df0:	df 91       	pop	r29
     df2:	cf 91       	pop	r28
     df4:	1f 91       	pop	r17
     df6:	0f 91       	pop	r16
     df8:	ff 90       	pop	r15
     dfa:	08 95       	ret

00000dfc <_ZN6Stream8parseIntE13LookaheadModec>:
// lookahead determines how parseInt looks ahead in the stream.
// See LookaheadMode enumeration at the top of the file.
// Lookahead is terminated by the first character that is not a valid part of an integer.
// Once parsing commences, 'ignore' will be skipped in the stream.
long Stream::parseInt(LookaheadMode lookahead, char ignore)
{
     dfc:	cf 92       	push	r12
     dfe:	df 92       	push	r13
     e00:	ff 92       	push	r15
     e02:	0f 93       	push	r16
     e04:	1f 93       	push	r17
     e06:	cf 93       	push	r28
     e08:	df 93       	push	r29
     e0a:	00 d0       	rcall	.+0      	; 0xe0c <_ZN6Stream8parseIntE13LookaheadModec+0x10>
     e0c:	00 d0       	rcall	.+0      	; 0xe0e <_ZN6Stream8parseIntE13LookaheadModec+0x12>
     e0e:	cd b7       	in	r28, 0x3d	; 61
     e10:	de b7       	in	r29, 0x3e	; 62
     e12:	6c 01       	movw	r12, r24
     e14:	04 2f       	mov	r16, r20
  bool isNegative = false;
  long value = 0;
  int c;

  c = peekNextDigit(lookahead, false);
     e16:	40 e0       	ldi	r20, 0x00	; 0
     e18:	0e 94 bb 06 	call	0xd76	; 0xd76 <_ZN6Stream13peekNextDigitE13LookaheadModeb>
     e1c:	fc 01       	movw	r30, r24
  // ignore non numeric leading characters
  if(c < 0)
     e1e:	97 fd       	sbrc	r25, 7
     e20:	4b c0       	rjmp	.+150    	; 0xeb8 <_ZN6Stream8parseIntE13LookaheadModec+0xbc>
     e22:	20 e0       	ldi	r18, 0x00	; 0
     e24:	30 e0       	ldi	r19, 0x00	; 0
     e26:	a9 01       	movw	r20, r18
     e28:	f1 2c       	mov	r15, r1
    return 0; // zero returned if timeout

  do{
    if(c == ignore)
     e2a:	11 27       	eor	r17, r17
     e2c:	07 fd       	sbrc	r16, 7
     e2e:	10 95       	com	r17
     e30:	e0 17       	cp	r30, r16
     e32:	f1 07       	cpc	r31, r17
     e34:	e9 f0       	breq	.+58     	; 0xe70 <_ZN6Stream8parseIntE13LookaheadModec+0x74>
      ; // ignore this character
    else if(c == '-')
     e36:	ed 32       	cpi	r30, 0x2D	; 45
     e38:	f1 05       	cpc	r31, r1
     e3a:	c1 f0       	breq	.+48     	; 0xe6c <_ZN6Stream8parseIntE13LookaheadModec+0x70>
      isNegative = true;
    else if(c >= '0' && c <= '9')        // is c a digit?
     e3c:	cf 01       	movw	r24, r30
     e3e:	c0 97       	sbiw	r24, 0x30	; 48
     e40:	0a 97       	sbiw	r24, 0x0a	; 10
     e42:	b0 f4       	brcc	.+44     	; 0xe70 <_ZN6Stream8parseIntE13LookaheadModec+0x74>
      value = value * 10 + c - '0';
     e44:	aa e0       	ldi	r26, 0x0A	; 10
     e46:	b0 e0       	ldi	r27, 0x00	; 0
     e48:	0e 94 bd 0f 	call	0x1f7a	; 0x1f7a <__muluhisi3>
     e4c:	9b 01       	movw	r18, r22
     e4e:	ac 01       	movw	r20, r24
     e50:	cf 01       	movw	r24, r30
     e52:	aa 27       	eor	r26, r26
     e54:	97 fd       	sbrc	r25, 7
     e56:	a0 95       	com	r26
     e58:	ba 2f       	mov	r27, r26
     e5a:	28 0f       	add	r18, r24
     e5c:	39 1f       	adc	r19, r25
     e5e:	4a 1f       	adc	r20, r26
     e60:	5b 1f       	adc	r21, r27
     e62:	20 53       	subi	r18, 0x30	; 48
     e64:	31 09       	sbc	r19, r1
     e66:	41 09       	sbc	r20, r1
     e68:	51 09       	sbc	r21, r1
     e6a:	02 c0       	rjmp	.+4      	; 0xe70 <_ZN6Stream8parseIntE13LookaheadModec+0x74>

  do{
    if(c == ignore)
      ; // ignore this character
    else if(c == '-')
      isNegative = true;
     e6c:	ff 24       	eor	r15, r15
     e6e:	f3 94       	inc	r15
    else if(c >= '0' && c <= '9')        // is c a digit?
      value = value * 10 + c - '0';
    read();  // consume the character we got with peek
     e70:	d6 01       	movw	r26, r12
     e72:	ed 91       	ld	r30, X+
     e74:	fc 91       	ld	r31, X
     e76:	06 80       	ldd	r0, Z+6	; 0x06
     e78:	f7 81       	ldd	r31, Z+7	; 0x07
     e7a:	e0 2d       	mov	r30, r0
     e7c:	c6 01       	movw	r24, r12
     e7e:	29 83       	std	Y+1, r18	; 0x01
     e80:	3a 83       	std	Y+2, r19	; 0x02
     e82:	4b 83       	std	Y+3, r20	; 0x03
     e84:	5c 83       	std	Y+4, r21	; 0x04
     e86:	09 95       	icall
    c = timedPeek();
     e88:	c6 01       	movw	r24, r12
     e8a:	0e 94 8d 06 	call	0xd1a	; 0xd1a <_ZN6Stream9timedPeekEv>
     e8e:	fc 01       	movw	r30, r24
  }
  while( (c >= '0' && c <= '9') || c == ignore );
     e90:	c0 97       	sbiw	r24, 0x30	; 48
  c = peekNextDigit(lookahead, false);
  // ignore non numeric leading characters
  if(c < 0)
    return 0; // zero returned if timeout

  do{
     e92:	29 81       	ldd	r18, Y+1	; 0x01
     e94:	3a 81       	ldd	r19, Y+2	; 0x02
     e96:	4b 81       	ldd	r20, Y+3	; 0x03
     e98:	5c 81       	ldd	r21, Y+4	; 0x04
     e9a:	0a 97       	sbiw	r24, 0x0a	; 10
     e9c:	48 f2       	brcs	.-110    	; 0xe30 <_ZN6Stream8parseIntE13LookaheadModec+0x34>
    else if(c >= '0' && c <= '9')        // is c a digit?
      value = value * 10 + c - '0';
    read();  // consume the character we got with peek
    c = timedPeek();
  }
  while( (c >= '0' && c <= '9') || c == ignore );
     e9e:	e0 17       	cp	r30, r16
     ea0:	f1 07       	cpc	r31, r17
     ea2:	31 f2       	breq	.-116    	; 0xe30 <_ZN6Stream8parseIntE13LookaheadModec+0x34>

  if(isNegative)
     ea4:	ff 20       	and	r15, r15
     ea6:	61 f0       	breq	.+24     	; 0xec0 <_ZN6Stream8parseIntE13LookaheadModec+0xc4>
    value = -value;
     ea8:	66 27       	eor	r22, r22
     eaa:	77 27       	eor	r23, r23
     eac:	cb 01       	movw	r24, r22
     eae:	62 1b       	sub	r22, r18
     eb0:	73 0b       	sbc	r23, r19
     eb2:	84 0b       	sbc	r24, r20
     eb4:	95 0b       	sbc	r25, r21
     eb6:	06 c0       	rjmp	.+12     	; 0xec4 <_ZN6Stream8parseIntE13LookaheadModec+0xc8>
  int c;

  c = peekNextDigit(lookahead, false);
  // ignore non numeric leading characters
  if(c < 0)
    return 0; // zero returned if timeout
     eb8:	60 e0       	ldi	r22, 0x00	; 0
     eba:	70 e0       	ldi	r23, 0x00	; 0
     ebc:	cb 01       	movw	r24, r22
     ebe:	02 c0       	rjmp	.+4      	; 0xec4 <_ZN6Stream8parseIntE13LookaheadModec+0xc8>
     ec0:	ca 01       	movw	r24, r20
     ec2:	b9 01       	movw	r22, r18
  while( (c >= '0' && c <= '9') || c == ignore );

  if(isNegative)
    value = -value;
  return value;
}
     ec4:	0f 90       	pop	r0
     ec6:	0f 90       	pop	r0
     ec8:	0f 90       	pop	r0
     eca:	0f 90       	pop	r0
     ecc:	df 91       	pop	r29
     ece:	cf 91       	pop	r28
     ed0:	1f 91       	pop	r17
     ed2:	0f 91       	pop	r16
     ed4:	ff 90       	pop	r15
     ed6:	df 90       	pop	r13
     ed8:	cf 90       	pop	r12
     eda:	08 95       	ret

00000edc <_ZL4RecvPVhh>:
	UEINTX = ~(1<<RXOUTI);
}

static inline void Recv(volatile u8* data, u8 count)
{
	while (count--)
     edc:	61 50       	subi	r22, 0x01	; 1
     ede:	30 f0       	brcs	.+12     	; 0xeec <_ZL4RecvPVhh+0x10>
		*data++ = UEDATX;
     ee0:	20 91 f1 00 	lds	r18, 0x00F1
     ee4:	fc 01       	movw	r30, r24
     ee6:	20 83       	st	Z, r18
     ee8:	01 96       	adiw	r24, 0x01	; 1
     eea:	f8 cf       	rjmp	.-16     	; 0xedc <_ZL4RecvPVhh>
	
	RXLED1;					// light the RX LED
     eec:	28 98       	cbi	0x05, 0	; 5
	RxLEDPulse = TX_RX_LED_PULSE_MS;	
     eee:	84 e6       	ldi	r24, 0x64	; 100
     ef0:	80 93 5f 02 	sts	0x025F, r24
     ef4:	08 95       	ret

00000ef6 <_ZL11SendControlh>:
}

static
bool SendControl(u8 d)
{
	if (_cmark < _cend)
     ef6:	40 91 5a 02 	lds	r20, 0x025A
     efa:	50 91 5b 02 	lds	r21, 0x025B
     efe:	20 91 58 02 	lds	r18, 0x0258
     f02:	30 91 59 02 	lds	r19, 0x0259
     f06:	42 17       	cp	r20, r18
     f08:	53 07       	cpc	r21, r19
     f0a:	b4 f4       	brge	.+44     	; 0xf38 <_ZL11SendControlh+0x42>
		;
}

static inline u8 WaitForINOrOUT()
{
	while (!(UEINTX & ((1<<TXINI)|(1<<RXOUTI))))
     f0c:	90 91 e8 00 	lds	r25, 0x00E8
     f10:	95 70       	andi	r25, 0x05	; 5
     f12:	e1 f3       	breq	.-8      	; 0xf0c <_ZL11SendControlh+0x16>
		;
	return (UEINTX & (1<<RXOUTI)) == 0;
     f14:	90 91 e8 00 	lds	r25, 0x00E8
static
bool SendControl(u8 d)
{
	if (_cmark < _cend)
	{
		if (!WaitForINOrOUT())
     f18:	92 fd       	sbrc	r25, 2
     f1a:	19 c0       	rjmp	.+50     	; 0xf4e <_ZL11SendControlh+0x58>
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
     f1c:	80 93 f1 00 	sts	0x00F1, r24
	if (_cmark < _cend)
	{
		if (!WaitForINOrOUT())
			return false;
		Send8(d);
		if (!((_cmark + 1) & 0x3F))
     f20:	80 91 5a 02 	lds	r24, 0x025A
     f24:	90 91 5b 02 	lds	r25, 0x025B
     f28:	01 96       	adiw	r24, 0x01	; 1
     f2a:	8f 73       	andi	r24, 0x3F	; 63
     f2c:	99 27       	eor	r25, r25
     f2e:	89 2b       	or	r24, r25
     f30:	19 f4       	brne	.+6      	; 0xf38 <_ZL11SendControlh+0x42>
		;
}

static inline void ClearIN(void)
{
	UEINTX = ~(1<<TXINI);
     f32:	8e ef       	ldi	r24, 0xFE	; 254
     f34:	80 93 e8 00 	sts	0x00E8, r24
			return false;
		Send8(d);
		if (!((_cmark + 1) & 0x3F))
			ClearIN();	// Fifo is full, release this packet
	}
	_cmark++;
     f38:	80 91 5a 02 	lds	r24, 0x025A
     f3c:	90 91 5b 02 	lds	r25, 0x025B
     f40:	01 96       	adiw	r24, 0x01	; 1
     f42:	90 93 5b 02 	sts	0x025B, r25
     f46:	80 93 5a 02 	sts	0x025A, r24
	return true;
     f4a:	81 e0       	ldi	r24, 0x01	; 1
     f4c:	08 95       	ret
bool SendControl(u8 d)
{
	if (_cmark < _cend)
	{
		if (!WaitForINOrOUT())
			return false;
     f4e:	80 e0       	ldi	r24, 0x00	; 0
		if (!((_cmark + 1) & 0x3F))
			ClearIN();	// Fifo is full, release this packet
	}
	_cmark++;
	return true;
};
     f50:	08 95       	ret

00000f52 <_ZL24USB_SendStringDescriptorPKhhh>:
}

// Send a USB descriptor string. The string is stored in PROGMEM as a
// plain ASCII string but is sent out as UTF-16 with the correct 2-byte
// prefix
static bool USB_SendStringDescriptor(const u8*string_P, u8 string_len, uint8_t flags) {
     f52:	cf 92       	push	r12
     f54:	df 92       	push	r13
     f56:	ef 92       	push	r14
     f58:	ff 92       	push	r15
     f5a:	0f 93       	push	r16
     f5c:	1f 93       	push	r17
     f5e:	cf 93       	push	r28
     f60:	df 93       	push	r29
     f62:	1f 92       	push	r1
     f64:	cd b7       	in	r28, 0x3d	; 61
     f66:	de b7       	in	r29, 0x3e	; 62
     f68:	08 2f       	mov	r16, r24
     f6a:	16 2f       	mov	r17, r22
     f6c:	f4 2e       	mov	r15, r20
        SendControl(2 + string_len * 2);
     f6e:	86 2f       	mov	r24, r22
     f70:	88 0f       	add	r24, r24
     f72:	8e 5f       	subi	r24, 0xFE	; 254
     f74:	99 83       	std	Y+1, r25	; 0x01
     f76:	0e 94 7b 07 	call	0xef6	; 0xef6 <_ZL11SendControlh>
        SendControl(3);
     f7a:	83 e0       	ldi	r24, 0x03	; 3
     f7c:	0e 94 7b 07 	call	0xef6	; 0xef6 <_ZL11SendControlh>
     f80:	e0 2e       	mov	r14, r16
        bool pgm = flags & TRANSFER_PGM;
        for(u8 i = 0; i < string_len; i++) {
     f82:	c0 2e       	mov	r12, r16
     f84:	99 81       	ldd	r25, Y+1	; 0x01
     f86:	d9 2e       	mov	r13, r25
     f88:	8c 2d       	mov	r24, r12
     f8a:	8e 19       	sub	r24, r14
     f8c:	81 17       	cp	r24, r17
     f8e:	98 f4       	brcc	.+38     	; 0xfb6 <_ZL24USB_SendStringDescriptorPKhhh+0x64>
                bool r = SendControl(pgm ? pgm_read_byte(&string_P[i]) : string_P[i]);
     f90:	f6 01       	movw	r30, r12
     f92:	f7 fe       	sbrs	r15, 7
     f94:	02 c0       	rjmp	.+4      	; 0xf9a <_ZL24USB_SendStringDescriptorPKhhh+0x48>
     f96:	84 91       	lpm	r24, Z
     f98:	01 c0       	rjmp	.+2      	; 0xf9c <_ZL24USB_SendStringDescriptorPKhhh+0x4a>
     f9a:	80 81       	ld	r24, Z
     f9c:	0e 94 7b 07 	call	0xef6	; 0xef6 <_ZL11SendControlh>
     fa0:	08 2f       	mov	r16, r24
                r &= SendControl(0); // high byte
     fa2:	80 e0       	ldi	r24, 0x00	; 0
     fa4:	0e 94 7b 07 	call	0xef6	; 0xef6 <_ZL11SendControlh>
     fa8:	80 23       	and	r24, r16
     faa:	ff ef       	ldi	r31, 0xFF	; 255
     fac:	cf 1a       	sub	r12, r31
     fae:	df 0a       	sbc	r13, r31
                if(!r) {
     fb0:	81 11       	cpse	r24, r1
     fb2:	ea cf       	rjmp	.-44     	; 0xf88 <_ZL24USB_SendStringDescriptorPKhhh+0x36>
     fb4:	01 c0       	rjmp	.+2      	; 0xfb8 <_ZL24USB_SendStringDescriptorPKhhh+0x66>
                        return false;
                }
        }
        return true;
     fb6:	81 e0       	ldi	r24, 0x01	; 1
}
     fb8:	0f 90       	pop	r0
     fba:	df 91       	pop	r29
     fbc:	cf 91       	pop	r28
     fbe:	1f 91       	pop	r17
     fc0:	0f 91       	pop	r16
     fc2:	ff 90       	pop	r15
     fc4:	ef 90       	pop	r14
     fc6:	df 90       	pop	r13
     fc8:	cf 90       	pop	r12
     fca:	08 95       	ret

00000fcc <_ZL15USB_ClockEnablev>:
}

static inline void USB_ClockEnable()
{
#if defined(UHWCON)
	UHWCON |= (1<<UVREGE);			// power internal reg
     fcc:	80 91 d7 00 	lds	r24, 0x00D7
     fd0:	81 60       	ori	r24, 0x01	; 1
     fd2:	80 93 d7 00 	sts	0x00D7, r24
#endif
	USBCON = (1<<USBE) | (1<<FRZCLK);	// clock frozen, usb enabled
     fd6:	80 ea       	ldi	r24, 0xA0	; 160
     fd8:	80 93 d8 00 	sts	0x00D8, r24

// ATmega32U4
#if defined(PINDIV)
#if F_CPU == 16000000UL
	PLLCSR |= (1<<PINDIV);                   // Need 16 MHz xtal
     fdc:	89 b5       	in	r24, 0x29	; 41
     fde:	80 61       	ori	r24, 0x10	; 16
     fe0:	89 bd       	out	0x29, r24	; 41
#endif
#else
#error "USB Chip not supported, please defined method of USB PLL initialization"
#endif

	PLLCSR |= (1<<PLLE);
     fe2:	89 b5       	in	r24, 0x29	; 41
     fe4:	82 60       	ori	r24, 0x02	; 2
     fe6:	89 bd       	out	0x29, r24	; 41
	while (!(PLLCSR & (1<<PLOCK)))		// wait for lock pll
     fe8:	09 b4       	in	r0, 0x29	; 41
     fea:	00 fe       	sbrs	r0, 0
     fec:	fd cf       	rjmp	.-6      	; 0xfe8 <_ZL15USB_ClockEnablev+0x1c>
	}

	// Some tests on specific versions of macosx (10.7.3), reported some
	// strange behaviors when the board is reset using the serial
	// port touch at 1200 bps. This delay fixes this behavior.
	delay(1);
     fee:	61 e0       	ldi	r22, 0x01	; 1
     ff0:	70 e0       	ldi	r23, 0x00	; 0
     ff2:	80 e0       	ldi	r24, 0x00	; 0
     ff4:	90 e0       	ldi	r25, 0x00	; 0
     ff6:	0e 94 4a 0c 	call	0x1894	; 0x1894 <delay>
#if defined(OTGPADE)
	USBCON = (USBCON & ~(1<<FRZCLK)) | (1<<OTGPADE);	// start USB clock, enable VBUS Pad
     ffa:	80 91 d8 00 	lds	r24, 0x00D8
     ffe:	8f 7c       	andi	r24, 0xCF	; 207
    1000:	80 61       	ori	r24, 0x10	; 16
    1002:	80 93 d8 00 	sts	0x00D8, r24
	USBCON &= ~(1 << FRZCLK);	// start USB clock
#endif

#if defined(RSTCPU)
#if defined(LSM)
	UDCON &= ~((1<<RSTCPU) | (1<<LSM) | (1<<RMWKUP) | (1<<DETACH));	// enable attach resistor, set full speed mode
    1006:	80 91 e0 00 	lds	r24, 0x00E0
    100a:	80 7f       	andi	r24, 0xF0	; 240
    100c:	80 93 e0 00 	sts	0x00E0, r24
    1010:	08 95       	ret

00001012 <_ZL14SendInterfacesv>:
	}
	return len;
}

static u8 SendInterfaces()
{
    1012:	cf 93       	push	r28
    1014:	df 93       	push	r29
    1016:	1f 92       	push	r1
    1018:	cd b7       	in	r28, 0x3d	; 61
    101a:	de b7       	in	r29, 0x3e	; 62
	u8 interfaces = 0;
    101c:	19 82       	std	Y+1, r1	; 0x01

	CDC_GetInterface(&interfaces);
    101e:	ce 01       	movw	r24, r28
    1020:	01 96       	adiw	r24, 0x01	; 1
    1022:	0e 94 ea 03 	call	0x7d4	; 0x7d4 <_Z16CDC_GetInterfacePh>

#ifdef PLUGGABLE_USB_ENABLED
	PluggableUSB().getInterface(&interfaces);
    1026:	0e 94 61 0d 	call	0x1ac2	; 0x1ac2 <_Z12PluggableUSBv>
    102a:	be 01       	movw	r22, r28
    102c:	6f 5f       	subi	r22, 0xFF	; 255
    102e:	7f 4f       	sbci	r23, 0xFF	; 255
    1030:	0e 94 da 0c 	call	0x19b4	; 0x19b4 <_ZN13PluggableUSB_12getInterfaceEPh>
#endif

	return interfaces;
}
    1034:	89 81       	ldd	r24, Y+1	; 0x01
    1036:	0f 90       	pop	r0
    1038:	df 91       	pop	r29
    103a:	cf 91       	pop	r28
    103c:	08 95       	ret

0000103e <_ZN6LockEPC1Eh>:
#define USB_RECV_TIMEOUT
class LockEP
{
	u8 _sreg;
public:
	LockEP(u8 ep) : _sreg(SREG)
    103e:	2f b7       	in	r18, 0x3f	; 63
    1040:	fc 01       	movw	r30, r24
    1042:	20 83       	st	Z, r18
	{
		cli();
    1044:	f8 94       	cli
		SetEP(ep & 7);
    1046:	67 70       	andi	r22, 0x07	; 7
	UEDATX = d;
}

static inline void SetEP(u8 ep)
{
	UENUM = ep;
    1048:	60 93 e9 00 	sts	0x00E9, r22
    104c:	08 95       	ret

0000104e <_Z13USB_Availableh>:
	}
};

//	Number of bytes, assumes a rx endpoint
u8 USB_Available(u8 ep)
{
    104e:	cf 93       	push	r28
    1050:	df 93       	push	r29
    1052:	1f 92       	push	r1
    1054:	cd b7       	in	r28, 0x3d	; 61
    1056:	de b7       	in	r29, 0x3e	; 62
	LockEP lock(ep);
    1058:	68 2f       	mov	r22, r24
    105a:	ce 01       	movw	r24, r28
    105c:	01 96       	adiw	r24, 0x01	; 1
    105e:	0e 94 1f 08 	call	0x103e	; 0x103e <_ZN6LockEPC1Eh>
	UENUM = ep;
}

static inline u8 FifoByteCount()
{
	return UEBCLX;
    1062:	80 91 f2 00 	lds	r24, 0x00F2
		cli();
		SetEP(ep & 7);
	}
	~LockEP()
	{
		SREG = _sreg;
    1066:	99 81       	ldd	r25, Y+1	; 0x01
    1068:	9f bf       	out	0x3f, r25	; 63
//	Number of bytes, assumes a rx endpoint
u8 USB_Available(u8 ep)
{
	LockEP lock(ep);
	return FifoByteCount();
}
    106a:	0f 90       	pop	r0
    106c:	df 91       	pop	r29
    106e:	cf 91       	pop	r28
    1070:	08 95       	ret

00001072 <_Z8USB_RecvhPvi>:

//	Non Blocking receive
//	Return number of bytes read
int USB_Recv(u8 ep, void* d, int len)
{
    1072:	ff 92       	push	r15
    1074:	0f 93       	push	r16
    1076:	1f 93       	push	r17
    1078:	cf 93       	push	r28
    107a:	df 93       	push	r29
    107c:	00 d0       	rcall	.+0      	; 0x107e <_Z8USB_RecvhPvi+0xc>
    107e:	cd b7       	in	r28, 0x3d	; 61
    1080:	de b7       	in	r29, 0x3e	; 62
    1082:	f6 2e       	mov	r15, r22
    1084:	8a 01       	movw	r16, r20
	if (!_usbConfiguration || len < 0)
    1086:	90 91 5e 02 	lds	r25, 0x025E
    108a:	99 23       	and	r25, r25
    108c:	11 f0       	breq	.+4      	; 0x1092 <_Z8USB_RecvhPvi+0x20>
    108e:	57 ff       	sbrs	r21, 7
    1090:	03 c0       	rjmp	.+6      	; 0x1098 <_Z8USB_RecvhPvi+0x26>
		return -1;
    1092:	8f ef       	ldi	r24, 0xFF	; 255
    1094:	9f ef       	ldi	r25, 0xFF	; 255
    1096:	2b c0       	rjmp	.+86     	; 0x10ee <_Z8USB_RecvhPvi+0x7c>
	
	LockEP lock(ep);
    1098:	68 2f       	mov	r22, r24
    109a:	ce 01       	movw	r24, r28
    109c:	01 96       	adiw	r24, 0x01	; 1
    109e:	7a 83       	std	Y+2, r23	; 0x02
    10a0:	0e 94 1f 08 	call	0x103e	; 0x103e <_ZN6LockEPC1Eh>
	UENUM = ep;
}

static inline u8 FifoByteCount()
{
	return UEBCLX;
    10a4:	80 91 f2 00 	lds	r24, 0x00F2
	if (!_usbConfiguration || len < 0)
		return -1;
	
	LockEP lock(ep);
	u8 n = FifoByteCount();
	len = min(n,len);
    10a8:	90 e0       	ldi	r25, 0x00	; 0
    10aa:	a8 01       	movw	r20, r16
    10ac:	7a 81       	ldd	r23, Y+2	; 0x02
    10ae:	80 17       	cp	r24, r16
    10b0:	91 07       	cpc	r25, r17
    10b2:	0c f4       	brge	.+2      	; 0x10b6 <_Z8USB_RecvhPvi+0x44>
    10b4:	ac 01       	movw	r20, r24
    10b6:	ef 2d       	mov	r30, r15
    10b8:	f7 2f       	mov	r31, r23
    10ba:	f4 0e       	add	r15, r20
}

static inline u8 Recv8()
{
	RXLED1;					// light the RX LED
	RxLEDPulse = TX_RX_LED_PULSE_MS;
    10bc:	84 e6       	ldi	r24, 0x64	; 100
	LockEP lock(ep);
	u8 n = FifoByteCount();
	len = min(n,len);
	n = len;
	u8* dst = (u8*)d;
	while (n--)
    10be:	fe 16       	cp	r15, r30
    10c0:	39 f0       	breq	.+14     	; 0x10d0 <_Z8USB_RecvhPvi+0x5e>
	RxLEDPulse = TX_RX_LED_PULSE_MS;	
}

static inline u8 Recv8()
{
	RXLED1;					// light the RX LED
    10c2:	28 98       	cbi	0x05, 0	; 5
	RxLEDPulse = TX_RX_LED_PULSE_MS;
    10c4:	80 93 5f 02 	sts	0x025F, r24

	return UEDATX;	
    10c8:	90 91 f1 00 	lds	r25, 0x00F1
	u8 n = FifoByteCount();
	len = min(n,len);
	n = len;
	u8* dst = (u8*)d;
	while (n--)
		*dst++ = Recv8();
    10cc:	91 93       	st	Z+, r25
    10ce:	f7 cf       	rjmp	.-18     	; 0x10be <_Z8USB_RecvhPvi+0x4c>
	if (len && !FifoByteCount())	// release empty buffer
    10d0:	41 15       	cp	r20, r1
    10d2:	51 05       	cpc	r21, r1
    10d4:	21 f0       	breq	.+8      	; 0x10de <_Z8USB_RecvhPvi+0x6c>
	UENUM = ep;
}

static inline u8 FifoByteCount()
{
	return UEBCLX;
    10d6:	80 91 f2 00 	lds	r24, 0x00F2
	len = min(n,len);
	n = len;
	u8* dst = (u8*)d;
	while (n--)
		*dst++ = Recv8();
	if (len && !FifoByteCount())	// release empty buffer
    10da:	88 23       	and	r24, r24
    10dc:	21 f0       	breq	.+8      	; 0x10e6 <_Z8USB_RecvhPvi+0x74>
		cli();
		SetEP(ep & 7);
	}
	~LockEP()
	{
		SREG = _sreg;
    10de:	89 81       	ldd	r24, Y+1	; 0x01
    10e0:	8f bf       	out	0x3f, r24	; 63
	if (!_usbConfiguration || len < 0)
		return -1;
	
	LockEP lock(ep);
	u8 n = FifoByteCount();
	len = min(n,len);
    10e2:	ca 01       	movw	r24, r20
    10e4:	04 c0       	rjmp	.+8      	; 0x10ee <_Z8USB_RecvhPvi+0x7c>
	return UEINTX & (1<<FIFOCON);
}

static inline void ReleaseRX()
{
	UEINTX = 0x6B;	// FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
    10e6:	8b e6       	ldi	r24, 0x6B	; 107
    10e8:	80 93 e8 00 	sts	0x00E8, r24
    10ec:	f8 cf       	rjmp	.-16     	; 0x10de <_Z8USB_RecvhPvi+0x6c>
		*dst++ = Recv8();
	if (len && !FifoByteCount())	// release empty buffer
		ReleaseRX();
	
	return len;
}
    10ee:	0f 90       	pop	r0
    10f0:	0f 90       	pop	r0
    10f2:	df 91       	pop	r29
    10f4:	cf 91       	pop	r28
    10f6:	1f 91       	pop	r17
    10f8:	0f 91       	pop	r16
    10fa:	ff 90       	pop	r15
    10fc:	08 95       	ret

000010fe <_Z8USB_Recvh>:

//	Recv 1 byte if ready
int USB_Recv(u8 ep)
{
    10fe:	cf 93       	push	r28
    1100:	df 93       	push	r29
    1102:	1f 92       	push	r1
    1104:	cd b7       	in	r28, 0x3d	; 61
    1106:	de b7       	in	r29, 0x3e	; 62
	u8 c;
	if (USB_Recv(ep,&c,1) != 1)
    1108:	41 e0       	ldi	r20, 0x01	; 1
    110a:	50 e0       	ldi	r21, 0x00	; 0
    110c:	be 01       	movw	r22, r28
    110e:	6f 5f       	subi	r22, 0xFF	; 255
    1110:	7f 4f       	sbci	r23, 0xFF	; 255
    1112:	0e 94 39 08 	call	0x1072	; 0x1072 <_Z8USB_RecvhPvi>
    1116:	01 97       	sbiw	r24, 0x01	; 1
    1118:	19 f4       	brne	.+6      	; 0x1120 <_Z8USB_Recvh+0x22>
		return -1;
	return c;
    111a:	89 81       	ldd	r24, Y+1	; 0x01
    111c:	90 e0       	ldi	r25, 0x00	; 0
    111e:	02 c0       	rjmp	.+4      	; 0x1124 <_Z8USB_Recvh+0x26>
//	Recv 1 byte if ready
int USB_Recv(u8 ep)
{
	u8 c;
	if (USB_Recv(ep,&c,1) != 1)
		return -1;
    1120:	8f ef       	ldi	r24, 0xFF	; 255
    1122:	9f ef       	ldi	r25, 0xFF	; 255
	return c;
}
    1124:	0f 90       	pop	r0
    1126:	df 91       	pop	r29
    1128:	cf 91       	pop	r28
    112a:	08 95       	ret

0000112c <_Z13USB_SendSpaceh>:

//	Space in send EP
u8 USB_SendSpace(u8 ep)
{
    112c:	cf 93       	push	r28
    112e:	df 93       	push	r29
    1130:	1f 92       	push	r1
    1132:	cd b7       	in	r28, 0x3d	; 61
    1134:	de b7       	in	r29, 0x3e	; 62
	LockEP lock(ep);
    1136:	68 2f       	mov	r22, r24
    1138:	ce 01       	movw	r24, r28
    113a:	01 96       	adiw	r24, 0x01	; 1
    113c:	0e 94 1f 08 	call	0x103e	; 0x103e <_ZN6LockEPC1Eh>
	UECONX = (1<<STALLRQ) | (1<<EPEN);
}

static inline u8 ReadWriteAllowed()
{
	return UEINTX & (1<<RWAL);
    1140:	90 91 e8 00 	lds	r25, 0x00E8

//	Space in send EP
u8 USB_SendSpace(u8 ep)
{
	LockEP lock(ep);
	if (!ReadWriteAllowed())
    1144:	89 2f       	mov	r24, r25
    1146:	80 72       	andi	r24, 0x20	; 32
    1148:	95 ff       	sbrs	r25, 5
    114a:	04 c0       	rjmp	.+8      	; 0x1154 <_Z13USB_SendSpaceh+0x28>
	UENUM = ep;
}

static inline u8 FifoByteCount()
{
	return UEBCLX;
    114c:	90 91 f2 00 	lds	r25, 0x00F2
u8 USB_SendSpace(u8 ep)
{
	LockEP lock(ep);
	if (!ReadWriteAllowed())
		return 0;
	return USB_EP_SIZE - FifoByteCount();
    1150:	80 e4       	ldi	r24, 0x40	; 64
    1152:	89 1b       	sub	r24, r25
		cli();
		SetEP(ep & 7);
	}
	~LockEP()
	{
		SREG = _sreg;
    1154:	99 81       	ldd	r25, Y+1	; 0x01
    1156:	9f bf       	out	0x3f, r25	; 63
{
	LockEP lock(ep);
	if (!ReadWriteAllowed())
		return 0;
	return USB_EP_SIZE - FifoByteCount();
}
    1158:	0f 90       	pop	r0
    115a:	df 91       	pop	r29
    115c:	cf 91       	pop	r28
    115e:	08 95       	ret

00001160 <_Z8USB_SendhPKvi>:

//	Blocking Send of data to an endpoint
int USB_Send(u8 ep, const void* d, int len)
{
    1160:	6f 92       	push	r6
    1162:	7f 92       	push	r7
    1164:	8f 92       	push	r8
    1166:	9f 92       	push	r9
    1168:	af 92       	push	r10
    116a:	bf 92       	push	r11
    116c:	cf 92       	push	r12
    116e:	df 92       	push	r13
    1170:	ef 92       	push	r14
    1172:	ff 92       	push	r15
    1174:	0f 93       	push	r16
    1176:	1f 93       	push	r17
    1178:	cf 93       	push	r28
    117a:	df 93       	push	r29
    117c:	1f 92       	push	r1
    117e:	cd b7       	in	r28, 0x3d	; 61
    1180:	de b7       	in	r29, 0x3e	; 62
    1182:	78 2e       	mov	r7, r24
    1184:	7b 01       	movw	r14, r22
    1186:	c4 2e       	mov	r12, r20
    1188:	b5 2e       	mov	r11, r21
	if (!_usbConfiguration)
    118a:	80 91 5e 02 	lds	r24, 0x025E
    118e:	88 23       	and	r24, r24
    1190:	69 f0       	breq	.+26     	; 0x11ac <_Z8USB_SendhPKvi+0x4c>
    1192:	04 2f       	mov	r16, r20
    1194:	15 2f       	mov	r17, r21
    1196:	8a ef       	ldi	r24, 0xFA	; 250
    1198:	d8 2e       	mov	r13, r24
			LockEP lock(ep);
			// Frame may have been released by the SOF interrupt handler
			if (!ReadWriteAllowed())
				continue;
			len -= n;
			if (ep & TRANSFER_ZERO)
    119a:	87 2d       	mov	r24, r7
    119c:	80 72       	andi	r24, 0x20	; 32
    119e:	98 2e       	mov	r9, r24
	UEINTX = 0x6B;	// FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static inline void ReleaseTX()
{
	UEINTX = 0x3A;	// FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
    11a0:	9a e3       	ldi	r25, 0x3A	; 58
    11a2:	a9 2e       	mov	r10, r25
			else
			{
				while (n--)
					Send8(*data++);
			}
			if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE)))	// Release full buffer
    11a4:	87 2d       	mov	r24, r7
    11a6:	80 74       	andi	r24, 0x40	; 64
    11a8:	88 2e       	mov	r8, r24
    11aa:	11 c0       	rjmp	.+34     	; 0x11ce <_Z8USB_SendhPKvi+0x6e>

//	Blocking Send of data to an endpoint
int USB_Send(u8 ep, const void* d, int len)
{
	if (!_usbConfiguration)
		return -1;
    11ac:	8f ef       	ldi	r24, 0xFF	; 255
    11ae:	9f ef       	ldi	r25, 0xFF	; 255
    11b0:	57 c0       	rjmp	.+174    	; 0x1260 <_Z8USB_SendhPKvi+0x100>
	int r = len;
	const u8* data = (const u8*)d;
	u8 timeout = 250;		// 250ms timeout on send? TODO
	while (len)
	{
		u8 n = USB_SendSpace(ep);
    11b2:	87 2d       	mov	r24, r7
    11b4:	0e 94 96 08 	call	0x112c	; 0x112c <_Z13USB_SendSpaceh>
    11b8:	68 2e       	mov	r6, r24
		if (n == 0)
    11ba:	81 11       	cpse	r24, r1
    11bc:	0c c0       	rjmp	.+24     	; 0x11d6 <_Z8USB_SendhPKvi+0x76>
		{
			if (!(--timeout))
    11be:	da 94       	dec	r13
    11c0:	a9 f3       	breq	.-22     	; 0x11ac <_Z8USB_SendhPKvi+0x4c>
				return -1;
			delay(1);
    11c2:	61 e0       	ldi	r22, 0x01	; 1
    11c4:	70 e0       	ldi	r23, 0x00	; 0
    11c6:	80 e0       	ldi	r24, 0x00	; 0
    11c8:	90 e0       	ldi	r25, 0x00	; 0
    11ca:	0e 94 4a 0c 	call	0x1894	; 0x1894 <delay>
		return -1;

	int r = len;
	const u8* data = (const u8*)d;
	u8 timeout = 250;		// 250ms timeout on send? TODO
	while (len)
    11ce:	01 15       	cp	r16, r1
    11d0:	11 05       	cpc	r17, r1
    11d2:	79 f7       	brne	.-34     	; 0x11b2 <_Z8USB_SendhPKvi+0x52>
    11d4:	3b c0       	rjmp	.+118    	; 0x124c <_Z8USB_SendhPKvi+0xec>
				return -1;
			delay(1);
			continue;
		}

		if (n > len)
    11d6:	28 2f       	mov	r18, r24
    11d8:	30 e0       	ldi	r19, 0x00	; 0
    11da:	02 17       	cp	r16, r18
    11dc:	13 07       	cpc	r17, r19
    11de:	0c f4       	brge	.+2      	; 0x11e2 <_Z8USB_SendhPKvi+0x82>
			n = len;
    11e0:	60 2e       	mov	r6, r16
		{
			LockEP lock(ep);
    11e2:	67 2d       	mov	r22, r7
    11e4:	ce 01       	movw	r24, r28
    11e6:	01 96       	adiw	r24, 0x01	; 1
    11e8:	0e 94 1f 08 	call	0x103e	; 0x103e <_ZN6LockEPC1Eh>
	UECONX = (1<<STALLRQ) | (1<<EPEN);
}

static inline u8 ReadWriteAllowed()
{
	return UEINTX & (1<<RWAL);
    11ec:	80 91 e8 00 	lds	r24, 0x00E8
		if (n > len)
			n = len;
		{
			LockEP lock(ep);
			// Frame may have been released by the SOF interrupt handler
			if (!ReadWriteAllowed())
    11f0:	85 ff       	sbrs	r24, 5
    11f2:	29 c0       	rjmp	.+82     	; 0x1246 <_Z8USB_SendhPKvi+0xe6>
				continue;
			len -= n;
    11f4:	26 2d       	mov	r18, r6
    11f6:	30 e0       	ldi	r19, 0x00	; 0
    11f8:	02 1b       	sub	r16, r18
    11fa:	13 0b       	sbc	r17, r19
			if (ep & TRANSFER_ZERO)
    11fc:	99 20       	and	r9, r9
    11fe:	39 f0       	breq	.+14     	; 0x120e <_Z8USB_SendhPKvi+0xae>
			{
				while (n--)
    1200:	6a 94       	dec	r6
    1202:	8f ef       	ldi	r24, 0xFF	; 255
    1204:	68 16       	cp	r6, r24
    1206:	b1 f0       	breq	.+44     	; 0x1234 <_Z8USB_SendhPKvi+0xd4>
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
    1208:	10 92 f1 00 	sts	0x00F1, r1
    120c:	f9 cf       	rjmp	.-14     	; 0x1200 <_Z8USB_SendhPKvi+0xa0>
			if (ep & TRANSFER_ZERO)
			{
				while (n--)
					Send8(0);
			}
			else if (ep & TRANSFER_PGM)
    120e:	f7 01       	movw	r30, r14
    1210:	86 2d       	mov	r24, r6
    1212:	77 fe       	sbrs	r7, 7
    1214:	07 c0       	rjmp	.+14     	; 0x1224 <_Z8USB_SendhPKvi+0xc4>
			{
				while (n--)
    1216:	81 50       	subi	r24, 0x01	; 1
    1218:	58 f0       	brcs	.+22     	; 0x1230 <_Z8USB_SendhPKvi+0xd0>
					Send8(pgm_read_byte(data++));
    121a:	94 91       	lpm	r25, Z
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
    121c:	90 93 f1 00 	sts	0x00F1, r25
    1220:	31 96       	adiw	r30, 0x01	; 1
    1222:	f9 cf       	rjmp	.-14     	; 0x1216 <_Z8USB_SendhPKvi+0xb6>
				while (n--)
					Send8(pgm_read_byte(data++));
			}
			else
			{
				while (n--)
    1224:	81 50       	subi	r24, 0x01	; 1
    1226:	20 f0       	brcs	.+8      	; 0x1230 <_Z8USB_SendhPKvi+0xd0>
					Send8(*data++);
    1228:	91 91       	ld	r25, Z+
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
    122a:	90 93 f1 00 	sts	0x00F1, r25
    122e:	fa cf       	rjmp	.-12     	; 0x1224 <_Z8USB_SendhPKvi+0xc4>
    1230:	e2 0e       	add	r14, r18
    1232:	f3 1e       	adc	r15, r19
	UECONX = (1<<STALLRQ) | (1<<EPEN);
}

static inline u8 ReadWriteAllowed()
{
	return UEINTX & (1<<RWAL);
    1234:	80 91 e8 00 	lds	r24, 0x00E8
			else
			{
				while (n--)
					Send8(*data++);
			}
			if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE)))	// Release full buffer
    1238:	85 ff       	sbrs	r24, 5
    123a:	0f c0       	rjmp	.+30     	; 0x125a <_Z8USB_SendhPKvi+0xfa>
    123c:	01 15       	cp	r16, r1
    123e:	11 05       	cpc	r17, r1
    1240:	11 f4       	brne	.+4      	; 0x1246 <_Z8USB_SendhPKvi+0xe6>
    1242:	81 10       	cpse	r8, r1
    1244:	0a c0       	rjmp	.+20     	; 0x125a <_Z8USB_SendhPKvi+0xfa>
		cli();
		SetEP(ep & 7);
	}
	~LockEP()
	{
		SREG = _sreg;
    1246:	89 81       	ldd	r24, Y+1	; 0x01
    1248:	8f bf       	out	0x3f, r24	; 63
    124a:	c1 cf       	rjmp	.-126    	; 0x11ce <_Z8USB_SendhPKvi+0x6e>
			}
			if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE)))	// Release full buffer
				ReleaseTX();
		}
	}
	TXLED1;					// light the TX LED
    124c:	5d 98       	cbi	0x0b, 5	; 11
	TxLEDPulse = TX_RX_LED_PULSE_MS;
    124e:	84 e6       	ldi	r24, 0x64	; 100
    1250:	80 93 60 02 	sts	0x0260, r24
	return r;
    1254:	8c 2d       	mov	r24, r12
    1256:	9b 2d       	mov	r25, r11
    1258:	03 c0       	rjmp	.+6      	; 0x1260 <_Z8USB_SendhPKvi+0x100>
	UEINTX = 0x6B;	// FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static inline void ReleaseTX()
{
	UEINTX = 0x3A;	// FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
    125a:	a0 92 e8 00 	sts	0x00E8, r10
    125e:	f3 cf       	rjmp	.-26     	; 0x1246 <_Z8USB_SendhPKvi+0xe6>
		}
	}
	TXLED1;					// light the TX LED
	TxLEDPulse = TX_RX_LED_PULSE_MS;
	return r;
}
    1260:	0f 90       	pop	r0
    1262:	df 91       	pop	r29
    1264:	cf 91       	pop	r28
    1266:	1f 91       	pop	r17
    1268:	0f 91       	pop	r16
    126a:	ff 90       	pop	r15
    126c:	ef 90       	pop	r14
    126e:	df 90       	pop	r13
    1270:	cf 90       	pop	r12
    1272:	bf 90       	pop	r11
    1274:	af 90       	pop	r10
    1276:	9f 90       	pop	r9
    1278:	8f 90       	pop	r8
    127a:	7f 90       	pop	r7
    127c:	6f 90       	pop	r6
    127e:	08 95       	ret

00001280 <_Z11InitControli>:
	UEDATX = d;
}

static inline void SetEP(u8 ep)
{
	UENUM = ep;
    1280:	10 92 e9 00 	sts	0x00E9, r1
static int _cmark;
static int _cend;
void InitControl(int end)
{
	SetEP(0);
	_cmark = 0;
    1284:	10 92 5b 02 	sts	0x025B, r1
    1288:	10 92 5a 02 	sts	0x025A, r1
	_cend = end;
    128c:	90 93 59 02 	sts	0x0259, r25
    1290:	80 93 58 02 	sts	0x0258, r24
    1294:	08 95       	ret

00001296 <_Z15USB_SendControlhPKvi>:
	return true;
};

//	Clipped by _cmark/_cend
int USB_SendControl(u8 flags, const void* d, int len)
{
    1296:	cf 92       	push	r12
    1298:	df 92       	push	r13
    129a:	ff 92       	push	r15
    129c:	0f 93       	push	r16
    129e:	1f 93       	push	r17
    12a0:	cf 93       	push	r28
    12a2:	df 93       	push	r29
    12a4:	00 d0       	rcall	.+0      	; 0x12a6 <_Z15USB_SendControlhPKvi+0x10>
    12a6:	cd b7       	in	r28, 0x3d	; 61
    12a8:	de b7       	in	r29, 0x3e	; 62
    12aa:	f8 2e       	mov	r15, r24
    12ac:	8a 01       	movw	r16, r20
	int sent = len;
	const u8* data = (const u8*)d;
	bool pgm = flags & TRANSFER_PGM;
	while (len--)
    12ae:	6b 01       	movw	r12, r22
    12b0:	01 15       	cp	r16, r1
    12b2:	11 05       	cpc	r17, r1
    12b4:	b1 f0       	breq	.+44     	; 0x12e2 <_Z15USB_SendControlhPKvi+0x4c>
	{
		u8 c = pgm ? pgm_read_byte(data++) : *data++;
    12b6:	f6 01       	movw	r30, r12
    12b8:	f7 fe       	sbrs	r15, 7
    12ba:	02 c0       	rjmp	.+4      	; 0x12c0 <_Z15USB_SendControlhPKvi+0x2a>
    12bc:	84 91       	lpm	r24, Z
    12be:	01 c0       	rjmp	.+2      	; 0x12c2 <_Z15USB_SendControlhPKvi+0x2c>
    12c0:	80 81       	ld	r24, Z
		if (!SendControl(c))
    12c2:	49 83       	std	Y+1, r20	; 0x01
    12c4:	5a 83       	std	Y+2, r21	; 0x02
    12c6:	0e 94 7b 07 	call	0xef6	; 0xef6 <_ZL11SendControlh>
    12ca:	01 50       	subi	r16, 0x01	; 1
    12cc:	11 09       	sbc	r17, r1
    12ce:	ff ef       	ldi	r31, 0xFF	; 255
    12d0:	cf 1a       	sub	r12, r31
    12d2:	df 0a       	sbc	r13, r31
    12d4:	49 81       	ldd	r20, Y+1	; 0x01
    12d6:	5a 81       	ldd	r21, Y+2	; 0x02
    12d8:	81 11       	cpse	r24, r1
    12da:	ea cf       	rjmp	.-44     	; 0x12b0 <_Z15USB_SendControlhPKvi+0x1a>
			return -1;
    12dc:	8f ef       	ldi	r24, 0xFF	; 255
    12de:	9f ef       	ldi	r25, 0xFF	; 255
    12e0:	01 c0       	rjmp	.+2      	; 0x12e4 <_Z15USB_SendControlhPKvi+0x4e>
	}
	return sent;
    12e2:	ca 01       	movw	r24, r20
}
    12e4:	0f 90       	pop	r0
    12e6:	0f 90       	pop	r0
    12e8:	df 91       	pop	r29
    12ea:	cf 91       	pop	r28
    12ec:	1f 91       	pop	r17
    12ee:	0f 91       	pop	r16
    12f0:	ff 90       	pop	r15
    12f2:	df 90       	pop	r13
    12f4:	cf 90       	pop	r12
    12f6:	08 95       	ret

000012f8 <_Z15USB_RecvControlPvi>:
        return true;
}

//	Does not timeout or cross fifo boundaries
int USB_RecvControl(void* d, int len)
{
    12f8:	bf 92       	push	r11
    12fa:	cf 92       	push	r12
    12fc:	df 92       	push	r13
    12fe:	ef 92       	push	r14
    1300:	ff 92       	push	r15
    1302:	0f 93       	push	r16
    1304:	1f 93       	push	r17
    1306:	cf 93       	push	r28
    1308:	df 93       	push	r29
    130a:	6c 01       	movw	r12, r24
    130c:	8b 01       	movw	r16, r22
	auto length = len;
    130e:	eb 01       	movw	r28, r22
	return (UEINTX & (1<<RXOUTI)) == 0;
}

static inline void ClearOUT(void)
{
	UEINTX = ~(1<<RXOUTI);
    1310:	3b ef       	ldi	r19, 0xFB	; 251
    1312:	b3 2e       	mov	r11, r19

//	Does not timeout or cross fifo boundaries
int USB_RecvControl(void* d, int len)
{
	auto length = len;
	while(length)
    1314:	20 97       	sbiw	r28, 0x00	; 0
    1316:	c1 f0       	breq	.+48     	; 0x1348 <_Z15USB_RecvControlPvi+0x50>
    1318:	7e 01       	movw	r14, r28
    131a:	c1 34       	cpi	r28, 0x41	; 65
    131c:	d1 05       	cpc	r29, r1
    131e:	1c f0       	brlt	.+6      	; 0x1326 <_Z15USB_RecvControlPvi+0x2e>
    1320:	20 e4       	ldi	r18, 0x40	; 64
    1322:	e2 2e       	mov	r14, r18
    1324:	f1 2c       	mov	r15, r1
	UEINTX = ~(1<<TXINI);
}

static inline void WaitOUT(void)
{
	while (!(UEINTX & (1<<RXOUTI)))
    1326:	80 91 e8 00 	lds	r24, 0x00E8
    132a:	82 ff       	sbrs	r24, 2
    132c:	fc cf       	rjmp	.-8      	; 0x1326 <_Z15USB_RecvControlPvi+0x2e>
			recvLength = 64;
		}

		// Write data to fit to the end (not the beginning) of the array
		WaitOUT();
		Recv((u8*)d + len - length, recvLength);
    132e:	c8 01       	movw	r24, r16
    1330:	8c 1b       	sub	r24, r28
    1332:	9d 0b       	sbc	r25, r29
    1334:	6e 2d       	mov	r22, r14
    1336:	8c 0d       	add	r24, r12
    1338:	9d 1d       	adc	r25, r13
    133a:	0e 94 6e 07 	call	0xedc	; 0xedc <_ZL4RecvPVhh>
	return (UEINTX & (1<<RXOUTI)) == 0;
}

static inline void ClearOUT(void)
{
	UEINTX = ~(1<<RXOUTI);
    133e:	b0 92 e8 00 	sts	0x00E8, r11

		// Write data to fit to the end (not the beginning) of the array
		WaitOUT();
		Recv((u8*)d + len - length, recvLength);
		ClearOUT();
		length -= recvLength;
    1342:	ce 19       	sub	r28, r14
    1344:	df 09       	sbc	r29, r15
    1346:	e6 cf       	rjmp	.-52     	; 0x1314 <_Z15USB_RecvControlPvi+0x1c>
	}
	return len;
}
    1348:	c8 01       	movw	r24, r16
    134a:	df 91       	pop	r29
    134c:	cf 91       	pop	r28
    134e:	1f 91       	pop	r17
    1350:	0f 91       	pop	r16
    1352:	ff 90       	pop	r15
    1354:	ef 90       	pop	r14
    1356:	df 90       	pop	r13
    1358:	cf 90       	pop	r12
    135a:	bf 90       	pop	r11
    135c:	08 95       	ret

0000135e <__vector_11>:
	return true;
}

//	Endpoint 0 interrupt
ISR(USB_COM_vect)
{
    135e:	1f 92       	push	r1
    1360:	0f 92       	push	r0
    1362:	0f b6       	in	r0, 0x3f	; 63
    1364:	0f 92       	push	r0
    1366:	11 24       	eor	r1, r1
    1368:	df 92       	push	r13
    136a:	ef 92       	push	r14
    136c:	ff 92       	push	r15
    136e:	0f 93       	push	r16
    1370:	1f 93       	push	r17
    1372:	2f 93       	push	r18
    1374:	3f 93       	push	r19
    1376:	4f 93       	push	r20
    1378:	5f 93       	push	r21
    137a:	6f 93       	push	r22
    137c:	7f 93       	push	r23
    137e:	8f 93       	push	r24
    1380:	9f 93       	push	r25
    1382:	af 93       	push	r26
    1384:	bf 93       	push	r27
    1386:	ef 93       	push	r30
    1388:	ff 93       	push	r31
    138a:	cf 93       	push	r28
    138c:	df 93       	push	r29
    138e:	cd b7       	in	r28, 0x3d	; 61
    1390:	de b7       	in	r29, 0x3e	; 62
    1392:	6c 97       	sbiw	r28, 0x1c	; 28
    1394:	de bf       	out	0x3e, r29	; 62
    1396:	cd bf       	out	0x3d, r28	; 61
	UEDATX = d;
}

static inline void SetEP(u8 ep)
{
	UENUM = ep;
    1398:	10 92 e9 00 	sts	0x00E9, r1
	return UEBCLX;
}

static inline u8 ReceivedSetupInt()
{
	return UEINTX & (1<<RXSTPI);
    139c:	80 91 e8 00 	lds	r24, 0x00E8

//	Endpoint 0 interrupt
ISR(USB_COM_vect)
{
    SetEP(0);
	if (!ReceivedSetupInt())
    13a0:	83 ff       	sbrs	r24, 3
    13a2:	23 c1       	rjmp	.+582    	; 0x15ea <__vector_11+0x28c>
		return;

	USBSetup setup;
	Recv((u8*)&setup,8);
    13a4:	68 e0       	ldi	r22, 0x08	; 8
    13a6:	ce 01       	movw	r24, r28
    13a8:	45 96       	adiw	r24, 0x15	; 21
    13aa:	0e 94 6e 07 	call	0xedc	; 0xedc <_ZL4RecvPVhh>
	return UEINTX & (1<<RXSTPI);
}

static inline void ClearSetupInt()
{
	UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
    13ae:	82 ef       	ldi	r24, 0xF2	; 242
    13b0:	80 93 e8 00 	sts	0x00E8, r24

	USBSetup setup;
	Recv((u8*)&setup,8);
	ClearSetupInt();

	u8 requestType = setup.bmRequestType;
    13b4:	3d 89       	ldd	r19, Y+21	; 0x15
	if (requestType & REQUEST_DEVICETOHOST)
    13b6:	37 ff       	sbrs	r19, 7
    13b8:	05 c0       	rjmp	.+10     	; 0x13c4 <__vector_11+0x66>
volatile u8 _usbCurrentStatus = 0; // meaning of bits see usb_20.pdf, Figure 9-4. Information Returned by a GetStatus() Request to a Device
volatile u8 _usbSuspendState = 0; // copy of UDINT to check SUSPI and WAKEUPI bits

static inline void WaitIN(void)
{
	while (!(UEINTX & (1<<TXINI)))
    13ba:	80 91 e8 00 	lds	r24, 0x00E8
    13be:	80 ff       	sbrs	r24, 0
    13c0:	fc cf       	rjmp	.-8      	; 0x13ba <__vector_11+0x5c>
    13c2:	03 c0       	rjmp	.+6      	; 0x13ca <__vector_11+0x6c>
		;
}

static inline void ClearIN(void)
{
	UEINTX = ~(1<<TXINI);
    13c4:	8e ef       	ldi	r24, 0xFE	; 254
    13c6:	80 93 e8 00 	sts	0x00E8, r24
		WaitIN();
	else
		ClearIN();

    bool ok = true;
	if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
    13ca:	83 2f       	mov	r24, r19
    13cc:	80 76       	andi	r24, 0x60	; 96
    13ce:	09 f0       	breq	.+2      	; 0x13d2 <__vector_11+0x74>
    13d0:	f0 c0       	rjmp	.+480    	; 0x15b2 <__vector_11+0x254>
	{
		//	Standard Requests
		u8 r = setup.bRequest;
    13d2:	2e 89       	ldd	r18, Y+22	; 0x16
		u16 wValue = setup.wValueL | (setup.wValueH << 8);
    13d4:	4f 89       	ldd	r20, Y+23	; 0x17
    13d6:	d8 8c       	ldd	r13, Y+24	; 0x18
		if (GET_STATUS == r)
    13d8:	21 11       	cpse	r18, r1
    13da:	0c c0       	rjmp	.+24     	; 0x13f4 <__vector_11+0x96>
		{
			if (requestType == (REQUEST_DEVICETOHOST | REQUEST_STANDARD | REQUEST_DEVICE))
    13dc:	30 38       	cpi	r19, 0x80	; 128
    13de:	29 f4       	brne	.+10     	; 0x13ea <__vector_11+0x8c>
			{
				Send8(_usbCurrentStatus);
    13e0:	80 91 5d 02 	lds	r24, 0x025D
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
    13e4:	80 93 f1 00 	sts	0x00F1, r24
    13e8:	02 c0       	rjmp	.+4      	; 0x13ee <__vector_11+0x90>
    13ea:	10 92 f1 00 	sts	0x00F1, r1
    13ee:	10 92 f1 00 	sts	0x00F1, r1
    13f2:	f4 c0       	rjmp	.+488    	; 0x15dc <__vector_11+0x27e>
    bool ok = true;
	if (REQUEST_STANDARD == (requestType & REQUEST_TYPE))
	{
		//	Standard Requests
		u8 r = setup.bRequest;
		u16 wValue = setup.wValueL | (setup.wValueH << 8);
    13f4:	84 2f       	mov	r24, r20
    13f6:	90 e0       	ldi	r25, 0x00	; 0
    13f8:	9d 29       	or	r25, r13
				// see "Figure 9-6. Information Returned by a GetStatus() Request to an Endpoint" in usb_20.pdf for more information
				Send8(0);
				Send8(0);
			}
		}
		else if (CLEAR_FEATURE == r)
    13fa:	21 30       	cpi	r18, 0x01	; 1
    13fc:	49 f4       	brne	.+18     	; 0x1410 <__vector_11+0xb2>
		{
			if((requestType == (REQUEST_HOSTTODEVICE | REQUEST_STANDARD | REQUEST_DEVICE))
    13fe:	31 11       	cpse	r19, r1
    1400:	ed c0       	rjmp	.+474    	; 0x15dc <__vector_11+0x27e>
				&& (wValue == DEVICE_REMOTE_WAKEUP))
    1402:	01 97       	sbiw	r24, 0x01	; 1
    1404:	09 f0       	breq	.+2      	; 0x1408 <__vector_11+0xaa>
    1406:	ea c0       	rjmp	.+468    	; 0x15dc <__vector_11+0x27e>
			{
				_usbCurrentStatus &= ~FEATURE_REMOTE_WAKEUP_ENABLED;
    1408:	80 91 5d 02 	lds	r24, 0x025D
    140c:	8d 7f       	andi	r24, 0xFD	; 253
    140e:	0a c0       	rjmp	.+20     	; 0x1424 <__vector_11+0xc6>
			}
		}
		else if (SET_FEATURE == r)
    1410:	23 30       	cpi	r18, 0x03	; 3
    1412:	59 f4       	brne	.+22     	; 0x142a <__vector_11+0xcc>
		{
			if((requestType == (REQUEST_HOSTTODEVICE | REQUEST_STANDARD | REQUEST_DEVICE))
    1414:	31 11       	cpse	r19, r1
    1416:	e2 c0       	rjmp	.+452    	; 0x15dc <__vector_11+0x27e>
				&& (wValue == DEVICE_REMOTE_WAKEUP))
    1418:	01 97       	sbiw	r24, 0x01	; 1
    141a:	09 f0       	breq	.+2      	; 0x141e <__vector_11+0xc0>
    141c:	df c0       	rjmp	.+446    	; 0x15dc <__vector_11+0x27e>
			{
				_usbCurrentStatus |= FEATURE_REMOTE_WAKEUP_ENABLED;
    141e:	80 91 5d 02 	lds	r24, 0x025D
    1422:	82 60       	ori	r24, 0x02	; 2
    1424:	80 93 5d 02 	sts	0x025D, r24
    1428:	d9 c0       	rjmp	.+434    	; 0x15dc <__vector_11+0x27e>
			}
		}
		else if (SET_ADDRESS == r)
    142a:	25 30       	cpi	r18, 0x05	; 5
    142c:	41 f4       	brne	.+16     	; 0x143e <__vector_11+0xe0>
volatile u8 _usbCurrentStatus = 0; // meaning of bits see usb_20.pdf, Figure 9-4. Information Returned by a GetStatus() Request to a Device
volatile u8 _usbSuspendState = 0; // copy of UDINT to check SUSPI and WAKEUPI bits

static inline void WaitIN(void)
{
	while (!(UEINTX & (1<<TXINI)))
    142e:	80 91 e8 00 	lds	r24, 0x00E8
    1432:	80 ff       	sbrs	r24, 0
    1434:	fc cf       	rjmp	.-8      	; 0x142e <__vector_11+0xd0>
			}
		}
		else if (SET_ADDRESS == r)
		{
			WaitIN();
			UDADDR = setup.wValueL | (1<<ADDEN);
    1436:	40 68       	ori	r20, 0x80	; 128
    1438:	40 93 e3 00 	sts	0x00E3, r20
    143c:	cf c0       	rjmp	.+414    	; 0x15dc <__vector_11+0x27e>
		}
		else if (GET_DESCRIPTOR == r)
    143e:	26 30       	cpi	r18, 0x06	; 6
    1440:	09 f0       	breq	.+2      	; 0x1444 <__vector_11+0xe6>
    1442:	8d c0       	rjmp	.+282    	; 0x155e <__vector_11+0x200>
    1444:	eb 8c       	ldd	r14, Y+27	; 0x1b
    1446:	fc 8c       	ldd	r15, Y+28	; 0x1c
static
bool SendDescriptor(USBSetup& setup)
{
	int ret;
	u8 t = setup.wValueH;
	if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
    1448:	22 e0       	ldi	r18, 0x02	; 2
    144a:	d2 12       	cpse	r13, r18
    144c:	2c c0       	rjmp	.+88     	; 0x14a6 <__vector_11+0x148>
//	TODO
static
bool SendConfiguration(int maxlen)
{
	//	Count and measure interfaces
	InitControl(0);	
    144e:	80 e0       	ldi	r24, 0x00	; 0
    1450:	90 e0       	ldi	r25, 0x00	; 0
    1452:	0e 94 40 09 	call	0x1280	; 0x1280 <_Z11InitControli>
	u8 interfaces = SendInterfaces();
    1456:	0e 94 09 08 	call	0x1012	; 0x1012 <_ZL14SendInterfacesv>
	ConfigDescriptor config = D_CONFIG(_cmark + sizeof(ConfigDescriptor),interfaces);
    145a:	99 e0       	ldi	r25, 0x09	; 9
    145c:	8e 01       	movw	r16, r28
    145e:	0f 5f       	subi	r16, 0xFF	; 255
    1460:	1f 4f       	sbci	r17, 0xFF	; 255
    1462:	f8 01       	movw	r30, r16
    1464:	29 2f       	mov	r18, r25
    1466:	11 92       	st	Z+, r1
    1468:	2a 95       	dec	r18
    146a:	e9 f7       	brne	.-6      	; 0x1466 <__vector_11+0x108>
    146c:	99 83       	std	Y+1, r25	; 0x01
    146e:	da 82       	std	Y+2, r13	; 0x02
    1470:	91 e0       	ldi	r25, 0x01	; 1
    1472:	9e 83       	std	Y+6, r25	; 0x06
    1474:	90 ea       	ldi	r25, 0xA0	; 160
    1476:	98 87       	std	Y+8, r25	; 0x08
    1478:	9a ef       	ldi	r25, 0xFA	; 250
    147a:	99 87       	std	Y+9, r25	; 0x09
    147c:	20 91 5a 02 	lds	r18, 0x025A
    1480:	30 91 5b 02 	lds	r19, 0x025B
    1484:	27 5f       	subi	r18, 0xF7	; 247
    1486:	3f 4f       	sbci	r19, 0xFF	; 255
    1488:	3c 83       	std	Y+4, r19	; 0x04
    148a:	2b 83       	std	Y+3, r18	; 0x03
    148c:	8d 83       	std	Y+5, r24	; 0x05

	//	Now send them
	InitControl(maxlen);
    148e:	c7 01       	movw	r24, r14
    1490:	0e 94 40 09 	call	0x1280	; 0x1280 <_Z11InitControli>
	USB_SendControl(0,&config,sizeof(ConfigDescriptor));
    1494:	49 e0       	ldi	r20, 0x09	; 9
    1496:	50 e0       	ldi	r21, 0x00	; 0
    1498:	b8 01       	movw	r22, r16
    149a:	80 e0       	ldi	r24, 0x00	; 0
    149c:	0e 94 4b 09 	call	0x1296	; 0x1296 <_Z15USB_SendControlhPKvi>
	SendInterfaces();
    14a0:	0e 94 09 08 	call	0x1012	; 0x1012 <_ZL14SendInterfacesv>
    14a4:	9b c0       	rjmp	.+310    	; 0x15dc <__vector_11+0x27e>
	int ret;
	u8 t = setup.wValueH;
	if (USB_CONFIGURATION_DESCRIPTOR_TYPE == t)
		return SendConfiguration(setup.wLength);

	InitControl(setup.wLength);
    14a6:	c7 01       	movw	r24, r14
    14a8:	0e 94 40 09 	call	0x1280	; 0x1280 <_Z11InitControli>
#ifdef PLUGGABLE_USB_ENABLED
	ret = PluggableUSB().getDescriptor(setup);
    14ac:	0e 94 61 0d 	call	0x1ac2	; 0x1ac2 <_Z12PluggableUSBv>
    14b0:	be 01       	movw	r22, r28
    14b2:	6b 5e       	subi	r22, 0xEB	; 235
    14b4:	7f 4f       	sbci	r23, 0xFF	; 255
    14b6:	0e 94 03 0d 	call	0x1a06	; 0x1a06 <_ZN13PluggableUSB_13getDescriptorER8USBSetup>
	if (ret != 0) {
    14ba:	00 97       	sbiw	r24, 0x00	; 0
    14bc:	19 f0       	breq	.+6      	; 0x14c4 <__vector_11+0x166>
		return (ret > 0 ? true : false);
    14be:	0c f0       	brlt	.+2      	; 0x14c2 <__vector_11+0x164>
    14c0:	8d c0       	rjmp	.+282    	; 0x15dc <__vector_11+0x27e>
    14c2:	90 c0       	rjmp	.+288    	; 0x15e4 <__vector_11+0x286>
	}
#endif

	const u8* desc_addr = 0;
	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
    14c4:	81 e0       	ldi	r24, 0x01	; 1
    14c6:	d8 12       	cpse	r13, r24
    14c8:	0e c0       	rjmp	.+28     	; 0x14e6 <__vector_11+0x188>
	{
		if (setup.wLength == 8)
    14ca:	8b 8d       	ldd	r24, Y+27	; 0x1b
    14cc:	9c 8d       	ldd	r25, Y+28	; 0x1c
    14ce:	08 97       	sbiw	r24, 0x08	; 8
    14d0:	19 f4       	brne	.+6      	; 0x14d8 <__vector_11+0x17a>
			_cdcComposite = 1;
    14d2:	81 e0       	ldi	r24, 0x01	; 1
    14d4:	80 93 57 02 	sts	0x0257, r24
		desc_addr = _cdcComposite ?  (const u8*)&USB_DeviceDescriptorB : (const u8*)&USB_DeviceDescriptor;
    14d8:	80 91 57 02 	lds	r24, 0x0257
    14dc:	81 11       	cpse	r24, r1
    14de:	2f c0       	rjmp	.+94     	; 0x153e <__vector_11+0x1e0>
    14e0:	67 e9       	ldi	r22, 0x97	; 151
    14e2:	71 e0       	ldi	r23, 0x01	; 1
    14e4:	31 c0       	rjmp	.+98     	; 0x1548 <__vector_11+0x1ea>
	}
	else if (USB_STRING_DESCRIPTOR_TYPE == t)
    14e6:	e3 e0       	ldi	r30, 0x03	; 3
    14e8:	de 12       	cpse	r13, r30
    14ea:	7c c0       	rjmp	.+248    	; 0x15e4 <__vector_11+0x286>
	{
		if (setup.wValueL == 0) {
    14ec:	8f 89       	ldd	r24, Y+23	; 0x17
    14ee:	88 23       	and	r24, r24
    14f0:	49 f1       	breq	.+82     	; 0x1544 <__vector_11+0x1e6>
			desc_addr = (const u8*)&STRING_LANGUAGE;
		}
		else if (setup.wValueL == IPRODUCT) {
    14f2:	82 30       	cpi	r24, 0x02	; 2
    14f4:	29 f4       	brne	.+10     	; 0x1500 <__vector_11+0x1a2>
			return USB_SendStringDescriptor(STRING_PRODUCT, strlen(USB_PRODUCT), TRANSFER_PGM);
    14f6:	40 e8       	ldi	r20, 0x80	; 128
    14f8:	60 e1       	ldi	r22, 0x10	; 16
    14fa:	85 eb       	ldi	r24, 0xB5	; 181
    14fc:	91 e0       	ldi	r25, 0x01	; 1
    14fe:	1c c0       	rjmp	.+56     	; 0x1538 <__vector_11+0x1da>
		}
		else if (setup.wValueL == IMANUFACTURER) {
    1500:	81 30       	cpi	r24, 0x01	; 1
    1502:	29 f4       	brne	.+10     	; 0x150e <__vector_11+0x1b0>
			return USB_SendStringDescriptor(STRING_MANUFACTURER, strlen(USB_MANUFACTURER), TRANSFER_PGM);
    1504:	40 e8       	ldi	r20, 0x80	; 128
    1506:	6b e0       	ldi	r22, 0x0B	; 11
    1508:	89 ea       	ldi	r24, 0xA9	; 169
    150a:	91 e0       	ldi	r25, 0x01	; 1
    150c:	15 c0       	rjmp	.+42     	; 0x1538 <__vector_11+0x1da>
		}
		else if (setup.wValueL == ISERIAL) {
    150e:	83 30       	cpi	r24, 0x03	; 3
    1510:	09 f0       	breq	.+2      	; 0x1514 <__vector_11+0x1b6>
    1512:	68 c0       	rjmp	.+208    	; 0x15e4 <__vector_11+0x286>
#ifdef PLUGGABLE_USB_ENABLED
			char name[ISERIAL_MAX_LEN];
			PluggableUSB().getShortName(name);
    1514:	0e 94 61 0d 	call	0x1ac2	; 0x1ac2 <_Z12PluggableUSBv>
    1518:	8e 01       	movw	r16, r28
    151a:	0f 5f       	subi	r16, 0xFF	; 255
    151c:	1f 4f       	sbci	r17, 0xFF	; 255
    151e:	b8 01       	movw	r22, r16
    1520:	0e 94 22 0d 	call	0x1a44	; 0x1a44 <_ZN13PluggableUSB_12getShortNameEPc>
			return USB_SendStringDescriptor((uint8_t*)name, strlen(name), 0);
    1524:	f8 01       	movw	r30, r16
    1526:	01 90       	ld	r0, Z+
    1528:	00 20       	and	r0, r0
    152a:	e9 f7       	brne	.-6      	; 0x1526 <__vector_11+0x1c8>
    152c:	31 97       	sbiw	r30, 0x01	; 1
    152e:	e0 1b       	sub	r30, r16
    1530:	f1 0b       	sbc	r31, r17
    1532:	40 e0       	ldi	r20, 0x00	; 0
    1534:	6e 2f       	mov	r22, r30
    1536:	c8 01       	movw	r24, r16
    1538:	0e 94 a9 07 	call	0xf52	; 0xf52 <_ZL24USB_SendStringDescriptorPKhhh>
    153c:	4d c0       	rjmp	.+154    	; 0x15d8 <__vector_11+0x27a>
	const u8* desc_addr = 0;
	if (USB_DEVICE_DESCRIPTOR_TYPE == t)
	{
		if (setup.wLength == 8)
			_cdcComposite = 1;
		desc_addr = _cdcComposite ?  (const u8*)&USB_DeviceDescriptorB : (const u8*)&USB_DeviceDescriptor;
    153e:	65 e8       	ldi	r22, 0x85	; 133
    1540:	71 e0       	ldi	r23, 0x01	; 1
    1542:	02 c0       	rjmp	.+4      	; 0x1548 <__vector_11+0x1ea>
	}
	else if (USB_STRING_DESCRIPTOR_TYPE == t)
	{
		if (setup.wValueL == 0) {
			desc_addr = (const u8*)&STRING_LANGUAGE;
    1544:	66 ec       	ldi	r22, 0xC6	; 198
    1546:	71 e0       	ldi	r23, 0x01	; 1
		}
		else
			return false;
	}

	if (desc_addr == 0)
    1548:	61 15       	cp	r22, r1
    154a:	71 05       	cpc	r23, r1
    154c:	09 f4       	brne	.+2      	; 0x1550 <__vector_11+0x1f2>
    154e:	4a c0       	rjmp	.+148    	; 0x15e4 <__vector_11+0x286>
		return false;
	u8 desc_length = pgm_read_byte(desc_addr);
    1550:	fb 01       	movw	r30, r22
    1552:	44 91       	lpm	r20, Z

	USB_SendControl(TRANSFER_PGM,desc_addr,desc_length);
    1554:	50 e0       	ldi	r21, 0x00	; 0
    1556:	80 e8       	ldi	r24, 0x80	; 128
    1558:	0e 94 4b 09 	call	0x1296	; 0x1296 <_Z15USB_SendControlhPKvi>
    155c:	3f c0       	rjmp	.+126    	; 0x15dc <__vector_11+0x27e>
		}
		else if (GET_DESCRIPTOR == r)
		{
			ok = SendDescriptor(setup);
		}
		else if (SET_DESCRIPTOR == r)
    155e:	27 30       	cpi	r18, 0x07	; 7
    1560:	09 f4       	brne	.+2      	; 0x1564 <__vector_11+0x206>
    1562:	40 c0       	rjmp	.+128    	; 0x15e4 <__vector_11+0x286>
		{
			ok = false;
		}
		else if (GET_CONFIGURATION == r)
    1564:	28 30       	cpi	r18, 0x08	; 8
    1566:	21 f4       	brne	.+8      	; 0x1570 <__vector_11+0x212>
	return UEDATX;	
}

static inline void Send8(u8 d)
{
	UEDATX = d;
    1568:	81 e0       	ldi	r24, 0x01	; 1
    156a:	80 93 f1 00 	sts	0x00F1, r24
    156e:	36 c0       	rjmp	.+108    	; 0x15dc <__vector_11+0x27e>
		}
		else if (GET_CONFIGURATION == r)
		{
			Send8(1);
		}
		else if (SET_CONFIGURATION == r)
    1570:	29 30       	cpi	r18, 0x09	; 9
    1572:	a1 f5       	brne	.+104    	; 0x15dc <__vector_11+0x27e>
		{
			if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
    1574:	33 70       	andi	r19, 0x03	; 3
    1576:	b1 f5       	brne	.+108    	; 0x15e4 <__vector_11+0x286>
    1578:	e7 e9       	ldi	r30, 0x97	; 151
    157a:	f1 e0       	ldi	r31, 0x01	; 1
    157c:	81 e0       	ldi	r24, 0x01	; 1
void InitEndpoints()
{
	for (u8 i = 1; i < sizeof(_initEndpoints) && _initEndpoints[i] != 0; i++)
	{
		UENUM = i;
		UECONX = (1<<EPEN);
    157e:	21 e0       	ldi	r18, 0x01	; 1
		UECFG0X = _initEndpoints[i];
#if USB_EP_SIZE == 16
		UECFG1X = EP_SINGLE_16;
#elif USB_EP_SIZE == 64
		UECFG1X = EP_DOUBLE_64;
    1580:	36 e3       	ldi	r19, 0x36	; 54
}

static
void InitEndpoints()
{
	for (u8 i = 1; i < sizeof(_initEndpoints) && _initEndpoints[i] != 0; i++)
    1582:	90 81       	ld	r25, Z
    1584:	99 23       	and	r25, r25
    1586:	61 f0       	breq	.+24     	; 0x15a0 <__vector_11+0x242>
	{
		UENUM = i;
    1588:	80 93 e9 00 	sts	0x00E9, r24
		UECONX = (1<<EPEN);
    158c:	20 93 eb 00 	sts	0x00EB, r18
		UECFG0X = _initEndpoints[i];
    1590:	91 91       	ld	r25, Z+
    1592:	90 93 ec 00 	sts	0x00EC, r25
#if USB_EP_SIZE == 16
		UECFG1X = EP_SINGLE_16;
#elif USB_EP_SIZE == 64
		UECFG1X = EP_DOUBLE_64;
    1596:	30 93 ed 00 	sts	0x00ED, r19
}

static
void InitEndpoints()
{
	for (u8 i = 1; i < sizeof(_initEndpoints) && _initEndpoints[i] != 0; i++)
    159a:	8f 5f       	subi	r24, 0xFF	; 255
    159c:	87 30       	cpi	r24, 0x07	; 7
    159e:	89 f7       	brne	.-30     	; 0x1582 <__vector_11+0x224>
		UECFG1X = EP_DOUBLE_64;
#else
#error Unsupported value for USB_EP_SIZE
#endif
	}
	UERST = 0x7E;	// And reset them
    15a0:	8e e7       	ldi	r24, 0x7E	; 126
    15a2:	80 93 ea 00 	sts	0x00EA, r24
	UERST = 0;
    15a6:	10 92 ea 00 	sts	0x00EA, r1
		else if (SET_CONFIGURATION == r)
		{
			if (REQUEST_DEVICE == (requestType & REQUEST_RECIPIENT))
			{
				InitEndpoints();
				_usbConfiguration = setup.wValueL;
    15aa:	8f 89       	ldd	r24, Y+23	; 0x17
    15ac:	80 93 5e 02 	sts	0x025E, r24
    15b0:	15 c0       	rjmp	.+42     	; 0x15dc <__vector_11+0x27e>
		{
		}
	}
	else
	{
		InitControl(setup.wLength);		//	Max length of transfer
    15b2:	8b 8d       	ldd	r24, Y+27	; 0x1b
    15b4:	9c 8d       	ldd	r25, Y+28	; 0x1c
    15b6:	0e 94 40 09 	call	0x1280	; 0x1280 <_Z11InitControli>
static
bool ClassInterfaceRequest(USBSetup& setup)
{
	u8 i = setup.wIndex;

	if (CDC_ACM_INTERFACE == i)
    15ba:	89 8d       	ldd	r24, Y+25	; 0x19
    15bc:	81 11       	cpse	r24, r1
    15be:	05 c0       	rjmp	.+10     	; 0x15ca <__vector_11+0x26c>
		return CDC_Setup(setup);
    15c0:	ce 01       	movw	r24, r28
    15c2:	45 96       	adiw	r24, 0x15	; 21
    15c4:	0e 94 f5 03 	call	0x7ea	; 0x7ea <_Z9CDC_SetupR8USBSetup>
    15c8:	07 c0       	rjmp	.+14     	; 0x15d8 <__vector_11+0x27a>

#ifdef PLUGGABLE_USB_ENABLED
	return PluggableUSB().setup(setup);
    15ca:	0e 94 61 0d 	call	0x1ac2	; 0x1ac2 <_Z12PluggableUSBv>
    15ce:	be 01       	movw	r22, r28
    15d0:	6b 5e       	subi	r22, 0xEB	; 235
    15d2:	7f 4f       	sbci	r23, 0xFF	; 255
    15d4:	0e 94 41 0d 	call	0x1a82	; 0x1a82 <_ZN13PluggableUSB_5setupER8USBSetup>
	{
		InitControl(setup.wLength);		//	Max length of transfer
		ok = ClassInterfaceRequest(setup);
	}

	if (ok)
    15d8:	88 23       	and	r24, r24
    15da:	21 f0       	breq	.+8      	; 0x15e4 <__vector_11+0x286>
		;
}

static inline void ClearIN(void)
{
	UEINTX = ~(1<<TXINI);
    15dc:	8e ef       	ldi	r24, 0xFE	; 254
    15de:	80 93 e8 00 	sts	0x00E8, r24
    15e2:	03 c0       	rjmp	.+6      	; 0x15ea <__vector_11+0x28c>
	UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
}

static inline void Stall()
{
	UECONX = (1<<STALLRQ) | (1<<EPEN);
    15e4:	81 e2       	ldi	r24, 0x21	; 33
    15e6:	80 93 eb 00 	sts	0x00EB, r24
		ClearIN();
	else
	{
		Stall();
	}
}
    15ea:	6c 96       	adiw	r28, 0x1c	; 28
    15ec:	0f b6       	in	r0, 0x3f	; 63
    15ee:	f8 94       	cli
    15f0:	de bf       	out	0x3e, r29	; 62
    15f2:	0f be       	out	0x3f, r0	; 63
    15f4:	cd bf       	out	0x3d, r28	; 61
    15f6:	df 91       	pop	r29
    15f8:	cf 91       	pop	r28
    15fa:	ff 91       	pop	r31
    15fc:	ef 91       	pop	r30
    15fe:	bf 91       	pop	r27
    1600:	af 91       	pop	r26
    1602:	9f 91       	pop	r25
    1604:	8f 91       	pop	r24
    1606:	7f 91       	pop	r23
    1608:	6f 91       	pop	r22
    160a:	5f 91       	pop	r21
    160c:	4f 91       	pop	r20
    160e:	3f 91       	pop	r19
    1610:	2f 91       	pop	r18
    1612:	1f 91       	pop	r17
    1614:	0f 91       	pop	r16
    1616:	ff 90       	pop	r15
    1618:	ef 90       	pop	r14
    161a:	df 90       	pop	r13
    161c:	0f 90       	pop	r0
    161e:	0f be       	out	0x3f, r0	; 63
    1620:	0f 90       	pop	r0
    1622:	1f 90       	pop	r1
    1624:	18 95       	reti

00001626 <_Z9USB_Flushh>:
	UEDATX = d;
}

static inline void SetEP(u8 ep)
{
	UENUM = ep;
    1626:	80 93 e9 00 	sts	0x00E9, r24
}

static inline u8 FifoByteCount()
{
	return UEBCLX;
    162a:	80 91 f2 00 	lds	r24, 0x00F2
}

void USB_Flush(u8 ep)
{
	SetEP(ep);
	if (FifoByteCount())
    162e:	88 23       	and	r24, r24
    1630:	19 f0       	breq	.+6      	; 0x1638 <_Z9USB_Flushh+0x12>
	UEINTX = 0x6B;	// FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static inline void ReleaseTX()
{
	UEINTX = 0x3A;	// FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
    1632:	8a e3       	ldi	r24, 0x3A	; 58
    1634:	80 93 e8 00 	sts	0x00E8, r24
    1638:	08 95       	ret

0000163a <__vector_10>:
#endif
}

//	General interrupt
ISR(USB_GEN_vect)
{
    163a:	1f 92       	push	r1
    163c:	0f 92       	push	r0
    163e:	0f b6       	in	r0, 0x3f	; 63
    1640:	0f 92       	push	r0
    1642:	11 24       	eor	r1, r1
    1644:	2f 93       	push	r18
    1646:	3f 93       	push	r19
    1648:	4f 93       	push	r20
    164a:	5f 93       	push	r21
    164c:	6f 93       	push	r22
    164e:	7f 93       	push	r23
    1650:	8f 93       	push	r24
    1652:	9f 93       	push	r25
    1654:	af 93       	push	r26
    1656:	bf 93       	push	r27
    1658:	cf 93       	push	r28
    165a:	ef 93       	push	r30
    165c:	ff 93       	push	r31
	u8 udint = UDINT;
    165e:	c0 91 e1 00 	lds	r28, 0x00E1
	UDINT = UDINT &= ~((1<<EORSTI) | (1<<SOFI)); // clear the IRQ flags for the IRQs which are handled here, except WAKEUPI and SUSPI (see below)
    1662:	80 91 e1 00 	lds	r24, 0x00E1
    1666:	83 7f       	andi	r24, 0xF3	; 243
    1668:	80 93 e1 00 	sts	0x00E1, r24
    166c:	80 93 e1 00 	sts	0x00E1, r24

	//	End of Reset
	if (udint & (1<<EORSTI))
    1670:	c3 ff       	sbrs	r28, 3
    1672:	0f c0       	rjmp	.+30     	; 0x1692 <__vector_10+0x58>
#define EP_SINGLE_16 0x12

static
void InitEP(u8 index, u8 type, u8 size)
{
	UENUM = index;
    1674:	10 92 e9 00 	sts	0x00E9, r1
	UECONX = (1<<EPEN);
    1678:	81 e0       	ldi	r24, 0x01	; 1
    167a:	80 93 eb 00 	sts	0x00EB, r24
	UECFG0X = type;
    167e:	10 92 ec 00 	sts	0x00EC, r1
	UECFG1X = size;
    1682:	82 e3       	ldi	r24, 0x32	; 50
    1684:	80 93 ed 00 	sts	0x00ED, r24

	//	End of Reset
	if (udint & (1<<EORSTI))
	{
		InitEP(0,EP_TYPE_CONTROL,EP_SINGLE_64);	// init ep0
		_usbConfiguration = 0;			// not configured yet
    1688:	10 92 5e 02 	sts	0x025E, r1
		UEIENX = 1 << RXSTPE;			// Enable interrupts for ep0
    168c:	88 e0       	ldi	r24, 0x08	; 8
    168e:	80 93 f0 00 	sts	0x00F0, r24
	}

	//	Start of Frame - happens every millisecond so we use it for TX and RX LED one-shot timing, too
	if (udint & (1<<SOFI))
    1692:	c2 ff       	sbrs	r28, 2
    1694:	19 c0       	rjmp	.+50     	; 0x16c8 <__vector_10+0x8e>
	{
		USB_Flush(CDC_TX);				// Send a tx frame if found
    1696:	83 e0       	ldi	r24, 0x03	; 3
    1698:	0e 94 13 0b 	call	0x1626	; 0x1626 <_Z9USB_Flushh>
		
		// check whether the one-shot period has elapsed.  if so, turn off the LED
		if (TxLEDPulse && !(--TxLEDPulse))
    169c:	80 91 60 02 	lds	r24, 0x0260
    16a0:	88 23       	and	r24, r24
    16a2:	39 f0       	breq	.+14     	; 0x16b2 <__vector_10+0x78>
    16a4:	80 91 60 02 	lds	r24, 0x0260
    16a8:	81 50       	subi	r24, 0x01	; 1
    16aa:	80 93 60 02 	sts	0x0260, r24
    16ae:	88 23       	and	r24, r24
    16b0:	89 f1       	breq	.+98     	; 0x1714 <__vector_10+0xda>
			TXLED0;
		if (RxLEDPulse && !(--RxLEDPulse))
    16b2:	80 91 5f 02 	lds	r24, 0x025F
    16b6:	88 23       	and	r24, r24
    16b8:	39 f0       	breq	.+14     	; 0x16c8 <__vector_10+0x8e>
    16ba:	80 91 5f 02 	lds	r24, 0x025F
    16be:	81 50       	subi	r24, 0x01	; 1
    16c0:	80 93 5f 02 	sts	0x025F, r24
    16c4:	88 23       	and	r24, r24
    16c6:	41 f1       	breq	.+80     	; 0x1718 <__vector_10+0xde>
	}

	// the WAKEUPI interrupt is triggered as soon as there are non-idle patterns on the data
	// lines. Thus, the WAKEUPI interrupt can occur even if the controller is not in the "suspend" mode.
	// Therefore the we enable it only when USB is suspended
	if (udint & (1<<WAKEUPI))
    16c8:	c4 ff       	sbrs	r28, 4
    16ca:	10 c0       	rjmp	.+32     	; 0x16ec <__vector_10+0xb2>
	{
		UDIEN = (UDIEN & ~(1<<WAKEUPE)) | (1<<SUSPE); // Disable interrupts for WAKEUP and enable interrupts for SUSPEND
    16cc:	80 91 e2 00 	lds	r24, 0x00E2
    16d0:	8e 7e       	andi	r24, 0xEE	; 238
    16d2:	81 60       	ori	r24, 0x01	; 1
    16d4:	80 93 e2 00 	sts	0x00E2, r24

		//TODO
		// WAKEUPI shall be cleared by software (USB clock inputs must be enabled before).
		//USB_ClockEnable();
		UDINT &= ~(1<<WAKEUPI);
    16d8:	80 91 e1 00 	lds	r24, 0x00E1
    16dc:	8f 7e       	andi	r24, 0xEF	; 239
    16de:	80 93 e1 00 	sts	0x00E1, r24
		_usbSuspendState = (_usbSuspendState & ~(1<<SUSPI)) | (1<<WAKEUPI);
    16e2:	80 91 5c 02 	lds	r24, 0x025C
    16e6:	8e 7e       	andi	r24, 0xEE	; 238
    16e8:	80 61       	ori	r24, 0x10	; 16
    16ea:	11 c0       	rjmp	.+34     	; 0x170e <__vector_10+0xd4>
	}
	else if (udint & (1<<SUSPI)) // only one of the WAKEUPI / SUSPI bits can be active at time
    16ec:	c0 ff       	sbrs	r28, 0
    16ee:	16 c0       	rjmp	.+44     	; 0x171c <__vector_10+0xe2>
	{
		UDIEN = (UDIEN & ~(1<<SUSPE)) | (1<<WAKEUPE); // Disable interrupts for SUSPEND and enable interrupts for WAKEUP
    16f0:	80 91 e2 00 	lds	r24, 0x00E2
    16f4:	8e 7e       	andi	r24, 0xEE	; 238
    16f6:	80 61       	ori	r24, 0x10	; 16
    16f8:	80 93 e2 00 	sts	0x00E2, r24

		//TODO
		//USB_ClockDisable();

		UDINT &= ~((1<<WAKEUPI) | (1<<SUSPI)); // clear any already pending WAKEUP IRQs and the SUSPI request
    16fc:	80 91 e1 00 	lds	r24, 0x00E1
    1700:	8e 7e       	andi	r24, 0xEE	; 238
    1702:	80 93 e1 00 	sts	0x00E1, r24
		_usbSuspendState = (_usbSuspendState & ~(1<<WAKEUPI)) | (1<<SUSPI);
    1706:	80 91 5c 02 	lds	r24, 0x025C
    170a:	8e 7e       	andi	r24, 0xEE	; 238
    170c:	81 60       	ori	r24, 0x01	; 1
    170e:	80 93 5c 02 	sts	0x025C, r24
    1712:	04 c0       	rjmp	.+8      	; 0x171c <__vector_10+0xe2>
	{
		USB_Flush(CDC_TX);				// Send a tx frame if found
		
		// check whether the one-shot period has elapsed.  if so, turn off the LED
		if (TxLEDPulse && !(--TxLEDPulse))
			TXLED0;
    1714:	5d 9a       	sbi	0x0b, 5	; 11
    1716:	cd cf       	rjmp	.-102    	; 0x16b2 <__vector_10+0x78>
		if (RxLEDPulse && !(--RxLEDPulse))
			RXLED0;
    1718:	28 9a       	sbi	0x05, 0	; 5
    171a:	d6 cf       	rjmp	.-84     	; 0x16c8 <__vector_10+0x8e>
		//USB_ClockDisable();

		UDINT &= ~((1<<WAKEUPI) | (1<<SUSPI)); // clear any already pending WAKEUP IRQs and the SUSPI request
		_usbSuspendState = (_usbSuspendState & ~(1<<WAKEUPI)) | (1<<SUSPI);
	}
}
    171c:	ff 91       	pop	r31
    171e:	ef 91       	pop	r30
    1720:	cf 91       	pop	r28
    1722:	bf 91       	pop	r27
    1724:	af 91       	pop	r26
    1726:	9f 91       	pop	r25
    1728:	8f 91       	pop	r24
    172a:	7f 91       	pop	r23
    172c:	6f 91       	pop	r22
    172e:	5f 91       	pop	r21
    1730:	4f 91       	pop	r20
    1732:	3f 91       	pop	r19
    1734:	2f 91       	pop	r18
    1736:	0f 90       	pop	r0
    1738:	0f be       	out	0x3f, r0	; 63
    173a:	0f 90       	pop	r0
    173c:	1f 90       	pop	r1
    173e:	18 95       	reti

00001740 <_ZN10USBDevice_6attachEv>:
{
}

void USBDevice_::attach()
{
	_usbConfiguration = 0;
    1740:	10 92 5e 02 	sts	0x025E, r1
	_usbCurrentStatus = 0;
    1744:	10 92 5d 02 	sts	0x025D, r1
	_usbSuspendState = 0;
    1748:	10 92 5c 02 	sts	0x025C, r1
	USB_ClockEnable();
    174c:	0e 94 e6 07 	call	0xfcc	; 0xfcc <_ZL15USB_ClockEnablev>

	UDINT &= ~((1<<WAKEUPI) | (1<<SUSPI)); // clear already pending WAKEUP / SUSPEND requests
    1750:	80 91 e1 00 	lds	r24, 0x00E1
    1754:	8e 7e       	andi	r24, 0xEE	; 238
    1756:	80 93 e1 00 	sts	0x00E1, r24
	UDIEN = (1<<EORSTE) | (1<<SOFE) | (1<<SUSPE);	// Enable interrupts for EOR (End of Reset), SOF (start of frame) and SUSPEND
    175a:	8d e0       	ldi	r24, 0x0D	; 13
    175c:	80 93 e2 00 	sts	0x00E2, r24
	
	TX_RX_LED_INIT;
    1760:	55 9a       	sbi	0x0a, 5	; 10
    1762:	20 9a       	sbi	0x04, 0	; 4

#if MAGIC_KEY_POS != (RAMEND-1)
	if (pgm_read_word(FLASHEND - 1) == NEW_LUFA_SIGNATURE) {
    1764:	ee ef       	ldi	r30, 0xFE	; 254
    1766:	ff e7       	ldi	r31, 0x7F	; 127
    1768:	85 91       	lpm	r24, Z+
    176a:	94 91       	lpm	r25, Z
    176c:	8b 3f       	cpi	r24, 0xFB	; 251
    176e:	9c 4d       	sbci	r25, 0xDC	; 220
    1770:	19 f4       	brne	.+6      	; 0x1778 <_ZN10USBDevice_6attachEv+0x38>
		_updatedLUFAbootloader = true;
    1772:	81 e0       	ldi	r24, 0x01	; 1
    1774:	80 93 55 02 	sts	0x0255, r24
    1778:	08 95       	ret

0000177a <initVariant>:
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }
    177a:	08 95       	ret

0000177c <main>:
void setupUSB() __attribute__((weak));
void setupUSB() { }

int main(void)
{
	init();
    177c:	0e 94 84 0c 	call	0x1908	; 0x1908 <init>

	initVariant();
    1780:	0e 94 bd 0b 	call	0x177a	; 0x177a <initVariant>

#if defined(USBCON)
	USBDevice.attach();
    1784:	86 e5       	ldi	r24, 0x56	; 86
    1786:	92 e0       	ldi	r25, 0x02	; 2
    1788:	0e 94 a0 0b 	call	0x1740	; 0x1740 <_ZN10USBDevice_6attachEv>
#endif
	
	setup();
    178c:	0e 94 2a 01 	call	0x254	; 0x254 <setup>
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
    1790:	c0 e0       	ldi	r28, 0x00	; 0
    1792:	d0 e0       	ldi	r29, 0x00	; 0
#endif
	
	setup();
    
	for (;;) {
		loop();
    1794:	0e 94 0d 02 	call	0x41a	; 0x41a <loop>
		if (serialEventRun) serialEventRun();
    1798:	20 97       	sbiw	r28, 0x00	; 0
    179a:	e1 f3       	breq	.-8      	; 0x1794 <main+0x18>
    179c:	0e 94 00 00 	call	0	; 0x0 <__vectors>
    17a0:	f9 cf       	rjmp	.-14     	; 0x1794 <main+0x18>

000017a2 <__vector_23>:
#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
    17a2:	1f 92       	push	r1
    17a4:	0f 92       	push	r0
    17a6:	0f b6       	in	r0, 0x3f	; 63
    17a8:	0f 92       	push	r0
    17aa:	11 24       	eor	r1, r1
    17ac:	2f 93       	push	r18
    17ae:	3f 93       	push	r19
    17b0:	8f 93       	push	r24
    17b2:	9f 93       	push	r25
    17b4:	af 93       	push	r26
    17b6:	bf 93       	push	r27
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
    17b8:	80 91 62 02 	lds	r24, 0x0262
    17bc:	90 91 63 02 	lds	r25, 0x0263
    17c0:	a0 91 64 02 	lds	r26, 0x0264
    17c4:	b0 91 65 02 	lds	r27, 0x0265
	unsigned char f = timer0_fract;
    17c8:	30 91 61 02 	lds	r19, 0x0261

	m += MILLIS_INC;
	f += FRACT_INC;
    17cc:	23 e0       	ldi	r18, 0x03	; 3
    17ce:	23 0f       	add	r18, r19
	if (f >= FRACT_MAX) {
    17d0:	2d 37       	cpi	r18, 0x7D	; 125
    17d2:	20 f4       	brcc	.+8      	; 0x17dc <__vector_23+0x3a>
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
    17d4:	01 96       	adiw	r24, 0x01	; 1
    17d6:	a1 1d       	adc	r26, r1
    17d8:	b1 1d       	adc	r27, r1
    17da:	05 c0       	rjmp	.+10     	; 0x17e6 <__vector_23+0x44>
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
    17dc:	26 e8       	ldi	r18, 0x86	; 134
    17de:	23 0f       	add	r18, r19
		m += 1;
    17e0:	02 96       	adiw	r24, 0x02	; 2
    17e2:	a1 1d       	adc	r26, r1
    17e4:	b1 1d       	adc	r27, r1
	}

	timer0_fract = f;
    17e6:	20 93 61 02 	sts	0x0261, r18
	timer0_millis = m;
    17ea:	80 93 62 02 	sts	0x0262, r24
    17ee:	90 93 63 02 	sts	0x0263, r25
    17f2:	a0 93 64 02 	sts	0x0264, r26
    17f6:	b0 93 65 02 	sts	0x0265, r27
	timer0_overflow_count++;
    17fa:	80 91 66 02 	lds	r24, 0x0266
    17fe:	90 91 67 02 	lds	r25, 0x0267
    1802:	a0 91 68 02 	lds	r26, 0x0268
    1806:	b0 91 69 02 	lds	r27, 0x0269
    180a:	01 96       	adiw	r24, 0x01	; 1
    180c:	a1 1d       	adc	r26, r1
    180e:	b1 1d       	adc	r27, r1
    1810:	80 93 66 02 	sts	0x0266, r24
    1814:	90 93 67 02 	sts	0x0267, r25
    1818:	a0 93 68 02 	sts	0x0268, r26
    181c:	b0 93 69 02 	sts	0x0269, r27
}
    1820:	bf 91       	pop	r27
    1822:	af 91       	pop	r26
    1824:	9f 91       	pop	r25
    1826:	8f 91       	pop	r24
    1828:	3f 91       	pop	r19
    182a:	2f 91       	pop	r18
    182c:	0f 90       	pop	r0
    182e:	0f be       	out	0x3f, r0	; 63
    1830:	0f 90       	pop	r0
    1832:	1f 90       	pop	r1
    1834:	18 95       	reti

00001836 <millis>:

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;
    1836:	2f b7       	in	r18, 0x3f	; 63

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
    1838:	f8 94       	cli
	m = timer0_millis;
    183a:	60 91 62 02 	lds	r22, 0x0262
    183e:	70 91 63 02 	lds	r23, 0x0263
    1842:	80 91 64 02 	lds	r24, 0x0264
    1846:	90 91 65 02 	lds	r25, 0x0265
	SREG = oldSREG;
    184a:	2f bf       	out	0x3f, r18	; 63

	return m;
}
    184c:	08 95       	ret

0000184e <micros>:

unsigned long micros() {
	unsigned long m;
	uint8_t oldSREG = SREG, t;
    184e:	3f b7       	in	r19, 0x3f	; 63
	
	cli();
    1850:	f8 94       	cli
	m = timer0_overflow_count;
    1852:	80 91 66 02 	lds	r24, 0x0266
    1856:	90 91 67 02 	lds	r25, 0x0267
    185a:	a0 91 68 02 	lds	r26, 0x0268
    185e:	b0 91 69 02 	lds	r27, 0x0269
#if defined(TCNT0)
	t = TCNT0;
    1862:	26 b5       	in	r18, 0x26	; 38
#else
	#error TIMER 0 not defined
#endif

#ifdef TIFR0
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
    1864:	a8 9b       	sbis	0x15, 0	; 21
    1866:	05 c0       	rjmp	.+10     	; 0x1872 <micros+0x24>
    1868:	2f 3f       	cpi	r18, 0xFF	; 255
    186a:	19 f0       	breq	.+6      	; 0x1872 <micros+0x24>
		m++;
    186c:	01 96       	adiw	r24, 0x01	; 1
    186e:	a1 1d       	adc	r26, r1
    1870:	b1 1d       	adc	r27, r1
#else
	if ((TIFR & _BV(TOV0)) && (t < 255))
		m++;
#endif

	SREG = oldSREG;
    1872:	3f bf       	out	0x3f, r19	; 63
	
	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
    1874:	66 27       	eor	r22, r22
    1876:	78 2f       	mov	r23, r24
    1878:	89 2f       	mov	r24, r25
    187a:	9a 2f       	mov	r25, r26
    187c:	62 0f       	add	r22, r18
    187e:	71 1d       	adc	r23, r1
    1880:	81 1d       	adc	r24, r1
    1882:	91 1d       	adc	r25, r1
    1884:	42 e0       	ldi	r20, 0x02	; 2
    1886:	66 0f       	add	r22, r22
    1888:	77 1f       	adc	r23, r23
    188a:	88 1f       	adc	r24, r24
    188c:	99 1f       	adc	r25, r25
    188e:	4a 95       	dec	r20
    1890:	d1 f7       	brne	.-12     	; 0x1886 <micros+0x38>
}
    1892:	08 95       	ret

00001894 <delay>:

void delay(unsigned long ms)
{
    1894:	8f 92       	push	r8
    1896:	9f 92       	push	r9
    1898:	af 92       	push	r10
    189a:	bf 92       	push	r11
    189c:	cf 92       	push	r12
    189e:	df 92       	push	r13
    18a0:	ef 92       	push	r14
    18a2:	ff 92       	push	r15
    18a4:	6b 01       	movw	r12, r22
    18a6:	7c 01       	movw	r14, r24
	uint32_t start = micros();
    18a8:	0e 94 27 0c 	call	0x184e	; 0x184e <micros>
    18ac:	4b 01       	movw	r8, r22
    18ae:	5c 01       	movw	r10, r24

	while (ms > 0) {
    18b0:	c1 14       	cp	r12, r1
    18b2:	d1 04       	cpc	r13, r1
    18b4:	e1 04       	cpc	r14, r1
    18b6:	f1 04       	cpc	r15, r1
    18b8:	f1 f0       	breq	.+60     	; 0x18f6 <delay+0x62>
		yield();
    18ba:	0e 94 75 0d 	call	0x1aea	; 0x1aea <yield>
		while ( ms > 0 && (micros() - start) >= 1000) {
    18be:	0e 94 27 0c 	call	0x184e	; 0x184e <micros>
    18c2:	68 19       	sub	r22, r8
    18c4:	79 09       	sbc	r23, r9
    18c6:	8a 09       	sbc	r24, r10
    18c8:	9b 09       	sbc	r25, r11
    18ca:	68 3e       	cpi	r22, 0xE8	; 232
    18cc:	73 40       	sbci	r23, 0x03	; 3
    18ce:	81 05       	cpc	r24, r1
    18d0:	91 05       	cpc	r25, r1
    18d2:	70 f3       	brcs	.-36     	; 0x18b0 <delay+0x1c>
			ms--;
    18d4:	21 e0       	ldi	r18, 0x01	; 1
    18d6:	c2 1a       	sub	r12, r18
    18d8:	d1 08       	sbc	r13, r1
    18da:	e1 08       	sbc	r14, r1
    18dc:	f1 08       	sbc	r15, r1
			start += 1000;
    18de:	88 ee       	ldi	r24, 0xE8	; 232
    18e0:	88 0e       	add	r8, r24
    18e2:	83 e0       	ldi	r24, 0x03	; 3
    18e4:	98 1e       	adc	r9, r24
    18e6:	a1 1c       	adc	r10, r1
    18e8:	b1 1c       	adc	r11, r1
{
	uint32_t start = micros();

	while (ms > 0) {
		yield();
		while ( ms > 0 && (micros() - start) >= 1000) {
    18ea:	c1 14       	cp	r12, r1
    18ec:	d1 04       	cpc	r13, r1
    18ee:	e1 04       	cpc	r14, r1
    18f0:	f1 04       	cpc	r15, r1
    18f2:	29 f7       	brne	.-54     	; 0x18be <delay+0x2a>
    18f4:	dd cf       	rjmp	.-70     	; 0x18b0 <delay+0x1c>
			ms--;
			start += 1000;
		}
	}
}
    18f6:	ff 90       	pop	r15
    18f8:	ef 90       	pop	r14
    18fa:	df 90       	pop	r13
    18fc:	cf 90       	pop	r12
    18fe:	bf 90       	pop	r11
    1900:	af 90       	pop	r10
    1902:	9f 90       	pop	r9
    1904:	8f 90       	pop	r8
    1906:	08 95       	ret

00001908 <init>:

void init()
{
	// this needs to be called before setup() or some functions won't
	// work there
	sei();
    1908:	78 94       	sei
	
	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
#if defined(TCCR0A) && defined(WGM01)
	sbi(TCCR0A, WGM01);
    190a:	84 b5       	in	r24, 0x24	; 36
    190c:	82 60       	ori	r24, 0x02	; 2
    190e:	84 bd       	out	0x24, r24	; 36
	sbi(TCCR0A, WGM00);
    1910:	84 b5       	in	r24, 0x24	; 36
    1912:	81 60       	ori	r24, 0x01	; 1
    1914:	84 bd       	out	0x24, r24	; 36
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
    1916:	85 b5       	in	r24, 0x25	; 37
    1918:	82 60       	ori	r24, 0x02	; 2
    191a:	85 bd       	out	0x25, r24	; 37
	sbi(TCCR0B, CS00);
    191c:	85 b5       	in	r24, 0x25	; 37
    191e:	81 60       	ori	r24, 0x01	; 1
    1920:	85 bd       	out	0x25, r24	; 37

	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
    1922:	ee e6       	ldi	r30, 0x6E	; 110
    1924:	f0 e0       	ldi	r31, 0x00	; 0
    1926:	80 81       	ld	r24, Z
    1928:	81 60       	ori	r24, 0x01	; 1
    192a:	80 83       	st	Z, r24
	// this is better for motors as it ensures an even waveform
	// note, however, that fast pwm mode can achieve a frequency of up
	// 8 MHz (with a 16 MHz clock) at 50% duty cycle

#if defined(TCCR1B) && defined(CS11) && defined(CS10)
	TCCR1B = 0;
    192c:	e1 e8       	ldi	r30, 0x81	; 129
    192e:	f0 e0       	ldi	r31, 0x00	; 0
    1930:	10 82       	st	Z, r1

	// set timer 1 prescale factor to 64
	sbi(TCCR1B, CS11);
    1932:	80 81       	ld	r24, Z
    1934:	82 60       	ori	r24, 0x02	; 2
    1936:	80 83       	st	Z, r24
#if F_CPU >= 8000000L
	sbi(TCCR1B, CS10);
    1938:	80 81       	ld	r24, Z
    193a:	81 60       	ori	r24, 0x01	; 1
    193c:	80 83       	st	Z, r24
	sbi(TCCR1, CS10);
#endif
#endif
	// put timer 1 in 8-bit phase correct pwm mode
#if defined(TCCR1A) && defined(WGM10)
	sbi(TCCR1A, WGM10);
    193e:	e0 e8       	ldi	r30, 0x80	; 128
    1940:	f0 e0       	ldi	r31, 0x00	; 0
    1942:	80 81       	ld	r24, Z
    1944:	81 60       	ori	r24, 0x01	; 1
    1946:	80 83       	st	Z, r24
//#else
	// Timer 2 not finished (may not be present on this CPU)
#endif

#if defined(TCCR3B) && defined(CS31) && defined(WGM30)
	sbi(TCCR3B, CS31);		// set timer 3 prescale factor to 64
    1948:	e1 e9       	ldi	r30, 0x91	; 145
    194a:	f0 e0       	ldi	r31, 0x00	; 0
    194c:	80 81       	ld	r24, Z
    194e:	82 60       	ori	r24, 0x02	; 2
    1950:	80 83       	st	Z, r24
	sbi(TCCR3B, CS30);
    1952:	80 81       	ld	r24, Z
    1954:	81 60       	ori	r24, 0x01	; 1
    1956:	80 83       	st	Z, r24
	sbi(TCCR3A, WGM30);		// put timer 3 in 8-bit phase correct pwm mode
    1958:	e0 e9       	ldi	r30, 0x90	; 144
    195a:	f0 e0       	ldi	r31, 0x00	; 0
    195c:	80 81       	ld	r24, Z
    195e:	81 60       	ori	r24, 0x01	; 1
    1960:	80 83       	st	Z, r24
#endif

#if defined(TCCR4A) && defined(TCCR4B) && defined(TCCR4D) /* beginning of timer4 block for 32U4 and similar */
	sbi(TCCR4B, CS42);		// set timer4 prescale factor to 64
    1962:	e1 ec       	ldi	r30, 0xC1	; 193
    1964:	f0 e0       	ldi	r31, 0x00	; 0
    1966:	80 81       	ld	r24, Z
    1968:	84 60       	ori	r24, 0x04	; 4
    196a:	80 83       	st	Z, r24
	sbi(TCCR4B, CS41);
    196c:	80 81       	ld	r24, Z
    196e:	82 60       	ori	r24, 0x02	; 2
    1970:	80 83       	st	Z, r24
	sbi(TCCR4B, CS40);
    1972:	80 81       	ld	r24, Z
    1974:	81 60       	ori	r24, 0x01	; 1
    1976:	80 83       	st	Z, r24
	sbi(TCCR4D, WGM40);		// put timer 4 in phase- and frequency-correct PWM mode	
    1978:	e3 ec       	ldi	r30, 0xC3	; 195
    197a:	f0 e0       	ldi	r31, 0x00	; 0
    197c:	80 81       	ld	r24, Z
    197e:	81 60       	ori	r24, 0x01	; 1
    1980:	80 83       	st	Z, r24
	sbi(TCCR4A, PWM4A);		// enable PWM mode for comparator OCR4A
    1982:	e0 ec       	ldi	r30, 0xC0	; 192
    1984:	f0 e0       	ldi	r31, 0x00	; 0
    1986:	80 81       	ld	r24, Z
    1988:	82 60       	ori	r24, 0x02	; 2
    198a:	80 83       	st	Z, r24
	sbi(TCCR4C, PWM4D);		// enable PWM mode for comparator OCR4D
    198c:	e2 ec       	ldi	r30, 0xC2	; 194
    198e:	f0 e0       	ldi	r31, 0x00	; 0
    1990:	80 81       	ld	r24, Z
    1992:	81 60       	ori	r24, 0x01	; 1
    1994:	80 83       	st	Z, r24
#endif

#if defined(ADCSRA)
	// set a2d prescaler so we are inside the desired 50-200 KHz range.
	#if F_CPU >= 16000000 // 16 MHz / 128 = 125 KHz
		sbi(ADCSRA, ADPS2);
    1996:	ea e7       	ldi	r30, 0x7A	; 122
    1998:	f0 e0       	ldi	r31, 0x00	; 0
    199a:	80 81       	ld	r24, Z
    199c:	84 60       	ori	r24, 0x04	; 4
    199e:	80 83       	st	Z, r24
		sbi(ADCSRA, ADPS1);
    19a0:	80 81       	ld	r24, Z
    19a2:	82 60       	ori	r24, 0x02	; 2
    19a4:	80 83       	st	Z, r24
		sbi(ADCSRA, ADPS0);
    19a6:	80 81       	ld	r24, Z
    19a8:	81 60       	ori	r24, 0x01	; 1
    19aa:	80 83       	st	Z, r24
		cbi(ADCSRA, ADPS2);
		cbi(ADCSRA, ADPS1);
		sbi(ADCSRA, ADPS0);
	#endif
	// enable a2d conversions
	sbi(ADCSRA, ADEN);
    19ac:	80 81       	ld	r24, Z
    19ae:	80 68       	ori	r24, 0x80	; 128
    19b0:	80 83       	st	Z, r24
    19b2:	08 95       	ret

000019b4 <_ZN13PluggableUSB_12getInterfaceEPh>:
#ifdef PLUGGABLE_USB_ENABLED

extern uint8_t _initEndpoints[];

int PluggableUSB_::getInterface(uint8_t* interfaceCount)
{
    19b4:	ef 92       	push	r14
    19b6:	ff 92       	push	r15
    19b8:	0f 93       	push	r16
    19ba:	1f 93       	push	r17
    19bc:	cf 93       	push	r28
    19be:	df 93       	push	r29
    19c0:	7b 01       	movw	r14, r22
	int sent = 0;
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    19c2:	fc 01       	movw	r30, r24
    19c4:	c2 81       	ldd	r28, Z+2	; 0x02
    19c6:	d3 81       	ldd	r29, Z+3	; 0x03

extern uint8_t _initEndpoints[];

int PluggableUSB_::getInterface(uint8_t* interfaceCount)
{
	int sent = 0;
    19c8:	00 e0       	ldi	r16, 0x00	; 0
    19ca:	10 e0       	ldi	r17, 0x00	; 0
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    19cc:	20 97       	sbiw	r28, 0x00	; 0
    19ce:	81 f0       	breq	.+32     	; 0x19f0 <_ZN13PluggableUSB_12getInterfaceEPh+0x3c>
		int res = node->getInterface(interfaceCount);
    19d0:	e8 81       	ld	r30, Y
    19d2:	f9 81       	ldd	r31, Y+1	; 0x01
    19d4:	02 80       	ldd	r0, Z+2	; 0x02
    19d6:	f3 81       	ldd	r31, Z+3	; 0x03
    19d8:	e0 2d       	mov	r30, r0
    19da:	b7 01       	movw	r22, r14
    19dc:	ce 01       	movw	r24, r28
    19de:	09 95       	icall
		if (res < 0)
    19e0:	97 fd       	sbrc	r25, 7
    19e2:	08 c0       	rjmp	.+16     	; 0x19f4 <_ZN13PluggableUSB_12getInterfaceEPh+0x40>
			return -1;
		sent += res;
    19e4:	08 0f       	add	r16, r24
    19e6:	19 1f       	adc	r17, r25

int PluggableUSB_::getInterface(uint8_t* interfaceCount)
{
	int sent = 0;
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    19e8:	08 84       	ldd	r0, Y+8	; 0x08
    19ea:	d9 85       	ldd	r29, Y+9	; 0x09
    19ec:	c0 2d       	mov	r28, r0
    19ee:	ee cf       	rjmp	.-36     	; 0x19cc <_ZN13PluggableUSB_12getInterfaceEPh+0x18>
    19f0:	c8 01       	movw	r24, r16
    19f2:	02 c0       	rjmp	.+4      	; 0x19f8 <_ZN13PluggableUSB_12getInterfaceEPh+0x44>
		int res = node->getInterface(interfaceCount);
		if (res < 0)
			return -1;
    19f4:	8f ef       	ldi	r24, 0xFF	; 255
    19f6:	9f ef       	ldi	r25, 0xFF	; 255
		sent += res;
	}
	return sent;
}
    19f8:	df 91       	pop	r29
    19fa:	cf 91       	pop	r28
    19fc:	1f 91       	pop	r17
    19fe:	0f 91       	pop	r16
    1a00:	ff 90       	pop	r15
    1a02:	ef 90       	pop	r14
    1a04:	08 95       	ret

00001a06 <_ZN13PluggableUSB_13getDescriptorER8USBSetup>:

int PluggableUSB_::getDescriptor(USBSetup& setup)
{
    1a06:	0f 93       	push	r16
    1a08:	1f 93       	push	r17
    1a0a:	cf 93       	push	r28
    1a0c:	df 93       	push	r29
    1a0e:	8b 01       	movw	r16, r22
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    1a10:	fc 01       	movw	r30, r24
    1a12:	c2 81       	ldd	r28, Z+2	; 0x02
    1a14:	d3 81       	ldd	r29, Z+3	; 0x03
    1a16:	20 97       	sbiw	r28, 0x00	; 0
    1a18:	71 f0       	breq	.+28     	; 0x1a36 <_ZN13PluggableUSB_13getDescriptorER8USBSetup+0x30>
		int ret = node->getDescriptor(setup);
    1a1a:	e8 81       	ld	r30, Y
    1a1c:	f9 81       	ldd	r31, Y+1	; 0x01
    1a1e:	04 80       	ldd	r0, Z+4	; 0x04
    1a20:	f5 81       	ldd	r31, Z+5	; 0x05
    1a22:	e0 2d       	mov	r30, r0
    1a24:	b8 01       	movw	r22, r16
    1a26:	ce 01       	movw	r24, r28
    1a28:	09 95       	icall
		// ret!=0 -> request has been processed
		if (ret)
    1a2a:	00 97       	sbiw	r24, 0x00	; 0
    1a2c:	31 f4       	brne	.+12     	; 0x1a3a <_ZN13PluggableUSB_13getDescriptorER8USBSetup+0x34>
}

int PluggableUSB_::getDescriptor(USBSetup& setup)
{
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    1a2e:	08 84       	ldd	r0, Y+8	; 0x08
    1a30:	d9 85       	ldd	r29, Y+9	; 0x09
    1a32:	c0 2d       	mov	r28, r0
    1a34:	f0 cf       	rjmp	.-32     	; 0x1a16 <_ZN13PluggableUSB_13getDescriptorER8USBSetup+0x10>
		int ret = node->getDescriptor(setup);
		// ret!=0 -> request has been processed
		if (ret)
			return ret;
	}
	return 0;
    1a36:	80 e0       	ldi	r24, 0x00	; 0
    1a38:	90 e0       	ldi	r25, 0x00	; 0
}
    1a3a:	df 91       	pop	r29
    1a3c:	cf 91       	pop	r28
    1a3e:	1f 91       	pop	r17
    1a40:	0f 91       	pop	r16
    1a42:	08 95       	ret

00001a44 <_ZN13PluggableUSB_12getShortNameEPc>:

void PluggableUSB_::getShortName(char *iSerialNum)
{
    1a44:	0f 93       	push	r16
    1a46:	1f 93       	push	r17
    1a48:	cf 93       	push	r28
    1a4a:	df 93       	push	r29
    1a4c:	8b 01       	movw	r16, r22
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    1a4e:	fc 01       	movw	r30, r24
    1a50:	c2 81       	ldd	r28, Z+2	; 0x02
    1a52:	d3 81       	ldd	r29, Z+3	; 0x03
    1a54:	20 97       	sbiw	r28, 0x00	; 0
    1a56:	71 f0       	breq	.+28     	; 0x1a74 <_ZN13PluggableUSB_12getShortNameEPc+0x30>
		iSerialNum += node->getShortName(iSerialNum);
    1a58:	e8 81       	ld	r30, Y
    1a5a:	f9 81       	ldd	r31, Y+1	; 0x01
    1a5c:	06 80       	ldd	r0, Z+6	; 0x06
    1a5e:	f7 81       	ldd	r31, Z+7	; 0x07
    1a60:	e0 2d       	mov	r30, r0
    1a62:	b8 01       	movw	r22, r16
    1a64:	ce 01       	movw	r24, r28
    1a66:	09 95       	icall
    1a68:	08 0f       	add	r16, r24
    1a6a:	11 1d       	adc	r17, r1
}

void PluggableUSB_::getShortName(char *iSerialNum)
{
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    1a6c:	08 84       	ldd	r0, Y+8	; 0x08
    1a6e:	d9 85       	ldd	r29, Y+9	; 0x09
    1a70:	c0 2d       	mov	r28, r0
    1a72:	f0 cf       	rjmp	.-32     	; 0x1a54 <_ZN13PluggableUSB_12getShortNameEPc+0x10>
		iSerialNum += node->getShortName(iSerialNum);
	}
	*iSerialNum = 0;
    1a74:	f8 01       	movw	r30, r16
    1a76:	10 82       	st	Z, r1
}
    1a78:	df 91       	pop	r29
    1a7a:	cf 91       	pop	r28
    1a7c:	1f 91       	pop	r17
    1a7e:	0f 91       	pop	r16
    1a80:	08 95       	ret

00001a82 <_ZN13PluggableUSB_5setupER8USBSetup>:

bool PluggableUSB_::setup(USBSetup& setup)
{
    1a82:	0f 93       	push	r16
    1a84:	1f 93       	push	r17
    1a86:	cf 93       	push	r28
    1a88:	df 93       	push	r29
    1a8a:	8b 01       	movw	r16, r22
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    1a8c:	fc 01       	movw	r30, r24
    1a8e:	c2 81       	ldd	r28, Z+2	; 0x02
    1a90:	d3 81       	ldd	r29, Z+3	; 0x03
    1a92:	20 97       	sbiw	r28, 0x00	; 0
    1a94:	71 f0       	breq	.+28     	; 0x1ab2 <_ZN13PluggableUSB_5setupER8USBSetup+0x30>
		if (node->setup(setup)) {
    1a96:	e8 81       	ld	r30, Y
    1a98:	f9 81       	ldd	r31, Y+1	; 0x01
    1a9a:	01 90       	ld	r0, Z+
    1a9c:	f0 81       	ld	r31, Z
    1a9e:	e0 2d       	mov	r30, r0
    1aa0:	b8 01       	movw	r22, r16
    1aa2:	ce 01       	movw	r24, r28
    1aa4:	09 95       	icall
    1aa6:	81 11       	cpse	r24, r1
    1aa8:	06 c0       	rjmp	.+12     	; 0x1ab6 <_ZN13PluggableUSB_5setupER8USBSetup+0x34>
}

bool PluggableUSB_::setup(USBSetup& setup)
{
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
    1aaa:	08 84       	ldd	r0, Y+8	; 0x08
    1aac:	d9 85       	ldd	r29, Y+9	; 0x09
    1aae:	c0 2d       	mov	r28, r0
    1ab0:	f0 cf       	rjmp	.-32     	; 0x1a92 <_ZN13PluggableUSB_5setupER8USBSetup+0x10>
		if (node->setup(setup)) {
			return true;
		}
	}
	return false;
    1ab2:	80 e0       	ldi	r24, 0x00	; 0
    1ab4:	01 c0       	rjmp	.+2      	; 0x1ab8 <_ZN13PluggableUSB_5setupER8USBSetup+0x36>
bool PluggableUSB_::setup(USBSetup& setup)
{
	PluggableUSBModule* node;
	for (node = rootNode; node; node = node->next) {
		if (node->setup(setup)) {
			return true;
    1ab6:	81 e0       	ldi	r24, 0x01	; 1
		}
	}
	return false;
}
    1ab8:	df 91       	pop	r29
    1aba:	cf 91       	pop	r28
    1abc:	1f 91       	pop	r17
    1abe:	0f 91       	pop	r16
    1ac0:	08 95       	ret

00001ac2 <_Z12PluggableUSBv>:
	// restart USB layer???
}

PluggableUSB_& PluggableUSB()
{
	static PluggableUSB_ obj;
    1ac2:	80 91 6e 02 	lds	r24, 0x026E
    1ac6:	81 11       	cpse	r24, r1
    1ac8:	0d c0       	rjmp	.+26     	; 0x1ae4 <_Z12PluggableUSBv+0x22>
	return obj;
}

PluggableUSB_::PluggableUSB_() : lastIf(CDC_ACM_INTERFACE + CDC_INTERFACE_COUNT),
                                 lastEp(CDC_FIRST_ENDPOINT + CDC_ENPOINT_COUNT),
                                 rootNode(NULL)
    1aca:	82 e0       	ldi	r24, 0x02	; 2
    1acc:	80 93 6a 02 	sts	0x026A, r24
    1ad0:	84 e0       	ldi	r24, 0x04	; 4
    1ad2:	80 93 6b 02 	sts	0x026B, r24
    1ad6:	10 92 6d 02 	sts	0x026D, r1
    1ada:	10 92 6c 02 	sts	0x026C, r1
	// restart USB layer???
}

PluggableUSB_& PluggableUSB()
{
	static PluggableUSB_ obj;
    1ade:	81 e0       	ldi	r24, 0x01	; 1
    1ae0:	80 93 6e 02 	sts	0x026E, r24
	return obj;
}
    1ae4:	8a e6       	ldi	r24, 0x6A	; 106
    1ae6:	92 e0       	ldi	r25, 0x02	; 2
    1ae8:	08 95       	ret

00001aea <yield>:
 * libraries or sketches that supports cooperative threads.
 *
 * Its defined as a weak symbol and it can be redefined to implement a
 * real cooperative scheduler.
 */
static void __empty() {
    1aea:	08 95       	ret

00001aec <__subsf3>:
    1aec:	50 58       	subi	r21, 0x80	; 128

00001aee <__addsf3>:
    1aee:	bb 27       	eor	r27, r27
    1af0:	aa 27       	eor	r26, r26
    1af2:	0e d0       	rcall	.+28     	; 0x1b10 <__addsf3x>
    1af4:	75 c1       	rjmp	.+746    	; 0x1de0 <__fp_round>
    1af6:	66 d1       	rcall	.+716    	; 0x1dc4 <__fp_pscA>
    1af8:	30 f0       	brcs	.+12     	; 0x1b06 <__addsf3+0x18>
    1afa:	6b d1       	rcall	.+726    	; 0x1dd2 <__fp_pscB>
    1afc:	20 f0       	brcs	.+8      	; 0x1b06 <__addsf3+0x18>
    1afe:	31 f4       	brne	.+12     	; 0x1b0c <__addsf3+0x1e>
    1b00:	9f 3f       	cpi	r25, 0xFF	; 255
    1b02:	11 f4       	brne	.+4      	; 0x1b08 <__addsf3+0x1a>
    1b04:	1e f4       	brtc	.+6      	; 0x1b0c <__addsf3+0x1e>
    1b06:	5b c1       	rjmp	.+694    	; 0x1dbe <__fp_nan>
    1b08:	0e f4       	brtc	.+2      	; 0x1b0c <__addsf3+0x1e>
    1b0a:	e0 95       	com	r30
    1b0c:	e7 fb       	bst	r30, 7
    1b0e:	51 c1       	rjmp	.+674    	; 0x1db2 <__fp_inf>

00001b10 <__addsf3x>:
    1b10:	e9 2f       	mov	r30, r25
    1b12:	77 d1       	rcall	.+750    	; 0x1e02 <__fp_split3>
    1b14:	80 f3       	brcs	.-32     	; 0x1af6 <__addsf3+0x8>
    1b16:	ba 17       	cp	r27, r26
    1b18:	62 07       	cpc	r22, r18
    1b1a:	73 07       	cpc	r23, r19
    1b1c:	84 07       	cpc	r24, r20
    1b1e:	95 07       	cpc	r25, r21
    1b20:	18 f0       	brcs	.+6      	; 0x1b28 <__addsf3x+0x18>
    1b22:	71 f4       	brne	.+28     	; 0x1b40 <__addsf3x+0x30>
    1b24:	9e f5       	brtc	.+102    	; 0x1b8c <__addsf3x+0x7c>
    1b26:	8f c1       	rjmp	.+798    	; 0x1e46 <__fp_zero>
    1b28:	0e f4       	brtc	.+2      	; 0x1b2c <__addsf3x+0x1c>
    1b2a:	e0 95       	com	r30
    1b2c:	0b 2e       	mov	r0, r27
    1b2e:	ba 2f       	mov	r27, r26
    1b30:	a0 2d       	mov	r26, r0
    1b32:	0b 01       	movw	r0, r22
    1b34:	b9 01       	movw	r22, r18
    1b36:	90 01       	movw	r18, r0
    1b38:	0c 01       	movw	r0, r24
    1b3a:	ca 01       	movw	r24, r20
    1b3c:	a0 01       	movw	r20, r0
    1b3e:	11 24       	eor	r1, r1
    1b40:	ff 27       	eor	r31, r31
    1b42:	59 1b       	sub	r21, r25
    1b44:	99 f0       	breq	.+38     	; 0x1b6c <__addsf3x+0x5c>
    1b46:	59 3f       	cpi	r21, 0xF9	; 249
    1b48:	50 f4       	brcc	.+20     	; 0x1b5e <__addsf3x+0x4e>
    1b4a:	50 3e       	cpi	r21, 0xE0	; 224
    1b4c:	68 f1       	brcs	.+90     	; 0x1ba8 <__addsf3x+0x98>
    1b4e:	1a 16       	cp	r1, r26
    1b50:	f0 40       	sbci	r31, 0x00	; 0
    1b52:	a2 2f       	mov	r26, r18
    1b54:	23 2f       	mov	r18, r19
    1b56:	34 2f       	mov	r19, r20
    1b58:	44 27       	eor	r20, r20
    1b5a:	58 5f       	subi	r21, 0xF8	; 248
    1b5c:	f3 cf       	rjmp	.-26     	; 0x1b44 <__addsf3x+0x34>
    1b5e:	46 95       	lsr	r20
    1b60:	37 95       	ror	r19
    1b62:	27 95       	ror	r18
    1b64:	a7 95       	ror	r26
    1b66:	f0 40       	sbci	r31, 0x00	; 0
    1b68:	53 95       	inc	r21
    1b6a:	c9 f7       	brne	.-14     	; 0x1b5e <__addsf3x+0x4e>
    1b6c:	7e f4       	brtc	.+30     	; 0x1b8c <__addsf3x+0x7c>
    1b6e:	1f 16       	cp	r1, r31
    1b70:	ba 0b       	sbc	r27, r26
    1b72:	62 0b       	sbc	r22, r18
    1b74:	73 0b       	sbc	r23, r19
    1b76:	84 0b       	sbc	r24, r20
    1b78:	ba f0       	brmi	.+46     	; 0x1ba8 <__addsf3x+0x98>
    1b7a:	91 50       	subi	r25, 0x01	; 1
    1b7c:	a1 f0       	breq	.+40     	; 0x1ba6 <__addsf3x+0x96>
    1b7e:	ff 0f       	add	r31, r31
    1b80:	bb 1f       	adc	r27, r27
    1b82:	66 1f       	adc	r22, r22
    1b84:	77 1f       	adc	r23, r23
    1b86:	88 1f       	adc	r24, r24
    1b88:	c2 f7       	brpl	.-16     	; 0x1b7a <__addsf3x+0x6a>
    1b8a:	0e c0       	rjmp	.+28     	; 0x1ba8 <__addsf3x+0x98>
    1b8c:	ba 0f       	add	r27, r26
    1b8e:	62 1f       	adc	r22, r18
    1b90:	73 1f       	adc	r23, r19
    1b92:	84 1f       	adc	r24, r20
    1b94:	48 f4       	brcc	.+18     	; 0x1ba8 <__addsf3x+0x98>
    1b96:	87 95       	ror	r24
    1b98:	77 95       	ror	r23
    1b9a:	67 95       	ror	r22
    1b9c:	b7 95       	ror	r27
    1b9e:	f7 95       	ror	r31
    1ba0:	9e 3f       	cpi	r25, 0xFE	; 254
    1ba2:	08 f0       	brcs	.+2      	; 0x1ba6 <__addsf3x+0x96>
    1ba4:	b3 cf       	rjmp	.-154    	; 0x1b0c <__addsf3+0x1e>
    1ba6:	93 95       	inc	r25
    1ba8:	88 0f       	add	r24, r24
    1baa:	08 f0       	brcs	.+2      	; 0x1bae <__addsf3x+0x9e>
    1bac:	99 27       	eor	r25, r25
    1bae:	ee 0f       	add	r30, r30
    1bb0:	97 95       	ror	r25
    1bb2:	87 95       	ror	r24
    1bb4:	08 95       	ret

00001bb6 <__cmpsf2>:
    1bb6:	d9 d0       	rcall	.+434    	; 0x1d6a <__fp_cmp>
    1bb8:	08 f4       	brcc	.+2      	; 0x1bbc <__cmpsf2+0x6>
    1bba:	81 e0       	ldi	r24, 0x01	; 1
    1bbc:	08 95       	ret

00001bbe <__divsf3>:
    1bbe:	0c d0       	rcall	.+24     	; 0x1bd8 <__divsf3x>
    1bc0:	0f c1       	rjmp	.+542    	; 0x1de0 <__fp_round>
    1bc2:	07 d1       	rcall	.+526    	; 0x1dd2 <__fp_pscB>
    1bc4:	40 f0       	brcs	.+16     	; 0x1bd6 <__divsf3+0x18>
    1bc6:	fe d0       	rcall	.+508    	; 0x1dc4 <__fp_pscA>
    1bc8:	30 f0       	brcs	.+12     	; 0x1bd6 <__divsf3+0x18>
    1bca:	21 f4       	brne	.+8      	; 0x1bd4 <__divsf3+0x16>
    1bcc:	5f 3f       	cpi	r21, 0xFF	; 255
    1bce:	19 f0       	breq	.+6      	; 0x1bd6 <__divsf3+0x18>
    1bd0:	f0 c0       	rjmp	.+480    	; 0x1db2 <__fp_inf>
    1bd2:	51 11       	cpse	r21, r1
    1bd4:	39 c1       	rjmp	.+626    	; 0x1e48 <__fp_szero>
    1bd6:	f3 c0       	rjmp	.+486    	; 0x1dbe <__fp_nan>

00001bd8 <__divsf3x>:
    1bd8:	14 d1       	rcall	.+552    	; 0x1e02 <__fp_split3>
    1bda:	98 f3       	brcs	.-26     	; 0x1bc2 <__divsf3+0x4>

00001bdc <__divsf3_pse>:
    1bdc:	99 23       	and	r25, r25
    1bde:	c9 f3       	breq	.-14     	; 0x1bd2 <__divsf3+0x14>
    1be0:	55 23       	and	r21, r21
    1be2:	b1 f3       	breq	.-20     	; 0x1bd0 <__divsf3+0x12>
    1be4:	95 1b       	sub	r25, r21
    1be6:	55 0b       	sbc	r21, r21
    1be8:	bb 27       	eor	r27, r27
    1bea:	aa 27       	eor	r26, r26
    1bec:	62 17       	cp	r22, r18
    1bee:	73 07       	cpc	r23, r19
    1bf0:	84 07       	cpc	r24, r20
    1bf2:	38 f0       	brcs	.+14     	; 0x1c02 <__divsf3_pse+0x26>
    1bf4:	9f 5f       	subi	r25, 0xFF	; 255
    1bf6:	5f 4f       	sbci	r21, 0xFF	; 255
    1bf8:	22 0f       	add	r18, r18
    1bfa:	33 1f       	adc	r19, r19
    1bfc:	44 1f       	adc	r20, r20
    1bfe:	aa 1f       	adc	r26, r26
    1c00:	a9 f3       	breq	.-22     	; 0x1bec <__divsf3_pse+0x10>
    1c02:	33 d0       	rcall	.+102    	; 0x1c6a <__divsf3_pse+0x8e>
    1c04:	0e 2e       	mov	r0, r30
    1c06:	3a f0       	brmi	.+14     	; 0x1c16 <__divsf3_pse+0x3a>
    1c08:	e0 e8       	ldi	r30, 0x80	; 128
    1c0a:	30 d0       	rcall	.+96     	; 0x1c6c <__divsf3_pse+0x90>
    1c0c:	91 50       	subi	r25, 0x01	; 1
    1c0e:	50 40       	sbci	r21, 0x00	; 0
    1c10:	e6 95       	lsr	r30
    1c12:	00 1c       	adc	r0, r0
    1c14:	ca f7       	brpl	.-14     	; 0x1c08 <__divsf3_pse+0x2c>
    1c16:	29 d0       	rcall	.+82     	; 0x1c6a <__divsf3_pse+0x8e>
    1c18:	fe 2f       	mov	r31, r30
    1c1a:	27 d0       	rcall	.+78     	; 0x1c6a <__divsf3_pse+0x8e>
    1c1c:	66 0f       	add	r22, r22
    1c1e:	77 1f       	adc	r23, r23
    1c20:	88 1f       	adc	r24, r24
    1c22:	bb 1f       	adc	r27, r27
    1c24:	26 17       	cp	r18, r22
    1c26:	37 07       	cpc	r19, r23
    1c28:	48 07       	cpc	r20, r24
    1c2a:	ab 07       	cpc	r26, r27
    1c2c:	b0 e8       	ldi	r27, 0x80	; 128
    1c2e:	09 f0       	breq	.+2      	; 0x1c32 <__divsf3_pse+0x56>
    1c30:	bb 0b       	sbc	r27, r27
    1c32:	80 2d       	mov	r24, r0
    1c34:	bf 01       	movw	r22, r30
    1c36:	ff 27       	eor	r31, r31
    1c38:	93 58       	subi	r25, 0x83	; 131
    1c3a:	5f 4f       	sbci	r21, 0xFF	; 255
    1c3c:	2a f0       	brmi	.+10     	; 0x1c48 <__divsf3_pse+0x6c>
    1c3e:	9e 3f       	cpi	r25, 0xFE	; 254
    1c40:	51 05       	cpc	r21, r1
    1c42:	68 f0       	brcs	.+26     	; 0x1c5e <__divsf3_pse+0x82>
    1c44:	b6 c0       	rjmp	.+364    	; 0x1db2 <__fp_inf>
    1c46:	00 c1       	rjmp	.+512    	; 0x1e48 <__fp_szero>
    1c48:	5f 3f       	cpi	r21, 0xFF	; 255
    1c4a:	ec f3       	brlt	.-6      	; 0x1c46 <__divsf3_pse+0x6a>
    1c4c:	98 3e       	cpi	r25, 0xE8	; 232
    1c4e:	dc f3       	brlt	.-10     	; 0x1c46 <__divsf3_pse+0x6a>
    1c50:	86 95       	lsr	r24
    1c52:	77 95       	ror	r23
    1c54:	67 95       	ror	r22
    1c56:	b7 95       	ror	r27
    1c58:	f7 95       	ror	r31
    1c5a:	9f 5f       	subi	r25, 0xFF	; 255
    1c5c:	c9 f7       	brne	.-14     	; 0x1c50 <__divsf3_pse+0x74>
    1c5e:	88 0f       	add	r24, r24
    1c60:	91 1d       	adc	r25, r1
    1c62:	96 95       	lsr	r25
    1c64:	87 95       	ror	r24
    1c66:	97 f9       	bld	r25, 7
    1c68:	08 95       	ret
    1c6a:	e1 e0       	ldi	r30, 0x01	; 1
    1c6c:	66 0f       	add	r22, r22
    1c6e:	77 1f       	adc	r23, r23
    1c70:	88 1f       	adc	r24, r24
    1c72:	bb 1f       	adc	r27, r27
    1c74:	62 17       	cp	r22, r18
    1c76:	73 07       	cpc	r23, r19
    1c78:	84 07       	cpc	r24, r20
    1c7a:	ba 07       	cpc	r27, r26
    1c7c:	20 f0       	brcs	.+8      	; 0x1c86 <__divsf3_pse+0xaa>
    1c7e:	62 1b       	sub	r22, r18
    1c80:	73 0b       	sbc	r23, r19
    1c82:	84 0b       	sbc	r24, r20
    1c84:	ba 0b       	sbc	r27, r26
    1c86:	ee 1f       	adc	r30, r30
    1c88:	88 f7       	brcc	.-30     	; 0x1c6c <__divsf3_pse+0x90>
    1c8a:	e0 95       	com	r30
    1c8c:	08 95       	ret

00001c8e <__fixsfsi>:
    1c8e:	04 d0       	rcall	.+8      	; 0x1c98 <__fixunssfsi>
    1c90:	68 94       	set
    1c92:	b1 11       	cpse	r27, r1
    1c94:	d9 c0       	rjmp	.+434    	; 0x1e48 <__fp_szero>
    1c96:	08 95       	ret

00001c98 <__fixunssfsi>:
    1c98:	bc d0       	rcall	.+376    	; 0x1e12 <__fp_splitA>
    1c9a:	88 f0       	brcs	.+34     	; 0x1cbe <__fixunssfsi+0x26>
    1c9c:	9f 57       	subi	r25, 0x7F	; 127
    1c9e:	90 f0       	brcs	.+36     	; 0x1cc4 <__fixunssfsi+0x2c>
    1ca0:	b9 2f       	mov	r27, r25
    1ca2:	99 27       	eor	r25, r25
    1ca4:	b7 51       	subi	r27, 0x17	; 23
    1ca6:	a0 f0       	brcs	.+40     	; 0x1cd0 <__fixunssfsi+0x38>
    1ca8:	d1 f0       	breq	.+52     	; 0x1cde <__fixunssfsi+0x46>
    1caa:	66 0f       	add	r22, r22
    1cac:	77 1f       	adc	r23, r23
    1cae:	88 1f       	adc	r24, r24
    1cb0:	99 1f       	adc	r25, r25
    1cb2:	1a f0       	brmi	.+6      	; 0x1cba <__fixunssfsi+0x22>
    1cb4:	ba 95       	dec	r27
    1cb6:	c9 f7       	brne	.-14     	; 0x1caa <__fixunssfsi+0x12>
    1cb8:	12 c0       	rjmp	.+36     	; 0x1cde <__fixunssfsi+0x46>
    1cba:	b1 30       	cpi	r27, 0x01	; 1
    1cbc:	81 f0       	breq	.+32     	; 0x1cde <__fixunssfsi+0x46>
    1cbe:	c3 d0       	rcall	.+390    	; 0x1e46 <__fp_zero>
    1cc0:	b1 e0       	ldi	r27, 0x01	; 1
    1cc2:	08 95       	ret
    1cc4:	c0 c0       	rjmp	.+384    	; 0x1e46 <__fp_zero>
    1cc6:	67 2f       	mov	r22, r23
    1cc8:	78 2f       	mov	r23, r24
    1cca:	88 27       	eor	r24, r24
    1ccc:	b8 5f       	subi	r27, 0xF8	; 248
    1cce:	39 f0       	breq	.+14     	; 0x1cde <__fixunssfsi+0x46>
    1cd0:	b9 3f       	cpi	r27, 0xF9	; 249
    1cd2:	cc f3       	brlt	.-14     	; 0x1cc6 <__fixunssfsi+0x2e>
    1cd4:	86 95       	lsr	r24
    1cd6:	77 95       	ror	r23
    1cd8:	67 95       	ror	r22
    1cda:	b3 95       	inc	r27
    1cdc:	d9 f7       	brne	.-10     	; 0x1cd4 <__fixunssfsi+0x3c>
    1cde:	3e f4       	brtc	.+14     	; 0x1cee <__fixunssfsi+0x56>
    1ce0:	90 95       	com	r25
    1ce2:	80 95       	com	r24
    1ce4:	70 95       	com	r23
    1ce6:	61 95       	neg	r22
    1ce8:	7f 4f       	sbci	r23, 0xFF	; 255
    1cea:	8f 4f       	sbci	r24, 0xFF	; 255
    1cec:	9f 4f       	sbci	r25, 0xFF	; 255
    1cee:	08 95       	ret

00001cf0 <__floatunsisf>:
    1cf0:	e8 94       	clt
    1cf2:	09 c0       	rjmp	.+18     	; 0x1d06 <__floatsisf+0x12>

00001cf4 <__floatsisf>:
    1cf4:	97 fb       	bst	r25, 7
    1cf6:	3e f4       	brtc	.+14     	; 0x1d06 <__floatsisf+0x12>
    1cf8:	90 95       	com	r25
    1cfa:	80 95       	com	r24
    1cfc:	70 95       	com	r23
    1cfe:	61 95       	neg	r22
    1d00:	7f 4f       	sbci	r23, 0xFF	; 255
    1d02:	8f 4f       	sbci	r24, 0xFF	; 255
    1d04:	9f 4f       	sbci	r25, 0xFF	; 255
    1d06:	99 23       	and	r25, r25
    1d08:	a9 f0       	breq	.+42     	; 0x1d34 <__floatsisf+0x40>
    1d0a:	f9 2f       	mov	r31, r25
    1d0c:	96 e9       	ldi	r25, 0x96	; 150
    1d0e:	bb 27       	eor	r27, r27
    1d10:	93 95       	inc	r25
    1d12:	f6 95       	lsr	r31
    1d14:	87 95       	ror	r24
    1d16:	77 95       	ror	r23
    1d18:	67 95       	ror	r22
    1d1a:	b7 95       	ror	r27
    1d1c:	f1 11       	cpse	r31, r1
    1d1e:	f8 cf       	rjmp	.-16     	; 0x1d10 <__floatsisf+0x1c>
    1d20:	fa f4       	brpl	.+62     	; 0x1d60 <__floatsisf+0x6c>
    1d22:	bb 0f       	add	r27, r27
    1d24:	11 f4       	brne	.+4      	; 0x1d2a <__floatsisf+0x36>
    1d26:	60 ff       	sbrs	r22, 0
    1d28:	1b c0       	rjmp	.+54     	; 0x1d60 <__floatsisf+0x6c>
    1d2a:	6f 5f       	subi	r22, 0xFF	; 255
    1d2c:	7f 4f       	sbci	r23, 0xFF	; 255
    1d2e:	8f 4f       	sbci	r24, 0xFF	; 255
    1d30:	9f 4f       	sbci	r25, 0xFF	; 255
    1d32:	16 c0       	rjmp	.+44     	; 0x1d60 <__floatsisf+0x6c>
    1d34:	88 23       	and	r24, r24
    1d36:	11 f0       	breq	.+4      	; 0x1d3c <__floatsisf+0x48>
    1d38:	96 e9       	ldi	r25, 0x96	; 150
    1d3a:	11 c0       	rjmp	.+34     	; 0x1d5e <__floatsisf+0x6a>
    1d3c:	77 23       	and	r23, r23
    1d3e:	21 f0       	breq	.+8      	; 0x1d48 <__floatsisf+0x54>
    1d40:	9e e8       	ldi	r25, 0x8E	; 142
    1d42:	87 2f       	mov	r24, r23
    1d44:	76 2f       	mov	r23, r22
    1d46:	05 c0       	rjmp	.+10     	; 0x1d52 <__floatsisf+0x5e>
    1d48:	66 23       	and	r22, r22
    1d4a:	71 f0       	breq	.+28     	; 0x1d68 <__floatsisf+0x74>
    1d4c:	96 e8       	ldi	r25, 0x86	; 134
    1d4e:	86 2f       	mov	r24, r22
    1d50:	70 e0       	ldi	r23, 0x00	; 0
    1d52:	60 e0       	ldi	r22, 0x00	; 0
    1d54:	2a f0       	brmi	.+10     	; 0x1d60 <__floatsisf+0x6c>
    1d56:	9a 95       	dec	r25
    1d58:	66 0f       	add	r22, r22
    1d5a:	77 1f       	adc	r23, r23
    1d5c:	88 1f       	adc	r24, r24
    1d5e:	da f7       	brpl	.-10     	; 0x1d56 <__floatsisf+0x62>
    1d60:	88 0f       	add	r24, r24
    1d62:	96 95       	lsr	r25
    1d64:	87 95       	ror	r24
    1d66:	97 f9       	bld	r25, 7
    1d68:	08 95       	ret

00001d6a <__fp_cmp>:
    1d6a:	99 0f       	add	r25, r25
    1d6c:	00 08       	sbc	r0, r0
    1d6e:	55 0f       	add	r21, r21
    1d70:	aa 0b       	sbc	r26, r26
    1d72:	e0 e8       	ldi	r30, 0x80	; 128
    1d74:	fe ef       	ldi	r31, 0xFE	; 254
    1d76:	16 16       	cp	r1, r22
    1d78:	17 06       	cpc	r1, r23
    1d7a:	e8 07       	cpc	r30, r24
    1d7c:	f9 07       	cpc	r31, r25
    1d7e:	c0 f0       	brcs	.+48     	; 0x1db0 <__fp_cmp+0x46>
    1d80:	12 16       	cp	r1, r18
    1d82:	13 06       	cpc	r1, r19
    1d84:	e4 07       	cpc	r30, r20
    1d86:	f5 07       	cpc	r31, r21
    1d88:	98 f0       	brcs	.+38     	; 0x1db0 <__fp_cmp+0x46>
    1d8a:	62 1b       	sub	r22, r18
    1d8c:	73 0b       	sbc	r23, r19
    1d8e:	84 0b       	sbc	r24, r20
    1d90:	95 0b       	sbc	r25, r21
    1d92:	39 f4       	brne	.+14     	; 0x1da2 <__fp_cmp+0x38>
    1d94:	0a 26       	eor	r0, r26
    1d96:	61 f0       	breq	.+24     	; 0x1db0 <__fp_cmp+0x46>
    1d98:	23 2b       	or	r18, r19
    1d9a:	24 2b       	or	r18, r20
    1d9c:	25 2b       	or	r18, r21
    1d9e:	21 f4       	brne	.+8      	; 0x1da8 <__fp_cmp+0x3e>
    1da0:	08 95       	ret
    1da2:	0a 26       	eor	r0, r26
    1da4:	09 f4       	brne	.+2      	; 0x1da8 <__fp_cmp+0x3e>
    1da6:	a1 40       	sbci	r26, 0x01	; 1
    1da8:	a6 95       	lsr	r26
    1daa:	8f ef       	ldi	r24, 0xFF	; 255
    1dac:	81 1d       	adc	r24, r1
    1dae:	81 1d       	adc	r24, r1
    1db0:	08 95       	ret

00001db2 <__fp_inf>:
    1db2:	97 f9       	bld	r25, 7
    1db4:	9f 67       	ori	r25, 0x7F	; 127
    1db6:	80 e8       	ldi	r24, 0x80	; 128
    1db8:	70 e0       	ldi	r23, 0x00	; 0
    1dba:	60 e0       	ldi	r22, 0x00	; 0
    1dbc:	08 95       	ret

00001dbe <__fp_nan>:
    1dbe:	9f ef       	ldi	r25, 0xFF	; 255
    1dc0:	80 ec       	ldi	r24, 0xC0	; 192
    1dc2:	08 95       	ret

00001dc4 <__fp_pscA>:
    1dc4:	00 24       	eor	r0, r0
    1dc6:	0a 94       	dec	r0
    1dc8:	16 16       	cp	r1, r22
    1dca:	17 06       	cpc	r1, r23
    1dcc:	18 06       	cpc	r1, r24
    1dce:	09 06       	cpc	r0, r25
    1dd0:	08 95       	ret

00001dd2 <__fp_pscB>:
    1dd2:	00 24       	eor	r0, r0
    1dd4:	0a 94       	dec	r0
    1dd6:	12 16       	cp	r1, r18
    1dd8:	13 06       	cpc	r1, r19
    1dda:	14 06       	cpc	r1, r20
    1ddc:	05 06       	cpc	r0, r21
    1dde:	08 95       	ret

00001de0 <__fp_round>:
    1de0:	09 2e       	mov	r0, r25
    1de2:	03 94       	inc	r0
    1de4:	00 0c       	add	r0, r0
    1de6:	11 f4       	brne	.+4      	; 0x1dec <__fp_round+0xc>
    1de8:	88 23       	and	r24, r24
    1dea:	52 f0       	brmi	.+20     	; 0x1e00 <__fp_round+0x20>
    1dec:	bb 0f       	add	r27, r27
    1dee:	40 f4       	brcc	.+16     	; 0x1e00 <__fp_round+0x20>
    1df0:	bf 2b       	or	r27, r31
    1df2:	11 f4       	brne	.+4      	; 0x1df8 <__fp_round+0x18>
    1df4:	60 ff       	sbrs	r22, 0
    1df6:	04 c0       	rjmp	.+8      	; 0x1e00 <__fp_round+0x20>
    1df8:	6f 5f       	subi	r22, 0xFF	; 255
    1dfa:	7f 4f       	sbci	r23, 0xFF	; 255
    1dfc:	8f 4f       	sbci	r24, 0xFF	; 255
    1dfe:	9f 4f       	sbci	r25, 0xFF	; 255
    1e00:	08 95       	ret

00001e02 <__fp_split3>:
    1e02:	57 fd       	sbrc	r21, 7
    1e04:	90 58       	subi	r25, 0x80	; 128
    1e06:	44 0f       	add	r20, r20
    1e08:	55 1f       	adc	r21, r21
    1e0a:	59 f0       	breq	.+22     	; 0x1e22 <__fp_splitA+0x10>
    1e0c:	5f 3f       	cpi	r21, 0xFF	; 255
    1e0e:	71 f0       	breq	.+28     	; 0x1e2c <__fp_splitA+0x1a>
    1e10:	47 95       	ror	r20

00001e12 <__fp_splitA>:
    1e12:	88 0f       	add	r24, r24
    1e14:	97 fb       	bst	r25, 7
    1e16:	99 1f       	adc	r25, r25
    1e18:	61 f0       	breq	.+24     	; 0x1e32 <__fp_splitA+0x20>
    1e1a:	9f 3f       	cpi	r25, 0xFF	; 255
    1e1c:	79 f0       	breq	.+30     	; 0x1e3c <__fp_splitA+0x2a>
    1e1e:	87 95       	ror	r24
    1e20:	08 95       	ret
    1e22:	12 16       	cp	r1, r18
    1e24:	13 06       	cpc	r1, r19
    1e26:	14 06       	cpc	r1, r20
    1e28:	55 1f       	adc	r21, r21
    1e2a:	f2 cf       	rjmp	.-28     	; 0x1e10 <__fp_split3+0xe>
    1e2c:	46 95       	lsr	r20
    1e2e:	f1 df       	rcall	.-30     	; 0x1e12 <__fp_splitA>
    1e30:	08 c0       	rjmp	.+16     	; 0x1e42 <__fp_splitA+0x30>
    1e32:	16 16       	cp	r1, r22
    1e34:	17 06       	cpc	r1, r23
    1e36:	18 06       	cpc	r1, r24
    1e38:	99 1f       	adc	r25, r25
    1e3a:	f1 cf       	rjmp	.-30     	; 0x1e1e <__fp_splitA+0xc>
    1e3c:	86 95       	lsr	r24
    1e3e:	71 05       	cpc	r23, r1
    1e40:	61 05       	cpc	r22, r1
    1e42:	08 94       	sec
    1e44:	08 95       	ret

00001e46 <__fp_zero>:
    1e46:	e8 94       	clt

00001e48 <__fp_szero>:
    1e48:	bb 27       	eor	r27, r27
    1e4a:	66 27       	eor	r22, r22
    1e4c:	77 27       	eor	r23, r23
    1e4e:	cb 01       	movw	r24, r22
    1e50:	97 f9       	bld	r25, 7
    1e52:	08 95       	ret

00001e54 <__gesf2>:
    1e54:	8a df       	rcall	.-236    	; 0x1d6a <__fp_cmp>
    1e56:	08 f4       	brcc	.+2      	; 0x1e5a <__gesf2+0x6>
    1e58:	8f ef       	ldi	r24, 0xFF	; 255
    1e5a:	08 95       	ret

00001e5c <__mulsf3>:
    1e5c:	0b d0       	rcall	.+22     	; 0x1e74 <__mulsf3x>
    1e5e:	c0 cf       	rjmp	.-128    	; 0x1de0 <__fp_round>
    1e60:	b1 df       	rcall	.-158    	; 0x1dc4 <__fp_pscA>
    1e62:	28 f0       	brcs	.+10     	; 0x1e6e <__mulsf3+0x12>
    1e64:	b6 df       	rcall	.-148    	; 0x1dd2 <__fp_pscB>
    1e66:	18 f0       	brcs	.+6      	; 0x1e6e <__mulsf3+0x12>
    1e68:	95 23       	and	r25, r21
    1e6a:	09 f0       	breq	.+2      	; 0x1e6e <__mulsf3+0x12>
    1e6c:	a2 cf       	rjmp	.-188    	; 0x1db2 <__fp_inf>
    1e6e:	a7 cf       	rjmp	.-178    	; 0x1dbe <__fp_nan>
    1e70:	11 24       	eor	r1, r1
    1e72:	ea cf       	rjmp	.-44     	; 0x1e48 <__fp_szero>

00001e74 <__mulsf3x>:
    1e74:	c6 df       	rcall	.-116    	; 0x1e02 <__fp_split3>
    1e76:	a0 f3       	brcs	.-24     	; 0x1e60 <__mulsf3+0x4>

00001e78 <__mulsf3_pse>:
    1e78:	95 9f       	mul	r25, r21
    1e7a:	d1 f3       	breq	.-12     	; 0x1e70 <__mulsf3+0x14>
    1e7c:	95 0f       	add	r25, r21
    1e7e:	50 e0       	ldi	r21, 0x00	; 0
    1e80:	55 1f       	adc	r21, r21
    1e82:	62 9f       	mul	r22, r18
    1e84:	f0 01       	movw	r30, r0
    1e86:	72 9f       	mul	r23, r18
    1e88:	bb 27       	eor	r27, r27
    1e8a:	f0 0d       	add	r31, r0
    1e8c:	b1 1d       	adc	r27, r1
    1e8e:	63 9f       	mul	r22, r19
    1e90:	aa 27       	eor	r26, r26
    1e92:	f0 0d       	add	r31, r0
    1e94:	b1 1d       	adc	r27, r1
    1e96:	aa 1f       	adc	r26, r26
    1e98:	64 9f       	mul	r22, r20
    1e9a:	66 27       	eor	r22, r22
    1e9c:	b0 0d       	add	r27, r0
    1e9e:	a1 1d       	adc	r26, r1
    1ea0:	66 1f       	adc	r22, r22
    1ea2:	82 9f       	mul	r24, r18
    1ea4:	22 27       	eor	r18, r18
    1ea6:	b0 0d       	add	r27, r0
    1ea8:	a1 1d       	adc	r26, r1
    1eaa:	62 1f       	adc	r22, r18
    1eac:	73 9f       	mul	r23, r19
    1eae:	b0 0d       	add	r27, r0
    1eb0:	a1 1d       	adc	r26, r1
    1eb2:	62 1f       	adc	r22, r18
    1eb4:	83 9f       	mul	r24, r19
    1eb6:	a0 0d       	add	r26, r0
    1eb8:	61 1d       	adc	r22, r1
    1eba:	22 1f       	adc	r18, r18
    1ebc:	74 9f       	mul	r23, r20
    1ebe:	33 27       	eor	r19, r19
    1ec0:	a0 0d       	add	r26, r0
    1ec2:	61 1d       	adc	r22, r1
    1ec4:	23 1f       	adc	r18, r19
    1ec6:	84 9f       	mul	r24, r20
    1ec8:	60 0d       	add	r22, r0
    1eca:	21 1d       	adc	r18, r1
    1ecc:	82 2f       	mov	r24, r18
    1ece:	76 2f       	mov	r23, r22
    1ed0:	6a 2f       	mov	r22, r26
    1ed2:	11 24       	eor	r1, r1
    1ed4:	9f 57       	subi	r25, 0x7F	; 127
    1ed6:	50 40       	sbci	r21, 0x00	; 0
    1ed8:	8a f0       	brmi	.+34     	; 0x1efc <__mulsf3_pse+0x84>
    1eda:	e1 f0       	breq	.+56     	; 0x1f14 <__mulsf3_pse+0x9c>
    1edc:	88 23       	and	r24, r24
    1ede:	4a f0       	brmi	.+18     	; 0x1ef2 <__mulsf3_pse+0x7a>
    1ee0:	ee 0f       	add	r30, r30
    1ee2:	ff 1f       	adc	r31, r31
    1ee4:	bb 1f       	adc	r27, r27
    1ee6:	66 1f       	adc	r22, r22
    1ee8:	77 1f       	adc	r23, r23
    1eea:	88 1f       	adc	r24, r24
    1eec:	91 50       	subi	r25, 0x01	; 1
    1eee:	50 40       	sbci	r21, 0x00	; 0
    1ef0:	a9 f7       	brne	.-22     	; 0x1edc <__mulsf3_pse+0x64>
    1ef2:	9e 3f       	cpi	r25, 0xFE	; 254
    1ef4:	51 05       	cpc	r21, r1
    1ef6:	70 f0       	brcs	.+28     	; 0x1f14 <__mulsf3_pse+0x9c>
    1ef8:	5c cf       	rjmp	.-328    	; 0x1db2 <__fp_inf>
    1efa:	a6 cf       	rjmp	.-180    	; 0x1e48 <__fp_szero>
    1efc:	5f 3f       	cpi	r21, 0xFF	; 255
    1efe:	ec f3       	brlt	.-6      	; 0x1efa <__mulsf3_pse+0x82>
    1f00:	98 3e       	cpi	r25, 0xE8	; 232
    1f02:	dc f3       	brlt	.-10     	; 0x1efa <__mulsf3_pse+0x82>
    1f04:	86 95       	lsr	r24
    1f06:	77 95       	ror	r23
    1f08:	67 95       	ror	r22
    1f0a:	b7 95       	ror	r27
    1f0c:	f7 95       	ror	r31
    1f0e:	e7 95       	ror	r30
    1f10:	9f 5f       	subi	r25, 0xFF	; 255
    1f12:	c1 f7       	brne	.-16     	; 0x1f04 <__mulsf3_pse+0x8c>
    1f14:	fe 2b       	or	r31, r30
    1f16:	88 0f       	add	r24, r24
    1f18:	91 1d       	adc	r25, r1
    1f1a:	96 95       	lsr	r25
    1f1c:	87 95       	ror	r24
    1f1e:	97 f9       	bld	r25, 7
    1f20:	08 95       	ret

00001f22 <__unordsf2>:
    1f22:	23 df       	rcall	.-442    	; 0x1d6a <__fp_cmp>
    1f24:	88 0b       	sbc	r24, r24
    1f26:	99 0b       	sbc	r25, r25
    1f28:	08 95       	ret

00001f2a <__udivmodsi4>:
    1f2a:	a1 e2       	ldi	r26, 0x21	; 33
    1f2c:	1a 2e       	mov	r1, r26
    1f2e:	aa 1b       	sub	r26, r26
    1f30:	bb 1b       	sub	r27, r27
    1f32:	fd 01       	movw	r30, r26
    1f34:	0d c0       	rjmp	.+26     	; 0x1f50 <__udivmodsi4_ep>

00001f36 <__udivmodsi4_loop>:
    1f36:	aa 1f       	adc	r26, r26
    1f38:	bb 1f       	adc	r27, r27
    1f3a:	ee 1f       	adc	r30, r30
    1f3c:	ff 1f       	adc	r31, r31
    1f3e:	a2 17       	cp	r26, r18
    1f40:	b3 07       	cpc	r27, r19
    1f42:	e4 07       	cpc	r30, r20
    1f44:	f5 07       	cpc	r31, r21
    1f46:	20 f0       	brcs	.+8      	; 0x1f50 <__udivmodsi4_ep>
    1f48:	a2 1b       	sub	r26, r18
    1f4a:	b3 0b       	sbc	r27, r19
    1f4c:	e4 0b       	sbc	r30, r20
    1f4e:	f5 0b       	sbc	r31, r21

00001f50 <__udivmodsi4_ep>:
    1f50:	66 1f       	adc	r22, r22
    1f52:	77 1f       	adc	r23, r23
    1f54:	88 1f       	adc	r24, r24
    1f56:	99 1f       	adc	r25, r25
    1f58:	1a 94       	dec	r1
    1f5a:	69 f7       	brne	.-38     	; 0x1f36 <__udivmodsi4_loop>
    1f5c:	60 95       	com	r22
    1f5e:	70 95       	com	r23
    1f60:	80 95       	com	r24
    1f62:	90 95       	com	r25
    1f64:	9b 01       	movw	r18, r22
    1f66:	ac 01       	movw	r20, r24
    1f68:	bd 01       	movw	r22, r26
    1f6a:	cf 01       	movw	r24, r30
    1f6c:	08 95       	ret

00001f6e <__tablejump2__>:
    1f6e:	ee 0f       	add	r30, r30
    1f70:	ff 1f       	adc	r31, r31

00001f72 <__tablejump__>:
    1f72:	05 90       	lpm	r0, Z+
    1f74:	f4 91       	lpm	r31, Z
    1f76:	e0 2d       	mov	r30, r0
    1f78:	09 94       	ijmp

00001f7a <__muluhisi3>:
    1f7a:	0e 94 c8 0f 	call	0x1f90	; 0x1f90 <__umulhisi3>
    1f7e:	a5 9f       	mul	r26, r21
    1f80:	90 0d       	add	r25, r0
    1f82:	b4 9f       	mul	r27, r20
    1f84:	90 0d       	add	r25, r0
    1f86:	a4 9f       	mul	r26, r20
    1f88:	80 0d       	add	r24, r0
    1f8a:	91 1d       	adc	r25, r1
    1f8c:	11 24       	eor	r1, r1
    1f8e:	08 95       	ret

00001f90 <__umulhisi3>:
    1f90:	a2 9f       	mul	r26, r18
    1f92:	b0 01       	movw	r22, r0
    1f94:	b3 9f       	mul	r27, r19
    1f96:	c0 01       	movw	r24, r0
    1f98:	a3 9f       	mul	r26, r19
    1f9a:	70 0d       	add	r23, r0
    1f9c:	81 1d       	adc	r24, r1
    1f9e:	11 24       	eor	r1, r1
    1fa0:	91 1d       	adc	r25, r1
    1fa2:	b2 9f       	mul	r27, r18
    1fa4:	70 0d       	add	r23, r0
    1fa6:	81 1d       	adc	r24, r1
    1fa8:	11 24       	eor	r1, r1
    1faa:	91 1d       	adc	r25, r1
    1fac:	08 95       	ret

00001fae <_exit>:
    1fae:	f8 94       	cli

00001fb0 <__stop_program>:
    1fb0:	ff cf       	rjmp	.-2      	; 0x1fb0 <__stop_program>
