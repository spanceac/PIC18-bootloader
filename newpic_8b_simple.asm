; TODO INSERT CONFIG CODE HERE USING CONFIG BITS GENERATOR
#include "p18f25k22.inc"

radix dec ; decimal values used in code, unless expressed otherwise

max_flash equ 0x7ffe
end_flash_hi equ (max_flash >> 8)
end_flash_lo equ (max_flash & 0x00ff) - 2
jmp_to_user_code_addr_hi equ 0x0
jmp_to_user_code_addr_lo equ 0x4 
btld_code_start equ max_flash - 0x240
btld_code_start_hi equ (btld_code_start >> 8)
btld_code_start_lo equ (btld_code_start & 0x00ff)
user_prog_start equ 0x4
NEW_LINE equ 0x0a

cblock 0x0
recv_line:50
fl_write_buf:16
flash_addr_hi
flash_addr_lo
data_bytes_nr
data_bytes_nr_copy
ck_sum_calc
ck_sum_from_hex
record_type
count
local_count
data_ended
extended_rec_started
temp_data_byte
flash_erase_addr_hi
flash_erase_addr_lo
first_jmp_inst_buf:4
count2
flash_addr_zone
endc
    
  CONFIG  FOSC = INTIO67        ; Oscillator Selection bits (Internal oscillator block)
  CONFIG  PLLCFG = ON           ; 4X PLL Enable (Oscillator multiplied by 4)
  CONFIG  PRICLKEN = ON         ; Primary clock enable bit (Primary clock enabled)
  CONFIG  WDTEN = OFF
  CONFIG  IESO = OFF

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    StartBtld                   ; go to beginning of program

; TODO ADD INTERRUPTS HERE IF USED
    
org btld_code_start

uart_init:
    bcf TRISC, TRISC6 ; UART TX
    bsf TRISC, TRISC7 ; UART RX
    banksel ANSELC
    bcf ANSELC, ANSC7
    movlb 0 ; select bank 0
    bcf TXSTA1, SYNC
    bsf TXSTA1, TXEN
    bsf RCSTA1, SPEN
    bsf RCSTA1, CREN
    movlw 8
    movwf SPBRG1 ; 115200 baudrate
    ;;; let's make ourselves heard
    movlw 'W'
    movwf TXREG1
    btfss TXSTA1, TRMT
    goto $-2
    return

xoff:
    movlw 0x13
    movwf TXREG1
    btfss TXSTA1, TRMT
    goto $-2
    return

xon:
    movlw 0x11
    movwf TXREG1
    btfss TXSTA1, TRMT
    goto $-2
    return

receive_line:
    btfss PIR1, RC1IF
    goto $-2
    
    movf RCREG1, W
    ; test if received a line feed
    xorlw NEW_LINE
    bz exit_flow ; if not start, take normal flow
    
normal_flow
    movf RCREG1, W ; put UART rx data in W
    movwf POSTINC0 ; put UART rx data in recv_line buffer
    incf count
    goto receive_line

exit_flow
    ;call xoff ; stop receive, time to flash
    return

ascii2hex_line:
    movf count, w
    movwf local_count ; create a copy of count in local_count
convert_next_byte
    movlw 48
    subwf INDF0 ; subtract 48 from number
    movlw 10
    cpfslt INDF0 ; if nr is bigger than 9, then is in A - F range
    goto continue_for_a_f
    goto continue_for_0_9
continue_for_a_f
    movlw 7
    subwf INDF0 ; subtract 7 from number
continue_for_0_9
    movf POSTINC0, w ; increment FSR
    decf local_count
    bnz convert_next_byte
exit_conversion
    return

extract_write_data:
    clrf data_bytes_nr
    clrf flash_addr_hi
    clrf flash_addr_lo
    clrf ck_sum_calc
    clrf record_type
    
    lfsr FSR1, fl_write_buf ; point FSR1 to write buffer
    lfsr FSR0, recv_line
    movf POSTINC0, w ; go past start char ':'
    
    ;;; get nr of data bytes
    movf POSTINC0, w ; first byte of data_bytes_nr
    movwf data_bytes_nr
    swapf data_bytes_nr
    movf POSTINC0, w ; second byte of data_bytes_nr
    iorwf data_bytes_nr
    movff data_bytes_nr, ck_sum_calc
    
    ;;; get flash addres high and low
    movf POSTINC0, w ; first byte of flash_addr_hi
    movwf flash_addr_hi
    swapf flash_addr_hi
    movf POSTINC0, w ; second byte of flash_addr_hi
    iorwf flash_addr_hi
    movf flash_addr_hi, w
    addwf ck_sum_calc, f
    movf POSTINC0, w ; first byte of flash_addr_lo
    movwf flash_addr_lo
    swapf flash_addr_lo
    movf POSTINC0, w ; second byte of flash_addr_lo
    iorwf flash_addr_lo
    movf flash_addr_lo, w
    addwf ck_sum_calc, f
    
    ;; get record type
    movf POSTINC0, w
    movf POSTINC0, w ; get record type
    movwf record_type
    movf record_type, w
    addwf ck_sum_calc, f

    ;;; create a copy of data_bytes_nr, to use in copy loop
    movf data_bytes_nr, w
    movwf data_bytes_nr_copy
    bnz write_buf_fill ; if data_bytes_nr is not zero, fill write buffer
    return
write_buf_fill    
    ;;; copy the data to write from receive buffer to write buffer
    movf POSTINC0, w ; first byte of data to write
    movwf temp_data_byte
    swapf temp_data_byte
    movf POSTINC0, w ; second byte of data to write
    iorwf temp_data_byte
   
    movf temp_data_byte, w
    movwf POSTINC1 ; copy to write buffer and increment buffer address
    addwf ck_sum_calc, f
    
    decfsz data_bytes_nr_copy ;;; repeat for all
    bra write_buf_fill
    
    movf POSTINC0, w ; first byte of ck_sum_from_hex
    movwf ck_sum_from_hex
    swapf ck_sum_from_hex
    movf POSTINC0, w ; second byte of ck_sum_from_hex
    iorwf ck_sum_from_hex
    movf ck_sum_from_hex, w
    addwf ck_sum_calc
    bnz ck_sum_err
    return
ck_sum_err
    ; write to UART we have checksum error and reset
    movlw 'E'
    movwf TXREG1
    btfss TXSTA1, TRMT
    goto $-2
    reset
    
erase_flash:
    ;;; test to not overwrite the bootloader
    movlw btld_code_start_hi
    cpfseq flash_erase_addr_hi ; if high part of address is not 0x7d we can continue
    return ; reached address of bootloader, erase procedure ended

continue_erase
    clrf TBLPTRU
    movf flash_erase_addr_hi , w
    movwf TBLPTRH ; load flash address high
    movf flash_erase_addr_lo, w
    movwf TBLPTRL ; load flash address low
    bsf EECON1, EEPGD
    bcf EECON1, CFGS
    bsf EECON1, WREN
    bsf EECON1, FREE
    movlw 55h
    movwf EECON2
    movlw 0xaa
    movwf EECON2
    bsf EECON1, WR   
    
    movlw 64
    addwf flash_erase_addr_lo, f ; add 64 to the flash address to erase next block
    bnc erase_flash
    incf flash_erase_addr_hi ; increase also the high bytes if carry from previous addition
    goto erase_flash
    
write_flash:
    ;;; start the hw flashing procedure
    bsf EECON1, EEPGD ; point to Flash memory
    bcf EECON1, CFGS ; acces Flash program memory
    bsf EECON1, WREN ; enable write to memory
    movlw 0x55
    movwf EECON2
    movlw 0xaa
    movwf EECON2
    bsf EECON1, WR ; start program (CPU stall until done)
    return

flash_line:
    ;;; test for recored type
    tstfsz record_type
    bra record_not_zero
    
    ;;; is the addr 0?, if not -> continue flashing, no need for erase or address change
    tstfsz flash_addr_hi
    bra flash_wr_buf
    tstfsz flash_addr_lo
    bra flash_wr_buf
    
    ;; let's see if this could be part of an extended record
    ;; we don't flash extended records
    movf extended_rec_started, w
    xorlw 1
    btfsc STATUS, Z
    return ;; if this is a part of an extended record, return
    
    ;; if address is zero, we are at start and we need to erase the flash
    ;; save first jump to bootloader instructions before erasing flash
    lfsr FSR2, first_jmp_inst_buf
    clrf TBLPTRU
    clrf TBLPTRL
    clrf TBLPTRH
    movlw 4
    movwf count2
    
repeat_read    
    TBLRD*+
    movf TABLAT, w
    movwf POSTINC2
    decfsz count2
    goto repeat_read
    
    call erase_flash ; erase the flash
    
    ;; write back the first jump to bootloader instruction
    clrf TBLPTRL
    clrf TBLPTRH
    lfsr FSR2, first_jmp_inst_buf
    movlw 4
    movwf count2
    
repeat_buf_fill
    movf POSTINC2, w
    movwf TABLAT
    TBLWT*+
    decfsz count2
    goto repeat_buf_fill
    TBLRD*- ; point the TBLPTR to the last written address, otherwise crazy things happen
    call write_flash
    
    ;;; addresses are 0, we need to put the goto instruction of the user program at the address 4
    movlw jmp_to_user_code_addr_hi
    movwf flash_addr_hi
    movlw jmp_to_user_code_addr_lo
    movwf flash_addr_lo
    
flash_wr_buf
    lfsr FSR1, fl_write_buf ; point FSR1 to write buffer
    clrf TBLPTRU
    movf flash_addr_hi , w
    movwf TBLPTRH ; load flash address high
    movf flash_addr_lo, w
    movwf TBLPTRL ; load flash address low
    movwf flash_addr_zone
    movlw 0xc0
    andwf flash_addr_zone, f

fill_holding_regs    
    ;;; here comes the real flashing part ... YAAAY!!!
    movf POSTINC1, w ; put byte from write buffer in w and increment buffer addr
    movwf TABLAT ; put data in write latch
    TBLWT*+ ; put data in holding register and increment the table

    ; verify if bits 7:6 in TBLTPTRL changed -> address range change
    movf TBLPTRL, w
    andlw 0xc0
    xorwf flash_addr_zone, w
    bz no_addr_zone_change
    movlw 1
    cpfseq data_bytes_nr ; if TBLPTRL bits 7:6 changed && data_bytes_nr = 1 -> last byte -> no addr range change
    bra addr_zone_has_change

no_addr_zone_change
    decf data_bytes_nr
    bnz fill_holding_regs
    TBLRD*- ; point the TBLPTR to the last written address, otherwise crazy things happen
    call write_flash
    return

addr_zone_has_change
    TBLRD*- ; point the TBLPTR to the last written address, otherwise crazy things happen
    call write_flash ; flash the first part
    decf data_bytes_nr
    ; now flash the remaining part
    TBLRD*+ ; increment TBLPTR to the new values
    goto fill_holding_regs

record_not_zero
    movlw 1
    xorwf record_type, w
    ;;; if record is 1 then data ended, otherwise record is 4 -> extended record
    bnz record_is_extended
    bsf data_ended, 0
    return
    
record_is_extended
    ;;; is the flash addr 0?, if yes -> don't set the extended_rec_started flag
    tstfsz flash_addr_hi
    return
    tstfsz flash_addr_lo
    return
    movlw 1
    movwf extended_rec_started
    return

StartBtld
    movlw 0x70
    iorwf OSCCON ; select 16 MHz internal oscillator
    bsf OSCTUNE, PLLEN
    bsf TRISB, RB5
    banksel ANSELB
    bcf ANSELB, ANSB5
    btfsc PORTB, RB5 ; RB5 low --> goto bootloader, RB5 high --> goto user code
    goto user_prog_start
    lfsr FSR0, recv_line
    clrf flash_erase_addr_hi
    clrf flash_erase_addr_lo
    clrf data_ended
    clrf count
    clrf extended_rec_started
    btfss OSCCON2, PLLRDY ; wait for PLL to become active
    goto $-2
    call uart_init
    
receive_and_flash
    call receive_line
    lfsr FSR0, recv_line
    call ascii2hex_line
    call extract_write_data
    call flash_line
    clrf count ; clear count
    lfsr FSR0, recv_line 
    call xon ; re-start receiving
    btfss data_ended, 0
    goto receive_and_flash
    goto user_prog_start ; flashing is done, start user program

    END