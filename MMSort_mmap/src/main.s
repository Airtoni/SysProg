@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@@@     main.s
@@@ ---------------------------------------------------------------------------
@@@     author:  ...
@@@     target:  Raspberry Pi
@@@     project: MM-Sorting-Machine
@@@     date:    YYYY/MM/DD
@@@     version: ...
@@@ ---------------------------------------------------------------------------
@@@ This program controls the MM-Sorting-Machine by reading two inputs,
@@@ controlling the motors(, serving the 7-segment display) and interacting
@@@ with the co-processor.
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

@ Constants for assembler
@ The following are defined in /usr/include/asm-generic/fcntl.h:
@ Note that the values are specified in octal.
        .equ      O_RDWR,00000002             @ open for read/write
        .equ      O_DSYNC,00010000            @ synchronize virtual memory
        .equ      __O_SYNC,04000000           @      programming changes with
        .equ      O_SYNC,__O_SYNC|O_DSYNC     @ I/O memory
@ The following are defined in /usr/include/asm-generic/mman-common.h:
        .equ      PROT_READ,0x1               @ page can be read
        .equ      PROT_WRITE,0x2              @ page can be written
        .equ      MAP_SHARED,0x01             @ share changes
@ The following are defined by me:
@        .equ      PERIPH,0x3f000000           @ RPi 2 & 3 peripherals
        .equ      PERIPH,0x20000000           @ RPi zero & 1 peripherals
        .equ      GPIO_OFFSET,0x200000        @ start of GPIO device
        .equ      TIMERIR_OFFSET,0xB000       @ start f´of IR and timer
        .equ      O_FLAGS,O_RDWR|O_SYNC       @ open file flags
        .equ      PROT_RDWR,PROT_READ|PROT_WRITE
        .equ      NO_PREF,0
        .equ      PAGE_SIZE,4096              @ Raspbian memory page
        .equ      FILE_DESCRP_ARG,0           @ file descriptor
        .equ      DEVICE_ARG,4                @ device address
        .equ      STACK_ARGS,8                @ sp already 8-byte aligned

 @Teh following are defined by us
 	    .equ    PIN9,13
    .equ    PIN11,14
    .equ    PIN12,26
    .equ    PIN13,23
    .equ    PIN16,27
    .equ    PIN17,0
    .equ    PIN19,24
    .equ	PIN22,3
    .equ	PIN23,4
    .equ	PIN24,5
    .equ    PIN26,25
    .equ    PIN27,2

    .equ INT_EDGE_RISING,1


LOOPCOUNT .req    r4
TMPREG  .req      r5
RETREG  .req      r6
POSOUT  .req      r7
WAITREG .req      r8
RLDREG  .req      r9
GPIOREG .req      r10
COLREG  .req      r11



@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .data
        .balign   4

gpiomem:
        .asciz    "/dev/gpiomem"
mem:
        .asciz    "/dev/mem"
fdMsg:
        .asciz    "File descriptor = %i\n"
memMsgGpio:
        .asciz    "(GPIO) Using memory at %p\n"
memMsgTimerIR:
        .asciz    "(Timer + IR) Using memory at %p\n"

IntroMsg:
        .asciz    "Welcome to the MM-Sorting-Machine!\n"

        .balign   4
gpio_mmap_adr:
        .word     0               @ ...
gpio_mmap_fd:
        .word     0
timerir_mmap_adr:
        .word     0
timerir_mmap_fd:
        .word     0

@ - END OF DATA SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ - START OF TEXT SECTION @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        .text

        @ externals for making use of std-functions
        .extern printf

        @ externals for making use of wiringPI
        .extern wiringPiSetup
        .extern delay
        .extern digitalWrite
        .extern pinMode
        .extern wiringPiISR


        @ externals for RGB LEDs
        .extern WS2812RPi_Init
        .extern WS2812RPi_DeInit
        .extern WS2812RPi_SetBrightness       @ provide (uint8_t brightness);
        .extern WS2812RPi_SetSingle           @ provide (uint8_t pos, uint32_t color);
        .extern WS2812RPi_SetOthersOff        @ provide (uint8_t pos);
        .extern WS2812RPi_AllOff              @ provide (void);
        .extern WS2812RPi_AnimDo              @ provide (uint32_t cntCycles);
        .extern WS2812RPi_Show

        .balign   4
        .global   main
        .type     main, %function
@ -----------------------------------------------------------------------------
@ main entry point of the application
@   param:     none
@   return:    none
@ -----------------------------------------------------------------------------
main:
        ldr r0, =IntroMsg
        bl  printf

        @ GET GPIO VIRTUAL MEMORY ---------------------------------------------
        @ create backup and reserve stack space
        sub       sp, sp, #16                 @ space for saving regs
        str       r4, [sp, #0]                @ save r4
        str       r5, [sp, #4]                @      r5
        str       fp, [sp, #8]                @      fp
        str       lr, [sp, #12]               @      lr
        add       fp, sp, #12                 @ set our frame pointer
        sub       sp, sp, #STACK_ARGS         @ sp on 8-byte boundary

        @ open /dev/gpiomem for read/write and syncing
        ldr       r0, =gpiomem                 @ address of /dev/gpiomem
        ldr       r1, openMode                @ flags for accessing device
        bl        open
        mov       r4, r0                      @ use r4 for file descriptor

        @ display file descriptor
        ldr       r0, =fdMsg                  @ format for printf
        mov       r1, r4                      @ file descriptor
        bl        printf

        @ map the GPIO registers to a virtual memory location so we can access them
        str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/gpiomem file descriptor
        ldr       r0, gpio                    @ address of GPIO
        str       r0, [sp, #DEVICE_ARG]       @ location of GPIO
        mov       r0, #NO_PREF                @ let kernel pick memory
        mov       r1, #PAGE_SIZE              @ get 1 page of memory
        mov       r2, #PROT_RDWR              @ read/write this memory
        mov       r3, #MAP_SHARED             @ share with other processes
        bl        mmap

        @ save virtual memory address
        ldr       r1, =gpio_mmap_adr          @ store gpio mmap (virtual address)
        str       r0, [r1]
        ldr       r1, =gpio_mmap_fd           @ store the file descriptor
        str       r4, [r1]

        ldr       r6, [r1]
        mov       r1, r0                      @ display virtual address
        ldr       r0, =memMsgGpio
        bl        printf
        mov       r1, r6
        ldr       r0, =memMsgGpio
        bl        printf

        @ restore sp and free stack
        add       sp, sp, #STACK_ARGS         @ fix sp
        ldr       r4, [sp, #0]                @ restore r4
        ldr       r5, [sp, #4]                @      r5
        ldr       fp, [sp, #8]                @         fp
        ldr       lr, [sp, #12]               @         lr
        add       sp, sp, #16                 @ restore sp

        @ GET TIMER + IR VIRTUAL MEMORY ---------------------------------------
        @ create backup and reserve stack space
        sub       sp, sp, #16                 @ space for saving regs
        str       r4, [sp, #0]                @ save r4
        str       r5, [sp, #4]                @      r5
        str       fp, [sp, #8]                @      fp
        str       lr, [sp, #12]               @      lr
        add       fp, sp, #12                 @ set our frame pointer
        sub       sp, sp, #STACK_ARGS         @ sp on 8-byte boundary

        @ open /dev/gpiomem for read/write and syncing
        ldr       r0, =mem                    @ address of /dev/mem
        ldr       r1, openMode                @ flags for accessing device
        bl        open
        mov       r4, r0                      @ use r4 for file descriptor

        @ display file descriptor
        ldr       r0, =fdMsg                  @ format for printf
        mov       r1, r4                      @ file descriptor
        bl        printf

        @ map the GPIO registers to a virtual memory location so we can access them
        str       r4, [sp, #FILE_DESCRP_ARG]  @ /dev/mem file descriptor
        ldr       r0, timerIR                 @ address of timer + IR
        str       r0, [sp, #DEVICE_ARG]       @ location of timer +IR
        mov       r0, #NO_PREF                @ let kernel pick memory
        mov       r1, #PAGE_SIZE              @ get 1 page of memory
        mov       r2, #PROT_RDWR              @ read/write this memory
        mov       r3, #MAP_SHARED             @ share with other processes
        bl        mmap

        @ save virtual memory address
        ldr       r1, =timerir_mmap_adr       @ store timer + IR mmap (virtual address)
        str       r0, [r1]
        ldr       r1, =timerir_mmap_fd        @ store the file descriptor
        str       r4, [r1]

        ldr       r6, [r1]
        mov       r1, r0                      @ display virtual address
        ldr       r0, =memMsgTimerIR
        bl        printf
        mov       r1, r6
        ldr       r0, =memMsgTimerIR
        bl        printf

        @ restore sp and free stack
        add       sp, sp, #STACK_ARGS         @ fix sp
        ldr       r4, [sp, #0]                @ restore r4
        ldr       r5, [sp, #4]                @      r5
        ldr       fp, [sp, #8]                @         fp
        ldr       lr, [sp, #12]               @         lr
        add       sp, sp, #16                 @ restore sp


        @ TODO: PLEASE INIT HW HERE
        @ HINT:
        @   configuration of inputs is not necessary cause the pins are
        @   configured as inputs after reset

        @ TODO: BRANCH HERE TO YOUR APPLICATION CODE
        @ b         ...

        @ WARNING:
        @   call "end_of_app" if you're done with your application


        bl          wiringPiSetup
        bl          hw_init                 @ calling other hardware initalize
        bl          led_init                @ calling led initalize

        bl          coproc_wakeup           @ getting the coprocessor out of sleep mode
        bl          motor_wakeup            @ getting the motors ready to work

        bl          go_default_color_wheel  @ color wheel searches for the next magnet and stops
        bl          go_default_outlet       @ outlet moves until it finds the magnet

        mov         r0, #5                  @ setting r0 to 5 to go throw the sorting 5 times
        bl          start_feeder            @ starting the feeder

main_loop:

        bl          rotate_color_wheel_90_degrees @rotate M&M in front of color sensor
        bl 			go_default_color_wheel
        ldr         r0,=33333333               @ Wait for 1s ??
        bl          wait

        bl          read_color
        bl          rotate_outlet_to_color
     @   bl rotate_color_wheel_90_degrees

        b          main_loop

        @push        {r0}                    @ moveing r0 to the stack because this register is used at other places

        @bl          rotate_color_wheel      @ rotates the color wheel by 90° to get an object infront of the color detector
       @ bl          read_color              @ reading the color detector result is stored in r0 as a number in range 1-6
       @ mov         r1, #67                 @ ca. 67 steps per glass bowl
        @mul         r2, r0, r1              @ multiply bowl number by steps per bowl to get the goal positon
        @mov         r0, r2

  @     bl          go_default_outlet       @ outlet moves to default to simplefy the algorythem

rotate_loop:
        cmp         r0, POSOUT
        push        {r0}                    @ saveing the goal position on the stack
        blne        step_outlet             @ outlet makes one step if goal position and current position (POSOUT) are not equal
        pop         {r0}                    @ getting the goal position from the stack
        bne         rotate_loop             @ jumps back if goal position and current position are not equal

        pop         {r0}
        sub         r0, r0, #1              @ decrementing r0 to create a for loop: start = 5 end = 0 decremanting
        cmp         r0, #1
        bleq        stop_feeder             @ stops the noisy feeder before going trew the last sort
        cmp         r0, #0
        bne         main_loop

        b           end_of_app

@read Color
read_color:
        push        {lr}
    @    mov			r0,#PIN22
     @   bl			digitalRead
      @  mov			COLREG,r1

       @ mov			r0,#PIN23
        @bl 			digitalRead
        @lsl			r1,#1
        @orr			COLREG,COLREG,r1


        @mov			r0,#PIN24
        @bl 			digitalRead
        @lsl			r1,#2
      @  orr			COLREG,COLREG,r1




        mov         COLREG,#0b111
        lsl         COLREG,#22

        ldr         r2,[GPIOREG,#0x34]
        and         COLREG,r2,COLREG

        lsr         COLREG,#22

	pop         {lr}
        bx          lr

@Read Pin in r0 and returns in r1
digitalRead:
        mov         r1,#1
        lsl         r1,r0

        ldr         r2,[GPIOREG,#0x34]
        and         r1,r2,r1
        lsr         r1,r0

@ waits three times the tick count privided in r0
wait:
        subs        r0, r0, #1
        cmp         r0, #0
        bne         wait
        bx          lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@               FEEDER CONTROL
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


@ starts the feeder
start_feeder:
        push        {lr}
        mov         r0, #PIN19
        mov         r1, #1
        bl          digitalWrite
        pop         {lr}
        bx          lr

@ starts the feeder
stop_feeder:
        push        {lr}
        mov         r0, #PIN19
        mov         r1, #0
        bl          digitalWrite
        pop         {lr}
        bx          lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@               COLOR WHEEL CONTROL
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

rotate_color_wheel_90_degrees: @Set POSOUT to 0 before starting
        push        {lr}
        mov          LOOPCOUNT,#0
rotate_cw_loop:
        bl          step_color_wheel
        add         LOOPCOUNT, LOOPCOUNT, #1
        cmp         LOOPCOUNT, #400
        bne         rotate_cw_loop

        pop          {lr}
        bx            lr


@ color_wheel steps one time, takes about 1ms
step_color_wheel:
        push        {lr}                    @ saveing lr for more branches
        mov         r0, #PIN13              @ set StepCW to HIGH
        mov         r1, #1
        bl          digitalWrite

        mov         r0, #0x51000            @ set r0 to ca 200 000d = 31000h   3 Zeros in the at are rounded to be able to just shift the value to the right position
        bl          wait

        mov         r0, #PIN13              @ set StepCW to LOW
        mov         r1, #0
        bl          digitalWrite

        mov         r0, #0x51000            @ set r0 to ca 200 000 as number of cycles to wait
        bl          wait
        pop         {lr}                    @ restores lr
        bx          lr

go_default_color_wheel:
        push        {lr}

        @mov         r0,#0

        @ read hallsensor of color wheel, connected to PIN20
        mov         r0,#20
        bl digitalRead

        cmp         r1, #0
        beq         end_go_default@ exit go_to_default if hall sensor is 1

        bl          step_color_wheel
        pop         {lr}
        b           go_default_color_wheel

end_go_default:
        pop         {lr}
        bx          lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@               OUTLET CONTROL
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

rotate_outlet_to_color:
        push        {lr}

        cmp         COLREG, #0
        popeq       {pc}

        cmp 		COLREG, POSOUT
        popeq		{pc}

        cmp         COLREG, POSOUT
        bgt         rotate_forward

rotate_backward:

        mov         r0, #PIN26
        mov         r1, #0
        bl          digitalWrite

        sub         r6, POSOUT, COLREG

rotc_loop:
        bl          rotate_outlet_one_pos
        sub         r6, r6, #1
        cmp         r6, #0
        bne         rotc_loop
		mov			POSOUT,COLREG
        pop         {lr}
        bx          lr

rotate_forward:

        mov         r0, #PIN26
        mov         r1, #1
        bl          digitalWrite

        sub         r6, COLREG, POSOUT
        b           rotc_loop


rotate_outlet_one_pos:
        push        {lr}
        mov         LOOPCOUNT,#0
rotate_outlet_loop:
        bl          step_outlet
        add         LOOPCOUNT,LOOPCOUNT,#1
        cmp         LOOPCOUNT,#67
        bne         rotate_outlet_loop
        pop         {lr}
        bx          lr



@ outlet steps one time, takes about 1ms
step_outlet:
        push        {lr}                    @ saveing for more branches
        mov         r0, #PIN12              @ set StepOut to HIGH
        mov         r1, #1
        bl          digitalWrite

        mov         r0, #0x130000           @ schreibt 0x31 in r0 und shifted 12 nach links, somit steht 200 000 dec in r0
        bl          wait

        mov         r0, #PIN12              @ set StepOut to LOW
        mov         r1, #0
        bl          digitalWrite

        mov         r0, #0x130000            @ schreibt 0x31 in r0 und shifted 12 nach links, somit steht 200 000 dec in r0
        bl          wait
        pop         {lr}                    @ restores lr
        bx          lr

go_default_outlet:
        push        {lr}
        mov          POSOUT,#2
        @ read hallsensor of outlet, connected to PIN21
        mov         r0,#21
        bl          digitalRead
        cmp			r1,#1
		beq			from_outside

        mov			r0,#PIN26
        mov			r1,#0
        bl			digitalWrite

go_outside_loop:
		bl			step_outlet
		mov         r0,#21
        bl          digitalRead
        cmp         r1, #0
        beq			go_outside_loop

from_outside:
		mov			r0,#PIN26
		mov			r1,#1
		bl 			digitalWrite
go_to_first_end_loop:
		bl			step_outlet
		mov         r0,#21
        bl          digitalRead
        cmp         r1, #1
        beq			go_to_first_end_loop

        mov			LOOPCOUNT,#0
find_second_end_loop:
		bl step_outlet
		mov         r0,#21
        bl          digitalRead
        cmp         r1, #0
        add			LOOPCOUNT,LOOPCOUNT,#1
        beq			find_second_end_loop

		lsr			LOOPCOUNT,#1
		mov			r0,#PIN26
		mov			r1,#0
		bl 			digitalWrite


go_to_middle_loop:
		bl			step_outlet
		sub			LOOPCOUNT,LOOPCOUNT,#1
		cmp			LOOPCOUNT,#0
		bne			go_to_middle_loop

		pop			{lr}
		bx			lr




@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@               Initialisation
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


hw_init:

        push        {lr}

        ldr         r1, =gpio_mmap_adr      @ reload the addr for accessing the GPIOs
        ldr         GPIOREG, [r1]

        mov         r1,#1                   @ pinMode uses r0 to spesify the GPIO-Pin and r1 for input(0)/output(1)

        mov         r0,#PIN11               @ Outlet (nRSTOut)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN12               @ Outlet (StepOut)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN13               @ ColorWheel (StepCW)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN16               @ ColorWheel (DirCW)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN17               @ ColorWheel (nRSTCW)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN19               @ Feeder (GoStop)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN26               @ Outlet (DirOut)
        bl          pinMode

        mov         r1,#1
        mov         r0,#PIN27               @ Coprocessor (nSLP)
        bl          pinMode

		mov         r0,#PIN9
        mov         r1,#INT_EDGE_RISING
        ldr         r2,=end_of_app
        bl          wiringPiISR

        pop         {lr}
        bx          lr


led_init:
        push {lr}
        bl WS2812RPi_Init


        mov         r0,#1                    @Pos: 1 Farbe: Red
        ldr r1,=0xFF0000
        bl WS2812RPi_SetSingle


        mov r0,#2                    @Pos: 2 Farbe: Green
        ldr r1,=0x00FF00
        bl WS2812RPi_SetSingle


        mov r0,#3
        ldr r1,=0xFFFF00               @Pos: 3 Farbe: Yellow
        bl WS2812RPi_SetSingle


        mov r0,#4
        ldr r1,=0x0000FF                  @Pos: 4 Farbe: Blue
        bl WS2812RPi_SetSingle


        mov r0,#5                       @Pos: 5 Farbe: Orange
        ldr r1,=0xFF8000
        bl WS2812RPi_SetSingle


        mov r0,#6

        ldr r1,=0x7B3503                 @Pos: 1 Farbe: Brown
        bl WS2812RPi_SetSingle

        mov r0,#100
        bl WS2812RPi_SetBrightness


        bl WS2812RPi_Show

        pop {lr}
        bx lr

@ pin 11 und 17 auf HIGH um Motoren aufzuwecken
motor_wakeup:
        push        {lr}
        mov         r0, #PIN11
        mov         r1, #1
        bl          digitalWrite

        mov         r0, #PIN17
        mov         r1, #1
        bl          digitalWrite
        pop         {lr}
        bx          lr


@ pin 27 auf HIGH um coproc aufwecken
coproc_wakeup:
        push        {lr}
        mov         r0, #PIN27
        mov         r1, #1
        bl          digitalWrite
        pop         {lr}
        bx          lr



@ --------------------------------------------------------------------------------------------------------------------
@
@ ADDRESSES: Further definitions.
@
@ --------------------------------------------------------------------------------------------------------------------
        .balign   4
@ addresses of messages
openMode:
        .word     O_FLAGS
gpio:
        .word     PERIPH+GPIO_OFFSET
timerIR:
        .word     PERIPH+TIMERIR_OFFSET

@ --------------------------------------------------------------------------------------------------------------------
@
@ END OF APPLICATION
@
@ --------------------------------------------------------------------------------------------------------------------
end_of_app:
        ldr       r1, =gpio_mmap_adr          @ reload the addr for accessing the GPIOs
        ldr       r0, [r1]                    @ memory to unmap
        mov       r1, #PAGE_SIZE              @ amount we mapped
        bl        munmap                      @ unmap it
        ldr       r1, =gpio_mmap_fd           @ reload the addr for accessing the GPIOs
        ldr       r0, [r1]                    @ memory to unmap
        bl        close                       @ close the file

        ldr       r1, =timerir_mmap_adr       @ reload the addr for accessing the Timer + IR
        ldr       r0, [r1]                    @ memory to unmap
        mov       r1, #PAGE_SIZE              @ amount we mapped
        bl        munmap                      @ unmap it
        ldr       r1, =timerir_mmap_fd        @ reload the addr for accessing the Timer + IR
        ldr       r0, [r1]                    @ memory to unmap
        bl        close                       @ close the file

        mov       r0, #0                      @ return code 0
        mov       r7, #1                      @ exit app
        svc       0
        .end

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
