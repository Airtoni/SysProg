## How to Microprozessor:



### GPIO initalisieren

GPIO angucken, Register Adresse rausfinden, dann schauen was wo in das Register geschrieben werden muss.

#### Beispiel um zwei Pins einfügen: 

Zwischenregister: Pin eins konfigurieren
Zwischenregister: Pin zwei konfigurieren
dann beide ”oder verknüpfen”
dann in das “echte Register” schreiben
Siehe ARM Befehlssatz

![image-20200213163920143](C:\Users\paulr\AppData\Roaming\Typora\typora-user-images\image-20200213163920143.png)

### Moves: kann nur 8 bit werte verwenden!

```assembly
MOV R1, R2        	@ Ziel r1, wert von r2 wird in r1 gemoved
MOV R7 #128       	@ Ziel r7, wert 128(dec) wird in r7 gemoved
MOV R7 #0xAB01    	@ Ziel r7, wert AB01(hex) wird in r7 gemoved
```

### Shifts

```assembly
LSL R1, #27			@ R1 wird 27 mal nach left geshiftet
LSL R1, R2			@ In R2 steht, wie oft R1 geschiftet werden soll
LSLS R1, #27		@ Das S gibt wie bei arithmetischen Befehlen dafür Verantworlich, ob das CPSR verändert wird
```



### Arithmetische Befehle

```assembly
ADD R1, R2, R3 		@ r2 + r3 in r1
SUB R1, R1, R3      @ r1 - r3 in r1 ohne veränderung der Flags
SUBS R1, R1, R3     @ r1 - r3 in r1 mit veränderung der Flags
```

flags können in CPSR (Current Program Status Register) nachgeguckt werden

### Laden und Speichern ARM

LDR wird benutzt, um etwas aus dem Speicher in ein Register zu laden, STR wird benutzt um einen Wert aus einem Register an einer Speicheradresse zu schreiben.

Beispiel:

```assembly
LDR R2, =0x7E20004	@ Speichert Wert von der Adresse 0x7E20004 in R1
funktioniert nicht wegen der virtuellen Adresse
LDR R1, [GPIOREG,#0x4]		@ Speichert Wert von der Adresse, welche in Register R2 steht in Register R1

STR R1,[R2]  		@ Speichert den Wert, welcher in R1 an die Adresse, welche in R2 zu finden ist
```



##### Beispiel aus der Praxis initialisiere PIN 19 als OUTPUT :

im Datenblatt schauen, wohin geschrieben werden muss, GPIOREG gibt startwert an, danach muss man den Offset angeben als Hex Wert. In GPIOREG steht der Wert für 0x 7E20 0000, allerdings kann dieser Wert nicht fest eingetragen werden, da das Linux System diese Verändert. 

Aus dem Datenblatt wird entnommen: An Adresse 0x 7E20 0004 (Aus Seite 90) muss Bit 29-27 den Wert 001 annehmen (Seite 92)

```assembly
@ in r1 gewünschten bit wert schreiben
MOV R1 #0x1				@ In Register eins steht nun 000...00001
LSL	R1 #27				@ Register wird nun 27 mal nach links geshiftet

STR R1, [GPIOREG,#0x4]	@ GPIOREG ist vorgeben, es wird 4 aufaddiert um auf  0x 7E20 0004 zu kommen
```

##### Beispiel aus der Praxis: Pin 19 auf HIGH setzten 

Zuerst muss Pin 19 als Output definiert werden, anschließend im Datenblatt nachschauen: 

Aus dem Datenblatt wird entnommen: Ins das GPIO Output Set Register 0, welches an Adresse 0x 7E20 001C zu finden ist, soll an Bit 19 eine 1 geschrieben werden.

```assembly
@ in r1 gewünschten bit wert schreiben
MOV R1 #0x1				@ In Register eins steht nun 000...00001
LSL	R1 #19				@ Register wird nun 19 mal nach links geshiftet

STR R1, [GPIOREG,#0x1C]	@ GPIOREG ist vorgeben, es wird 0x1C aufaddiert um auf  0x 7E20 001C zu kommen
```
