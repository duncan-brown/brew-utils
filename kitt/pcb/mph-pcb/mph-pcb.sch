EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A2 23386 16535
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Driver_LED:TLC5940NT U1
U 1 1 6110428D
P 2400 11700
F 0 "U1" V 2446 10556 50  0000 R CNN
F 1 "TLC5940NT" V 2355 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 2450 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 2000 12400 50  0001 C CNN
	1    2400 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61118DF9
P 1400 11700
F 0 "#PWR?" H 1400 11550 50  0001 C CNN
F 1 "+5V" H 1415 11873 50  0000 C CNN
F 2 "" H 1400 11700 50  0001 C CNN
F 3 "" H 1400 11700 50  0001 C CNN
	1    1400 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61119761
P 3500 11700
F 0 "#PWR?" H 3500 11450 50  0001 C CNN
F 1 "GND" H 3505 11527 50  0000 C CNN
F 2 "" H 3500 11700 50  0001 C CNN
F 3 "" H 3500 11700 50  0001 C CNN
	1    3500 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6111817F
P 1850 15050
F 0 "#PWR?" H 1850 14800 50  0001 C CNN
F 1 "GND" V 1855 14922 50  0000 R CNN
F 2 "" H 1850 15050 50  0001 C CNN
F 3 "" H 1850 15050 50  0001 C CNN
	1    1850 15050
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61118275
P 1850 14950
F 0 "#PWR?" H 1850 14700 50  0001 C CNN
F 1 "GND" V 1855 14822 50  0000 R CNN
F 2 "" H 1850 14950 50  0001 C CNN
F 3 "" H 1850 14950 50  0001 C CNN
	1    1850 14950
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 611170F6
P 2950 15050
F 0 "#PWR?" H 2950 14800 50  0001 C CNN
F 1 "GND" V 2955 14922 50  0000 R CNN
F 2 "" H 2950 15050 50  0001 C CNN
F 3 "" H 2950 15050 50  0001 C CNN
	1    2950 15050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 611160CA
P 1850 14150
F 0 "#PWR?" H 1850 13900 50  0001 C CNN
F 1 "GND" V 1855 14022 50  0000 R CNN
F 2 "" H 1850 14150 50  0001 C CNN
F 3 "" H 1850 14150 50  0001 C CNN
	1    1850 14150
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61114FA9
P 1850 14050
F 0 "#PWR?" H 1850 13800 50  0001 C CNN
F 1 "GND" V 1855 13922 50  0000 R CNN
F 2 "" H 1850 14050 50  0001 C CNN
F 3 "" H 1850 14050 50  0001 C CNN
	1    1850 14050
	0    1    -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61112410
P 2950 15150
F 0 "#PWR?" H 2950 15000 50  0001 C CNN
F 1 "+5V" V 2965 15278 50  0000 L CNN
F 2 "" H 2950 15150 50  0001 C CNN
F 3 "" H 2950 15150 50  0001 C CNN
	1    2950 15150
	0    1    1    0   
$EndComp
$Comp
L ttgo_esp32:TTGO_ESP32_TDISPLAY_V1.1 TTGO1
U 1 1 610FD390
P 2050 15200
F 0 "TTGO1" H 2400 16637 60  0000 C CNN
F 1 "TTGO_ESP32_TDISPLAY_V1.1" H 2400 16531 60  0000 C CNN
F 2 "TTGO-tdisplay:TTGO_ESP32_TDisplay_v1.1" H 2050 15200 60  0001 C CNN
F 3 "" H 2050 15200 60  0001 C CNN
	1    2050 15200
	1    0    0    -1  
$EndComp
Text Label 2950 14950 0    50   ~ 0
BLANK
Text Label 2950 14850 0    50   ~ 0
XLAT
Text Label 2950 14750 0    50   ~ 0
SCLK
Text Label 2950 14650 0    50   ~ 0
SIG_1
Text Label 2950 14550 0    50   ~ 0
VPRG
Text Label 1850 14850 2    50   ~ 0
GSCLK
Text Label 2950 14250 0    50   ~ 0
DCPRG
Wire Wire Line
	2950 14350 2950 14250
Text Label 1850 14750 2    50   ~ 0
SOUT
Connection ~ 3550 13300
Entry Wire Line
	3450 14950 3550 14850
Wire Wire Line
	2950 14950 3450 14950
Entry Wire Line
	3450 14550 3550 14450
Entry Wire Line
	3450 14850 3550 14750
Entry Wire Line
	3450 14750 3550 14650
Entry Wire Line
	3450 14250 3550 14150
Wire Wire Line
	2950 14850 3450 14850
Wire Wire Line
	2950 14750 3450 14750
Wire Wire Line
	3450 14550 2950 14550
Wire Wire Line
	3450 14250 2950 14250
Connection ~ 2950 14250
Entry Wire Line
	1300 14750 1400 14850
Wire Wire Line
	1400 14850 1850 14850
Text Label 3200 12400 3    50   ~ 0
SIG_2
Text Label 3100 12400 3    50   ~ 0
SIG_1
Text Label 3000 12400 3    50   ~ 0
SCLK
Text Label 2300 12400 3    50   ~ 0
XLAT
Text Label 2200 12400 3    50   ~ 0
BLANK
Text Label 2000 12400 3    50   ~ 0
GSCLK
Text Label 1900 12400 3    50   ~ 0
DCPRG
Text Label 1700 12400 3    50   ~ 0
VPRG
Entry Wire Line
	3450 14650 3550 14550
Wire Wire Line
	2950 14650 3450 14650
$Comp
L Device:R R1
U 1 1 6113DDEC
P 1800 12800
F 0 "R1" H 1870 12846 50  0000 L CNN
F 1 "2.2K" H 1870 12755 50  0000 L CNN
F 2 "" V 1730 12800 50  0001 C CNN
F 3 "~" H 1800 12800 50  0001 C CNN
	1    1800 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 12400 1800 12650
$Comp
L power:GND #PWR?
U 1 1 61140F39
P 1800 12950
F 0 "#PWR?" H 1800 12700 50  0001 C CNN
F 1 "GND" H 1805 12777 50  0000 C CNN
F 2 "" H 1800 12950 50  0001 C CNN
F 3 "" H 1800 12950 50  0001 C CNN
	1    1800 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	1600 13300 1700 13200
Wire Wire Line
	1700 12400 1700 13200
Entry Wire Line
	1900 13300 2000 13200
Wire Wire Line
	2000 12400 2000 13200
Wire Wire Line
	2200 12400 2200 13200
Wire Wire Line
	2300 12400 2300 13200
Wire Wire Line
	3000 12400 3000 13200
Wire Wire Line
	3100 12400 3100 13200
Wire Wire Line
	3200 12400 3200 13200
Entry Wire Line
	2100 13300 2200 13200
Entry Wire Line
	2200 13300 2300 13200
Entry Wire Line
	2900 13300 3000 13200
Entry Wire Line
	3000 13300 3100 13200
Entry Wire Line
	3100 13300 3200 13200
$Comp
L Driver_LED:TLC5940NT U2
U 1 1 6114CC1E
P 5000 11700
F 0 "U2" V 5046 10556 50  0000 R CNN
F 1 "TLC5940NT" V 4955 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 5050 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 4600 12400 50  0001 C CNN
	1    5000 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6114CC24
P 4000 11700
F 0 "#PWR?" H 4000 11550 50  0001 C CNN
F 1 "+5V" H 4015 11873 50  0000 C CNN
F 2 "" H 4000 11700 50  0001 C CNN
F 3 "" H 4000 11700 50  0001 C CNN
	1    4000 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6114CC2A
P 6100 11700
F 0 "#PWR?" H 6100 11450 50  0001 C CNN
F 1 "GND" H 6105 11527 50  0000 C CNN
F 2 "" H 6100 11700 50  0001 C CNN
F 3 "" H 6100 11700 50  0001 C CNN
	1    6100 11700
	1    0    0    -1  
$EndComp
Text Label 5800 12400 3    50   ~ 0
SIG_3
Text Label 5700 12400 3    50   ~ 0
SIG_2
Text Label 5600 12400 3    50   ~ 0
SCLK
Text Label 4900 12400 3    50   ~ 0
XLAT
Text Label 4800 12400 3    50   ~ 0
BLANK
Text Label 4600 12400 3    50   ~ 0
GSCLK
Text Label 4500 12400 3    50   ~ 0
DCPRG
Text Label 4300 12400 3    50   ~ 0
VPRG
$Comp
L Device:R R2
U 1 1 6114CC38
P 4400 12800
F 0 "R2" H 4470 12846 50  0000 L CNN
F 1 "2.2K" H 4470 12755 50  0000 L CNN
F 2 "" V 4330 12800 50  0001 C CNN
F 3 "~" H 4400 12800 50  0001 C CNN
	1    4400 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 12400 4400 12650
$Comp
L power:GND #PWR?
U 1 1 6114CC3F
P 4400 12950
F 0 "#PWR?" H 4400 12700 50  0001 C CNN
F 1 "GND" H 4405 12777 50  0000 C CNN
F 2 "" H 4400 12950 50  0001 C CNN
F 3 "" H 4400 12950 50  0001 C CNN
	1    4400 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	4200 13300 4300 13200
Wire Wire Line
	4300 12400 4300 13200
Entry Wire Line
	4500 13300 4600 13200
Wire Wire Line
	4600 12400 4600 13200
Wire Wire Line
	4800 12400 4800 13200
Wire Wire Line
	4900 12400 4900 13200
Wire Wire Line
	5600 12400 5600 13200
Wire Wire Line
	5700 12400 5700 13200
Wire Wire Line
	5800 12400 5800 13200
Entry Wire Line
	4700 13300 4800 13200
Entry Wire Line
	4800 13300 4900 13200
Entry Wire Line
	5500 13300 5600 13200
Entry Wire Line
	5600 13300 5700 13200
Entry Wire Line
	5700 13300 5800 13200
$Comp
L Driver_LED:TLC5940NT U3
U 1 1 6115B8C6
P 7600 11700
F 0 "U3" V 7646 10556 50  0000 R CNN
F 1 "TLC5940NT" V 7555 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 7650 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 7200 12400 50  0001 C CNN
	1    7600 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6115B8CC
P 6600 11700
F 0 "#PWR?" H 6600 11550 50  0001 C CNN
F 1 "+5V" H 6615 11873 50  0000 C CNN
F 2 "" H 6600 11700 50  0001 C CNN
F 3 "" H 6600 11700 50  0001 C CNN
	1    6600 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6115B8D2
P 8700 11700
F 0 "#PWR?" H 8700 11450 50  0001 C CNN
F 1 "GND" H 8705 11527 50  0000 C CNN
F 2 "" H 8700 11700 50  0001 C CNN
F 3 "" H 8700 11700 50  0001 C CNN
	1    8700 11700
	1    0    0    -1  
$EndComp
Text Label 8400 12400 3    50   ~ 0
SIG_4
Text Label 8300 12400 3    50   ~ 0
SIG_3
Text Label 8200 12400 3    50   ~ 0
SCLK
Text Label 7500 12400 3    50   ~ 0
XLAT
Text Label 7400 12400 3    50   ~ 0
BLANK
Text Label 7200 12400 3    50   ~ 0
GSCLK
Text Label 7100 12400 3    50   ~ 0
DCPRG
Text Label 6900 12400 3    50   ~ 0
VPRG
$Comp
L Device:R R3
U 1 1 6115B8E0
P 7000 12800
F 0 "R3" H 7070 12846 50  0000 L CNN
F 1 "2.2K" H 7070 12755 50  0000 L CNN
F 2 "" V 6930 12800 50  0001 C CNN
F 3 "~" H 7000 12800 50  0001 C CNN
	1    7000 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 12400 7000 12650
$Comp
L power:GND #PWR?
U 1 1 6115B8E7
P 7000 12950
F 0 "#PWR?" H 7000 12700 50  0001 C CNN
F 1 "GND" H 7005 12777 50  0000 C CNN
F 2 "" H 7000 12950 50  0001 C CNN
F 3 "" H 7000 12950 50  0001 C CNN
	1    7000 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	6800 13300 6900 13200
Wire Wire Line
	6900 12400 6900 13200
Entry Wire Line
	7100 13300 7200 13200
Wire Wire Line
	7200 12400 7200 13200
Wire Wire Line
	7400 12400 7400 13200
Wire Wire Line
	7500 12400 7500 13200
Wire Wire Line
	8200 12400 8200 13200
Wire Wire Line
	8300 12400 8300 13200
Wire Wire Line
	8400 12400 8400 13200
Entry Wire Line
	7300 13300 7400 13200
Entry Wire Line
	7400 13300 7500 13200
Entry Wire Line
	8100 13300 8200 13200
Entry Wire Line
	8200 13300 8300 13200
Entry Wire Line
	8300 13300 8400 13200
$Comp
L Driver_LED:TLC5940NT U4
U 1 1 61185C8B
P 10200 11700
F 0 "U4" V 10246 10556 50  0000 R CNN
F 1 "TLC5940NT" V 10155 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 10250 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 9800 12400 50  0001 C CNN
	1    10200 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61185C91
P 9200 11700
F 0 "#PWR?" H 9200 11550 50  0001 C CNN
F 1 "+5V" H 9215 11873 50  0000 C CNN
F 2 "" H 9200 11700 50  0001 C CNN
F 3 "" H 9200 11700 50  0001 C CNN
	1    9200 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61185C97
P 11300 11700
F 0 "#PWR?" H 11300 11450 50  0001 C CNN
F 1 "GND" H 11305 11527 50  0000 C CNN
F 2 "" H 11300 11700 50  0001 C CNN
F 3 "" H 11300 11700 50  0001 C CNN
	1    11300 11700
	1    0    0    -1  
$EndComp
Text Label 11000 12400 3    50   ~ 0
SIG_5
Text Label 10900 12400 3    50   ~ 0
SIG_4
Text Label 10800 12400 3    50   ~ 0
SCLK
Text Label 10100 12400 3    50   ~ 0
XLAT
Text Label 10000 12400 3    50   ~ 0
BLANK
Text Label 9800 12400 3    50   ~ 0
GSCLK
Text Label 9700 12400 3    50   ~ 0
DCPRG
Text Label 9500 12400 3    50   ~ 0
VPRG
$Comp
L Device:R R4
U 1 1 61185CA5
P 9600 12800
F 0 "R4" H 9670 12846 50  0000 L CNN
F 1 "2.2K" H 9670 12755 50  0000 L CNN
F 2 "" V 9530 12800 50  0001 C CNN
F 3 "~" H 9600 12800 50  0001 C CNN
	1    9600 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 12400 9600 12650
$Comp
L power:GND #PWR?
U 1 1 61185CAC
P 9600 12950
F 0 "#PWR?" H 9600 12700 50  0001 C CNN
F 1 "GND" H 9605 12777 50  0000 C CNN
F 2 "" H 9600 12950 50  0001 C CNN
F 3 "" H 9600 12950 50  0001 C CNN
	1    9600 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	9400 13300 9500 13200
Wire Wire Line
	9500 12400 9500 13200
Entry Wire Line
	9700 13300 9800 13200
Wire Wire Line
	9800 12400 9800 13200
Wire Wire Line
	10000 12400 10000 13200
Wire Wire Line
	10100 12400 10100 13200
Wire Wire Line
	10800 12400 10800 13200
Wire Wire Line
	10900 12400 10900 13200
Wire Wire Line
	11000 12400 11000 13200
Entry Wire Line
	9900 13300 10000 13200
Entry Wire Line
	10000 13300 10100 13200
Entry Wire Line
	10700 13300 10800 13200
Entry Wire Line
	10800 13300 10900 13200
Entry Wire Line
	10900 13300 11000 13200
$Comp
L Driver_LED:TLC5940NT U5
U 1 1 6118DCBD
P 12800 11700
F 0 "U5" V 12846 10556 50  0000 R CNN
F 1 "TLC5940NT" V 12755 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 12850 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 12400 12400 50  0001 C CNN
	1    12800 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6118DCC3
P 11800 11700
F 0 "#PWR?" H 11800 11550 50  0001 C CNN
F 1 "+5V" H 11815 11873 50  0000 C CNN
F 2 "" H 11800 11700 50  0001 C CNN
F 3 "" H 11800 11700 50  0001 C CNN
	1    11800 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6118DCC9
P 13900 11700
F 0 "#PWR?" H 13900 11450 50  0001 C CNN
F 1 "GND" H 13905 11527 50  0000 C CNN
F 2 "" H 13900 11700 50  0001 C CNN
F 3 "" H 13900 11700 50  0001 C CNN
	1    13900 11700
	1    0    0    -1  
$EndComp
Text Label 13600 12400 3    50   ~ 0
SIG_6
Text Label 13500 12400 3    50   ~ 0
SIG_5
Text Label 13400 12400 3    50   ~ 0
SCLK
Text Label 12700 12400 3    50   ~ 0
XLAT
Text Label 12600 12400 3    50   ~ 0
BLANK
Text Label 12400 12400 3    50   ~ 0
GSCLK
Text Label 12300 12400 3    50   ~ 0
DCPRG
Text Label 12100 12400 3    50   ~ 0
VPRG
$Comp
L Device:R R5
U 1 1 6118DCD7
P 12200 12800
F 0 "R5" H 12270 12846 50  0000 L CNN
F 1 "2.2K" H 12270 12755 50  0000 L CNN
F 2 "" V 12130 12800 50  0001 C CNN
F 3 "~" H 12200 12800 50  0001 C CNN
	1    12200 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	12200 12400 12200 12650
$Comp
L power:GND #PWR?
U 1 1 6118DCDE
P 12200 12950
F 0 "#PWR?" H 12200 12700 50  0001 C CNN
F 1 "GND" H 12205 12777 50  0000 C CNN
F 2 "" H 12200 12950 50  0001 C CNN
F 3 "" H 12200 12950 50  0001 C CNN
	1    12200 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	12000 13300 12100 13200
Wire Wire Line
	12100 12400 12100 13200
Entry Wire Line
	12300 13300 12400 13200
Wire Wire Line
	12400 12400 12400 13200
Wire Wire Line
	12600 12400 12600 13200
Wire Wire Line
	12700 12400 12700 13200
Wire Wire Line
	13400 12400 13400 13200
Wire Wire Line
	13500 12400 13500 13200
Wire Wire Line
	13600 12400 13600 13200
Entry Wire Line
	12500 13300 12600 13200
Entry Wire Line
	12600 13300 12700 13200
Entry Wire Line
	13300 13300 13400 13200
Entry Wire Line
	13400 13300 13500 13200
Entry Wire Line
	13500 13300 13600 13200
$Comp
L Driver_LED:TLC5940NT U6
U 1 1 6119255F
P 15400 11700
F 0 "U6" V 15446 10556 50  0000 R CNN
F 1 "TLC5940NT" V 15355 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 15450 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 15000 12400 50  0001 C CNN
	1    15400 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61192565
P 14400 11700
F 0 "#PWR?" H 14400 11550 50  0001 C CNN
F 1 "+5V" H 14415 11873 50  0000 C CNN
F 2 "" H 14400 11700 50  0001 C CNN
F 3 "" H 14400 11700 50  0001 C CNN
	1    14400 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6119256B
P 16500 11700
F 0 "#PWR?" H 16500 11450 50  0001 C CNN
F 1 "GND" H 16505 11527 50  0000 C CNN
F 2 "" H 16500 11700 50  0001 C CNN
F 3 "" H 16500 11700 50  0001 C CNN
	1    16500 11700
	1    0    0    -1  
$EndComp
Text Label 16100 12400 3    50   ~ 0
SIG_6
Text Label 16000 12400 3    50   ~ 0
SCLK
Text Label 15300 12400 3    50   ~ 0
XLAT
Text Label 15200 12400 3    50   ~ 0
BLANK
Text Label 15000 12400 3    50   ~ 0
GSCLK
Text Label 14900 12400 3    50   ~ 0
DCPRG
Text Label 14700 12400 3    50   ~ 0
VPRG
$Comp
L Device:R R6
U 1 1 61192579
P 14800 12800
F 0 "R6" H 14870 12846 50  0000 L CNN
F 1 "2.2K" H 14870 12755 50  0000 L CNN
F 2 "" V 14730 12800 50  0001 C CNN
F 3 "~" H 14800 12800 50  0001 C CNN
	1    14800 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	14800 12400 14800 12650
$Comp
L power:GND #PWR?
U 1 1 61192580
P 14800 12950
F 0 "#PWR?" H 14800 12700 50  0001 C CNN
F 1 "GND" H 14805 12777 50  0000 C CNN
F 2 "" H 14800 12950 50  0001 C CNN
F 3 "" H 14800 12950 50  0001 C CNN
	1    14800 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	14600 13300 14700 13200
Wire Wire Line
	14700 12400 14700 13200
Entry Wire Line
	14900 13300 15000 13200
Wire Wire Line
	15000 12400 15000 13200
Wire Wire Line
	15200 12400 15200 13200
Wire Wire Line
	15300 12400 15300 13200
Wire Wire Line
	16000 12400 16000 13200
Wire Wire Line
	16100 12400 16100 13200
Wire Wire Line
	16200 12400 16200 13200
Entry Wire Line
	15100 13300 15200 13200
Entry Wire Line
	15200 13300 15300 13200
Entry Wire Line
	15900 13300 16000 13200
Entry Wire Line
	16000 13300 16100 13200
Entry Wire Line
	16100 13300 16200 13200
$Comp
L Driver_LED:TLC5940NT U7
U 1 1 61197871
P 18000 11700
F 0 "U7" V 18046 10556 50  0000 R CNN
F 1 "TLC5940NT" V 17955 10556 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 18050 10725 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc5940.pdf" H 17600 12400 50  0001 C CNN
	1    18000 11700
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61197877
P 17000 11700
F 0 "#PWR?" H 17000 11550 50  0001 C CNN
F 1 "+5V" H 17015 11873 50  0000 C CNN
F 2 "" H 17000 11700 50  0001 C CNN
F 3 "" H 17000 11700 50  0001 C CNN
	1    17000 11700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6119787D
P 19100 11700
F 0 "#PWR?" H 19100 11450 50  0001 C CNN
F 1 "GND" H 19105 11527 50  0000 C CNN
F 2 "" H 19100 11700 50  0001 C CNN
F 3 "" H 19100 11700 50  0001 C CNN
	1    19100 11700
	1    0    0    -1  
$EndComp
Text Label 18800 12400 3    50   ~ 0
SOUT
Text Label 18700 12400 3    50   ~ 0
SIG_7
Text Label 18600 12400 3    50   ~ 0
SCLK
Text Label 17900 12400 3    50   ~ 0
XLAT
Text Label 17800 12400 3    50   ~ 0
BLANK
Text Label 17600 12400 3    50   ~ 0
GSCLK
Text Label 17500 12400 3    50   ~ 0
DCPRG
Text Label 17300 12400 3    50   ~ 0
VPRG
$Comp
L Device:R R7
U 1 1 6119788B
P 17400 12800
F 0 "R7" H 17470 12846 50  0000 L CNN
F 1 "2.2K" H 17470 12755 50  0000 L CNN
F 2 "" V 17330 12800 50  0001 C CNN
F 3 "~" H 17400 12800 50  0001 C CNN
	1    17400 12800
	1    0    0    -1  
$EndComp
Wire Wire Line
	17400 12400 17400 12650
$Comp
L power:GND #PWR?
U 1 1 61197892
P 17400 12950
F 0 "#PWR?" H 17400 12700 50  0001 C CNN
F 1 "GND" H 17405 12777 50  0000 C CNN
F 2 "" H 17400 12950 50  0001 C CNN
F 3 "" H 17400 12950 50  0001 C CNN
	1    17400 12950
	1    0    0    -1  
$EndComp
Entry Wire Line
	17200 13300 17300 13200
Wire Wire Line
	17300 12400 17300 13200
Entry Wire Line
	17500 13300 17600 13200
Wire Wire Line
	17600 12400 17600 13200
Wire Wire Line
	17800 12400 17800 13200
Wire Wire Line
	17900 12400 17900 13200
Wire Wire Line
	18600 12400 18600 13200
Wire Wire Line
	18700 12400 18700 13200
Wire Wire Line
	18800 12400 18800 13200
Entry Wire Line
	17700 13300 17800 13200
Entry Wire Line
	17800 13300 17900 13200
Entry Wire Line
	18500 13300 18600 13200
Entry Wire Line
	18600 13300 18700 13200
Entry Wire Line
	18700 13300 18800 13200
Text Label 16200 12400 3    50   ~ 0
SIG_7
Entry Wire Line
	1300 14650 1400 14750
Wire Wire Line
	1400 14750 1850 14750
Wire Bus Line
	1300 13300 1300 14750
Wire Bus Line
	3550 13300 3550 14850
Wire Bus Line
	1300 13300 3550 13300
Wire Bus Line
	3550 13300 18700 13300
$EndSCHEMATC