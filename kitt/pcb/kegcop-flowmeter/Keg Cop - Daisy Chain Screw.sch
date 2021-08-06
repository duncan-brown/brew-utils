EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
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
Wire Wire Line
	5600 4700 5600 4600
Wire Wire Line
	5600 4600 5600 4300
Wire Wire Line
	5600 4300 5600 3600
Wire Wire Line
	5600 4600 5400 4600
Wire Wire Line
	5400 4600 5400 4700
Wire Wire Line
	5400 4600 5200 4600
Wire Wire Line
	5200 4600 5200 4700
Wire Wire Line
	4900 4700 4900 4600
Wire Wire Line
	5200 4600 4900 4600
Wire Wire Line
	4900 3500 4900 3600
Wire Wire Line
	5600 3600 5600 3500
Wire Wire Line
	5300 3600 5300 3500
Wire Wire Line
	5300 3600 5600 3600
Wire Wire Line
	5100 3500 5100 3600
Wire Wire Line
	5100 3600 5300 3600
Wire Wire Line
	5100 3600 4900 3600
Wire Wire Line
	6400 4300 5600 4300
Connection ~ 5600 4600
Connection ~ 5400 4600
Connection ~ 5200 4600
Connection ~ 5300 3600
Connection ~ 5100 3600
Connection ~ 5600 3600
Connection ~ 5600 4300
Text Label 6200 4300 0    70   ~ 0
GND
Wire Wire Line
	5500 4700 5500 4100
Wire Wire Line
	5500 4100 6400 4100
Text Label 6200 4100 0    70   ~ 0
DATA0
Wire Wire Line
	5000 4700 5000 4300
Wire Wire Line
	5000 4300 5400 4300
Wire Wire Line
	5400 4300 5400 3900
Wire Wire Line
	5400 3900 5500 3900
Wire Wire Line
	5500 3900 5500 3500
Wire Wire Line
	6400 3900 5500 3900
Connection ~ 5500 3900
Text Label 6200 3900 0    70   ~ 0
5VD
Wire Wire Line
	5300 4700 5300 3900
Wire Wire Line
	5300 3900 5000 3900
Wire Wire Line
	5000 3900 5000 3500
Wire Wire Line
	5100 4700 5100 4100
Wire Wire Line
	5100 4100 5200 4100
Wire Wire Line
	5200 4100 5200 3500
$Comp
L Keg_Cop_-_Daisy_Chain_Screw-eagle-import:RJ45 IN1
U 1 1 234ED39C
P 5300 4800
F 0 "IN1" H 5200 5200 59  0000 L BNN
F 1 "RJ45" V 5400 4680 59  0000 R TNN
F 2 "Keg Cop - Daisy Chain Screw:RJ45-NO-SHIELD" H 5300 4800 50  0001 C CNN
F 3 "" H 5300 4800 50  0001 C CNN
	1    5300 4800
	0    1    1    0   
$EndComp
$Comp
L Keg_Cop_-_Daisy_Chain_Screw-eagle-import:RJ45 OUT1
U 1 1 00E7AA9E
P 5200 3400
F 0 "OUT1" H 5100 3800 59  0000 L BNN
F 1 "RJ45" V 5300 3280 59  0000 R TNN
F 2 "Keg Cop - Daisy Chain Screw:RJ45-NO-SHIELD" H 5200 3400 50  0001 C CNN
F 3 "" H 5200 3400 50  0001 C CNN
	1    5200 3400
	0    -1   -1   0   
$EndComp
$Comp
L Keg_Cop_-_Daisy_Chain_Screw-eagle-import:W237-103 FLOW1
U 1 1 C8124B2D
P 6600 3900
F 0 "FLOW1" H 6600 3935 59  0000 R TNN
F 1 "W237-103" H 6500 3755 59  0001 L BNN
F 2 "Keg Cop - Daisy Chain Screw:W237-103" H 6600 3900 50  0001 C CNN
F 3 "" H 6600 3900 50  0001 C CNN
	1    6600 3900
	-1   0    0    1   
$EndComp
$Comp
L Keg_Cop_-_Daisy_Chain_Screw-eagle-import:W237-103 FLOW1
U 2 1 C8124B21
P 6600 4100
F 0 "FLOW1" H 6600 4135 59  0000 R TNN
F 1 "W237-103" H 6500 3955 59  0001 L BNN
F 2 "Keg Cop - Daisy Chain Screw:W237-103" H 6600 4100 50  0001 C CNN
F 3 "" H 6600 4100 50  0001 C CNN
	2    6600 4100
	-1   0    0    1   
$EndComp
$Comp
L Keg_Cop_-_Daisy_Chain_Screw-eagle-import:W237-103 FLOW1
U 3 1 C8124B25
P 6600 4300
F 0 "FLOW1" H 6600 4335 59  0000 R TNN
F 1 "W237-103" H 6500 4155 59  0000 L BNN
F 2 "Keg Cop - Daisy Chain Screw:W237-103" H 6600 4300 50  0001 C CNN
F 3 "" H 6600 4300 50  0001 C CNN
	3    6600 4300
	-1   0    0    1   
$EndComp
$EndSCHEMATC
