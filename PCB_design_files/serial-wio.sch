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
Text Label 1600 1550 2    50   ~ 0
UART_RXD
Text Label 1600 1450 2    50   ~ 0
UART_TXD
$Comp
L Connector:Raspberry_Pi_2_3 J1
U 1 1 62313230
P 2400 2350
F 0 "J1" H 2400 3831 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 2400 3740 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical" H 2400 2350 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 2400 2350 50  0001 C CNN
	1    2400 2350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0101
U 1 1 6234D28A
P 2850 1050
F 0 "#PWR0101" H 2850 900 50  0001 C CNN
F 1 "+3.3V" H 2865 1223 50  0000 C CNN
F 2 "" H 2850 1050 50  0001 C CNN
F 3 "" H 2850 1050 50  0001 C CNN
	1    2850 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1050 2600 1050
Wire Wire Line
	2850 1050 2600 1050
Connection ~ 2600 1050
NoConn ~ 2200 1050
NoConn ~ 2300 1050
NoConn ~ 3200 1450
NoConn ~ 3200 1550
NoConn ~ 3200 1750
NoConn ~ 3200 1850
NoConn ~ 3200 2050
NoConn ~ 3200 2150
NoConn ~ 3200 2250
NoConn ~ 3200 2450
NoConn ~ 3200 2550
NoConn ~ 3200 2650
NoConn ~ 3200 2750
NoConn ~ 3200 2850
NoConn ~ 3200 3050
NoConn ~ 3200 3150
NoConn ~ 1600 1750
NoConn ~ 1600 1850
NoConn ~ 1600 1950
NoConn ~ 1600 2150
NoConn ~ 1600 2250
NoConn ~ 1600 2350
NoConn ~ 1600 2550
NoConn ~ 1600 2650
NoConn ~ 1600 2750
NoConn ~ 1600 2850
NoConn ~ 1600 2950
NoConn ~ 1600 3050
$Comp
L power:GND #PWR0102
U 1 1 6235459C
P 2300 3800
F 0 "#PWR0102" H 2300 3550 50  0001 C CNN
F 1 "GND" H 2305 3627 50  0000 C CNN
F 2 "" H 2300 3800 50  0001 C CNN
F 3 "" H 2300 3800 50  0001 C CNN
	1    2300 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3650 2000 3800
Wire Wire Line
	2000 3800 2100 3800
Wire Wire Line
	2100 3650 2100 3800
Connection ~ 2100 3800
Wire Wire Line
	2100 3800 2200 3800
Connection ~ 2200 3800
Wire Wire Line
	2200 3800 2300 3800
Wire Wire Line
	2200 3650 2200 3800
Wire Wire Line
	2300 3650 2300 3800
Connection ~ 2300 3800
Wire Wire Line
	2300 3800 2400 3800
Wire Wire Line
	2700 3800 2700 3650
Wire Wire Line
	2600 3650 2600 3800
Connection ~ 2600 3800
Wire Wire Line
	2600 3800 2700 3800
Wire Wire Line
	2500 3650 2500 3800
Connection ~ 2500 3800
Wire Wire Line
	2500 3800 2600 3800
Connection ~ 2400 3800
Wire Wire Line
	2400 3800 2500 3800
Wire Wire Line
	2400 3650 2400 3800
$Comp
L Connector:Conn_01x03_Male J2
U 1 1 62359750
P 7050 3300
F 0 "J2" H 7022 3232 50  0000 R CNN
F 1 "Conn_01x03_Male" H 7022 3323 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 7050 3300 50  0001 C CNN
F 3 "~" H 7050 3300 50  0001 C CNN
	1    7050 3300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 6235D0E2
P 6700 3400
F 0 "#PWR0103" H 6700 3150 50  0001 C CNN
F 1 "GND" H 6705 3227 50  0000 C CNN
F 2 "" H 6700 3400 50  0001 C CNN
F 3 "" H 6700 3400 50  0001 C CNN
	1    6700 3400
	1    0    0    -1  
$EndComp
Text Label 5300 3650 2    50   ~ 0
UART_RXD
Text Label 5300 3200 2    50   ~ 0
UART_TXD
Wire Wire Line
	6850 3400 6700 3400
Wire Notes Line
	950  700  3750 700 
Wire Notes Line
	3750 700  3750 4800
Wire Notes Line
	3750 4800 950  4800
Wire Notes Line
	950  4800 950  700 
Wire Notes Line
	4850 700  4850 2550
Wire Notes Line
	4850 2550 8000 2550
Wire Notes Line
	8000 2550 8000 700 
Wire Notes Line
	8000 700  4850 700 
Wire Notes Line
	4850 2700 8000 2700
Wire Notes Line
	8000 2700 8000 4250
Wire Notes Line
	8000 4250 4850 4250
Wire Notes Line
	4850 4250 4850 2700
Text Notes 1650 4750 0    50   ~ 0
Connecteur RaspPi 40 broches
Text Notes 5850 800  0    50   ~ 0
Adaptation de niveau (Sonde)
Text Notes 5800 4150 0    50   ~ 0
Connectique UART (3.3V)
$Comp
L Connector:Conn_01x01_Female TEST_GND1
U 1 1 623F3093
P 6900 1900
F 0 "TEST_GND1" H 6928 1926 50  0000 L CNN
F 1 "Conn_01x01_Female" H 6928 1835 50  0000 L CNN
F 2 "serial-wio:4mm-test-conn-socket" H 6900 1900 50  0001 C CNN
F 3 "~" H 6900 1900 50  0001 C CNN
	1    6900 1900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Female TEST_SIG1
U 1 1 623F3830
P 6900 1700
F 0 "TEST_SIG1" H 6928 1726 50  0000 L CNN
F 1 "Conn_01x01_Female" H 6928 1635 50  0000 L CNN
F 2 "serial-wio:4mm-test-conn-socket" H 6900 1700 50  0001 C CNN
F 3 "~" H 6900 1700 50  0001 C CNN
	1    6900 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1700 6150 1700
$Comp
L power:+3.3V #PWR01
U 1 1 6240D238
P 5600 1050
F 0 "#PWR01" H 5600 900 50  0001 C CNN
F 1 "+3.3V" H 5615 1223 50  0000 C CNN
F 2 "" H 5600 1050 50  0001 C CNN
F 3 "" H 5600 1050 50  0001 C CNN
	1    5600 1050
	1    0    0    -1  
$EndComp
Text Label 5500 1700 2    50   ~ 0
UART_RXD
$Comp
L power:GND #PWR02
U 1 1 6241B813
P 6600 1900
F 0 "#PWR02" H 6600 1650 50  0001 C CNN
F 1 "GND" H 6605 1727 50  0000 C CNN
F 2 "" H 6600 1900 50  0001 C CNN
F 3 "" H 6600 1900 50  0001 C CNN
	1    6600 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1900 6600 1900
$Comp
L Mechanical:MountingHole H1
U 1 1 6240971A
P 8750 1050
F 0 "H1" H 8850 1096 50  0000 L CNN
F 1 "MountingHole" H 8850 1000 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 8750 1050 50  0001 C CNN
F 3 "~" H 8750 1050 50  0001 C CNN
	1    8750 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 6240A9F5
P 8750 1250
F 0 "H2" H 8850 1296 50  0000 L CNN
F 1 "MountingHole" H 8850 1205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 8750 1250 50  0001 C CNN
F 3 "~" H 8750 1250 50  0001 C CNN
	1    8750 1250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H5
U 1 1 6240AC73
P 8750 1850
F 0 "H5" H 8850 1896 50  0000 L CNN
F 1 "MountingHole" H 8850 1805 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 8750 1850 50  0001 C CNN
F 3 "~" H 8750 1850 50  0001 C CNN
	1    8750 1850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 6240B200
P 8750 1450
F 0 "H3" H 8850 1496 50  0000 L CNN
F 1 "MountingHole" H 8850 1405 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 8750 1450 50  0001 C CNN
F 3 "~" H 8750 1450 50  0001 C CNN
	1    8750 1450
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 6240B7E8
P 8750 1650
F 0 "H4" H 8850 1696 50  0000 L CNN
F 1 "MountingHole" H 8850 1605 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 8750 1650 50  0001 C CNN
F 3 "~" H 8750 1650 50  0001 C CNN
	1    8750 1650
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H6
U 1 1 62418969
P 8750 2050
F 0 "H6" H 8850 2096 50  0000 L CNN
F 1 "MountingHole" H 8850 2005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 8750 2050 50  0001 C CNN
F 3 "~" H 8750 2050 50  0001 C CNN
	1    8750 2050
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:MMBF170 Q1
U 1 1 62428D3D
P 5800 3550
F 0 "Q1" V 6049 3550 50  0000 C CNN
F 1 "MMBF170" V 6140 3550 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6000 3475 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30104.pdf" H 5800 3550 50  0001 L CNN
	1    5800 3550
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 6243B3BA
P 5800 2900
F 0 "#PWR0104" H 5800 2750 50  0001 C CNN
F 1 "+3.3V" H 5815 3073 50  0000 C CNN
F 2 "" H 5800 2900 50  0001 C CNN
F 3 "" H 5800 2900 50  0001 C CNN
	1    5800 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6243E0EF
P 5600 1350
F 0 "R2" H 5670 1396 50  0000 L CNN
F 1 "10k" H 5670 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5530 1350 50  0001 C CNN
F 3 "~" H 5600 1350 50  0001 C CNN
	1    5600 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1050 5950 1050
Wire Wire Line
	5500 1700 5600 1700
$Comp
L Transistor_FET:MMBF170 Q2
U 1 1 6240D231
P 5950 1600
F 0 "Q2" V 6199 1600 50  0000 C CNN
F 1 "MMBF170" V 6290 1600 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6150 1525 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30104.pdf" H 5950 1600 50  0001 L CNN
	1    5950 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 1050 5600 1200
Connection ~ 5600 1050
Wire Wire Line
	5600 1500 5600 1700
Connection ~ 5600 1700
Wire Wire Line
	5600 1700 5750 1700
Wire Wire Line
	5950 1050 5950 1400
$Comp
L Transistor_FET:MMBF170 Q3
U 1 1 62454A08
P 6200 3100
F 0 "Q3" V 6449 3100 50  0000 C CNN
F 1 "MMBF170" V 6540 3100 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6400 3025 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30104.pdf" H 6200 3100 50  0001 L CNN
	1    6200 3100
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 62457446
P 5450 3450
F 0 "R1" H 5520 3496 50  0000 L CNN
F 1 "10k" H 5520 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5380 3450 50  0001 C CNN
F 3 "~" H 5450 3450 50  0001 C CNN
	1    5450 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 624580B8
P 5900 3050
F 0 "R3" H 5970 3096 50  0000 L CNN
F 1 "10k" H 5970 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5830 3050 50  0001 C CNN
F 3 "~" H 5900 3050 50  0001 C CNN
	1    5900 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3200 5900 3200
Wire Wire Line
	5900 3200 6000 3200
Connection ~ 5900 3200
Wire Wire Line
	5800 2900 5900 2900
Wire Wire Line
	5900 2900 6200 2900
Connection ~ 5900 2900
Wire Wire Line
	6400 3200 6850 3200
Wire Wire Line
	6000 3650 6500 3650
Wire Wire Line
	6500 3650 6500 3300
Wire Wire Line
	6500 3300 6850 3300
Wire Wire Line
	5600 3650 5450 3650
Wire Wire Line
	5450 3600 5450 3650
Connection ~ 5450 3650
Wire Wire Line
	5450 3650 5300 3650
Wire Wire Line
	5450 3300 5450 2900
Wire Wire Line
	5450 2900 5800 2900
Connection ~ 5800 2900
Wire Wire Line
	5800 2900 5800 3350
$EndSCHEMATC
