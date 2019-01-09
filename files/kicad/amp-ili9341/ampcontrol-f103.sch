EESchema Schematic File Version 4
LIBS:ampcontrol-f103-cache
EELAYER 26 0
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
$Comp
L power:GNDD #PWR010
U 1 1 5C043025
P 4800 5200
F 0 "#PWR010" H 4800 4950 50  0001 C CNN
F 1 "GNDD" H 4804 5045 50  0000 C CNN
F 2 "" H 4800 5200 50  0001 C CNN
F 3 "" H 4800 5200 50  0001 C CNN
	1    4800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 5000 4700 5100
Wire Wire Line
	4700 5100 4800 5100
Wire Wire Line
	4900 5100 4900 5000
Wire Wire Line
	4800 5000 4800 5100
Connection ~ 4800 5100
Wire Wire Line
	4800 5100 4900 5100
Wire Wire Line
	4800 5200 4800 5100
$Comp
L power:+3V3 #PWR09
U 1 1 5C0430F5
P 4700 1800
F 0 "#PWR09" H 4700 1650 50  0001 C CNN
F 1 "+3V3" H 4715 1973 50  0000 C CNN
F 2 "" H 4700 1800 50  0001 C CNN
F 3 "" H 4700 1800 50  0001 C CNN
	1    4700 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2000 4600 1900
Wire Wire Line
	4600 1900 4700 1900
Wire Wire Line
	4800 1900 4800 2000
Wire Wire Line
	4700 2000 4700 1900
Connection ~ 4700 1900
Wire Wire Line
	4700 1900 4800 1900
Wire Wire Line
	4700 1800 4700 1900
$Comp
L power:+3V3 #PWR02
U 1 1 5C0435B6
P 1250 5600
F 0 "#PWR02" H 1250 5450 50  0001 C CNN
F 1 "+3V3" H 1265 5773 50  0000 C CNN
F 2 "" H 1250 5600 50  0001 C CNN
F 3 "" H 1250 5600 50  0001 C CNN
	1    1250 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR01
U 1 1 5C0435EA
P 1050 5800
F 0 "#PWR01" H 1050 5550 50  0001 C CNN
F 1 "GNDD" H 1054 5645 50  0000 C CNN
F 2 "" H 1050 5800 50  0001 C CNN
F 3 "" H 1050 5800 50  0001 C CNN
	1    1050 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 5700 1050 5700
Wire Wire Line
	1050 5700 1050 5800
Wire Wire Line
	1250 5800 1250 5600
NoConn ~ 1450 5900
Wire Wire Line
	1450 6000 1200 6000
Text Label 1200 6000 0    50   ~ 0
RS
Wire Wire Line
	1450 6100 1200 6100
Wire Wire Line
	1250 5800 1450 5800
Wire Wire Line
	1450 7100 1200 7100
Wire Wire Line
	1450 7200 1200 7200
Wire Wire Line
	1450 7300 1200 7300
Wire Wire Line
	1450 7500 1200 7500
Text Label 1200 6100 0    50   ~ 0
WR
Text Label 1200 6200 0    50   ~ 0
RD
Text Label 1200 7100 0    50   ~ 0
CS
Text Label 1200 7200 0    50   ~ 0
F_CS
Text Label 1200 7300 0    50   ~ 0
NRST
Text Label 1200 7500 0    50   ~ 0
BCKL
NoConn ~ 1450 7600
$Comp
L Connector_Generic:Conn_02x20_Top_Bottom J1
U 1 1 5C047533
P 1650 6600
F 0 "J1" H 1700 7717 50  0000 C CNN
F 1 "Conn_LCD" H 1700 7626 50  0000 C CNN
F 2 "ampcontrol-f103:PinSocket_2x20_P2.54mm_Vertical_Top_Bottom" H 1650 6600 50  0001 C CNN
F 3 "~" H 1650 6600 50  0001 C CNN
	1    1650 6600
	1    0    0    -1  
$EndComp
NoConn ~ 1950 7600
NoConn ~ 1950 7500
NoConn ~ 1950 6800
Wire Wire Line
	1950 5700 2200 5700
Wire Wire Line
	2200 5800 1950 5800
Wire Wire Line
	1950 5900 2200 5900
Wire Wire Line
	2200 6000 1950 6000
Wire Wire Line
	1950 6100 2200 6100
Wire Wire Line
	2200 6200 1950 6200
Wire Wire Line
	1950 6300 2200 6300
Wire Wire Line
	2200 6400 1950 6400
Wire Wire Line
	1950 6500 2200 6500
Wire Wire Line
	1950 6600 2200 6600
Wire Wire Line
	1950 6700 2200 6700
Wire Wire Line
	1950 6900 2200 6900
Wire Wire Line
	1950 7000 2200 7000
Wire Wire Line
	1950 7100 2200 7100
Wire Wire Line
	1950 7200 2200 7200
Wire Wire Line
	1950 7300 2200 7300
Wire Wire Line
	1950 7400 2200 7400
NoConn ~ 1450 7400
Text Label 2200 5700 2    50   ~ 0
DB0
Text Label 2200 5800 2    50   ~ 0
DB1
Text Label 2200 5900 2    50   ~ 0
DB2
Text Label 2200 6000 2    50   ~ 0
DB3
Text Label 2200 6100 2    50   ~ 0
DB4
Text Label 2200 6200 2    50   ~ 0
DB5
Text Label 2200 6300 2    50   ~ 0
DB6
Text Label 2200 6400 2    50   ~ 0
DB7
Text Label 2200 6500 2    50   ~ 0
T_CLK
Text Label 2200 6600 2    50   ~ 0
T_CS
Text Label 2200 6700 2    50   ~ 0
T_DIN
Text Label 2200 6900 2    50   ~ 0
T_DO
Text Label 2200 7000 2    50   ~ 0
T_IRQ
Text Label 2200 7100 2    50   ~ 0
SD_DO
Text Label 2200 7200 2    50   ~ 0
SD_CLK
Text Label 2200 7300 2    50   ~ 0
SD_DIN
Text Label 2200 7400 2    50   ~ 0
SD_CS
$Comp
L MCU_ST_STM32F1:STM32F103CBTx U2
U 1 1 5C050C11
P 4700 3500
F 0 "U2" H 4150 4950 50  0000 C CNN
F 1 "STM32F103CBTx" H 4700 3500 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 4100 2100 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 4700 3500 50  0001 C CNN
	1    4700 3500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4100 3300 3700 3300
Wire Wire Line
	4100 3400 3700 3400
Wire Wire Line
	3700 3500 4100 3500
Wire Wire Line
	4100 3600 3700 3600
Wire Wire Line
	4100 3700 3700 3700
Wire Wire Line
	4100 3800 3700 3800
Wire Wire Line
	4100 3900 3700 3900
Wire Wire Line
	4100 4000 3700 4000
Text Label 3700 3300 0    50   ~ 0
DB0
Text Label 3700 3400 0    50   ~ 0
DB1
Text Label 3700 3500 0    50   ~ 0
DB2
Text Label 3700 3600 0    50   ~ 0
DB3
Text Label 3700 3700 0    50   ~ 0
DB4
Text Label 3700 3800 0    50   ~ 0
DB5
Text Label 3700 3900 0    50   ~ 0
DB6
Text Label 3700 4000 0    50   ~ 0
DB7
Wire Wire Line
	4100 4800 3700 4800
Text Label 3700 4800 0    50   ~ 0
HW_RST
Wire Wire Line
	5400 3700 5800 3700
Text Label 5800 3700 2    50   ~ 0
RD
Wire Wire Line
	5400 3800 5800 3800
Text Label 5800 3800 2    50   ~ 0
WR
Wire Wire Line
	5400 3900 5800 3900
Text Label 5800 3900 2    50   ~ 0
RS
Wire Wire Line
	5400 4000 5800 4000
Text Label 5800 4000 2    50   ~ 0
CS
Text Label 5800 2900 2    50   ~ 0
BCKL
NoConn ~ 1200 6300
NoConn ~ 1200 6400
NoConn ~ 1200 6500
NoConn ~ 1200 6600
NoConn ~ 1200 6700
NoConn ~ 1200 6800
NoConn ~ 1200 6900
NoConn ~ 1200 7000
NoConn ~ 1200 7200
NoConn ~ 2200 6500
NoConn ~ 2200 6600
NoConn ~ 2200 6700
NoConn ~ 2200 6900
NoConn ~ 2200 7000
NoConn ~ 2200 7100
NoConn ~ 2200 7200
NoConn ~ 2200 7300
NoConn ~ 2200 7400
$Comp
L Device:Crystal_Small Y2
U 1 1 5C0E4C09
P 7100 2700
F 0 "Y2" H 7100 2475 50  0000 C CNN
F 1 "8MHz" H 7100 2566 50  0000 C CNN
F 2 "Crystal:Crystal_HC49-4H_Vertical" H 7100 2700 50  0001 C CNN
F 3 "~" H 7100 2700 50  0001 C CNN
	1    7100 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	5400 2900 5800 2900
Wire Wire Line
	5800 4100 5400 4100
Wire Wire Line
	5800 4200 5400 4200
Text Label 5800 4100 2    50   ~ 0
MUTE
Text Label 5800 4200 2    50   ~ 0
STBY
Wire Wire Line
	4100 4100 3700 4100
Text Label 3700 4100 0    50   ~ 0
IR
Wire Wire Line
	4100 4200 3700 4200
Wire Wire Line
	3700 4300 4100 4300
Text Label 3700 4200 0    50   ~ 0
TX
Text Label 3700 4300 0    50   ~ 0
RX
$Comp
L Device:C C6
U 1 1 5C12356D
P 7300 3250
F 0 "C6" H 7185 3204 50  0000 R CNN
F 1 "20p" H 7185 3295 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7338 3100 50  0001 C CNN
F 3 "~" H 7300 3250 50  0001 C CNN
	1    7300 3250
	-1   0    0    1   
$EndComp
$Comp
L Device:C C5
U 1 1 5C1235AB
P 6900 3250
F 0 "C5" H 6785 3204 50  0000 R CNN
F 1 "20p" H 6785 3295 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6938 3100 50  0001 C CNN
F 3 "~" H 6900 3250 50  0001 C CNN
	1    6900 3250
	-1   0    0    1   
$EndComp
$Comp
L Device:C C4
U 1 1 5C12CD09
P 6500 3250
F 0 "C4" H 6385 3204 50  0000 R CNN
F 1 "6p2" H 6385 3295 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6538 3100 50  0001 C CNN
F 3 "~" H 6500 3250 50  0001 C CNN
	1    6500 3250
	-1   0    0    1   
$EndComp
$Comp
L Device:C C3
U 1 1 5C12CD53
P 6100 3250
F 0 "C3" H 6215 3296 50  0000 L CNN
F 1 "6p2" H 6215 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6138 3100 50  0001 C CNN
F 3 "~" H 6100 3250 50  0001 C CNN
	1    6100 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR014
U 1 1 5C138140
P 6500 3400
F 0 "#PWR014" H 6500 3150 50  0001 C CNN
F 1 "GNDD" H 6504 3245 50  0000 C CNN
F 2 "" H 6500 3400 50  0001 C CNN
F 3 "" H 6500 3400 50  0001 C CNN
	1    6500 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 3000 6500 3100
$Comp
L Device:Crystal_Small Y1
U 1 1 5C138248
P 6300 3100
F 0 "Y1" H 6300 3325 50  0000 C CNN
F 1 "32768" H 6300 3234 50  0000 C CNN
F 2 "Crystal:Crystal_C38-LF_D3.0mm_L8.0mm_Vertical" H 6300 3100 50  0001 C CNN
F 3 "~" H 6300 3100 50  0001 C CNN
	1    6300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3000 6500 3000
Wire Wire Line
	6400 3100 6500 3100
Connection ~ 6500 3100
$Comp
L power:GNDD #PWR012
U 1 1 5C144D0F
P 6100 3400
F 0 "#PWR012" H 6100 3150 50  0001 C CNN
F 1 "GNDD" H 6104 3245 50  0000 C CNN
F 2 "" H 6100 3400 50  0001 C CNN
F 3 "" H 6100 3400 50  0001 C CNN
	1    6100 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2700 7200 2700
Wire Wire Line
	7000 2700 6900 2700
Wire Wire Line
	5400 2700 6900 2700
Wire Wire Line
	5400 2600 7300 2600
$Comp
L power:GNDD #PWR016
U 1 1 5C14C7F5
P 6900 3400
F 0 "#PWR016" H 6900 3150 50  0001 C CNN
F 1 "GNDD" H 6904 3245 50  0000 C CNN
F 2 "" H 6900 3400 50  0001 C CNN
F 3 "" H 6900 3400 50  0001 C CNN
	1    6900 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR017
U 1 1 5C14C810
P 7300 3400
F 0 "#PWR017" H 7300 3150 50  0001 C CNN
F 1 "GNDD" H 7304 3245 50  0000 C CNN
F 2 "" H 7300 3400 50  0001 C CNN
F 3 "" H 7300 3400 50  0001 C CNN
	1    7300 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3100 6100 3100
Connection ~ 6100 3100
Wire Wire Line
	6100 3100 6200 3100
Connection ~ 6900 2700
Wire Wire Line
	7300 2600 7300 2700
Wire Wire Line
	6900 3100 6900 2700
Connection ~ 7300 2700
Wire Wire Line
	7300 3100 7300 2700
Wire Wire Line
	5800 3300 5400 3300
Wire Wire Line
	5800 3400 5400 3400
Text Label 5800 3300 2    50   ~ 0
SP_L
Text Label 5800 3400 2    50   ~ 0
SP_R
Wire Wire Line
	5400 3500 5800 3500
Text Label 5800 3500 2    50   ~ 0
BOOT1
Wire Wire Line
	5400 2400 5800 2400
Text Label 5800 2400 2    50   ~ 0
BOOT0
Wire Wire Line
	5400 3600 5800 3600
Text Label 5800 3600 2    50   ~ 0
SCK
$Comp
L power:+5V #PWR013
U 1 1 5C166122
P 6100 3900
F 0 "#PWR013" H 6100 3750 50  0001 C CNN
F 1 "+5V" H 6115 4073 50  0000 C CNN
F 2 "" H 6100 3900 50  0001 C CNN
F 3 "" H 6100 3900 50  0001 C CNN
	1    6100 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 5C166181
P 6100 4100
F 0 "R18" H 6170 4146 50  0000 L CNN
F 1 "4k7" H 6170 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6030 4100 50  0001 C CNN
F 3 "~" H 6100 4100 50  0001 C CNN
	1    6100 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4300 6100 4300
$Comp
L power:+5V #PWR015
U 1 1 5C16F716
P 6500 3900
F 0 "#PWR015" H 6500 3750 50  0001 C CNN
F 1 "+5V" H 6515 4073 50  0000 C CNN
F 2 "" H 6500 3900 50  0001 C CNN
F 3 "" H 6500 3900 50  0001 C CNN
	1    6500 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R19
U 1 1 5C16F733
P 6500 4100
F 0 "R19" H 6570 4146 50  0000 L CNN
F 1 "4k7" H 6570 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6430 4100 50  0001 C CNN
F 3 "~" H 6500 4100 50  0001 C CNN
	1    6500 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4250 6100 4300
Wire Wire Line
	6100 3950 6100 3900
Wire Wire Line
	6500 3900 6500 3950
Wire Wire Line
	6500 4250 6500 4400
Wire Wire Line
	6500 4400 5400 4400
Text Label 5800 4300 2    50   ~ 0
SCL
Text Label 5800 4400 2    50   ~ 0
SDA
Wire Wire Line
	4100 4600 3700 4600
Text Label 3700 4600 0    50   ~ 0
SWCLK
Wire Wire Line
	3700 4700 4100 4700
Text Label 3700 4700 0    50   ~ 0
SWDIO
Wire Wire Line
	5400 4500 5800 4500
Wire Wire Line
	5800 4600 5400 4600
Wire Wire Line
	5400 4700 5800 4700
Wire Wire Line
	5800 4800 5400 4800
Text Label 5800 4500 2    50   ~ 0
SD_CS
Text Label 5800 4600 2    50   ~ 0
SD_CLK
Text Label 5800 4700 2    50   ~ 0
SD_DO
Text Label 5800 4800 2    50   ~ 0
SD_DIN
Wire Wire Line
	4100 4400 3700 4400
Text Label 3700 4400 0    50   ~ 0
USB_DM
Wire Wire Line
	3700 4500 4100 4500
Text Label 3700 4500 0    50   ~ 0
USB_DP
$Comp
L Device:R R16
U 1 1 5C1D0086
P 3650 7350
F 0 "R16" V 3550 7325 50  0000 R CNN
F 1 "1k" V 3550 7375 50  0000 L CNN
F 2 "" V 3580 7350 50  0001 C CNN
F 3 "~" H 3650 7350 50  0001 C CNN
	1    3650 7350
	0    1    1    0   
$EndComp
$Comp
L Device:R R15
U 1 1 5C1D00F4
P 3650 7150
F 0 "R15" V 3550 7125 50  0000 R CNN
F 1 "1k" V 3550 7175 50  0000 L CNN
F 2 "" V 3580 7150 50  0001 C CNN
F 3 "~" H 3650 7150 50  0001 C CNN
	1    3650 7150
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 5C1D01B1
P 3650 6550
F 0 "R12" V 3550 6525 50  0000 R CNN
F 1 "1k" V 3550 6575 50  0000 L CNN
F 2 "" V 3580 6550 50  0001 C CNN
F 3 "~" H 3650 6550 50  0001 C CNN
	1    3650 6550
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5C1D01B7
P 3650 6350
F 0 "R11" V 3550 6325 50  0000 R CNN
F 1 "1k" V 3550 6375 50  0000 L CNN
F 2 "" V 3580 6350 50  0001 C CNN
F 3 "~" H 3650 6350 50  0001 C CNN
	1    3650 6350
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5C1D2991
P 3650 6150
F 0 "R10" V 3550 6125 50  0000 R CNN
F 1 "1k" V 3550 6175 50  0000 L CNN
F 2 "" V 3580 6150 50  0001 C CNN
F 3 "~" H 3650 6150 50  0001 C CNN
	1    3650 6150
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 5C1D2997
P 3650 5950
F 0 "R9" V 3550 5925 50  0000 R CNN
F 1 "1k" V 3550 5975 50  0000 L CNN
F 2 "" V 3580 5950 50  0001 C CNN
F 3 "~" H 3650 5950 50  0001 C CNN
	1    3650 5950
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 5C1E191D
P 3650 6950
F 0 "R14" V 3550 6925 50  0000 R CNN
F 1 "1k" V 3550 6975 50  0000 L CNN
F 2 "" V 3580 6950 50  0001 C CNN
F 3 "~" H 3650 6950 50  0001 C CNN
	1    3650 6950
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 5C1E1923
P 3650 6750
F 0 "R13" V 3550 6725 50  0000 R CNN
F 1 "1k" V 3550 6775 50  0000 L CNN
F 2 "" V 3580 6750 50  0001 C CNN
F 3 "~" H 3650 6750 50  0001 C CNN
	1    3650 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 5950 3200 5950
Text Label 3200 5950 0    50   ~ 0
BTN_0
Wire Wire Line
	3200 6150 3500 6150
Text Label 3200 6150 0    50   ~ 0
BTN_1
Wire Wire Line
	3500 6350 3200 6350
Wire Wire Line
	3200 6550 3500 6550
Wire Wire Line
	3200 6750 3500 6750
Wire Wire Line
	3200 6950 3500 6950
Wire Wire Line
	3200 7150 3500 7150
Wire Wire Line
	3200 7350 3500 7350
Text Label 3200 6350 0    50   ~ 0
BTN_2
Text Label 3200 6550 0    50   ~ 0
BTN_3
Text Label 3200 6750 0    50   ~ 0
BTN_4
Text Label 3200 6950 0    50   ~ 0
BTN_5
Text Label 3200 7150 0    50   ~ 0
ENC_A
Text Label 3200 7350 0    50   ~ 0
ENC_B
Wire Wire Line
	3800 5950 4050 5950
Wire Wire Line
	4050 6150 3800 6150
Wire Wire Line
	3800 6350 4050 6350
Wire Wire Line
	4050 6550 3800 6550
Wire Wire Line
	3800 6750 4050 6750
Wire Wire Line
	4050 6950 3800 6950
Wire Wire Line
	3800 7350 4050 7350
Text Label 4050 5950 2    50   ~ 0
DB0
Text Label 4050 6150 2    50   ~ 0
DB1
Text Label 4050 6350 2    50   ~ 0
DB2
Text Label 4050 6550 2    50   ~ 0
DB3
Text Label 4050 6750 2    50   ~ 0
DB4
Text Label 4050 6950 2    50   ~ 0
DB5
Text Label 4050 7350 2    50   ~ 0
DB7
Wire Wire Line
	3800 7150 4050 7150
Text Label 4050 7150 2    50   ~ 0
DB6
$Comp
L Amplifier_Operational:MCP602 U1
U 1 1 5C17DABA
P 2350 1600
F 0 "U1" H 2350 1967 50  0000 C CNN
F 1 "MCP602" H 2350 1876 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2350 1600 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 2350 1600 50  0001 C CNN
	1    2350 1600
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP602 U1
U 2 1 5C17DBE3
P 2350 3050
F 0 "U1" H 2350 3417 50  0000 C CNN
F 1 "MCP602" H 2350 3326 50  0000 C CNN
F 2 "" H 2350 3050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 2350 3050 50  0001 C CNN
	2    2350 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R17
U 1 1 5C182671
P 5500 2000
F 0 "R17" H 5570 2046 50  0000 L CNN
F 1 "22k" H 5570 1955 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 5430 2000 50  0001 C CNN
F 3 "~" H 5500 2000 50  0001 C CNN
	1    5500 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5C18283C
P 1900 1900
F 0 "R4" H 1970 1946 50  0000 L CNN
F 1 "22k" H 1970 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1830 1900 50  0001 C CNN
F 3 "~" H 1900 1900 50  0001 C CNN
	1    1900 1900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2050 1500 1900 1500
Connection ~ 1900 1500
$Comp
L Device:R R1
U 1 1 5C18A86C
P 1700 1700
F 0 "R1" H 1770 1746 50  0000 L CNN
F 1 "10k" H 1770 1655 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1630 1700 50  0001 C CNN
F 3 "~" H 1700 1700 50  0001 C CNN
	1    1700 1700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 1700 2000 1700
Wire Wire Line
	1900 1500 1900 1750
$Comp
L Device:C C1
U 1 1 5C19B1FC
P 1350 1700
F 0 "C1" H 1235 1654 50  0000 R CNN
F 1 "20p" H 1235 1745 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1388 1550 50  0001 C CNN
F 3 "~" H 1350 1700 50  0001 C CNN
	1    1350 1700
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 1700 1500 1700
$Comp
L power:GNDD #PWR06
U 1 1 5C1A388F
P 1900 2100
F 0 "#PWR06" H 1900 1850 50  0001 C CNN
F 1 "GNDD" H 1904 1945 50  0000 C CNN
F 2 "" H 1900 2100 50  0001 C CNN
F 3 "" H 1900 2100 50  0001 C CNN
	1    1900 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2100 1900 2050
$Comp
L power:+3V3 #PWR011
U 1 1 5C1A7BB6
P 5500 1800
F 0 "#PWR011" H 5500 1650 50  0001 C CNN
F 1 "+3V3" H 5515 1973 50  0000 C CNN
F 2 "" H 5500 1800 50  0001 C CNN
F 3 "" H 5500 1800 50  0001 C CNN
	1    5500 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 1800 5500 1850
$Comp
L Device:R R7
U 1 1 5C1B03FD
P 2350 1900
F 0 "R7" H 2420 1946 50  0000 L CNN
F 1 "100k" H 2420 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2280 1900 50  0001 C CNN
F 3 "~" H 2350 1900 50  0001 C CNN
	1    2350 1900
	0    -1   1    0   
$EndComp
Wire Wire Line
	2200 1900 2000 1900
Wire Wire Line
	2000 1900 2000 1700
Connection ~ 2000 1700
Wire Wire Line
	2000 1700 1850 1700
Wire Wire Line
	2500 1900 2700 1900
Wire Wire Line
	2700 1900 2700 1600
Wire Wire Line
	2700 1600 2650 1600
$Comp
L Device:R R5
U 1 1 5C1CAA35
P 1900 2750
F 0 "R5" H 1970 2796 50  0000 L CNN
F 1 "22k" H 1970 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1830 2750 50  0001 C CNN
F 3 "~" H 1900 2750 50  0001 C CNN
	1    1900 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5C1CAA3C
P 1900 3350
F 0 "R6" H 1970 3396 50  0000 L CNN
F 1 "22k" H 1970 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1830 3350 50  0001 C CNN
F 3 "~" H 1900 3350 50  0001 C CNN
	1    1900 3350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2050 2950 1900 2950
Connection ~ 1900 2950
Wire Wire Line
	1900 2950 1900 2900
$Comp
L Device:R R2
U 1 1 5C1CAA46
P 1700 3150
F 0 "R2" H 1770 3196 50  0000 L CNN
F 1 "10k" H 1770 3105 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1630 3150 50  0001 C CNN
F 3 "~" H 1700 3150 50  0001 C CNN
	1    1700 3150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 3150 2000 3150
Wire Wire Line
	1900 2950 1900 3200
$Comp
L Device:C C2
U 1 1 5C1CAA4F
P 1350 3150
F 0 "C2" H 1235 3104 50  0000 R CNN
F 1 "20p" H 1235 3195 50  0000 R CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1388 3000 50  0001 C CNN
F 3 "~" H 1350 3150 50  0001 C CNN
	1    1350 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 3150 1500 3150
$Comp
L power:GNDD #PWR08
U 1 1 5C1CAA57
P 1900 3550
F 0 "#PWR08" H 1900 3300 50  0001 C CNN
F 1 "GNDD" H 1904 3395 50  0000 C CNN
F 2 "" H 1900 3550 50  0001 C CNN
F 3 "" H 1900 3550 50  0001 C CNN
	1    1900 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3550 1900 3500
$Comp
L power:+3V3 #PWR07
U 1 1 5C1CAA5E
P 1900 2550
F 0 "#PWR07" H 1900 2400 50  0001 C CNN
F 1 "+3V3" H 1915 2723 50  0000 C CNN
F 2 "" H 1900 2550 50  0001 C CNN
F 3 "" H 1900 2550 50  0001 C CNN
	1    1900 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 2550 1900 2600
$Comp
L Device:R R8
U 1 1 5C1CAA65
P 2350 3350
F 0 "R8" H 2420 3396 50  0000 L CNN
F 1 "100k" H 2420 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2280 3350 50  0001 C CNN
F 3 "~" H 2350 3350 50  0001 C CNN
	1    2350 3350
	0    -1   1    0   
$EndComp
Wire Wire Line
	2200 3350 2000 3350
Wire Wire Line
	2000 3350 2000 3150
Connection ~ 2000 3150
Wire Wire Line
	2000 3150 1850 3150
Wire Wire Line
	2500 3350 2700 3350
Wire Wire Line
	2700 3350 2700 3050
Wire Wire Line
	2700 3050 2650 3050
Text Label 2900 1600 2    50   ~ 0
SP_L
Text Label 2900 3050 2    50   ~ 0
SP_R
Connection ~ 2700 1600
Connection ~ 2700 3050
Wire Wire Line
	2700 3050 2900 3050
Wire Wire Line
	2700 1600 2900 1600
Wire Wire Line
	1000 1450 750  1450
Wire Wire Line
	750  2900 1000 2900
Text Label 750  1450 0    50   ~ 0
IN_L
Text Label 750  2900 0    50   ~ 0
IN_R
$Comp
L Device:R_POT RV1
U 1 1 5C232615
P 1000 1700
F 0 "RV1" H 930 1746 50  0000 R CNN
F 1 "22k" H 930 1655 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Piher_PT-6-V_Vertical" H 1000 1700 50  0001 C CNN
F 3 "~" H 1000 1700 50  0001 C CNN
	1    1000 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1700 1150 1700
$Comp
L power:GNDD #PWR03
U 1 1 5C23F573
P 1000 1950
F 0 "#PWR03" H 1000 1700 50  0001 C CNN
F 1 "GNDD" H 1004 1795 50  0000 C CNN
F 2 "" H 1000 1950 50  0001 C CNN
F 3 "" H 1000 1950 50  0001 C CNN
	1    1000 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 1950 1000 1850
Wire Wire Line
	1000 1450 1000 1550
$Comp
L Device:R_POT RV2
U 1 1 5C260260
P 1000 3150
F 0 "RV2" H 930 3196 50  0000 R CNN
F 1 "22k" H 930 3105 50  0000 R CNN
F 2 "" H 1000 3150 50  0001 C CNN
F 3 "~" H 1000 3150 50  0001 C CNN
	1    1000 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3150 1150 3150
$Comp
L power:GNDD #PWR04
U 1 1 5C260268
P 1000 3400
F 0 "#PWR04" H 1000 3150 50  0001 C CNN
F 1 "GNDD" H 1004 3245 50  0000 C CNN
F 2 "" H 1000 3400 50  0001 C CNN
F 3 "" H 1000 3400 50  0001 C CNN
	1    1000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 3400 1000 3300
Wire Wire Line
	1000 2900 1000 3000
Wire Wire Line
	5500 2150 5500 2200
Wire Wire Line
	5500 2200 5400 2200
$Comp
L Device:R R3
U 1 1 5C19A62C
P 1900 1250
F 0 "R3" H 1970 1296 50  0000 L CNN
F 1 "22k" H 1970 1205 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1830 1250 50  0001 C CNN
F 3 "~" H 1900 1250 50  0001 C CNN
	1    1900 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR05
U 1 1 5C19A634
P 1900 1050
F 0 "#PWR05" H 1900 900 50  0001 C CNN
F 1 "+3V3" H 1915 1223 50  0000 C CNN
F 2 "" H 1900 1050 50  0001 C CNN
F 3 "" H 1900 1050 50  0001 C CNN
	1    1900 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1050 1900 1100
Wire Wire Line
	1900 1400 1900 1500
Wire Wire Line
	5500 2200 5800 2200
Connection ~ 5500 2200
Text Label 5800 2200 2    50   ~ 0
NRST
$Comp
L Device:R R20
U 1 1 5C373CA2
P 700 5850
F 0 "R20" H 770 5896 50  0000 L CNN
F 1 "22k" H 770 5805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 630 5850 50  0001 C CNN
F 3 "~" H 700 5850 50  0001 C CNN
	1    700  5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  6200 700  6000
Wire Wire Line
	700  6200 1450 6200
$Comp
L power:+3V3 #PWR018
U 1 1 5C38284E
P 700 5600
F 0 "#PWR018" H 700 5450 50  0001 C CNN
F 1 "+3V3" H 715 5773 50  0000 C CNN
F 2 "" H 700 5600 50  0001 C CNN
F 3 "" H 700 5600 50  0001 C CNN
	1    700  5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  5600 700  5700
Wire Wire Line
	1200 6300 1450 6300
Wire Wire Line
	1200 6400 1450 6400
Wire Wire Line
	1450 6500 1200 6500
Wire Wire Line
	1200 6600 1450 6600
Wire Wire Line
	1200 6700 1450 6700
Wire Wire Line
	1200 6800 1450 6800
Wire Wire Line
	1200 6900 1450 6900
Wire Wire Line
	1200 7000 1450 7000
Text Label 1200 6300 0    50   ~ 0
DB8
Text Label 1200 6400 0    50   ~ 0
DB9
Text Label 1200 6500 0    50   ~ 0
DB10
Text Label 1200 6600 0    50   ~ 0
DB11
Text Label 1200 6700 0    50   ~ 0
DB12
Text Label 1200 6800 0    50   ~ 0
DB13
Text Label 1200 6900 0    50   ~ 0
DB14
Text Label 1200 7000 0    50   ~ 0
DB15
$EndSCHEMATC