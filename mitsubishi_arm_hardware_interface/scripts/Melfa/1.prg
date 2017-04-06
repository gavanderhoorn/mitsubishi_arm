1 ' Version 0.1.0
2 ' Wait for IP address from client, then open for MXT"
3 ' Declare variables
4 C1$="STORMLAB"  ' Password
5 CIP$= "ENET:192.168.0.3"
6 'Begin program
7 *BEGIN
8 Servo Off 'In case it is on
9 Close #1 'In case it is open
10 Close #2 'In case it is open
11 Open "COM1:" As #2 'Open TCP/IP on port 10003 and wait for password
12 Input #2, C2$
13 If C2$ = C1$ Then ' If password is valid
14 Open CIP$ As #1 ' Open UDP socket
15 Servo On ' Turn servo on
16 Print #2, 1 'Tell the client initialization has succeeded
17 Ovrd 5 ' Use reduced speed
18 Mxt 1,1,100 ' Start MXT in Joint mode
19 Dly 1 ' Wait for a second
20 Close #1
21 Servo Off ' Done with servo, so turn it off
22 Else 'Otherwise, print an error and go back to beggining
23 Print #2,-1
24 EndIf
25 GoTo *BEGIN ' Go back to beginning
26 End
