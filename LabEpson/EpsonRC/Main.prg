Function main
	Motor On
	Power High
	Accel 30, 30
	Speed 50
	Home
	Do
		If MemSw(512) Then
			Call paletizadoZ
			Home
		EndIf
		If MemSw(513) Then
			Call paletizadoS
			Home
		EndIf
		If MemSw(514) Then
			Call paletizadoExterno
			Home
		EndIf
	Loop
Fend


