Set fso = CreateObject("Scripting.FileSystemObject")
dir = fso.GetParentFolderName(WScript.ScriptFullName)
python = "%USERPROFILE%\.platformio\penv\Scripts\python.exe"
Set ws = CreateObject("Wscript.Shell")
ws.CurrentDirectory = dir
ws.Run """" & python & """ """ & dir & "\pc_receiver.py""", 1, False
