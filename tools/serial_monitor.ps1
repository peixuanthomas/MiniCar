param(
    [string]$Port = "COM4",
    [int]$Baud = 115200
)

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$serial.NewLine = "`n"
$serial.ReadTimeout = 500

try {
    $serial.Open()
    Write-Host "Listening on $Port @ $Baud. Press Ctrl+C to stop."

    while ($true) {
        try {
            $line = $serial.ReadLine()
            Write-Host $line.TrimEnd("`r", "`n")
        } catch [System.TimeoutException] {
        }
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
