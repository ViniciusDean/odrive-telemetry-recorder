$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $root '.venv\Scripts\python.exe'

if (-not (Test-Path -LiteralPath $python)) {
    throw 'Ambiente Python nao encontrado. Execute run.ps1 uma vez.'
}

& $python -m pip install pyinstaller
$arguments = @(
    '-m', 'PyInstaller', '--noconfirm', '--clean', '--windowed', '--onefile',
    '--name', 'OdriveTelemetryRecorder',
    '--distpath', (Join-Path $root 'dist'),
    '--workpath', (Join-Path $root 'build'),
    '--specpath', $root,
    (Join-Path $root 'odrive_telemetry_recorder.py')
)
& $python @arguments
