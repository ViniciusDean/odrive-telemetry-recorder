# ODrive Telemetry Recorder

Aplicativo desktop externo para gravar a telemetria HID do Odrive-Wheel. Ele nao abre o configurador, nao precisa de WebHID e nao envia comandos para o motor.

Baseado no protocolo de telemetria exposto pelo projeto principal [eagabriel/Odrive-Wheel](https://github.com/eagabriel/Odrive-Wheel).

## Uso

1. Conecte o volante por USB.
2. Execute run.ps1.
3. Clique em Procurar volante.
4. Confira range, current limit, resistor de freio e resistencia de fase.
5. Clique em Iniciar gravacao, rode a sessao e clique em Parar e analisar.

Os CSVs sao salvos na pasta recordings ao lado do executavel, nunca em pasta temporaria. O CSV bruto contem todas as amostras HID; o CSV de resumo contem os minimos, maximos, clipping de FFB pelo max torque configurado e clipping fisico por corrente.

## Gerar executavel

Execute build.ps1. O arquivo sera criado em dist/OdriveTelemetryRecorder.exe.
