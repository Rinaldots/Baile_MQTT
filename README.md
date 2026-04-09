# Baile de Robôs - ESP32 MQTT

Este repositório contém o firmware (C++) e a interface de telemetria 3D (HTML/JS) para o controle de um robô de acionamento diferencial (Differential Drive) utilizando um **ESP32** via **MQTT**. O projeto foca em navegação precisa baseada em odometria de encoders, integração com IMU (MPU6050) em alta frequência e visualização remota em tempo real.

## 🛠 Funcionalidades Principais
* **Controle Cinemático e PID:** Malha de controle rodando a 200Hz para leitura de encoders, estimação de odometria e cálculo da cinemática diferencial.
* **Integração IMU:** Leitura de Acelerômetro e Giroscópio (MPU6050) via FreeRTOS.
* **Comunicação MQTT:** Recebimento de comandos de navegação (X, Y, Theta) e transmissão de telemetria contínua.
* **Sistema de Áudio:** Integração de efeitos sonoros através do módulo DFPlayer Mini.
* **Dashboard 3D Web:** Interface desenvolvida com Three.js (com OrbitControls, GLTFLoader) exibindo um CAD interativo do robô em tempo real de acordo com as coordenadas e acelerações obtidas.
* **Análise de Sistemas de Controle:** Scripts Python avançados para levantar a função de transferência dos motores e aplicar regressões estatísticas para sintonia fina do PID.

## 📁 Estrutura do Projeto

```text
├── controle/         # Scripts Python (.py) e Jupyter (.ipynb) p/ coleta e análise das FF.TT. dos motores
├── include/          # Cabeçalhos globais e declarações (se necessário)
├── lib/
│   ├── diff_car/     # Lógica central: Cinematica, PID, Encoder, Ponte H, IMU, Leds e Configurações (200Hz/50Hz)
│   ├── mqtt/         # Utilitários de parsing de comandos e controle MQTT
│   └── network/      # Configurações confidenciais/pessoais da rede Wi-Fi e credenciais
├── src/
│   └── main.cpp      # Ponto de entrada, configuração do FreeRTOS e laços principais
├── test/             
│   ├── index.html    # Dashboard de telemetria Web 3D (Javascript + Three.js + MQTT via WebSockets)
│   └── models/       # Arquivos .gltf e .glb utilizados na montagem visual do robô no Dashboard
└── platformio.ini    # Informações de build, dependências de libs e board param
```

## ⚙️ Tecnologias e Bibliotecas (PlatformIO)
O projeto é compilado usando [PlatformIO](https://platformio.org/). Dependências automáticas presentes no `platformio.ini`:
* `PubSubClient` / `EspMQTTClient` (Pub/Sub do MQTT)
* `MPU6050` da ElectronicCats (Inércia)
* `DFRobotDFPlayerMini` e `EspSoftwareSerial` (Som)

## 🚀 Como Executar

### 1. Firmware (ESP32)
1. Instale o **PlatformIO** como extensão do VS Code.
2. Clone este repositório e abra a pasta raiz (`Baile_MQTT`).
3. Renomeie/configure os arquivos em `lib/network/` para apontar para o seu Roteador Wi-Fi e Broker MQTT.
4. Conecte o ESP32 na porta USB, clique na aba do PlatformIO e execute o `Upload`.

### 2. Dashboard Web (UI 3D)
1. Ter um broker MQTT configurado capaz de usar conexões via WebSockets (ex: Mosquitto alterado para a porta 9001).
2. Abra o arquivo `test/index.html` em um navegador.
3. Insira o endereço IP/porta correto do seu Broker WS no início da tag `<script>` no HTML.
4. Interaja livremente movimentando o cursor do mouse e dando "scroll" para dar zoom na visualização 3D do Caster e demais peças, ou utilizar os visores do `Chart.js` embutidos.

### 3. Análise de Malha de Controle
Para usar as ferramentas matemáticas desenvolvidas:
1. Navegue para o diretório `controle/`.
2. Instale os requerimentos (`pandas`, `scipy`, `matplotlib`, `pyserial`).
3. Rode `coleta_motor.py` para amostragem serial de degraus de velocidade.
4. Rode `analisar_transferencia.py` para levantar os CIs e dados de regime permanente em relação ao PWM.
