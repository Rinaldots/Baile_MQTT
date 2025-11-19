import serial
import csv
import time
import re
from datetime import datetime


class MotorDataCollector:
    def __init__(self, port='COM3', baudrate=115200, csv_filename=None):
        """
        Inicializa o coletor de dados do motor.
        """
        self.port = port
        self.baudrate = baudrate
        
        # Nome automático se não informado
        if csv_filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_filename = f"dados_motores_{timestamp}.csv"
        else:
            self.csv_filename = csv_filename

        self.serial_conn = None
        self.csv_file = None
        self.csv_writer = None

        # Regex para capturar valores
        self.data_pattern = re.compile(
            r'Left Velocity \(m/s\):\s*([-\d\.]+)\s*\|\s*Left PWM:\s*([-\d\.]+)\s*\|\|\s*Right Velocity \(m/s\):\s*([-\d\.]+)\s*\|\s*Right PWM:\s*([-\d\.]+)',
            re.IGNORECASE
        )

    def connect_serial(self):
        """Conecta à porta serial"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.002)  
            # timeout curtíssimo → leitura não bloqueante
            print(f"Conectado à porta {self.port} com baudrate {self.baudrate}")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"Erro ao conectar: {e}")
            return False

    def setup_csv(self):
        """Cria o arquivo CSV"""
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ['timestamp', 'datetime', 'left_velocity_ms', 'left_pwm', 'right_velocity_ms', 'right_pwm']
            )
            print(f"Arquivo CSV criado: {self.csv_filename}")
            return True
        except Exception as e:
            print(f"Erro ao criar CSV: {e}")
            return False

    def parse_motor_data(self, line: str):
        """Extrai dados usando regex"""
        match = self.data_pattern.search(line)
        if match:
            return (
                float(match.group(1)),
                float(match.group(2)),
                float(match.group(3)),
                float(match.group(4))
            )
        return None

    def collect_data(self, duration=None):
        """Coleta dados com máxima frequência possível"""
        if not self.connect_serial():
            return

        if not self.setup_csv():
            return

        print("\nIniciando coleta rápida...")
        start = time.time()
        count = 0

        try:
            buffer = ""

            while True:
                # Parar após 'duration' segundos
                if duration and (time.time() - start) >= duration:
                    break

                # Ler o máximo possível do buffer serial
                data = self.serial_conn.read(1024).decode('utf-8', errors='ignore')

                if data:
                    buffer += data

                    # Processar linhas completas
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        parsed = self.parse_motor_data(line)
                        if parsed:
                            lv, lpwm, rv, rpwm = parsed
                            now = time.time()
                            dt_str = datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                            # escrever no CSV
                            self.csv_writer.writerow([now, dt_str, lv, lpwm, rv, rpwm])

                            count += 1
                            if count % 20 == 0:
                                print(f"{count} | L={lv:.2f} m/s  R={rv:.2f} m/s  PWM_L={lpwm:.0f} PWM_R={rpwm:.0f}")

        except KeyboardInterrupt:
            print("\nInterrompido pelo usuário.")

        finally:
            self.cleanup()
            print(f"Total de linhas coletadas: {count}")
            print(f"Arquivo salvo: {self.csv_filename}")

    def cleanup(self):
        """Fecha arquivos e portas"""
        if self.serial_conn:
            self.serial_conn.close()
            print("Conexão serial fechada.")
        if self.csv_file:
            self.csv_file.close()
            print("Arquivo CSV fechado.")


def main():
        PORT = 'COM3'
        BAUDRATE = 115200

        collector = MotorDataCollector(port=PORT, baudrate=BAUDRATE)

        print("=== Coletor de Dados Rápido ===")
        collector.collect_data(duration=100)   # coleta por 100s


if __name__ == "__main__":
    main()
