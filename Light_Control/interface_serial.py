#!/usr/bin/env python3
#time,luminaire_ID,lux_ref,lux_meas,ldrvoltage,ldrresistance,duty_cycle,windup_state,setpoint-weight
"""
Script para ler dados do ADC do Raspberry Pi Pico e gravar num ficheiro.
"""

import serial
import serial.tools.list_ports
import time
from datetime import datetime
import threading
import sys
import argparse
import os

RPI_PORTS = {
    "rpi1": "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E66118604B886021-if00",
    "rpi2": "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E66118604B497921-if00",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interface serial para Raspberry Pi Pico"
    )
    parser.add_argument(
        "--rpi",
        choices=["rpi1", "rpi2"],
        help="Seleciona automaticamente a porta USB do RPI 1 ou RPI 2",
    )
    parser.add_argument(
        "--port",
        help="Define explicitamente a porta serial (ex: /dev/ttyACM0)",
    )
    return parser.parse_args()

def find_pico_port():
    """Procura automaticamente a porta do Raspberry Pi Pico."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # O Pico geralmente aparece como "USB Serial Device" ou similar
        if "USB" in port.description or "ACM" in port.device or "ttyUSB" in port.device:
            return port.device
    return None


def resolve_port(args):
    """Resolve a porta serial conforme prioridade: --port > --rpi > auto-deteção."""
    if args.port:
        return args.port

    if args.rpi:
        selected = RPI_PORTS[args.rpi]
        return selected

    return find_pico_port()


def wait_for_port(port_path, timeout_s=12.0, poll_s=0.2):
    """Espera a porta aparecer (útil após reset/upload do Pico)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if os.path.exists(port_path):
            return True
        time.sleep(poll_s)
    return False

def main():
    args = parse_args()

    # Tenta resolver porta por argumentos e fallback automático
    port = resolve_port(args)

    # Quando uma porta específica é pedida, aguarda reaparecer após upload/reset.
    if port and (args.rpi or args.port) and not os.path.exists(port):
        print(f"A aguardar porta {port} ficar disponível...")
        if not wait_for_port(port):
            print(f"Porta não apareceu a tempo: {port}")
            if args.rpi:
                print("Não vou usar auto-deteção para evitar ligar ao RPI errado.")
                print("Confirme se a board está ligada e se o ID no script está correto.")
            port = None
    
    if not port:
        if args.rpi:
            print("Porta USB do RPI selecionado não encontrada.")
            return
        print("Porta USB não encontrada automaticamente.")
        port = input("Insira a porta serial (ex: /dev/ttyACM0 ou /dev/ttyUSB0): ")
    else:
        print(f"Porta encontrada: {port}")
    
    # Nome do ficheiro com timestamp
    filename = f"adc_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    try:
        # Abre conexão serial
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Aguarda inicialização
        
        print(f"Conectado a {port}")
        print(f"A gravar dados em: {filename}")
        print("Digite comandos e pressione Enter para enviar ao Pico")
        print("Pressione Ctrl+C para parar...\n")
        
        # Flag para controlar threads
        running = True
        
        def read_commands():
            """Thread para ler comandos do utilizador"""
            nonlocal running
            try:
                while running:
                    try:
                        cmd = input()
                        if cmd.strip():
                            # Envia comando para o Pico
                            ser.write((cmd + "\n").encode('utf-8'))
                            print(f"[Enviado] {cmd}")
                    except EOFError:
                        break
                    except Exception as e:
                        if running:
                            print(f"Erro ao enviar comando: {e}")
            except KeyboardInterrupt:
                pass
        
        # Inicia thread de leitura de comandos
        cmd_thread = threading.Thread(target=read_commands, daemon=True)
        cmd_thread.start()
        
        # Abre ficheiro para escrita
        # with open(filename, 'w') as f:
            # Escreve cabeçalho
            # f.write("time,luminaire_ID,lux_ref,lux_meas,ldrvoltage,ldrresistance,duty_cycle,windup_state,setpoint-weight\n")
            
        while running:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                if line:
                    # Imprime linhas sem vírgulas (respostas do Pico a comandos)
                    if ',' not in line:
                        print(line)
                    # Grava linhas com vírgulas no ficheiro (dados CSV)
                    #elif not line.startswith("Sistema") and not line.startswith("LED"):
                    #    f.write(line + "\n")
                    #    f.flush()  # Garante que os dados são escritos imediatamente
                
    except KeyboardInterrupt:
        running = False
        print("\n\nInterrompido pelo utilizador.")
        print(f"Dados gravados em: {filename}")
        
    except serial.SerialException as e:
        print(f"Erro ao abrir porta serial: {e}")
        print("\nPortas disponíveis:")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  - {port.device}: {port.description}")
    
    except Exception as e:
        print(f"Erro: {e}")
    
    finally:
        running = False
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Porta serial fechada.")

if __name__ == "__main__":
    main()
