import serial
import serial.tools.list_ports
import ctypes
import time

def find_esp_port():
    """Автоматично шукає порт, до якого підключено ESP (UART адаптер)"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Шукаємо за ключовими словами драйверів, які ми встановлювали (CP210, FT232, USB Serial)
        if "UART" in port.description or "Serial" in port.description or "CP210" in port.description:
            return port.device
    return None

def main():
    port = find_esp_port()
    if not port:
        print("Помилка: Плату ESP не знайдено. Перевір підключення або вкажи порт вручну.")
        return

    baud_rate = 115200
    print(f"Підключення до {port} на швидкості {baud_rate}...")

    try:
        # Налаштування з'єднання
        ser = serial.Serial(port, baud_rate, timeout=1)
        print("Скрипт запущено. Очікування команди від ESP...")

        while True:
            if ser.in_waiting > 0:
                # Читаємо рядок з порту, декодуємо та прибираємо зайві пробіли/символи
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line == "CMD_LOCK_PC":
                    print(f"[{time.strftime('%H:%M:%S')}] Отримано команду! Блокування Windows...")
                    # Виклик системної функції блокування (Win + L)
                    ctypes.windll.user32.LockWorkStation()
                    # Затримка, щоб уникнути черги однакових команд
                    time.sleep(5) 
            
            time.sleep(0.1) # Зменшуємо навантаження на процесор

    except serial.SerialException as e:
        print(f"Помилка порту: {e}")
    except KeyboardInterrupt:
        print("\nСкрипт зупинено користувачем.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()