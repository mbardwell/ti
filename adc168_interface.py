# Do not use the logging module. It will interfere with std{in,out}.
import csv
import getopt
import sys
import termios
import tty
from time import sleep

from serial import Serial


def main(port: str, cmd: bytes) -> None:
    kBaudrate = 9600
    kTimeout = 0.1
    with Serial(port=port, baudrate=kBaudrate, timeout=kTimeout) as serial:
        if cmd == b"stdin":
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setraw(sys.stdin)
                while True:
                    char = sys.stdin.read(1)
                    if char == "\x03":  # This is the ASCII for Ctrl+C
                        break
                    sys.stdout.write(char)
                    sys.stdout.write("\r\n")
                    sys.stdout.flush()
                    serial.write(char.encode())
                    for c in serial.read(96):
                        sys.stdout.write(f"0x{c:02x} 0b{c:08b}\r\n")
                    sys.stdout.flush()
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        elif cmd == b"csv":
            for cmd in [b"r", b"a", b"y"]:
                serial.write(cmd)
                sleep(0.5)
                print(f"{int.from_bytes(serial.read(3), byteorder='big'):024b}")
            serial.write(b"7")  # capture
            received_data = serial.read(4 * 2 * 3)
            assert len(received_data) % 3 == 0
            adc_output = [
                int.from_bytes(received_data[i : i + 3], byteorder="big")
                for i in range(0, len(received_data), 3)
            ]
            with open("output.csv", "w", newline="") as csvfile:
                csvwriter = csv.writer(csvfile, delimiter=",")
                csvwriter.writerow(["channel", "adc code", "volts", "raw code"])
                for i, packet in enumerate(adc_output):
                    channel = f"{(i % 8) // 2}{'A' if i % 2 == 0 else 'B'}"
                    adc_code = (
                        packet << 2 & 0xFFFFFF
                    ) >> 8  # adc_output looks like [0, A/B bit, ...16 bit code..., 0...]
                    voltage = ((adc_code & 0x7FFF) / (2**15 - 1)) * 2.5 + (
                        0 if adc_code & (1 << 15) else 2.5
                    )
                    csvwriter.writerow(
                        [
                            channel,
                            f"{adc_code:016b}",
                            f"{voltage:0.6f}",
                            f"{packet:024b}",
                        ]
                    )


if __name__ == "__main__":
    opts, args = getopt.getopt(sys.argv[1:], "hp:c:", ["help", "port=", "cmd="])
    port = "/dev/ttyACM1"
    cmd: bytes = b"stdin"
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print("Help")
        elif opt in ("-p", "--port"):
            port = arg
        elif opt in ("-c", "--cmd"):
            cmd = arg.encode()
    main(port, cmd)
