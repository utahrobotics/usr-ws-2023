from serial import Serial


def main():
    controller = Serial(
        "/dev/drive_control",
        115200
    )

    while True:
        print(controller.readline().decode())


if __name__ == "__main__":
    main()
