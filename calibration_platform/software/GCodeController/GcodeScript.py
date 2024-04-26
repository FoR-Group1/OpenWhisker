import serial
import time


portName = "/dev/ttyACM0"


def portIsUsable(port=portName):
    try:
        ser = serial.Serial(port=port)
        return True
    except:
        return False


def sendGcode(gcode):
    print("sending")
    ser.write(gcode.encode())
    response = ser.readline()
    # print(portIsUsable())
    # while not portIsUsable():
    #     print("hello")
    #     print(response)
    print("hello_2")


def dabGcode():
    return NotImplementedError


def beamTheoryGcode():
    return NotImplementedError


def brushGcode():
    return NotImplementedError


def beamTheoryGcode():
    return "G1 X115: G1 X90: G1 X115: G1 X90:G1 X115: G1 X90:G1 X115: G1 X90:G1 X115: G1 X90:"


if __name__ == "__main__":
    print("-- starting --")
    ser = serial.Serial(portName, 115200)

    g_code_1 = "G28 XY: G1 Y100 X20:"
    sendGcode(g_code_1)
    # time.sleep(5)
    sendGcode("G1 F9000")

    # while True:
    #     print("Repeating")
    #     g_code_2 = "G0 X100: G4 S2:"
    #     time.sleep(2)
    #     sendGcode(g_code_2)
    #     g_code_3 = "G0 X200: G4 S2:"
    #     time.sleep(2)
    #     sendGcode(g_code_3)

    sendGcode(beamTheoryGcode())
    print("ja1")

    ser.close()

# time.sleep(2)
# ser.write("G28X\n:")
# time.sleep(1)
# ser.close()
