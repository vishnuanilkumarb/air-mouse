import serial
import pyautogui

ser = serial.Serial('COM15', 115200)

while True:
    try:
        data = ser.readline().decode().strip().split(',')
        gx, gy = int(data[0]), int(data[1])

        moveX = int(gx / 500)
        moveY = int(gy / 500)
        pyautogui.moveRel(moveX, moveY)

    except:
        pass
