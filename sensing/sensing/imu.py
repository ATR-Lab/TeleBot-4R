import serial

ser = serial.Serial("/dev/ttyACM0", baudrate=9600)

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8')[:-2]
        values = line.split('\t')
        split_values = []
        for value in values:
            split_values.append(float(value.split(": ")[1]))
        print(split_values)