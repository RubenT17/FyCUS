"""
/* **********************************************************************************
 *
 *  file            logger_python.py
 *  Created on: 	24.08.2022
 *  Author: 		Rubén Torres Bermúdez <rubentorresbermudez@gmail.com>
 *  Organism:		FyCUS. Universidad de Sevilla.
 *
 *	Description:
 *		This Phyton source file provides the data logger for the Cubesat.
 *      It is a simple data logger that saves the data in a .csv file in the
 *      current directory and in a backup directory "./data_backup". 
 *      
 *		logger_python.py Copyright (C) 2022  Rubén Torres Bermúdez
 *
 * **********************************************************************************
*/
"""



import serial
from os import mkdir, _exit, chdir
from os.path import isdir
from time import sleep, time, strftime
from shutil import move


PORT = "/dev/ttyACM0"
MIN_TO_SAVE = 0.12
DIRECTORY = "/home/pi/Desktop"
tiempo = time()
cuenta = 0


def check_path():
    if not isdir("./data_backup"):
        mkdir("./data_backup")        
        
def serial_init():
    ser = serial.Serial(PORT, 115200)
    return ser


# guardar datos en txt
def save_data(data):
    global tiempo, cuenta
    with open('data.csv', "a") as file:
        file.write(strftime("%H:%M:%S") + "," + data)
        file.close()
    if((time()-tiempo) > (60*MIN_TO_SAVE)):
        check_path()
        move("data.csv", "data_backup/" + strftime("%H:%M:%S") + f"_{cuenta}.csv")
        cuenta = cuenta + 1
        tiempo = time()




# recibir datos de serial y decodicarlos 
def data_Rx():
    while True:
        if ser.inWaiting():
            data = ser.readline().decode("utf-8").strip("\n")
            save_data(data)




if __name__ == "__main__":
    chdir(DIRECTORY)
    check_path()
    while True:
        sleep(3)
        try:
            ser = serial_init()
        except KeyboardInterrupt:
            print("KEYBOARD INTERRUPT")
            ser.close()
            _exit(0)    
        except:
            print("Fallo al iniciar serial")
            continue
        
        while True:
            try:
                data_Rx()
        
            except KeyboardInterrupt:
                print("KEYBOARD INTERRUPT")
                ser.close()
                _exit(0)     
    
            except:
                try:
                    print("Fallo en comunicación serial")
                    ser.close()
                finally:
                    break       
                
