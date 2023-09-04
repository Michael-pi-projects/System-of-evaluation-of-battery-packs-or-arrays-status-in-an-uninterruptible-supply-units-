#!/usr/bin/env python3

import serial 
import time
import yagmail
# we import the needed libraries 

password = ""
# a variable to store the email password of sender

with open ("/home/pi/.local/share/.email_password", "r") as f :
    # this code receives the password and logs to the sender e mai
    # from the path that we have seved the password
    password = f.read()
                        # reads the password from the saved file
yag = yagmail.SMTP('example_email_1@gmail.com', password)
# the first parameter of the comand is the email so send what we want example :'example_email_1@gmail.com'
# the second parameter is the password variable with app pasword stored 


while True:
# we want to make the connection betwenn the arduino and raspberry automatic
# so that's the reason we put in in a while loop
    try: # it tries to connect if it is possible and everything correct
        ser = serial.Serial('/dev/ttyACM0' , 115200 , timeout=1.0)
        # the '/dev/ttyUSB0' tells the raspberry which port the arduino is conected
        # the 115200 is the bouad rate of the arduino . It is VERY IMPORTANT  that
        # the bouad rate  we put here is the same as the bouad rate of the arduino
        # the ser is the variable that the data from the serial is stored

        # This line opens the serial communication
        # and if it works we get the serial in the
        # ser(variable) , if it doesn't work we get an
        # and the program is gone exit
        print("Successfully connected to Serial")
        break
        # if the connection is established it exits the loop
    except serial.SerialException:
        # if the connection is not possible (problem to connect)
        print("Could not connect to Serial. Trying to connect again..")
        time.sleep(2)
        # it waits 2 second until it tries to reconnect
        # It reconnects after the except because it goes back at the start of while
        # loop and execute the try command again
    

time.sleep(2)
# this command tels the raspberry to wait some time
# because every time that the raspberry opens the serial communication with
# the arduino , the arduino restarts and it needs some time
# to start runing again the code .
# The time  to wait till the arduino restarts it depends
# on how much time the arduino code takes when it restarts

ser.reset_input_buffer()
# it clears and resets the buffer.
# when the informtion comes from the arduino
# it is stored in a temporarry  memory the buffer
# so we clean it from everything that it is
# previusly stored , fro the new data to be stored

print ('Serial OK...')
# it print that the serial communication is ok

with open("/home/pi/Data/Data.csv", "a") as log:
    # we need to provide-write the corect path to
    # the folder that we want to store the data
    
    # It opens a Data.csv named file in a Data named directory
    # It also opens  it in append mode ( "a" ), so that lines
    # are only written to the end of the file.
    # the lines are not overwriten and they are stored one after another 
    try:
        # it runs the loop until we press ctrl+c to close the communicaton
        while True:
            time.sleep(0.01) #the speed  that the loop runs
            if ser.in_waiting > 0:
                # it checks if there is any byte available and if
                # there is any number ( number > 0)  of bytes it means tha some data have beed send
                time.sleep(1.010)
                # it waits 2 second for the value of the current sensor to be sent and write the value in proper
                # plae next to the other values
                # if we dont wait  it will send in one line all the other
                # measurments and the curent in
                # a new line , it takes one extra second
                # (or the same time in the arduino code )for the arduino
                # to measure the curent and display it next to
                # the other measurments
                line = ser.readline().decode('utf-8').rstrip()
                # it reads each  next liwne
                # then it decodes the data we recieve with the function .decode('utf-8')
                # then we use the function .rstrip() to clear any new line character or
                # any other character that is unnecessary
                # and it stores the data as a string
                log.write("{0},\n".format(line))
                # it stores the Data from the serial of arduino that
                # have been stored in a string  named Line
                # and in the end of each string it goes to a new line for
                # the next string
                if line.find('UnderVoltage')!= -1:
                    #  if we find in the string the word we want ( example : is )
                    # then we send an email with  the notification we want to send
                    # every time the the loop is runs ( example : every 2 seconds )
                    # usibg find ffunction
                    yag.send(to='example_email_2@gmail.com',
                             # the email of receiver
                             subject ="Arduino sensor notification",
                             # title of the notification
                             contents = "Undervoltage")
                             # title of the incident 
                             # attachments = "/home/pi/Data/Data.csv")
                             # here we write the path of the file we want to send if it is needed
                    print("Email sent")
                if line.find('OverCurrent')!= -1:
                    #  if we find in the string the word we want ( example : is )
                    # then we send an email with  the notification we want to send
                    # every time the the loop is runs ( example : every 2 seconds )
                    # usibg find ffunction
                    yag.send(to='example_email_2@gmail.com',
                             # the email of receiver
                             subject ="Arduino sensor notification",
                             # title of the notification
                             contents = "Overcurrent")
                             # title of the incident 
                             # attachments = "/home/pi/Data/Data.csv")
                             # here we write the path of the file we want to send if it is needed
                    print("Email sent")
                print(line)
                # it prints the line string variable
    except KeyboardInterrupt:
         yag.send(to='example_email_2@gmail.com',
                             # the email of receiver
                             #subject ="Arduino sensor notification",
                             # title of the notification
                             contents = "Data",
                             # title of the incident 
                             attachments = "/home/pi/Data/Data.csv")
                             # here we write the path of the file we want to send if it is needed
                             # this wait to press the ctrl+c to close the commuication
    print("Email sent")
    print("Close serial communication...")
    ser.close()
    # it closes the serial communication
    # in order to colse  the serial communication
    # we click in the sheel at thonny serial output window
    # and press ctrl+c
