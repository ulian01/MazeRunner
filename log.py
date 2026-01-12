import sqlite3, time, serial

PORT = "/dev/tty.usbmodemF0F5BD50E6942"  # change to port
ser = serial.Serial(PORT,9600,timeout=1) # port = hub port, 9600 serial.begin freq, timeout 1 = wait 1 sec untill continuing
db = sqlite3.connect("connectivity.db") # open database

while True:
    m = ser.readline().decode(errors = "ignore").strip() # read untill newline, remove /n and /r,
    if m:
        db.execute("INSERT INTO traffic(ts,msg) VALUES(?,?)",(time.strftime("%F %T"),m)) #insert into db
        db.commit() # save changes
        print(m)
